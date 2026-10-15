#include "bms_usb_host.h"
#include "config.h"

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstring>
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

// Portado da versão Arduino (bms_usb_host.cpp), que já usava a API USB Host
// do ESP-IDF. Diferenças: Serial.print virou ESP_LOGx, read() espera com
// timeout (a tarefa do parser bloqueia nele) e 'running' virou atomic.

namespace {

const char *TAG = "USB";

// ===================== Estado USB =====================
enum ChipType { CHIP_NONE, CHIP_CDC_ACM, CHIP_CH34X, CHIP_CP210X, CHIP_FTDI };
const char *const CHIP_NAMES[] = { "nenhum", "CDC-ACM", "CH340/CH341", "CP210x", "FTDI" };

const int NUM_IN_XFERS = 2;  // duas transferências na fila para não perder bytes

// Eventos do callback do cliente USB para a tarefa do dispositivo
const uint32_t EVT_NEW_DEV = 1u << 0;
const uint32_t EVT_DEV_GONE = 1u << 1;

uint32_t baudRate = BMS_BAUD;

usb_host_client_handle_t clientHdl = nullptr;
usb_device_handle_t devHdl = nullptr;
TaskHandle_t devTaskHdl = nullptr;
volatile uint8_t pendingDevAddr = 0;

ChipType chip = CHIP_NONE;
bool hasCdcComm = false;
uint8_t ifaceComm = 0;
uint8_t ifaceData = 0;
bool ifaceClaimed = false;
uint8_t epIn = 0;
uint16_t epInMps = 64;

usb_transfer_t *xferCtrl = nullptr;
usb_transfer_t *xferIn[NUM_IN_XFERS] = {};
SemaphoreHandle_t ctrlDone = nullptr;
SemaphoreHandle_t xfersIdle = nullptr;  // dado pelo inCb quando inFlight chega a 0
StreamBufferHandle_t rxStream = nullptr;
std::atomic<bool> running{false};
std::atomic<int> inFlight{0};

// ===================== Tarefas do USB Host =====================
void usbLibTask(void *) {
  while (true) {
    uint32_t flags = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
    if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      usb_host_device_free_all();
    }
  }
}

void usbClientTask(void *) {
  while (true) {
    usb_host_client_handle_events(clientHdl, portMAX_DELAY);
  }
}

// Roda dentro de usbClientTask: só repassa o evento para bmsDevTask.
void clientEventCb(const usb_host_client_event_msg_t *msg, void *) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    pendingDevAddr = msg->new_dev.address;
    xTaskNotify(devTaskHdl, EVT_NEW_DEV, eSetBits);
  } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    if (msg->dev_gone.dev_hdl == devHdl) xTaskNotify(devTaskHdl, EVT_DEV_GONE, eSetBits);
  }
}

// ===================== Transferências de controle =====================
void ctrlCb(usb_transfer_t *t) {
  xSemaphoreGive((SemaphoreHandle_t)t->context);
}

bool controlTransfer(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
                     uint16_t wIndex, uint8_t *data, uint16_t wLength) {
  usb_setup_packet_t *setup = (usb_setup_packet_t *)xferCtrl->data_buffer;
  setup->bmRequestType = bmRequestType;
  setup->bRequest = bRequest;
  setup->wValue = wValue;
  setup->wIndex = wIndex;
  setup->wLength = wLength;

  bool isIn = bmRequestType & 0x80;
  if (!isIn && wLength) memcpy(xferCtrl->data_buffer + sizeof(usb_setup_packet_t), data, wLength);

  xferCtrl->num_bytes = sizeof(usb_setup_packet_t) + wLength;
  xferCtrl->device_handle = devHdl;
  xferCtrl->bEndpointAddress = 0;
  xferCtrl->callback = ctrlCb;
  xferCtrl->context = ctrlDone;
  xferCtrl->timeout_ms = 1000;

  xSemaphoreTake(ctrlDone, 0);  // descarta sinal de um callback atrasado
  if (usb_host_transfer_submit_control(clientHdl, xferCtrl) != ESP_OK) return false;
  if (xSemaphoreTake(ctrlDone, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
  if (xferCtrl->status != USB_TRANSFER_STATUS_COMPLETED) return false;

  if (isIn && wLength) memcpy(data, xferCtrl->data_buffer + sizeof(usb_setup_packet_t), wLength);
  return true;
}

// ===================== Configuração de cada conversor =====================
bool configureCdcAcm() {
  uint8_t lineCoding[7] = {
    (uint8_t)(baudRate), (uint8_t)(baudRate >> 8), (uint8_t)(baudRate >> 16), (uint8_t)(baudRate >> 24),
    0,  // 1 stop bit
    0,  // sem paridade
    8   // 8 bits de dados
  };
  // Dispositivos com USB nativo podem ignorar/recusar o baud rate; não é erro fatal.
  if (!controlTransfer(0x21, 0x20, 0, ifaceComm, lineCoding, sizeof(lineCoding))) {
    ESP_LOGW(TAG, "Aviso: SET_LINE_CODING recusado");
  }
  // DTR+RTS: muitos dispositivos CDC só transmitem com DTR ativo
  if (!controlTransfer(0x21, 0x22, 0x0003, ifaceComm, nullptr, 0)) {
    ESP_LOGW(TAG, "Aviso: SET_CONTROL_LINE_STATE recusado");
  }
  return true;
}

// Mesmo cálculo do driver ch341 do Linux (clock de 48 MHz)
uint16_t ch341Divisor(uint32_t speed) {
  const uint32_t CLK = 48000000UL;
  auto clkDiv = [](int ps, int fact) -> uint32_t { return 1UL << (12 - 3 * ps - fact); };

  int ps;
  for (ps = 3; ps > 0; ps--) {
    if (speed >= CLK / (clkDiv(ps, 1) * 512)) break;
  }

  uint32_t fact = 1;
  uint32_t cd = clkDiv(ps, 1);
  uint32_t div = CLK / (cd * speed);
  if (div < 9 || div > 255) {
    div /= 2;
    cd *= 2;
    fact = 0;
  }
  if (16 * CLK / (cd * div) - 16 * speed >= 16 * speed - 16 * CLK / (cd * (div + 1))) div++;
  if (fact == 1 && div % 2 == 0) {
    div /= 2;
    fact = 0;
  }
  return (uint16_t)(((0x100 - div) << 8) | (fact << 2) | ps);
}

bool configureCh34x() {
  uint8_t ver[2] = { 0, 0 };
  controlTransfer(0xC0, 0x5F, 0, 0, ver, 2);  // versão do chip
  if (!controlTransfer(0x40, 0xA1, 0, 0, nullptr, 0)) return false;  // inicializa serial

  uint16_t div = ch341Divisor(baudRate);
  if (ver[0] > 0x27) div |= 0x80;
  if (!controlTransfer(0x40, 0x9A, 0x1312, div, nullptr, 0)) return false;  // baud rate
  if (ver[0] >= 0x30) {
    controlTransfer(0x40, 0x9A, 0x2518, 0x00C3, nullptr, 0);  // 8N1, RX/TX habilitados
  }
  // DTR/RTS não são alterados para não provocar reset no microcontrolador do BMS
  return true;
}

bool configureCp210x() {
  uint16_t idx = ifaceData;
  if (!controlTransfer(0x41, 0x00, 0x0001, idx, nullptr, 0)) return false;  // IFC_ENABLE
  uint8_t baud[4] = { (uint8_t)(baudRate), (uint8_t)(baudRate >> 8),
                      (uint8_t)(baudRate >> 16), (uint8_t)(baudRate >> 24) };
  if (!controlTransfer(0x41, 0x1E, 0, idx, baud, 4)) return false;           // SET_BAUDRATE
  controlTransfer(0x41, 0x03, 0x0800, idx, nullptr, 0);                      // SET_LINE_CTL 8N1
  return true;
}

bool configureFtdi() {
  const uint16_t port = 0;
  controlTransfer(0x40, 0x00, 0x0000, port, nullptr, 0);  // reset

  // Divisor sobre 3 MHz com fração em 1/8
  static const uint8_t fracCode[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
  uint32_t div8 = (3000000UL * 8 + baudRate / 2) / baudRate;
  uint32_t enc = (div8 >> 3) | ((uint32_t)fracCode[div8 & 7] << 14);
  if (!controlTransfer(0x40, 0x03, (uint16_t)enc, (uint16_t)(enc >> 16), nullptr, 0)) return false;

  controlTransfer(0x40, 0x04, 0x0008, port, nullptr, 0);  // 8N1
  controlTransfer(0x40, 0x02, 0x0000, port, nullptr, 0);  // sem controle de fluxo
  return true;
}

// ===================== Leitura bulk IN =====================
// Roda dentro de usbClientTask.
void inCb(usb_transfer_t *t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
    if (chip == CHIP_FTDI) {
      // Cada pacote FTDI começa com 2 bytes de status
      for (int off = 0; off < t->actual_num_bytes; off += epInMps) {
        int len = std::min<int>(epInMps, t->actual_num_bytes - off);
        if (len > 2) xStreamBufferSend(rxStream, t->data_buffer + off + 2, len - 2, 0);
      }
    } else {
      xStreamBufferSend(rxStream, t->data_buffer, t->actual_num_bytes, 0);
    }
  }

  if (running && t->status == USB_TRANSFER_STATUS_COMPLETED &&
      usb_host_transfer_submit(t) == ESP_OK) {
    return;  // continua na fila
  }
  if (--inFlight == 0) xSemaphoreGive(xfersIdle);
}

bool startReading() {
  size_t size = epInMps * ((512 + epInMps - 1) / epInMps);  // múltiplo do MPS
  for (int i = 0; i < NUM_IN_XFERS; i++) {
    if (usb_host_transfer_alloc(size, 0, &xferIn[i]) != ESP_OK) return false;
    xferIn[i]->device_handle = devHdl;
    xferIn[i]->bEndpointAddress = epIn;
    xferIn[i]->callback = inCb;
    xferIn[i]->context = nullptr;
    xferIn[i]->num_bytes = size;
  }

  running = true;
  for (int i = 0; i < NUM_IN_XFERS; i++) {
    inFlight++;
    if (usb_host_transfer_submit(xferIn[i]) != ESP_OK) {
      inFlight--;
      return false;
    }
  }
  return true;
}

// ===================== Abertura / fechamento do dispositivo =====================
void closeDevice() {
  running = false;
  if (!devHdl) return;

  xSemaphoreTake(xfersIdle, 0);  // descarta um sinal antigo
  if (inFlight > 0) {
    usb_host_endpoint_halt(devHdl, epIn);
    usb_host_endpoint_flush(devHdl, epIn);
    // Espera os callbacks devolverem as transferências
    if (xSemaphoreTake(xfersIdle, pdMS_TO_TICKS(1000)) != pdTRUE) {
      // Liberar transferência ainda em voo corromperia a pilha USB: vaza
      ESP_LOGE(TAG, "Transferências IN não retornaram; não liberadas");
      for (int i = 0; i < NUM_IN_XFERS; i++) xferIn[i] = nullptr;
    }
  }

  for (int i = 0; i < NUM_IN_XFERS; i++) {
    if (xferIn[i]) {
      usb_host_transfer_free(xferIn[i]);
      xferIn[i] = nullptr;
    }
  }
  if (ifaceClaimed) usb_host_interface_release(clientHdl, devHdl, ifaceData);
  usb_host_device_close(clientHdl, devHdl);

  devHdl = nullptr;
  ifaceClaimed = false;
  chip = CHIP_NONE;
  inFlight = 0;
  ESP_LOGW(TAG, "Dispositivo desconectado");
}

bool openDevice(uint8_t addr) {
  if (usb_host_device_open(clientHdl, addr, &devHdl) != ESP_OK) {
    devHdl = nullptr;
    return false;
  }

  const usb_device_desc_t *dd;
  const usb_config_desc_t *cd;
  usb_host_get_device_descriptor(devHdl, &dd);
  usb_host_get_active_config_descriptor(devHdl, &cd);

  // Procura a primeira interface com endpoint bulk IN e uma interface CDC de comunicação
  hasCdcComm = false;
  bool hasCdcData = false;
  bool found = false;
  int curIface = -1;
  uint8_t curAlt = 0;
  const uint8_t *p = (const uint8_t *)cd;
  for (uint16_t i = 0; i + 1 < cd->wTotalLength && p[i] > 0; i += p[i]) {
    uint8_t type = p[i + 1];
    if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      const usb_intf_desc_t *id = (const usb_intf_desc_t *)&p[i];
      curIface = id->bInterfaceNumber;
      curAlt = id->bAlternateSetting;
      if (id->bInterfaceClass == 0x02 && !hasCdcComm) {
        hasCdcComm = true;
        ifaceComm = curIface;
      }
      if (id->bInterfaceClass == 0x0A) hasCdcData = true;
    } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && !found && curIface >= 0 && curAlt == 0) {
      const usb_ep_desc_t *ed = (const usb_ep_desc_t *)&p[i];
      if ((ed->bmAttributes & 0x03) == 0x02 && (ed->bEndpointAddress & 0x80)) {
        epIn = ed->bEndpointAddress;
        epInMps = ed->wMaxPacketSize & 0x7FF;
        ifaceData = curIface;
        found = true;
      }
    }
  }

  if (hasCdcComm || hasCdcData) chip = CHIP_CDC_ACM;
  else if (dd->idVendor == 0x1A86) chip = CHIP_CH34X;
  else if (dd->idVendor == 0x10C4) chip = CHIP_CP210X;
  else if (dd->idVendor == 0x0403) chip = CHIP_FTDI;
  else chip = CHIP_NONE;

  ESP_LOGI(TAG, "Conectado VID=%04X PID=%04X, conversor: %s, EP IN=0x%02X (MPS %u)",
           dd->idVendor, dd->idProduct, CHIP_NAMES[chip], epIn, epInMps);

  if (!found) {
    ESP_LOGE(TAG, "Nenhum endpoint bulk IN encontrado");
    return false;
  }
  if (chip == CHIP_NONE) {
    ESP_LOGE(TAG, "Conversor USB-serial não suportado");
    return false;
  }
  if (usb_host_interface_claim(clientHdl, devHdl, ifaceData, 0) != ESP_OK) {
    ESP_LOGE(TAG, "Falha ao reivindicar a interface");
    return false;
  }
  ifaceClaimed = true;
  return true;
}

void startDevice(uint8_t addr) {
  if (!openDevice(addr)) {
    closeDevice();
    return;
  }

  bool ok = false;
  switch (chip) {
    case CHIP_CDC_ACM: ok = configureCdcAcm(); break;
    case CHIP_CH34X: ok = configureCh34x(); break;
    case CHIP_CP210X: ok = configureCp210x(); break;
    case CHIP_FTDI: ok = configureFtdi(); break;
    default: break;
  }
  if (!ok) {
    ESP_LOGE(TAG, "Falha ao configurar a serial");
    closeDevice();
    return;
  }
  if (!startReading()) {
    ESP_LOGE(TAG, "Falha ao iniciar a leitura");
    closeDevice();
    return;
  }
  ESP_LOGI(TAG, "Serial aberta a %" PRIu32 " baud, aguardando dados do BMS...", baudRate);
}

// Abre/fecha o conversor fora da tarefa do cliente: controlTransfer() espera
// em semáforo pelo callback, que roda em usbClientTask.
void bmsDevTask(void *) {
  while (true) {
    uint32_t events = 0;
    xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);

    if (events & EVT_DEV_GONE) closeDevice();

    if (events & EVT_NEW_DEV) {
      uint8_t addr = pendingDevAddr;
      pendingDevAddr = 0;
      if (addr && !devHdl) startDevice(addr);
    }
  }
}

}  // namespace

namespace BmsUsb {

bool begin(uint32_t baud) {
  baudRate = baud;

  rxStream = xStreamBufferCreate(BMS_RX_STREAM_SIZE, 1);
  ctrlDone = xSemaphoreCreateBinary();
  xfersIdle = xSemaphoreCreateBinary();
  if (!rxStream || !ctrlDone || !xfersIdle) return false;

  usb_host_config_t hostCfg = {};
  hostCfg.skip_phy_setup = false;
  hostCfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
  if (usb_host_install(&hostCfg) != ESP_OK) return false;

  usb_host_client_config_t clientCfg = {};
  clientCfg.is_synchronous = false;
  clientCfg.max_num_event_msg = 5;
  clientCfg.async.client_event_callback = clientEventCb;
  clientCfg.async.callback_arg = nullptr;
  if (usb_host_client_register(&clientCfg, &clientHdl) != ESP_OK) return false;

  if (usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + 64, 0, &xferCtrl) != ESP_OK) return false;

  // bmsDevTask precisa existir antes de usbClientTask entregar o primeiro evento
  xTaskCreatePinnedToCore(bmsDevTask, "bms_dev", 4096, nullptr, 1, &devTaskHdl, 0);
  xTaskCreatePinnedToCore(usbLibTask, "usb_lib", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(usbClientTask, "usb_client", 4096, nullptr, 2, nullptr, 0);

  ESP_LOGI(TAG, "Aguardando conexão do BMS na porta USB (OTG)...");
  return true;
}

size_t read(uint8_t *buf, size_t maxLen, TickType_t wait) {
  if (!rxStream) return 0;
  return xStreamBufferReceive(rxStream, buf, maxLen, wait);
}

bool isStreaming() {
  return running;
}

}  // namespace BmsUsb
