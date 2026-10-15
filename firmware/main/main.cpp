// ============================================================================
// Leitor do BMS EVPV por USB Host + resumo em CAN 0x300 - ESP32-S3-DevKitC-1
//
// Hardware:
//   - BMS EVPV V2.0: micro USB do BMS -> porta "USB" (OTG) do DevKitC-1
//   - CAN 500 kbit/s: MCP2515 + transceiver <-> SPI2 (pinos em config.h)
//   - Console: porta "UART" do DevKitC-1 (115200)
//
// Tarefas (no lugar do loop() da versão Arduino):
//   core 0: usb_lib, usb_client, bms_dev   (bms_usb_host.cpp)
//   core 1: bms_parser  stream buffer -> linhas -> estado compartilhado
//           can_tx      a cada BMS_CAN_TX_PERIOD_MS monta e envia o 0x300
//           report      a cada PRINT_PERIOD_MS imprime o relatório
//
// O estado compartilhado (último pacote do BMS, contadores, status do CAN)
// fica sob um único mutex; cada tarefa segura o mutex só para copiar.
// ============================================================================

#include <cinttypes>
#include <cstdio>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "BmsEvpvParser.h"
#include "BmsStatusFrame.h"
#include "bms_usb_host.h"
#include "config.h"
#include "drivers/Mcp2515.h"
#include "report.h"

namespace {

const char *TAG_BMS = "BMS";
const char *TAG_CAN = "CAN";

// ===================== Estado compartilhado =====================
// ponytail: um mutex para tudo; o volume é ~4 pacotes/s, não há disputa real.
SemaphoreHandle_t stateMutex = nullptr;
BmsData bmsData = {};
bool bmsValid = false;
int64_t lastPacketUs = 0;
uint32_t packets = 0;
uint32_t badPackets = 0;
Report::CanTxStatus canStatus = {};

struct Lock {
  Lock() { xSemaphoreTake(stateMutex, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(stateMutex); }
};

// Chamar com o mutex tomado
bool bmsFreshLocked() {
  return bmsValid && (esp_timer_get_time() - lastPacketUs) <= (int64_t)BMS_DATA_TIMEOUT_MS * 1000;
}

// ===================== Parser do BMS =====================
void handleLine(const char *line) {
  if (BMS_PRINT_RAW_LINES) ESP_LOGI(TAG_BMS, "RAW: %s", line);

  BmsData d;
  bool ok = BmsEvpvParser::parseLine(line, d);

  Lock lock;
  if (ok) {
    bmsData = d;
    bmsValid = true;
    lastPacketUs = esp_timer_get_time();
    packets++;
  } else {
    badPackets++;
  }
}

void bmsParserTask(void *) {
  char lineBuf[256];
  size_t lineLen = 0;
  uint8_t buf[64];

  while (true) {
    size_t n = BmsUsb::read(buf, sizeof(buf), portMAX_DELAY);
    for (size_t i = 0; i < n; i++) {
      char c = (char)buf[i];
      if (c == '\n' || c == '\r') {
        if (lineLen > 0) {
          lineBuf[lineLen] = '\0';
          handleLine(lineBuf);
          lineLen = 0;
        }
      } else if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;  // linha longa demais: descarta
      }
    }
  }
}

// ===================== TX CAN (MCP2515) =====================
Mcp2515::Mode wantedMode() {
  return LISTEN_ONLY_MODE ? Mcp2515::Mode::ListenOnly : Mcp2515::Mode::Normal;
}

bool startMcp(Mcp2515 &mcp) {
  return mcp.reset() && mcp.setBitrate500k(MCP2515_OSC_HZ) && mcp.setMode(wantedMode());
}

void canTxTask(void *) {
  Mcp2515 mcp;
  esp_err_t err = mcp.attach(SPI2_HOST, MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN,
                             MCP2515_CS_PIN, MCP2515_SPI_CLOCK_HZ);
  if (err != ESP_OK) {
    ESP_LOGE(TAG_CAN, "Falha ao iniciar o SPI do MCP2515: %s", esp_err_to_name(err));
    vTaskDelete(nullptr);
    return;
  }

  bool online = false;
  uint32_t failures = 0;
  int64_t nextRetryUs = 0;
  int64_t prevCycleUs = 0;
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BMS_CAN_TX_PERIOD_MS));
    int64_t now = esp_timer_get_time();
    uint32_t periodUs = prevCycleUs ? (uint32_t)(now - prevCycleUs) : 0;
    prevCycleUs = now;

    if (!online && now >= nextRetryUs) {
      online = startMcp(mcp);
      if (online) {
        ESP_LOGI(TAG_CAN, "MCP2515 iniciado (%s, cristal %" PRIu32 " Hz, 500 kbit/s)",
                 LISTEN_ONLY_MODE ? "listen-only" : "modo normal", MCP2515_OSC_HZ);
      } else {
        failures++;
        nextRetryUs = now + (int64_t)MCP2515_RETRY_MS * 1000;
        ESP_LOGE(TAG_CAN, "Falha ao iniciar o MCP2515 (tentativa %" PRIu32 "); nova tentativa em %" PRIu32 " ms",
                 failures, MCP2515_RETRY_MS);
      }
    } else if (online && mcp.currentMode() != wantedMode()) {
      // Ex.: o módulo perdeu alimentação e voltou em modo config
      online = false;
      ESP_LOGW(TAG_CAN, "MCP2515 fora do modo esperado; reiniciando");
    }

    BmsData d;
    bool fresh;
    {
      Lock lock;
      fresh = bmsFreshLocked();
      d = bmsData;
    }

    Mcp2515::ErrorState errState = {};
    bool spiError = false;
    Mcp2515::TxResult tx = Mcp2515::TxResult::Error;
    bool tried = false;
    if (online) {
      spiError = !mcp.readErrors(errState);
      // Dado velho não é retransmitido: quem recebe detecta a perda pelo timeout.
      // Em listen-only o MCP2515 não transmite: os buffers de TX ficariam presos.
      if (fresh && !LISTEN_ONLY_MODE) {
        auto frame = BmsStatusFrame::build(d);
        tx = mcp.sendStd(BmsStatusFrame::MSG_ID, frame.data(), BmsStatusFrame::DLC);
        tried = true;
      }
    }

    Lock lock;
    canStatus.online = online;
    canStatus.initFailures = failures;
    if (online) canStatus.err = errState;
    if (spiError) canStatus.spiErrors++;
    if (tried) {
      // AllBusy: os 3 buffers ainda esperam ACK (ex.: barramento sem outro nó).
      // Os erros de TX aparecem no TEC/ERROR-PASSIVE do relatório.
      if (tx == Mcp2515::TxResult::Ok) canStatus.sent++;
      else if (tx == Mcp2515::TxResult::AllBusy) canStatus.allBusy++;
      else canStatus.spiErrors++;
    }
    if (periodUs > canStatus.maxPeriodUs) canStatus.maxPeriodUs = periodUs;
    canStatus.cycles++;
  }
}

// ===================== Relatório =====================
void reportTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PRINT_PERIOD_MS));

    Report::Snapshot s;
    {
      Lock lock;
      s.usbStreaming = BmsUsb::isStreaming();
      s.bmsFresh = bmsFreshLocked();
      s.bms = bmsData;
      s.packets = packets;
      s.badPackets = badPackets;
      s.can = canStatus;
      // Latência é por janela; o resto é acumulado desde a partida
      canStatus.maxPeriodUs = 0;
      canStatus.cycles = 0;
    }
    Report::print(s);
  }
}

}  // namespace

extern "C" void app_main(void) {
  printf("\nLeitor BMS EVPV (USB Host) + CAN 0x300 - ESP32-S3\n");

  stateMutex = xSemaphoreCreateMutex();
  configASSERT(stateMutex);

  if (BmsUsb::begin(BMS_BAUD)) {
    xTaskCreatePinnedToCore(bmsParserTask, "bms_parser", 4096, nullptr, 4, nullptr, 1);
  } else {
    ESP_LOGE(TAG_BMS, "Falha ao instalar o USB Host; leitura do BMS desativada.");
  }

  xTaskCreatePinnedToCore(canTxTask, "can_tx", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(reportTask, "report", 4096, nullptr, 2, nullptr, 1);
}
