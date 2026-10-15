#include "Mcp2515.h"
#include "Mcp2515Timing.h"

#include <cstring>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

// Instruções SPI
constexpr uint8_t INSTR_RESET = 0xC0;
constexpr uint8_t INSTR_READ = 0x03;
constexpr uint8_t INSTR_WRITE = 0x02;
constexpr uint8_t INSTR_BIT_MODIFY = 0x05;
constexpr uint8_t INSTR_RTS = 0x80;  // | bit do TXB (0x01, 0x02, 0x04)

// Registradores
constexpr uint8_t REG_CANSTAT = 0x0E;
constexpr uint8_t REG_CANCTRL = 0x0F;
constexpr uint8_t REG_TEC = 0x1C;  // REC vem logo depois (0x1D)
constexpr uint8_t REG_CNF3 = 0x28;  // CNF2 = 0x29, CNF1 = 0x2A
constexpr uint8_t REG_EFLG = 0x2D;
constexpr uint8_t REG_TXB_CTRL[3] = { 0x30, 0x40, 0x50 };  // TXBnSIDH = CTRL + 1

constexpr uint8_t MODE_MASK = 0xE0;  // REQOP / OPMOD
constexpr uint8_t TXB_TXREQ = 0x08;

constexpr int64_t MODE_TIMEOUT_US = 50000;

}  // namespace

esp_err_t Mcp2515::attach(spi_host_device_t host, int sck, int miso, int mosi, int cs, int clockHz) {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = sck;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  // Transações curtas (até 15 bytes): sem DMA
  esp_err_t err = spi_bus_initialize(host, &bus, SPI_DMA_DISABLED);
  if (err != ESP_OK) return err;

  spi_device_interface_config_t devCfg = {};
  devCfg.mode = 0;  // MCP2515 aceita modo 0,0 e 1,1
  devCfg.clock_speed_hz = clockHz;
  devCfg.spics_io_num = cs;
  devCfg.queue_size = 1;
  return spi_bus_add_device(host, &devCfg, &dev_);
}

bool Mcp2515::transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
  if (!dev_) return false;
  spi_transaction_t t = {};
  t.length = len * 8;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  return spi_device_polling_transmit(dev_, &t) == ESP_OK;
}

bool Mcp2515::readRegs(uint8_t addr, uint8_t *values, size_t n) {
  uint8_t tx[2 + 16] = { INSTR_READ, addr };
  uint8_t rx[2 + 16] = {};
  if (n > 16 || !transfer(tx, rx, 2 + n)) return false;
  memcpy(values, rx + 2, n);
  return true;
}

bool Mcp2515::readReg(uint8_t addr, uint8_t &value) {
  return readRegs(addr, &value, 1);
}

bool Mcp2515::writeRegs(uint8_t addr, const uint8_t *values, size_t n) {
  uint8_t tx[2 + 16] = { INSTR_WRITE, addr };
  if (n > 16) return false;
  memcpy(tx + 2, values, n);
  return transfer(tx, nullptr, 2 + n);
}

bool Mcp2515::bitModify(uint8_t addr, uint8_t mask, uint8_t value) {
  const uint8_t tx[4] = { INSTR_BIT_MODIFY, addr, mask, value };
  return transfer(tx, nullptr, sizeof(tx));
}

Mcp2515::Mode Mcp2515::currentMode() {
  uint8_t stat;
  if (!readReg(REG_CANSTAT, stat)) return Mode::Unknown;
  return (Mode)(stat & MODE_MASK);
}

bool Mcp2515::reset() {
  const uint8_t tx = INSTR_RESET;
  if (!transfer(&tx, nullptr, 1)) return false;
  // Após o reset o oscilador precisa estabilizar (OST de 128 ciclos) e o
  // chip volta em modo config: espera isso aparecer no CANSTAT.
  int64_t deadline = esp_timer_get_time() + MODE_TIMEOUT_US;
  do {
    vTaskDelay(1);
    if (currentMode() == Mode::Config) return true;
  } while (esp_timer_get_time() < deadline);
  return false;
}

bool Mcp2515::setBitrate500k(uint32_t oscHz) {
  Mcp2515Timing::Cnf c;
  if (!Mcp2515Timing::cnfFor500k(oscHz, c)) return false;

  // CNF3, CNF2, CNF1 são consecutivos: uma escrita com autoincremento
  const uint8_t cnf[3] = { c.cnf3, c.cnf2, c.cnf1 };
  uint8_t check[3];
  if (!writeRegs(REG_CNF3, cnf, 3) || !readRegs(REG_CNF3, check, 3)) return false;
  // A leitura de volta também detecta SPI sem resposta (MISO preso em 0 ou 1)
  return memcmp(cnf, check, 3) == 0;
}

bool Mcp2515::setMode(Mode mode) {
  if (!bitModify(REG_CANCTRL, MODE_MASK, (uint8_t)mode)) return false;
  // A troca só acontece com o barramento livre (11 bits recessivos)
  int64_t deadline = esp_timer_get_time() + MODE_TIMEOUT_US;
  do {
    if (currentMode() == mode) return true;
    vTaskDelay(1);
  } while (esp_timer_get_time() < deadline);
  return false;
}

Mcp2515::TxResult Mcp2515::sendStd(uint16_t id, const uint8_t *data, uint8_t len) {
  if (id > 0x7FF || len > 8) return TxResult::Error;

  constexpr uint8_t CANCTRL_ABAT = 0x10;
  if (abortPending_) {
    // ABAT precisa voltar a 0 para liberar novas transmissões
    if (!bitModify(REG_CANCTRL, CANCTRL_ABAT, 0)) return TxResult::Error;
    abortPending_ = false;
  }

  for (int n = 0; n < 3; n++) {
    uint8_t ctrl;
    if (!readReg(REG_TXB_CTRL[n], ctrl)) return TxResult::Error;
    if (ctrl & TXB_TXREQ) continue;  // ainda esperando ACK/arbitragem

    // TXBnSIDH, SIDL, EID8, EID0, DLC, D0..D7 em sequência
    uint8_t buf[5 + 8] = {};
    buf[0] = (uint8_t)(id >> 3);           // SID10..SID3
    buf[1] = (uint8_t)((id & 0x07) << 5);  // SID2..SID0, EXIDE = 0 (standard)
    buf[4] = len;                          // RTR = 0 (frame de dados)
    memcpy(buf + 5, data, len);
    if (!writeRegs(REG_TXB_CTRL[n] + 1, buf, 5 + len)) return TxResult::Error;

    const uint8_t rts = INSTR_RTS | (uint8_t)(1u << n);
    return transfer(&rts, nullptr, 1) ? TxResult::Ok : TxResult::Error;
  }
  abortPending_ = bitModify(REG_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT);
  return TxResult::AllBusy;
}

bool Mcp2515::readErrors(ErrorState &out) {
  uint8_t counters[2];
  if (!readRegs(REG_TEC, counters, 2) || !readReg(REG_EFLG, out.eflg)) return false;
  out.tec = counters[0];
  out.rec = counters[1];
  return true;
}
