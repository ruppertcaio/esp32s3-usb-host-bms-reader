#pragma once

#include <cstddef>
#include <cstdint>
#include "driver/spi_master.h"
#include "esp_err.h"

// ============================================================================
// Driver mínimo do MCP2515 sobre o spi_master do ESP-IDF, escrito a partir do
// mapa de registradores e das instruções SPI do datasheet da Microchip.
//
// Só o necessário para este projeto: reset, bit timing de 500 kbit/s, troca
// de modo (normal / listen-only / config), envio de frame standard de 11 bits
// e leitura de EFLG/TEC/REC. Recepção não é usada: os buffers de RX enchem e
// ficam cheios, o que não afeta o ACK nem a transmissão.
//
// Não é thread-safe: usar de uma única tarefa.
// ============================================================================

class Mcp2515 {
 public:
  // Valores de REQOP (CANCTRL[7:5]) / OPMOD (CANSTAT[7:5])
  enum class Mode : uint8_t {
    Normal = 0x00,
    Sleep = 0x20,
    Loopback = 0x40,
    ListenOnly = 0x60,
    Config = 0x80,
    Unknown = 0xFF,  // falha de SPI
  };

  enum class TxResult { Ok, AllBusy, Error };

  struct ErrorState {
    uint8_t eflg;  // EFLG: RX1OVR RX0OVR TXBO TXEP RXEP TXWAR RXWAR EWARN
    uint8_t tec;
    uint8_t rec;
  };

  // Bits do EFLG
  static constexpr uint8_t EFLG_TXBO = 0x20;   // bus-off
  static constexpr uint8_t EFLG_TXEP = 0x10;   // TX error-passive (TEC >= 128)
  static constexpr uint8_t EFLG_RXEP = 0x08;   // RX error-passive (REC >= 128)
  static constexpr uint8_t EFLG_RXOVR = 0xC0;  // RX0OVR | RX1OVR

  // Inicializa o barramento SPI e registra o MCP2515 como dispositivo.
  // Chamar uma vez.
  esp_err_t attach(spi_host_device_t host, int sck, int miso, int mosi, int cs, int clockHz);

  // RESET (0xC0) e espera o chip voltar em modo config.
  bool reset();

  // Grava CNF1..3 para 500 kbit/s e confere lendo de volta. Só em modo config.
  bool setBitrate500k(uint32_t oscHz);

  // Pede o modo em CANCTRL.REQOP e confirma em CANSTAT.OPMOD.
  bool setMode(Mode mode);
  Mode currentMode();

  // Coloca um frame de dados standard (11 bits) no primeiro TXB livre
  // (TXREQ = 0) e pede a transmissão. Não espera o ACK.
  // Com os 3 TXB ocupados (ninguém dando ACK), pede ABAT para descartar
  // frames velhos: quando o barramento voltar sai o dado mais recente.
  TxResult sendStd(uint16_t id, const uint8_t *data, uint8_t len);

  bool readErrors(ErrorState &out);

  bool readReg(uint8_t addr, uint8_t &value);
  bool readRegs(uint8_t addr, uint8_t *values, size_t n);
  bool writeRegs(uint8_t addr, const uint8_t *values, size_t n);
  bool bitModify(uint8_t addr, uint8_t mask, uint8_t value);

 private:
  bool transfer(const uint8_t *tx, uint8_t *rx, size_t len);

  spi_device_handle_t dev_ = nullptr;
  bool abortPending_ = false;   // ABAT pedido na chamada anterior
};
