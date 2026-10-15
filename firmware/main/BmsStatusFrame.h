#pragma once

#include <array>
#include <cstdint>
#include "BmsEvpvParser.h"

// ============================================================================
// Frame CAN com o resumo do BMS, transmitido pelo MCP2515.
//
// Mensagem BMS_Status_1: ID 0x300 (standard, 11 bits), DLC 8, Little-Endian
// (Intel). Bit 0 = LSB do byte 0; valor físico = raw * escala.
//
//   Sinal            Byte  Bit ini/tam  Tipo      Escala      Faixa
//   BMS_uPack        0..1   0 / 16      uint16    0.01 V      0 .. 655.35 V
//   BMS_iPack        2..3  16 / 16      int16     0.1 A       -3276.8 .. 3276.7 A
//   BMS_uCellDelta   4..5  32 / 16      uint16    1 mV        0 .. 65535 mV
//   BMS_stAlert      6     48 /  1      bit       -           0 / 1
//   (reservado)      6..7  49 / 15      -         -           sempre 0
//
// Trecho DBC equivalente:
//   BO_ 768 BMS_Status_1: 8 Vector__XXX
//    SG_ BMS_uPack : 0|16@1+ (0.01,0) [0|655.35] "V" Vector__XXX
//    SG_ BMS_iPack : 16|16@1- (0.1,0) [-3276.8|3276.7] "A" Vector__XXX
//    SG_ BMS_uCellDelta : 32|16@1+ (1,0) [0|65535] "mV" Vector__XXX
//    SG_ BMS_stAlert : 48|1@1+ (1,0) [0|1] "" Vector__XXX
//
// Valores fora da faixa saturam no limite do tipo.
// Módulo puro (sem ESP-IDF): compilado também nos testes de host.
// ============================================================================

namespace BmsStatusFrame {

constexpr uint16_t MSG_ID = 0x300;
constexpr uint8_t DLC = 8;

// BMS_stAlert = 1 se pack < ALERT_PACK_MIN_MV OU delta >= ALERT_DELTA_MAX_MV
constexpr uint32_t ALERT_PACK_MIN_MV = 44000;
constexpr uint16_t ALERT_DELTA_MAX_MV = 1000;

// Monta os 8 bytes de dados do 0x300 a partir de um pacote do BMS.
std::array<uint8_t, DLC> build(const BmsData &d);

}  // namespace BmsStatusFrame
