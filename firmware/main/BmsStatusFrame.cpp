#include "BmsStatusFrame.h"

namespace {

uint16_t packVoltageRaw(uint32_t packMv) {
  uint32_t raw = (packMv + 5) / 10;  // 10 mV/bit, arredondado
  return raw > 0xFFFF ? 0xFFFF : (uint16_t)raw;
}

int16_t currentRaw(int32_t currentMa) {
  // 100 mA/bit, arredondado para longe do zero. Em 64 bits: currentMa ± 50
  // estouraria int32_t perto dos limites.
  int64_t ma = currentMa;
  int64_t raw = (ma >= 0) ? (ma + 50) / 100 : (ma - 50) / 100;
  if (raw > INT16_MAX) return INT16_MAX;
  if (raw < INT16_MIN) return INT16_MIN;
  return (int16_t)raw;
}

}  // namespace

namespace BmsStatusFrame {

std::array<uint8_t, DLC> build(const BmsData &d) {
  uint32_t packMv = 0;
  for (int i = 0; i < BmsData::NUM_CELLS; i++) packMv += d.cellMv[i];

  // Alerta avaliado com a resolução original (mV), antes da escala
  bool alert = (packMv < ALERT_PACK_MIN_MV) || (d.deltaMv >= ALERT_DELTA_MAX_MV);

  uint16_t uPack = packVoltageRaw(packMv);
  uint16_t iPack = (uint16_t)currentRaw(d.currentMa);  // complemento de 2
  uint16_t uDelta = d.deltaMv;

  return {
      (uint8_t)(uPack & 0xFF), (uint8_t)(uPack >> 8),    // bytes 0..1, bits 0..15: BMS_uPack, LSB primeiro
      (uint8_t)(iPack & 0xFF), (uint8_t)(iPack >> 8),    // bytes 2..3, bits 16..31: BMS_iPack
      (uint8_t)(uDelta & 0xFF), (uint8_t)(uDelta >> 8),  // bytes 4..5, bits 32..47: BMS_uCellDelta
      (uint8_t)(alert ? 0x01 : 0x00),                    // byte 6, bit 48: BMS_stAlert; bits 49..55 reservados em 0
      0x00,                                              // byte 7, bits 56..63: reservado
  };
}

}  // namespace BmsStatusFrame
