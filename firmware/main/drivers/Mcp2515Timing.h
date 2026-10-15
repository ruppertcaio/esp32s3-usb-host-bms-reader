#pragma once

#include <cstdint>

// ============================================================================
// Bit timing do MCP2515 para 500 kbit/s (datasheet do MCP2515, seção 5).
//
//   TQ  = 2 * (BRP + 1) / Fosc
//   bit = SyncSeg (1 TQ) + PropSeg + PS1 + PS2
//   ponto de amostragem = (SyncSeg + PropSeg + PS1) / bit
//
// Escolha: 8 TQ por bit de 2 us -> TQ = 250 ns
//   SyncSeg 1 + PropSeg 2 + PS1 3 + PS2 2 = 8 TQ, amostragem em 6/8 = 75 %
//   PropSeg = 500 ns: cobre o atraso de ida e volta de transceiver + cabo
//   numa bancada curta.
//
//   Fosc 8 MHz:  BRP = 8e6  / (2 * 8 * 500e3) - 1 = 0
//   Fosc 16 MHz: BRP = 16e6 / (2 * 8 * 500e3) - 1 = 1
//
// Requisitos do datasheet (conferidos nos static_assert abaixo):
//   PropSeg + PS1 >= PS2,  PropSeg + PS1 >= Tdelay (2 TQ),  PS2 > SJW
//
// Codificação dos registradores (cada campo grava valor - 1):
//   CNF1 = SJW[7:6] | BRP[5:0]
//   CNF2 = BTLMODE[7]=1 (PS2 vem do CNF3) | SAM[6]=0 (1 amostra) | PHSEG1[5:3] | PRSEG[2:0]
//   CNF3 = SOF[7]=0 | WAKFIL[6]=0 | PHSEG2[2:0]
//
//   8 MHz  -> CNF1 0x00, CNF2 0x91, CNF3 0x01
//   16 MHz -> CNF1 0x01, CNF2 0x91, CNF3 0x01
//
// Módulo puro (sem ESP-IDF): compilado também nos testes de host.
// ============================================================================

namespace Mcp2515Timing {

constexpr uint32_t BITRATE = 500000;
constexpr uint32_t PROP_SEG = 2;
constexpr uint32_t PS1 = 3;
constexpr uint32_t PS2 = 2;
constexpr uint32_t SJW = 1;
constexpr uint32_t TQ_PER_BIT = 1 + PROP_SEG + PS1 + PS2;

static_assert(TQ_PER_BIT == 8, "8 TQ por bit");
static_assert(PROP_SEG + PS1 >= PS2, "datasheet: PropSeg + PS1 >= PS2");
static_assert(PROP_SEG + PS1 >= 2, "datasheet: PropSeg + PS1 >= Tdelay");
static_assert(PS2 > SJW, "datasheet: PS2 > SJW");

struct Cnf {
  uint8_t cnf1;
  uint8_t cnf2;
  uint8_t cnf3;
};

// Calcula CNF1..3 para 500 kbit/s. Retorna false se Fosc não for múltiplo de
// 2 * TQ_PER_BIT * BITRATE (8 MHz) ou se o BRP passar de 6 bits.
inline bool cnfFor500k(uint32_t oscHz, Cnf &out) {
  const uint32_t step = 2 * TQ_PER_BIT * BITRATE;
  if (oscHz == 0 || oscHz % step != 0) return false;
  const uint32_t brp = oscHz / step - 1;
  if (brp > 63) return false;

  out.cnf1 = (uint8_t)(((SJW - 1) << 6) | brp);
  out.cnf2 = (uint8_t)(0x80 | ((PS1 - 1) << 3) | (PROP_SEG - 1));
  out.cnf3 = (uint8_t)(PS2 - 1);
  return true;
}

}  // namespace Mcp2515Timing
