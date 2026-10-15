#pragma once

#include <cstdint>

// ============================================================================
// Protocolo do BMS EVPV V2.0 (manual, pág. 9). Uma linha de texto por pacote:
//
//   c1,...,c16,descarga,recarga,delta,balanceando,corrente,t1,...,t8\n
//
//   - Tensão das células 1 a 16 (mV)
//   - Flag permite descarregar (0/1)
//   - Flag permite recarregar (0/1)
//   - Delta entre maior e menor célula (mV)
//   - Flag balanceando (0/1)
//   - Corrente (mA)
//   - Temperaturas 1 a 8 (ºC)
//
// Módulo puro (sem ESP-IDF): compilado também nos testes de host.
// ============================================================================

struct BmsData {
  static constexpr int NUM_CELLS = 16;
  static constexpr int NUM_TEMPS = 8;
  static constexpr int NUM_FIELDS = NUM_CELLS + 5 + NUM_TEMPS;  // 29 campos

  uint16_t cellMv[NUM_CELLS];
  bool dischargeAllowed;
  bool chargeAllowed;
  uint16_t deltaMv;
  bool balancing;
  int32_t currentMa;
  int16_t tempC[NUM_TEMPS];
};

namespace BmsEvpvParser {

// Decodifica uma linha (sem ou com '\r'/'\n' no fim). Retorna false e não
// altera 'out' se o número de campos estiver errado, algum campo não for
// inteiro, ou algum valor estiver fora da faixa.
bool parseLine(const char *line, BmsData &out);

}  // namespace BmsEvpvParser
