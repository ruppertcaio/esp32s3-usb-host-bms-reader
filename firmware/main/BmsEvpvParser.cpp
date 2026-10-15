#include "BmsEvpvParser.h"

#include <cstdlib>

// Parser portado da versão Arduino (bms_evpv.cpp). Diferenças: strtoll em vez
// de strtol (long tem 64 bits no host de teste e 32 no ESP32) e checagem de
// faixa também para delta, corrente e temperaturas (o valor tem que caber no
// tipo do campo, senão o cast truncaria lixo em silêncio).

namespace BmsEvpvParser {

namespace {

bool isBlank(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

bool inRange(long long v, long long lo, long long hi) {
  return v >= lo && v <= hi;
}

}  // namespace

bool parseLine(const char *s, BmsData &d) {
  long long v[BmsData::NUM_FIELDS];
  int n = 0;
  const char *p = s;

  while (true) {
    char *end;
    long long x = std::strtoll(p, &end, 10);
    if (end == p) return false;                 // campo vazio ou não numérico
    if (n >= BmsData::NUM_FIELDS) return false;  // campos a mais
    v[n++] = x;
    while (isBlank(*end)) end++;  // espaços e o CR/LF do fim da linha
    if (*end == '\0') break;
    if (*end != ',') return false;
    p = end + 1;
  }
  if (n != BmsData::NUM_FIELDS) return false;

  const int C = BmsData::NUM_CELLS;

  // Validação básica para descartar lixo (ex.: baud rate errado)
  for (int i = 0; i < C; i++) {
    if (!inRange(v[i], 0, 6000)) return false;
  }
  for (int i = C; i < C + 2; i++) {
    if (!inRange(v[i], 0, 1)) return false;
  }
  if (!inRange(v[C + 2], 0, UINT16_MAX)) return false;
  if (!inRange(v[C + 3], 0, 1)) return false;
  if (!inRange(v[C + 4], INT32_MIN, INT32_MAX)) return false;
  for (int i = C + 5; i < BmsData::NUM_FIELDS; i++) {
    if (!inRange(v[i], INT16_MIN, INT16_MAX)) return false;
  }

  for (int i = 0; i < C; i++) d.cellMv[i] = (uint16_t)v[i];
  d.dischargeAllowed = v[C] == 1;
  d.chargeAllowed = v[C + 1] == 1;
  d.deltaMv = (uint16_t)v[C + 2];
  d.balancing = v[C + 3] == 1;
  d.currentMa = (int32_t)v[C + 4];
  for (int i = 0; i < BmsData::NUM_TEMPS; i++) d.tempC[i] = (int16_t)v[C + 5 + i];
  return true;
}

}  // namespace BmsEvpvParser
