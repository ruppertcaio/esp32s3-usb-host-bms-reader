// Testes de host dos módulos puros: parser EVPV, frame 0x300 e CNF do MCP2515.
// Sem framework: CHECK conta as falhas e o processo sai com código != 0.

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>

#include "BmsEvpvParser.h"
#include "BmsStatusFrame.h"
#include "drivers/Mcp2515Timing.h"

static int failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FALHOU %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      failures++;                                                  \
    }                                                              \
  } while (0)

// Linha do exemplo do README (29 campos)
static const char *EXAMPLE_LINE =
    "3312,3308,3315,3310,3306,3313,3311,3309,3314,3307,3312,3310,3305,3316,3311,3309,"
    "1,1,11,0,12450,27,28,27,29,26,27,28,27";

static BmsData exampleData() {
  BmsData d{};
  CHECK(BmsEvpvParser::parseLine(EXAMPLE_LINE, d));
  return d;
}

static void testParserHappyPath() {
  BmsData d{};
  CHECK(BmsEvpvParser::parseLine(EXAMPLE_LINE, d));
  CHECK(d.cellMv[0] == 3312);
  CHECK(d.cellMv[13] == 3316);
  CHECK(d.cellMv[15] == 3309);
  CHECK(d.dischargeAllowed);
  CHECK(d.chargeAllowed);
  CHECK(d.deltaMv == 11);
  CHECK(!d.balancing);
  CHECK(d.currentMa == 12450);
  CHECK(d.tempC[0] == 27);
  CHECK(d.tempC[3] == 29);
  CHECK(d.tempC[7] == 27);
}

static void testParserLineEndings() {
  BmsData d{};
  CHECK(BmsEvpvParser::parseLine((std::string(EXAMPLE_LINE) + "\r\n").c_str(), d));
  CHECK(d.tempC[7] == 27);
  CHECK(BmsEvpvParser::parseLine((std::string(EXAMPLE_LINE) + "\r").c_str(), d));
  CHECK(BmsEvpvParser::parseLine("3312, 3308,3315,3310,3306,3313,3311,3309,3314,3307,3312,3310,3305,3316,3311,3309,"
                                 "0,1,11,1,-12450,27,28,27,29,26,27,28,-5",
                                 d));
  CHECK(!d.dischargeAllowed && d.balancing);
  CHECK(d.currentMa == -12450);
  CHECK(d.tempC[7] == -5);
}

static void testParserRejects() {
  BmsData d{};
  d.deltaMv = 1234;  // sentinela: linha rejeitada não altera a saída
  const std::string line = EXAMPLE_LINE;

  CHECK(!BmsEvpvParser::parseLine(line.substr(0, line.rfind(',')).c_str(), d));  // 28 campos
  CHECK(!BmsEvpvParser::parseLine((line + ",27").c_str(), d));                   // 30 campos
  CHECK(!BmsEvpvParser::parseLine("", d));
  CHECK(!BmsEvpvParser::parseLine("\r", d));
  CHECK(!BmsEvpvParser::parseLine("lixo", d));
  CHECK(!BmsEvpvParser::parseLine("\x80\xFF\x12\x7E", d));
  CHECK(!BmsEvpvParser::parseLine((line + "x").c_str(), d));
  CHECK(!BmsEvpvParser::parseLine(("," + line).c_str(), d));  // campo vazio
  CHECK(!BmsEvpvParser::parseLine(("3312.5" + line.substr(4)).c_str(), d));
  CHECK(!BmsEvpvParser::parseLine(("6001" + line.substr(4)).c_str(), d));  // célula > 6000 mV
  CHECK(!BmsEvpvParser::parseLine(("-1" + line.substr(4)).c_str(), d));    // célula negativa
  // flag de descarga = 2
  std::string badFlag = line;
  badFlag.replace(badFlag.find(",1,1,11,"), 8, ",2,1,11,");
  CHECK(!BmsEvpvParser::parseLine(badFlag.c_str(), d));
  // corrente que não cabe em int32
  std::string badCurrent = line;
  badCurrent.replace(badCurrent.find("12450"), 5, "99999999999");
  CHECK(!BmsEvpvParser::parseLine(badCurrent.c_str(), d));
  CHECK(d.deltaMv == 1234);
}

static void testFrameReadmeExample() {
  // Pack 52.968 V -> 5297 (0x14B1), 12.45 A -> 125 (0x007D), delta 11 mV
  const std::array<uint8_t, 8> expected = { 0xB1, 0x14, 0x7D, 0x00, 0x0B, 0x00, 0x00, 0x00 };
  CHECK(BmsStatusFrame::build(exampleData()) == expected);
  CHECK(BmsStatusFrame::MSG_ID == 0x300);
}

static void testFrameNegativeCurrent() {
  BmsData d = exampleData();
  d.currentMa = -12450;  // -124.5 -> -125 (arredonda para longe do zero) = 0xFF83
  auto f = BmsStatusFrame::build(d);
  CHECK(f[2] == 0x83 && f[3] == 0xFF);
  d.currentMa = -12449;  // -124.49 -> -124 = 0xFF84
  f = BmsStatusFrame::build(d);
  CHECK(f[2] == 0x84 && f[3] == 0xFF);
  d.currentMa = -1;  // -0.01 A -> 0
  f = BmsStatusFrame::build(d);
  CHECK(f[2] == 0x00 && f[3] == 0x00);
}

static void testFrameSaturation() {
  BmsData d{};
  for (auto &c : d.cellMv) c = 65535;  // 1048.56 V -> satura em 0xFFFF
  d.deltaMv = 65535;
  d.currentMa = INT32_MAX;
  auto f = BmsStatusFrame::build(d);
  CHECK(f[0] == 0xFF && f[1] == 0xFF);
  CHECK(f[2] == 0xFF && f[3] == 0x7F);  // INT16_MAX
  CHECK(f[4] == 0xFF && f[5] == 0xFF);
  d.currentMa = INT32_MIN;
  f = BmsStatusFrame::build(d);
  CHECK(f[2] == 0x00 && f[3] == 0x80);  // INT16_MIN
}

static void testFrameAlert() {
  BmsData d{};
  for (auto &c : d.cellMv) c = 2750;  // pack exatamente 44.000 V, delta 999: sem alerta
  d.deltaMv = 999;
  CHECK(BmsStatusFrame::build(d)[6] == 0x00);

  d.cellMv[0] = 2749;  // pack 43.999 V
  CHECK(BmsStatusFrame::build(d)[6] == 0x01);

  d = exampleData();
  d.deltaMv = 1000;  // delta >= 1000 mV
  auto f = BmsStatusFrame::build(d);
  CHECK(f[6] == 0x01);
  CHECK(f[7] == 0x00);
}

// Decodifica CNF1..3 de volta para bitrate e ponto de amostragem
static void checkCnf(uint32_t oscHz, uint8_t cnf1, uint8_t cnf2, uint8_t cnf3) {
  Mcp2515Timing::Cnf c{};
  CHECK(Mcp2515Timing::cnfFor500k(oscHz, c));
  CHECK(c.cnf1 == cnf1 && c.cnf2 == cnf2 && c.cnf3 == cnf3);

  const uint32_t brp = c.cnf1 & 0x3F;
  const uint32_t prop = (c.cnf2 & 0x07) + 1;
  const uint32_t ps1 = ((c.cnf2 >> 3) & 0x07) + 1;
  const uint32_t ps2 = (c.cnf3 & 0x07) + 1;
  const uint32_t tqPerBit = 1 + prop + ps1 + ps2;
  CHECK(c.cnf2 & 0x80);  // BTLMODE: PS2 vem do CNF3
  CHECK(oscHz / (2 * (brp + 1) * tqPerBit) == 500000);
  CHECK((1 + prop + ps1) * 100 / tqPerBit == 75);
}

static void testMcp2515Cnf() {
  checkCnf(8000000, 0x00, 0x91, 0x01);
  checkCnf(16000000, 0x01, 0x91, 0x01);
  Mcp2515Timing::Cnf c{};
  CHECK(!Mcp2515Timing::cnfFor500k(20000000, c));  // 20 MHz não divide em 8 TQ de 250 ns
  CHECK(!Mcp2515Timing::cnfFor500k(0, c));
}

int main() {
  testParserHappyPath();
  testParserLineEndings();
  testParserRejects();
  testFrameReadmeExample();
  testFrameNegativeCurrent();
  testFrameSaturation();
  testFrameAlert();
  testMcp2515Cnf();

  if (failures) {
    std::printf("%d falha(s)\n", failures);
    return 1;
  }
  std::printf("OK\n");
  return 0;
}
