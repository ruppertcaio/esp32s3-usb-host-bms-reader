#include "report.h"
#include "config.h"

#include <cinttypes>
#include <cstdio>

// Portado de report.cpp da versão Arduino (só as partes BMS, USB, TX CAN e
// latência). Usa printf direto: o relatório é um bloco de texto, não log.

namespace {

void printBms(const Report::Snapshot &s) {
  printf("================ BMS EVPV ================\n");

  if (!s.bmsFresh) {
    if (s.usbStreaming) {
      printf("Sem pacotes válidos (inválidos: %" PRIu32 "). Verifique o baud rate/cabo.\n", s.badPackets);
    } else {
      printf("Conversor USB do BMS não conectado.\n");
    }
    return;
  }

  const BmsData &bms = s.bms;
  uint32_t packMv = 0;
  int minIdx = 0, maxIdx = 0;
  for (int i = 0; i < BmsData::NUM_CELLS; i++) {
    packMv += bms.cellMv[i];
    if (bms.cellMv[i] < bms.cellMv[minIdx]) minIdx = i;
    if (bms.cellMv[i] > bms.cellMv[maxIdx]) maxIdx = i;
  }
  float packV = packMv / 1000.0f;
  float currentA = bms.currentMa / 1000.0f;

  for (int i = 0; i < BmsData::NUM_CELLS; i++) {
    printf("C%-2d %5u mV%s", i + 1, bms.cellMv[i], (i % 4 == 3) ? "\n" : "   ");
  }
  printf("Pack: %.3f V | Min: C%d %u mV | Max: C%d %u mV | Delta: %u mV\n",
         packV, minIdx + 1, bms.cellMv[minIdx], maxIdx + 1, bms.cellMv[maxIdx], bms.deltaMv);
  printf("Corrente: %.3f A | Potência: %.1f W\n", currentA, packV * currentA);
  printf("Descarga: %s | Recarga: %s | Balanceando: %s\n",
         bms.dischargeAllowed ? "PERMITIDA" : "BLOQUEADA",
         bms.chargeAllowed ? "PERMITIDA" : "BLOQUEADA",
         bms.balancing ? "SIM" : "NAO");
  printf("Temp (C):");
  for (int i = 0; i < BmsData::NUM_TEMPS; i++) printf(" T%d=%d", i + 1, bms.tempC[i]);
  printf("\nPacotes OK: %" PRIu32 " | Invalidos: %" PRIu32 "\n", s.packets, s.badPackets);
}

void printCan(const Report::CanTxStatus &c) {
  printf("================ CAN TX 0x300 (MCP2515) ================\n");

  if (!c.online) {
    printf("MCP2515 offline (falhas de init: %" PRIu32 "). Verifique SPI, alimentação e cristal.\n",
           c.initFailures);
  } else {
    // RX0OVR/RX1OVR ficam fora: a recepção não é lida e os buffers de RX
    // ficam sempre cheios.
    uint8_t eflg = c.err.eflg & (uint8_t)~Mcp2515::EFLG_RXOVR;
    const char *state = (eflg & Mcp2515::EFLG_TXBO)                          ? " | BUS-OFF"
                        : (eflg & (Mcp2515::EFLG_TXEP | Mcp2515::EFLG_RXEP)) ? " | ERROR-PASSIVE"
                                                                              : "";
    printf("Modo: %s | enviados=%" PRIu32 " | buffers ocupados=%" PRIu32 " | erros SPI=%" PRIu32 "\n",
           LISTEN_ONLY_MODE ? "listen-only (sem TX)" : "normal", c.sent, c.allBusy, c.spiErrors);
    printf("REC=%u TEC=%u EFLG=0x%02X%s\n", c.err.rec, c.err.tec, eflg, state);
  }
  printf("Tarefa CAN: periodo max %" PRIu32 " us (nominal %" PRIu32 " us) | %" PRIu32 " ciclos na janela\n",
         c.maxPeriodUs, BMS_CAN_TX_PERIOD_MS * 1000, c.cycles);
}

}  // namespace

namespace Report {

void print(const Snapshot &s) {
  flockfile(stdout);  // bloco inteiro sem linhas de log de outras tasks no meio
  printBms(s);
  printCan(s.can);
  printf("\n");
  funlockfile(stdout);
}

}  // namespace Report
