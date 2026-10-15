#pragma once

#include <cstdint>
#include "BmsEvpvParser.h"
#include "drivers/Mcp2515.h"

// ============================================================================
// Relatório periódico no console (porta UART).
//
// A tarefa de relatório copia o estado compartilhado (sob o mutex) para um
// Snapshot e imprime fora do mutex: o printf lento não segura o parser nem a
// tarefa CAN.
// ============================================================================

namespace Report {

struct CanTxStatus {
  bool online;
  uint32_t initFailures;
  uint32_t sent;       // frames 0x300 entregues a um TXB
  uint32_t allBusy;    // os 3 TXB ainda esperando ACK
  uint32_t spiErrors;  // falha de SPI durante o envio/leitura de erros
  Mcp2515::ErrorState err;
  // Latência da tarefa CAN na janela: maior intervalo entre dois ciclos
  // (nominal BMS_CAN_TX_PERIOD_MS) e quantos ciclos rodaram.
  uint32_t maxPeriodUs;
  uint32_t cycles;
};

struct Snapshot {
  bool usbStreaming;
  bool bmsFresh;
  BmsData bms;
  uint32_t packets;
  uint32_t badPackets;
  CanTxStatus can;
};

void print(const Snapshot &s);

}  // namespace Report
