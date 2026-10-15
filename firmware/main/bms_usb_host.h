#pragma once

#include <cstddef>
#include <cstdint>
#include "freertos/FreeRTOS.h"

// ============================================================================
// Transporte do BMS EVPV: ESP32-S3 como USB Host na porta "USB" (GPIO19/20).
//
// O micro USB do BMS é um conversor USB-serial. Suportados: CDC-ACM
// (CH343/CH9102/MCU com USB nativo), CH340/CH341, CP210x e FTDI.
//
// Toda a pilha USB (enumeração, configuração do conversor, transferências
// bulk IN) roda em tarefas FreeRTOS no core 0. Os bytes recebidos vão para um
// stream buffer que a tarefa do parser lê com read().
// ============================================================================

namespace BmsUsb {

// Instala o USB Host e cria as tarefas. Retorna false se o driver falhar.
bool begin(uint32_t baud);

// Copia até maxLen bytes recebidos do BMS, esperando até 'wait' ticks pelo
// primeiro byte. Retorna 0 se nada chegou no prazo.
size_t read(uint8_t *buf, size_t maxLen, TickType_t wait);

// true enquanto há um conversor aberto e com leitura bulk IN ativa.
bool isStreaming();

}  // namespace BmsUsb
