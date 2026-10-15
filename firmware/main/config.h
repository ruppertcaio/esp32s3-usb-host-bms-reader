#pragma once

// ============================================================================
// Configuração do leitor BMS EVPV (ESP32-S3-DevKitC-1, ESP-IDF v5.5.1)
//
//   - Console (logs + relatório) na porta "UART" do DevKitC-1 (UART0, 115200)
//   - Porta "USB" nativa (GPIO19/20) = USB Host do BMS (sdkconfig.defaults
//     deixa o console fora do USB nativo)
//
// Fonte do BMS: Manual de uso BMS EVPV V2 (serial 250 kbps, pacote CSV).
// ============================================================================

#include <cstddef>
#include <cstdint>
#include "soc/soc_caps.h"

#if !SOC_USB_OTG_SUPPORTED
#error "O leitor do BMS usa USB Host: compile para esp32s3 (idf.py set-target esp32s3)."
#endif

// ===================== Relatório (console UART) =====================
constexpr uint32_t PRINT_PERIOD_MS = 1000;

// ===================== BMS EVPV V2.0 (USB Host) =====================
constexpr uint32_t BMS_BAUD = 250000;           // velocidade serial do BMS (manual, pág. 3)
constexpr size_t BMS_RX_STREAM_SIZE = 4096;     // bytes entre o callback USB e o parser
constexpr uint32_t BMS_DATA_TIMEOUT_MS = 3000;  // sem pacote válido neste intervalo = dado velho
constexpr bool BMS_PRINT_RAW_LINES = false;     // imprime cada linha bruta recebida
constexpr uint32_t BMS_CAN_TX_PERIOD_MS = 250;  // frame 0x300; o BMS amostra a cada 250 ms (manual, pág. 3)

// ===================== MCP2515 (SPI2/FSPI) =====================
// Pinos da fiação da bancada (FSPI padrão do DevKitC-1). Não use GPIO19/20
// (USB do BMS) nem GPIO43/44 (UART do console).
// ATENÇÃO: módulos MCP2515+TJA1050 costumam ser 5 V; o ESP32-S3 é 3,3 V.
// Use level shifter no MISO se o módulo for alimentado em 5 V.
constexpr int MCP2515_SCK_PIN = 12;
constexpr int MCP2515_MISO_PIN = 13;
constexpr int MCP2515_MOSI_PIN = 11;
constexpr int MCP2515_CS_PIN = 10;
// INT não é usado: o firmware só transmite e lê os registradores de erro
// por polling a cada BMS_CAN_TX_PERIOD_MS. O fio INT pode ficar desligado.

// Cristal do módulo: 8 MHz ou 16 MHz (qualquer múltiplo de 8 MHz serve,
// ver drivers/Mcp2515Timing.h). Conferir o que está gravado no cristal.
constexpr uint32_t MCP2515_OSC_HZ = 8000000;
constexpr int MCP2515_SPI_CLOCK_HZ = 2500000;  // MCP2515 aceita até 10 MHz; reduza se usar level shifter lento
// Bitrate fixo em 500 kbit/s (Mcp2515Timing::BITRATE).

// false = modo normal: transmite o 0x300 e dá ACK no barramento
// true  = listen-only: passivo, não transmite nada (o 0x300 não sai)
// ATENÇÃO: em modo normal, bitrate/cristal errados injetam error frames no barramento.
constexpr bool LISTEN_ONLY_MODE = false;

constexpr uint32_t MCP2515_RETRY_MS = 2000;  // nova tentativa de init se o MCP2515 não responder
