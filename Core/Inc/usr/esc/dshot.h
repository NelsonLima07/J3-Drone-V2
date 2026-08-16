/**
 * @file    dshot.h
 * @brief   Codificacao do protocolo DShot (puro, sem dependencia de HAL).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Frame DShot de 16 bits (transmitido MSB primeiro):
 *   bits[0..10]  = valor (11 bits; bit 10 = request de telemetria)
 *   bits[11]     = stop bit (sempre 0)
 *   bits[12..15] = CRC4 = XOR dos nibbles de (valor << 1)
 *
 * Temporizacao DShot600 (spec Betaflight/BLHeli):
 *   periodo do bit  = 1,67 us  -> 200 MHz / 600 kHz = 333 ticks
 *   bit "0"         = 625 ns   -> 125 ticks
 *   bit "1"         = 1250 ns  -> 250 ticks
 *   pausa entre frames >= 2 us (bits de gap com CCR = 0 mantem a linha baixa)
 *
 * Faixa do valor: 0 = desarmado; 1..47 reservados; 48..2047 = throttle.
 */

#ifndef USR_ESC_DSHOT_H
#define USR_ESC_DSHOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_FRAME_BITS      16u
#define DSHOT_GAP_BITS        4u
#define DSHOT_BUFFER_TAMANHO  (DSHOT_FRAME_BITS + DSHOT_GAP_BITS)

#define DSHOT_MIN_THROTTLE    48u
#define DSHOT_MAX_THROTTLE    2047u
#define DSHOT_BIT_TELEMETRIA  0x400u

/**
 * @brief  CRC4 do DShot: XOR dos nibbles de (valor << 1), mascara 0xF.
 */
uint16_t dshot_crc4(uint16_t valor);

/**
 * @brief  Monta os 16 bits do quadro (MSB primeiro) a partir do valor.
 * @param  valor 0..2047 (bit 0x400 = telemetria).
 */
void dshot_quadro(uint16_t valor, uint16_t bits[DSHOT_FRAME_BITS]);

/**
 * @brief  Converte os bits do quadro em valores de CCR (com gap de CCR 0).
 * @param  ccr_zero CCR do bit "0"; ccr_um CCR do bit "1".
 */
void dshot_buffer_ccr(const uint16_t bits[DSHOT_FRAME_BITS],
                      uint32_t ccr_zero, uint32_t ccr_um,
                      uint32_t buffer[DSHOT_BUFFER_TAMANHO]);

/**
 * @brief  Mapeia o comando de motor normalizado 0..1 para 48..2047 (clamped).
 */
uint16_t dshot_valor_throttle(float motor);

/**
 * @brief  Monta o buffer de CCR completo para um motor (valor 48..2047).
 *         Use valor 0 (ou dshot_buffer_ccr com quadro 0) para desarmado.
 */
void dshot_buffer_motor(float motor, uint32_t ccr_zero, uint32_t ccr_um,
                        uint32_t buffer[DSHOT_BUFFER_TAMANHO]);

#ifdef __cplusplus
}
#endif

#endif /* USR_ESC_DSHOT_H */
