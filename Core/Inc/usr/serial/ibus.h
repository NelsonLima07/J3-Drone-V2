/**
 * @file    ibus.h
 * @brief   Decodificador do protocolo iBus (FlySky) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Nao depende do HAL: recebe o quadro de 32 bytes e entrega os 14 canais.
 * O transporte (USART2 + DMA + IDLE) fica no ibus_uart_hal.
 */

#ifndef USR_SERIAL_IBUS_H
#define USR_SERIAL_IBUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IBUS_NUM_CANAIS     14U
#define IBUS_FRAME_TAMANHO  32U
#define IBUS_SYNC_BYTE      0x20U
#define IBUS_SYNC2_BYTE     0x40U

/**
 * @brief  Decodifica um quadro iBus completo (IBUS_FRAME_TAMANHO bytes).
 * @param  bytes  quadro de 32 bytes: sync(2) + 14 canais LE + checksum(2).
 * @param  canais saida com IBUS_NUM_CANAIS valores (1000..2000).
 * @return 0 em caso de sucesso, < 0 se o quadro for invalido.
 * @retval -1 argumento nulo
 * @retval -2 sync invalido
 * @retval -3 checksum invalido (0xFFFF - soma dos 30 primeiros bytes)
 */
int ibus_decodifica(const uint8_t *bytes, uint16_t *canais);

#ifdef __cplusplus
}
#endif

#endif /* USR_SERIAL_IBUS_H */
