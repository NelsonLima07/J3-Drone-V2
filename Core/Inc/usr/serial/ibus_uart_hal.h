/**
 * @file    ibus_uart_hal.h
 * @brief   Recepcao iBus na USART2 via DMA + IDLE (GPDMA1 CH2).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Este arquivo e compilado apenas no firmware (fora da lib usr).
 * Usa o hdma_usart2_rx vinculado pelo hardware_glue.
 */

#ifndef USR_SERIAL_IBUS_UART_HAL_H
#define USR_SERIAL_IBUS_UART_HAL_H

#include "serial/ibus.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Janela (ms) sem quadro iBus para considerar o sinal perdido */
#define IBUS_SINAL_JANELA_MS 100U

/**
 * @brief  Habilita a inversao do RX (sinal iBus e invertido) e inicia a
 *         recepcao por DMA na USART2.
 * @return 0 em caso de sucesso, < 0 caso contrario.
 */
int ibus_uart_inicializa(void);

/**
 * @brief  Copia o ultimo quadro valido recebido.
 * @param  canais saida com IBUS_NUM_CANAIS valores.
 * @return 1 se houve sinal desde o boot, 0 caso contrario.
 */
int ibus_uart_canais(uint16_t *canais);

/**
 * @brief  Indica se ha sinal recente (ultimo quadro dentro de
 *         IBUS_SINAL_JANELA_MS de agora_ms).
 */
int ibus_uart_tem_sinal(uint32_t agora_ms);

/**
 * @brief  Tick (HAL_GetTick) do ultimo quadro iBus valido.
 */
uint32_t ibus_uart_ultimo_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* USR_SERIAL_IBUS_UART_HAL_H */
