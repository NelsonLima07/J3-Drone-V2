/**
 * @file    j3serial_uart_hal.h
 * @brief   Transporte do protocolo j3Serial na USART1 (RX por interrupcao,
 *          TX bloqueante).
 * @date    2026-08-18
 * @author  Nelson Lima
 * @ai      opencode (deepseek-v4-flash-free)
 *
 * Este arquivo e compilado apenas no firmware (fora da lib usr).
 * USART1: PB14 TX / PB15 RX, 115200 8N1 (configurada no CubeMX).
 * A IRQ da USART1 nao e gerada pelo CubeMX: habilitada aqui em codigo
 * (padrao do hardware_glue). O USART1_IRQHandler fica neste arquivo
 * (padrao do gps_uart_hal.c).
 */

#ifndef USR_SERIAL_J3SERIAL_UART_HAL_H
#define USR_SERIAL_J3SERIAL_UART_HAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Tamanho do ring de recepcao (frames max. 204 bytes). */
#define J3SERIAL_RING_TAMANHO 256u

/**
 * @brief  Habilita a IRQ da USART1 e inicia a recepcao por interrupcao
 *         (HAL_UART_Receive_IT, 1 byte por vez).
 * @return 0 em caso de sucesso, < 0 caso contrario.
 */
int j3serial_uart_inicializa(void);

/**
 * @brief  Esvazia o ring de recepcao.
 * @return quantidade de bytes copiados.
 */
uint32_t j3serial_uart_ler(uint8_t *buf, uint32_t max);

/**
 * @brief  Envia um buffer pela USART1 (bloqueante).
 * @return 0 em caso de sucesso, < 0 caso contrario.
 */
int j3serial_uart_envia(const uint8_t *dados, uint32_t n);

/**
 * @brief  Erro de RX (overrun/noise/frame): rearma a recepcao.
 *         Chamado pelo HAL_UART_ErrorCallback (hardware_glue).
 */
void j3serial_uart_erro(void);

#ifdef __cplusplus
}
#endif

#endif /* USR_SERIAL_J3SERIAL_UART_HAL_H */