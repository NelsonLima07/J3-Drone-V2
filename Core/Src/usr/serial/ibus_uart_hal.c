/**
 * @file    ibus_uart_hal.c
 * @brief   Recepcao iBus na USART2 via DMA + IDLE (GPDMA1 CH2).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Fluxo:
 *   - Sinal iBus e invertido (idle em nivel baixo): USART_CR2_RXINV.
 *   - HAL_UARTEx_ReceiveToIdle_DMA com buffer de 32 bytes em modo Normal.
 *   - Quadro completo (32 bytes) -> DMA TC; quadro parcial -> IDLE.
 *     Nos dois casos o HAL chama HAL_UARTEx_RxEventCallback com RxState
 *     READY, permitindo rearmar a recepcao no proprio callback.
 *   - Erros (overrun/noise) abortam a recepcao e chegam em
 *     HAL_UART_ErrorCallback; ali tambem rearmamos.
 *   - Um snapshot dos canais valida e mantido para a leitura do supervisor.
 */

#include "serial/ibus_uart_hal.h"

#include "usart.h"
#include "stm32h5xx_hal.h"

#include <string.h>

/* Buffer escrito pela DMA (32 bytes = quadro iBus completo) */
static volatile uint8_t ibus_rx_dma[IBUS_FRAME_TAMANHO];

/* Snapshot do ultimo quadro valido (escrito na IRQ, lido no supervisor) */
static volatile uint16_t ibus_canais_snap[IBUS_NUM_CANAIS];
static volatile uint32_t ibus_ultimo_tick;

int ibus_uart_inicializa(void)
{
  ibus_ultimo_tick = 0;
  memset((void *)ibus_canais_snap, 0, sizeof(ibus_canais_snap));

  /* Sinal iBus invertido: idle em nivel baixo */
  __HAL_UART_DISABLE(&huart2);
  MODIFY_REG(huart2.Instance->CR2, USART_CR2_RXINV, USART_CR2_RXINV);
  __HAL_UART_ENABLE(&huart2);

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)ibus_rx_dma,
                                   IBUS_FRAME_TAMANHO) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

int ibus_uart_canais(uint16_t *canais)
{
  uint32_t primask;
  uint16_t i;

  if (canais == 0)
  {
    return 0;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (i = 0; i < IBUS_NUM_CANAIS; ++i)
  {
    canais[i] = ibus_canais_snap[i];
  }
  __set_PRIMASK(primask);

  return (ibus_ultimo_tick != 0U) ? 1 : 0;
}

int ibus_uart_tem_sinal(uint32_t agora_ms)
{
  return ((uint32_t)(agora_ms - ibus_ultimo_tick) < IBUS_SINAL_JANELA_MS) ? 1 : 0;
}

uint32_t ibus_uart_ultimo_tick(void)
{
  return ibus_ultimo_tick;
}

/* --- Override das funcoes weak do HAL -------------------------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint16_t canais[IBUS_NUM_CANAIS];
  uint16_t i;
  uint32_t primask;

  if (huart->Instance != USART2)
  {
    return;
  }

  if (Size == IBUS_FRAME_TAMANHO &&
      ibus_decodifica((const uint8_t *)ibus_rx_dma, canais) == 0)
  {
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0; i < IBUS_NUM_CANAIS; ++i)
    {
      ibus_canais_snap[i] = canais[i];
    }
    ibus_ultimo_tick = HAL_GetTick();
    __set_PRIMASK(primask);
  }

  /* Rearma a recepcao: apos TC ou IDLE o HAL ja deixou RxState READY. */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)ibus_rx_dma,
                                     IBUS_FRAME_TAMANHO);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  /* Erro de RX (overrun/noise/frame): HAL abortou o DMA e deixou READY. */
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)ibus_rx_dma,
                                     IBUS_FRAME_TAMANHO);
}
