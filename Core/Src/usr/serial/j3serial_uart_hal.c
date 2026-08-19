/**
 * @file    j3serial_uart_hal.c
 * @brief   Transporte do protocolo j3Serial na USART1 (RX por interrupcao,
 *          TX bloqueante).
 * @date    2026-08-18
 * @author  Nelson Lima
 * @ai      opencode (deepseek-v4-flash-free)
 *
 * Fluxo:
 *   - HAL_UART_Receive_IT (1 byte) rearmado no HAL_UART_RxCpltCallback
 *     (despacho por instancia, mesmo padrao do hardware_glue).
 *   - Bytes recebidos vao para um ring de 256; o supervisor esvazia no
 *     parser j3serial via j3serial_uart_ler.
 *   - TX bloqueante (HAL_UART_Transmit): frames curtos (< 204 bytes),
 *     sem risco de travar o loop por muito tempo a 115200.
 */

#include "serial/j3serial_uart_hal.h"

#include "usart.h"
#include "stm32h5xx_hal.h"

#include <string.h>

#define J3SERIAL_TX_TIMEOUT_MS 100u

static volatile uint8_t j3s_ring[J3SERIAL_RING_TAMANHO];
static volatile uint16_t j3s_ring_leitura;
static volatile uint16_t j3s_ring_escrita;
static volatile uint8_t j3s_rx_byte; /* buffer do HAL_UART_Receive_IT */

static void j3s_ring_insere(const uint8_t *dados, uint16_t n)
{
  uint16_t i;

  for (i = 0; i < n; ++i)
  {
    uint16_t prox = (uint16_t)((j3s_ring_escrita + 1u) & (J3SERIAL_RING_TAMANHO - 1u));
    if (prox == j3s_ring_leitura)
    {
      break; /* cheio: descarta */
    }
    j3s_ring[j3s_ring_escrita] = dados[i];
    j3s_ring_escrita = prox;
  }
}

int j3serial_uart_inicializa(void)
{
  j3s_ring_leitura = 0;
  j3s_ring_escrita = 0;
  j3s_rx_byte = 0;

  /* A IRQ da USART1 nao existe no CubeMX: habilitada aqui (prio 5,
     mesmo padrao das demais IRQs do projeto). */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)&j3s_rx_byte, 1) != HAL_OK)
  {
    return -1;
  }
  return 0;
}

uint32_t j3serial_uart_ler(uint8_t *buf, uint32_t max)
{
  uint32_t n = 0;

  if (buf == 0)
  {
    return 0;
  }
  while (j3s_ring_leitura != j3s_ring_escrita && n < max)
  {
    buf[n++] = j3s_ring[j3s_ring_leitura];
    j3s_ring_leitura = (uint16_t)((j3s_ring_leitura + 1u) &
                                  (J3SERIAL_RING_TAMANHO - 1u));
  }
  return n;
}

int j3serial_uart_envia(const uint8_t *dados, uint32_t n)
{
  if (dados == 0 || n == 0)
  {
    return -1;
  }
  return (HAL_UART_Transmit(&huart1, (uint8_t *)dados, (uint16_t)n,
                            J3SERIAL_TX_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

void j3serial_uart_erro(void)
{
  /* Erro de RX: o HAL abortou a recepcao e deixou RxState READY. */
  (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&j3s_rx_byte, 1);
}

/* --- Overrides weak do HAL ------------------------------------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* O HAL ja leu o RDR para j3s_rx_byte antes deste callback. */
    j3s_ring_insere((const uint8_t *)&j3s_rx_byte, 1);
    (void)HAL_UART_Receive_IT(&huart1, (uint8_t *)&j3s_rx_byte, 1);
  }
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}