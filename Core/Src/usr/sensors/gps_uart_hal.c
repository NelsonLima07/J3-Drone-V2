/**
 * @file    gps_uart_hal.c
 * @brief   Transporte USART3 + DMA (GPDMA1 CH7) do GPS BN-220.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * PORTAS (ver docs/pinout_map.md - secao GPS):
 *   - USART3_TX = PC10 (AF7), USART3_RX = PC4 (AF7)   [CubeMX]
 *   - DMA GPDMA1 CH7, requisicao GPDMA1_REQUEST_USART3_RX (25)
 *   - NVIC: GPDMA1_Channel7_IRQn e USART3_IRQn prioridade 5
 * CONFIGURACAO: o CubeMX gera a USART3 a 115200; aqui reconfiguramos
 * para 9600 8N1 (baudrate padrao do BN-220). Se mudar o baud do
 * modulo, ajuste o valor abaixo.
 *
 * O GPDMA do STM32H5 nao tem modo circular classico (a constante
 * DMA_CIRCULAR nao existe no HAL H5). Usamos o mesmo padrao do iBus:
 * DMA Normal + HAL_UARTEx_ReceiveToIdle_DMA + re-arme no callback.
 * A cada evento TC (buffer cheio) ou IDLE (folga de 1 frame) o
 * callback copia os bytes recebidos para um ring e rearma a DMA.
 * Erros de RX (overrun) chegam em HAL_UART_ErrorCallback, que rearma.
 */

#include "sensors/gps_uart_hal.h"

#include "usart.h"
#include "stm32h5xx_hal.h"

#include <string.h>

#define GPS_BAUD_RATE 9600U

/* Buffer alvo da DMA (escrito por bloco em modo Normal). */
#define GPS_DMA_TAMANHO 512U

/* Ring onde o callback deposita os bytes. Deve ser potencia de 2. */
#define GPS_RING_TAMANHO 1024U

static volatile uint8_t gps_rx_dma[GPS_DMA_TAMANHO];
static volatile uint8_t gps_ring[GPS_RING_TAMANHO];
static volatile uint16_t gps_ring_escrita;
static volatile uint16_t gps_ring_leitura;

static DMA_HandleTypeDef hdma_usart3_rx;

static void gps_ring_insere(const uint8_t *buf, uint16_t n)
{
  uint16_t i;

  for (i = 0; i < n; ++i)
  {
    gps_ring[gps_ring_escrita] = buf[i];
    gps_ring_escrita = (uint16_t)((gps_ring_escrita + 1u) & (GPS_RING_TAMANHO - 1u));
    if (gps_ring_escrita == gps_ring_leitura)
    {
      gps_ring_leitura = (uint16_t)((gps_ring_leitura + 1u) & (GPS_RING_TAMANHO - 1u));
    }
  }
}

int gps_uart_inicializa(void)
{
  gps_ring_escrita = 0;
  gps_ring_leitura = 0;

  /* Reconfigura a USART3 para 9600 8N1 (o CubeMX gera 115200). */
  if (HAL_UART_DeInit(&huart3) != HAL_OK)
  {
    return -1;
  }
  huart3.Init.BaudRate = GPS_BAUD_RATE;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    return -2;
  }

  /* DMA GPDMA1 CH7: RX em modo Normal (requisicao USART3_RX = 25). */
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  hdma_usart3_rx.Instance = GPDMA1_Channel7;
  hdma_usart3_rx.Init.Request = GPDMA1_REQUEST_USART3_RX;
  hdma_usart3_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart3_rx.Init.SrcInc = DMA_SINC_FIXED;
  hdma_usart3_rx.Init.DestInc = DMA_DINC_INCREMENTED;
  hdma_usart3_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_usart3_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_usart3_rx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_usart3_rx.Init.SrcBurstLength = 1;
  hdma_usart3_rx.Init.DestBurstLength = 1;
  hdma_usart3_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma_usart3_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_usart3_rx.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
  {
    return -3;
  }
  if (HAL_DMA_ConfigChannelAttributes(&hdma_usart3_rx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    return -4;
  }

  __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);
  hdma_usart3_rx.Parent = &huart3;

  HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)gps_rx_dma,
                                   GPS_DMA_TAMANHO) != HAL_OK)
  {
    return -5;
  }
  return 0;
}

uint32_t gps_uart_ler(uint8_t *buf, uint32_t max)
{
  uint32_t n = 0;

  while (gps_ring_leitura != gps_ring_escrita && n < max)
  {
    buf[n++] = gps_ring[gps_ring_leitura];
    gps_ring_leitura = (uint16_t)((gps_ring_leitura + 1u) & (GPS_RING_TAMANHO - 1u));
  }
  return n;
}

/* --- Handlers chamados pelo dispatcher do hardware_glue -------------------- */

void gps_uart_rx_event(uint16_t size)
{
  gps_ring_insere((const uint8_t *)gps_rx_dma, size);

  /* Apos TC ou IDLE o HAL ja deixou RxState READY: rearma. */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)gps_rx_dma,
                                     GPS_DMA_TAMANHO);
}

void gps_uart_erro(void)
{
  /* Erro de RX (overrun/noise/frame): HAL abortou o DMA e deixou READY. */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)gps_rx_dma,
                                     GPS_DMA_TAMANHO);
}

void GPDMA1_Channel7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}
