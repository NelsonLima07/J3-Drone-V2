/**
 * @file    hardware_glue.c
 * @brief   Reparo em codigo da regressao do CubeMX (ver hardware_glue.h).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Ordem de inicializacao (importante):
 *   1. PLL2 habilitado primeiro (M=1, N=96, P=8 -> PLL2P = 48 MHz).
 *   2. SPI1 reconfigurado via DeInit+Init: o MspInit gerado pelo CubeMX
 *      re-seleciona PLL1Q como kernel; por isso o kernel PLL2P e
 *      selecionado por ultimo (apos o HAL_SPI_Init).
 *   3. DMA do SPI1 (GPDMA1 CH0=RX/req 6, CH1=TX/req 7) e da USART2
 *      (CH2=RX/req 23) vinculados aos handles, com NVIC prio 5.
 *
 * Este arquivo e compilado apenas no firmware (fora da lib usr).
 */

#include "system/hardware_glue.h"

#include "spi.h"
#include "usart.h"
#include "stm32h5xx_hal.h"

#include "serial/ibus_uart_hal.h"
#include "serial/j3serial_uart_hal.h"
#include "sensors/gps_uart_hal.h"

/* Handles de DMA (GPDMA1) usados pelo SPI1 e pela USART2 */
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* --- PLL2: fonte do kernel do SPI1 (48 MHz; SPI1/2 = 24 MHz) ------------- */

static void pll2_habilita(void)
{
  RCC_PLL2InitTypeDef pll2 = {0};

  pll2.PLL2Source   = RCC_PLL2_SOURCE_CSI;
  pll2.PLL2M        = 1;
  pll2.PLL2N        = 96;
  pll2.PLL2P        = 8;
  pll2.PLL2Q        = 2;
  pll2.PLL2R        = 2;
  pll2.PLL2RGE      = RCC_PLL2_VCIRANGE_2;
  pll2.PLL2VCOSEL   = RCC_PLL2_VCORANGE_WIDE;
  pll2.PLL2FRACN    = 0;
  pll2.PLL2ClockOut = RCC_PLL2_DIVP;

  if (HAL_RCCEx_EnablePLL2(&pll2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* --- SPI1: prescaler /2 + kernel PLL2P ------------------------------------ */

static void spi1_reconfigura(void)
{
  RCC_PeriphCLKInitTypeDef clk = {0};

  if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Kernel PLL2P por ultimo: o MspInit do HAL_SPI_Init reaplica PLL1Q. */
  clk.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
  clk.Spi1ClockSelection = RCC_SPI1CLKSOURCE_PLL2P;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
  {
    Error_Handler();
  }
}

/* --- DMA do SPI1 (GPDMA1 CH0 = RX, CH1 = TX) ------------------------------ */

static void dma_spi1_configura(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  hdma_spi1_rx.Instance = GPDMA1_Channel0;
  hdma_spi1_rx.Init.Request = GPDMA1_REQUEST_SPI1_RX;
  hdma_spi1_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.SrcInc = DMA_SINC_FIXED;
  hdma_spi1_rx.Init.DestInc = DMA_DINC_INCREMENTED;
  hdma_spi1_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi1_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi1_rx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_spi1_rx.Init.SrcBurstLength = 1;
  hdma_spi1_rx.Init.DestBurstLength = 1;
  hdma_spi1_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma_spi1_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi1_rx.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&hdma_spi1_rx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }

  /* Parent: os callbacks de DMA do HAL SPI (H5) resolvem o handle SPI via
     hdma->Parent; o HAL nunca o atribui (so o limpa). */
  hdma_spi1_rx.Parent = &hspi1;

  hdma_spi1_tx.Instance = GPDMA1_Channel1;
  hdma_spi1_tx.Init.Request = GPDMA1_REQUEST_SPI1_TX;
  hdma_spi1_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma_spi1_tx.Init.DestInc = DMA_DINC_FIXED;
  hdma_spi1_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_spi1_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_spi1_tx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_spi1_tx.Init.SrcBurstLength = 1;
  hdma_spi1_tx.Init.DestBurstLength = 1;
  hdma_spi1_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma_spi1_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&hdma_spi1_tx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }

  hdma_spi1_tx.Parent = &hspi1;

  __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);
  __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);

  HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
}

/* --- DMA da USART2 RX (GPDMA1 CH2) ---------------------------------------- */

static void dma_usart2_rx_configura(void)
{
  hdma_usart2_rx.Instance = GPDMA1_Channel2;
  hdma_usart2_rx.Init.Request = GPDMA1_REQUEST_USART2_RX;
  hdma_usart2_rx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart2_rx.Init.SrcInc = DMA_SINC_FIXED;
  hdma_usart2_rx.Init.DestInc = DMA_DINC_INCREMENTED;
  hdma_usart2_rx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_usart2_rx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_usart2_rx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_usart2_rx.Init.SrcBurstLength = 1;
  hdma_usart2_rx.Init.DestBurstLength = 1;
  hdma_usart2_rx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma_usart2_rx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_usart2_rx.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA_ConfigChannelAttributes(&hdma_usart2_rx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

  /* Parent: o UART HAL (H5) resolve huart via hdma->Parent nos callbacks. */
  hdma_usart2_rx.Parent = &huart2;

  HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
  /* O MspInit do CubeMX programa USART2 com prioridade 0; ajusta para 5. */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* --- API publica ----------------------------------------------------------- */

void hardware_glue_inicializa(void)
{
  pll2_habilita();
  spi1_reconfigura();
  dma_spi1_configura();
  dma_usart2_rx_configura();
}

/* --- IRQ handlers (weak no startup; ausentes no stm32h5xx_it.c) ------------ */

void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi1);
}

void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

void GPDMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

void GPDMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/* --- Overrides weak do HAL: despacho dos callbacks de RX ------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART2)
  {
    ibus_uart_rx_event(size);
  }
  else if (huart->Instance == USART3)
  {
    gps_uart_rx_event(size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    j3serial_uart_erro();
  }
  else if (huart->Instance == USART2)
  {
    ibus_uart_erro();
  }
  else if (huart->Instance == USART3)
  {
    gps_uart_erro();
  }
}
