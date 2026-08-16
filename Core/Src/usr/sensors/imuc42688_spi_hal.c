/**
 * @file    imuc42688_spi_hal.c
 * @brief   Glue HAL (STM32 + SPI1 + GPDMA1) do driver do ICM-42688-P
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Hardware (ver docs/pinout_map.md):
 *   - SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (AF5), kernel PLL2P = 24 MHz
 *   - CS  : PA4 (GPIO, ativo em nivel baixo)
 *   - INT1: PB0 (EXTI0 rising) - data ready do sensor
 *   - DMA : GPDMA1 Channel0 (RX, request 6) / Channel1 (TX, request 7)
 */

#include "sensors/imuc42688_spi_hal.h"

#include "spi.h"
#include "gpio.h"

#include <string.h>

#define IMUC_CS_ATIVO   GPIO_PIN_RESET
#define IMUC_CS_IDLE    GPIO_PIN_SET
#define IMUC_TIMEOUT_MS 10U

/* Estado global vinculado via imuc_hal_vincular */
static imuc42688_t *imu_estado = 0;
static imuc_hal_ao_medir_t ao_medir_cb = 0;

/* Buffers da leitura DMA: 1 byte de endereco + 14 bytes de dados */
static uint8_t spi_tx[ICM_BURST_TAMANHO + 1];
static uint8_t spi_rx[ICM_BURST_TAMANHO + 1];

/* --- Transporte bloqueante (usado na inicializacao) ----------------------- */

static int hal_ler(void *ctx, uint8_t endereco, uint8_t *dados, uint16_t n)
{
  uint8_t tx[ICM_BURST_TAMANHO + 1];
  uint8_t rx[ICM_BURST_TAMANHO + 1];
  HAL_StatusTypeDef st;
  uint16_t i;

  (void)ctx;
  if (n == 0 || n > ICM_BURST_TAMANHO ||
      HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  {
    return -1;
  }

  tx[0] = (uint8_t)(endereco | ICM_SPI_BIT_LEITURA);
  for (i = 1; i <= n; ++i)
  {
    tx[i] = 0;
  }

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_ATIVO);
  st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(n + 1), IMUC_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_IDLE);
  if (st != HAL_OK)
  {
    return -1;
  }

  memcpy(dados, &rx[1], n);
  return 0;
}

static int hal_escrever(void *ctx, uint8_t endereco, const uint8_t *dados, uint16_t n)
{
  uint8_t tx[ICM_BURST_TAMANHO + 1];
  HAL_StatusTypeDef st;
  uint16_t i;

  (void)ctx;
  if (n == 0 || n > ICM_BURST_TAMANHO ||
      HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  {
    return -1;
  }

  tx[0] = (uint8_t)(endereco & ICM_SPI_BIT_ESCRITA);
  for (i = 0; i < n; ++i)
  {
    tx[i + 1] = dados[i];
  }

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_ATIVO);
  st = HAL_SPI_Transmit(&hspi1, tx, (uint16_t)(n + 1), IMUC_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_IDLE);
  if (st != HAL_OK)
  {
    return -1;
  }

  return 0;
}

static void hal_atraso_us(void *ctx, uint32_t us)
{
  (void)ctx;
  if (us > 0)
  {
    HAL_Delay((uint32_t)((us + 999U) / 1000U));
  }
}

static const imuc_transport_t transporte_spi = {
  .ctx = 0,
  .ler = hal_ler,
  .escrever = hal_escrever,
  .atraso_us = hal_atraso_us,
};

/* --- Interface publica ------------------------------------------------------ */

int imuc_hal_vincular(imuc42688_t *imu, imuc_hal_ao_medir_t ao_medir)
{
  if (imu == 0)
  {
    return -1;
  }
  imu_estado = imu;
  ao_medir_cb = ao_medir;
  return imuc42688_inicializa(imu, &transporte_spi);
}

int imuc_hal_inicia_burst(void)
{
  uint16_t i;

  if (imu_estado == 0 ||
      HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
  {
    return -1;
  }

  spi_tx[0] = (uint8_t)(ICM_BURST_INICIO | ICM_SPI_BIT_LEITURA);
  for (i = 1; i <= ICM_BURST_TAMANHO; ++i)
  {
    spi_tx[i] = 0;
  }

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_ATIVO);
  if (HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx, spi_rx,
                                  (uint16_t)(ICM_BURST_TAMANHO + 1)) != HAL_OK)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_IDLE);
    return -1;
  }
  return 0;
}

/* --- Callbacks do HAL (override das funcoes weak) --------------------------- */

/**
 * @brief  Fim da transacao SPI TX+RX via DMA: levanta o CS, decodifica o
 *         burst de 14 bytes e notifica o sistema de controle.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, IMUC_CS_IDLE);

    if (imu_estado != 0)
    {
      imuc42688_processa_burst(imu_estado, &spi_rx[1]);
      if (ao_medir_cb != 0)
      {
        ao_medir_cb(&imu_estado->medida);
      }
    }
  }
}

/**
 * @brief  Exti de data ready (PB0/INT1): inicia a leitura do burst via DMA.
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin == IMU_INT1_Pin)
  {
    imuc_hal_inicia_burst();
  }
}
