/**
 * @file    i2c2_hal.c
 * @brief   Transporte I2C2 (polling) para LIS3MDL.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * PORTAS (ver docs/pinout_map.md - secao I2C2):
 *   - I2C2_SCL = PB10, I2C2_SDA = PB12 (AF4, open-drain)  [CubeMX]
 *   - LIS3MDL a 0x1C no barramento I2C2.
 * CONFIGURACAO: o CubeMX gera a I2C2 a ~400 kHz (Fast Mode). Os pinos
 * ficam sem pull-up interno (NOPULL): use os pull-ups dos modulos.
 * O BMP581 agora usa I3C1 (PB6/PB7) via i3c_bmp581_hal.
 */

#include "sensors/i2c2_hal.h"

#include "i2c.h"
#include "stm32h5xx_hal.h"

#define I2C2_TIMEOUT_MS 10U

static void i2c2_recupera(void)
{
  (void)HAL_I2C_DeInit(&hi2c2);
  (void)HAL_I2C_Init(&hi2c2);
}

static int i2c2_ler(void *ctx, uint8_t reg, uint8_t *dados, uint16_t n)
{
  uint8_t endereco = (uint8_t)(uintptr_t)ctx;
  HAL_StatusTypeDef st;
  int tentativa;

  for (tentativa = 0; tentativa < 2; ++tentativa)
  {
    st = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)endereco << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, dados, n, I2C2_TIMEOUT_MS);
    if (st == HAL_OK)
    {
      return 0;
    }
    if (st != HAL_BUSY && st != HAL_ERROR)
    {
      return -1;
    }
    i2c2_recupera();
  }
  return -2;
}

static int i2c2_escrever(void *ctx, uint8_t reg, const uint8_t *dados,
                         uint16_t n)
{
  uint8_t endereco = (uint8_t)(uintptr_t)ctx;
  HAL_StatusTypeDef st;
  int tentativa;

  for (tentativa = 0; tentativa < 2; ++tentativa)
  {
    st = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)endereco << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)dados, n,
                           I2C2_TIMEOUT_MS);
    if (st == HAL_OK)
    {
      return 0;
    }
    if (st != HAL_BUSY && st != HAL_ERROR)
    {
      return -1;
    }
    i2c2_recupera();
  }
  return -2;
}

static void i2c2_atraso_us(void *ctx, uint32_t us)
{
  volatile uint32_t i;
  (void)ctx;
  for (i = 0; i < us; ++i)
  {
  }
}

int i2c2_hal_vincula(imuc_transport_t *t, uint8_t endereco)
{
  if (t == 0)
  {
    return -1;
  }
  t->ctx = (void *)(uintptr_t)endereco;
  t->ler = i2c2_ler;
  t->escrever = i2c2_escrever;
  t->atraso_us = i2c2_atraso_us;
  return 0;
}
