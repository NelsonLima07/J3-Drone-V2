/**
 * @file    lis3mdl.c
 * @brief   Driver em C puro do magnetometro LIS3MDL (ST).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <string.h>
#include "sensors/lis3mdl.h"

/* CTRL1: TEMP_EN=0, DO=100 (100 Hz), MD=00 (continuo) -> 0x10 */
#define LIS3MDL_CTRL1_VALOR   0x10U
/* CTRL2: FS=00 (+-4 gauss), sem medicao de T compensada -> 0x00 */
#define LIS3MDL_CTRL2_VALOR   0x00U
/* CTRL3: modo continuo, sem interrupcoes -> 0x00 */
#define LIS3MDL_CTRL3_VALOR   0x00U
/* CTRL4: BDU=1 -> 0x80 */
#define LIS3MDL_CTRL4_VALOR   0x80U
/* CTRL5: fast read off, default -> 0x00 */
#define LIS3MDL_CTRL5_VALOR   0x00U

#define LIS3MDL_STATUS_ZYXDA  0x08U

int lis3mdl_inicializa(lis3mdl_t *mag, const imuc_transport_t *transporte)
{
  if (mag == 0 || transporte == 0)
  {
    return -1;
  }
  memset(mag, 0, sizeof(*mag));
  mag->transporte = transporte;
  return 0;
}

int lis3mdl_verifica_id(lis3mdl_t *mag)
{
  uint8_t id = 0;

  if (mag == 0 || mag->transporte == 0)
  {
    return 0;
  }
  if (mag->transporte->ler(mag->transporte->ctx, LIS3MDL_REG_WHO_AM_I,
                           &id, 1) != 0)
  {
    return 0;
  }
  return (id == LIS3MDL_WHO_AM_I) ? 1 : 0;
}

int lis3mdl_configura(lis3mdl_t *mag)
{
  static const uint8_t configs[5] = {
    LIS3MDL_CTRL1_VALOR,
    LIS3MDL_CTRL2_VALOR,
    LIS3MDL_CTRL3_VALOR,
    LIS3MDL_CTRL4_VALOR,
    LIS3MDL_CTRL5_VALOR
  };
  uint8_t reg;
  uint8_t i;

  if (mag == 0 || mag->transporte == 0)
  {
    return -1;
  }
  for (i = 0; i < 5u; ++i)
  {
    reg = LIS3MDL_REG_CTRL1 + i;
    if (mag->transporte->escrever(mag->transporte->ctx, reg,
                                  &configs[i], 1) != 0)
    {
      return -2;
    }
  }
  mag->ok = 1;
  return 0;
}

uint8_t lis3mdl_processa(lis3mdl_t *mag)
{
  uint8_t dados[6];
  uint8_t status = 0;
  int16_t raw[3];

  if (mag == 0 || mag->transporte == 0 || !mag->ok)
  {
    return 0;
  }
  if (mag->transporte->ler(mag->transporte->ctx, LIS3MDL_REG_STATUS,
                           &status, 1) != 0)
  {
    return 0;
  }
  if ((status & LIS3MDL_STATUS_ZYXDA) == 0)
  {
    return 0;
  }
  if (mag->transporte->ler(mag->transporte->ctx, LIS3MDL_REG_OUT_X_L,
                           dados, sizeof(dados)) != 0)
  {
    return 0;
  }
  raw[0] = (int16_t)(dados[0] | (uint16_t)(dados[1] << 8));
  raw[1] = (int16_t)(dados[2] | (uint16_t)(dados[3] << 8));
  raw[2] = (int16_t)(dados[4] | (uint16_t)(dados[5] << 8));

  mag->medida.x = (float)raw[0] / LIS3MDL_ESCALA_LSB_GAUSS;
  mag->medida.y = (float)raw[1] / LIS3MDL_ESCALA_LSB_GAUSS;
  mag->medida.z = (float)raw[2] / LIS3MDL_ESCALA_LSB_GAUSS;
  mag->contador_leituras++;
  return 1;
}

const mag_medida_t *lis3mdl_medida(const lis3mdl_t *mag)
{
  return (mag != 0 && mag->contador_leituras > 0u) ? &mag->medida : 0;
}
