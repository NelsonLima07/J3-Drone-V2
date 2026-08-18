/**
 * @file    bmp581.c
 * @brief   Driver em C puro do barometro BMP581 (Bosch) via I3C.
 * @date    2026-08-17
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Compensacao on-chip: sensor retorna valores ja linealizados.
 * Temperatura: 24-bit signed Q16.16 [degC] -> / 65536.0
 * Pressao:     24-bit signed Q17.6  [Pa]   -> / 64.0
 */

#include <math.h>
#include <string.h>
#include "sensors/bmp581.h"

int bmp581_inicializa(bmp581_t *baro, const imuc_transport_t *transporte)
{
  if (baro == 0 || transporte == 0)
  {
    return -1;
  }
  memset(baro, 0, sizeof(*baro));
  baro->transporte = transporte;
  baro->p0_pa = 101325.0f;
  return 0;
}

int bmp581_verifica_id(bmp581_t *baro)
{
  uint8_t id = 0;

  if (baro == 0 || baro->transporte == 0)
  {
    return 0;
  }
  if (baro->transporte->ler(baro->transporte->ctx, BMP581_REG_CHIP_ID,
                            &id, 1) != 0)
  {
    return 0;
  }
  return (id == BMP581_WHO_AM_I) ? 1 : 0;
}

int bmp581_configura(bmp581_t *baro)
{
  uint8_t v;

  if (baro == 0 || baro->transporte == 0)
  {
    return -1;
  }

  /* Soft reset: escreve 0xB6 no CMD (0x7E). */
  v = BMP581_SOFT_RESET_CMD;
  if (baro->transporte->escrever(baro->transporte->ctx, BMP581_REG_CMD,
                                 &v, 1) != 0)
  {
    return -2;
  }
  /* Aguarda reset interno (tsoft_res = 2 ms). */
  {
    volatile uint32_t i;
    for (i = 0; i < 8000u; ++i)
    {
    }
  }

  /* Verifica ID apos reset. */
  if (!bmp581_verifica_id(baro))
  {
    return -3;
  }

  /* DSP_CONFIG: habilita compensacao de pressao e temperatura. */
  v = BMP581_DSP_CONFIG_VALOR;
  if (baro->transporte->escrever(baro->transporte->ctx, BMP581_REG_DSP_CONFIG,
                                 &v, 1) != 0)
  {
    return -4;
  }

  /* OSR_CONFIG: T=1x, P=8x, press_en=1. */
  v = BMP581_OSR_CONFIG_VALOR;
  if (baro->transporte->escrever(baro->transporte->ctx, BMP581_REG_OSR_CONFIG,
                                 &v, 1) != 0)
  {
    return -5;
  }

  /* INT_SOURCE: habilita data ready para IBI. */
  v = BMP581_INT_SOURCE_VALOR;
  if (baro->transporte->escrever(baro->transporte->ctx, BMP581_REG_INT_SOURCE,
                                 &v, 1) != 0)
  {
    return -6;
  }

  /* ODR_CONFIG: ODR=25 Hz, pwr_mode=NORMAL. */
  v = BMP581_ODR_CONFIG_VALOR;
  if (baro->transporte->escrever(baro->transporte->ctx, BMP581_REG_ODR_CONFIG,
                                 &v, 1) != 0)
  {
    return -7;
  }

  baro->ok = 1;
  return 0;
}

uint8_t bmp581_processa(bmp581_t *baro)
{
  uint8_t dados[6];
  int32_t t_raw, p_raw;

  if (baro == 0 || baro->transporte == 0 || !baro->ok)
  {
    return 0;
  }

  /* Burst read: 6 bytes a partir de TEMP_XLSB (0x1D). */
  if (baro->transporte->ler(baro->transporte->ctx, BMP581_REG_TEMP_XLSB,
                            dados, sizeof(dados)) != 0)
  {
    return 0;
  }

  /* Temperatura: 24-bit signed, Q16.16 [degC]. */
  {
    uint32_t t_u = ((uint32_t)dados[2] << 16) | ((uint32_t)dados[1] << 8) |
                   (uint32_t)dados[0];
    t_raw = (int32_t)t_u;
    if (t_u & 0x800000u) { t_raw |= (int32_t)0xFF000000; }
  }
  /* Pressao: 24-bit signed, Q17.6 [Pa]. */
  {
    uint32_t p_u = ((uint32_t)dados[5] << 16) | ((uint32_t)dados[4] << 8) |
                   (uint32_t)dados[3];
    p_raw = (int32_t)p_u;
    if (p_u & 0x800000u) { p_raw |= (int32_t)0xFF000000; }
  }

  baro->medida.temperatura_c = (float)t_raw / 65536.0f;
  baro->medida.pressao_pa = (float)p_raw / 64.0f;
  baro->medida.altitude_m = bmp581_altitude(baro->medida.pressao_pa,
                                            baro->p0_pa);
  baro->contador_leituras++;
  return 1;
}

float bmp581_altitude(float pressao_pa, float p0_pa)
{
  float razao = pressao_pa / p0_pa;

  if (razao <= 0.0f)
  {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(razao, 0.19029495f));
}

const baro_medida_t *bmp581_medida(const bmp581_t *baro)
{
  return (baro != 0 && baro->contador_leituras > 0u) ? &baro->medida : 0;
}
