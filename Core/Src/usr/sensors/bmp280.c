/**
 * @file    bmp280.c
 * @brief   Driver em C puro do barometro BMP280 (Bosch).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <math.h>
#include <string.h>
#include "sensors/bmp280.h"

/* CTRL_MEAS: osrs_t=1x (001<<5), osrs_p=4x (011<<2), normal (11) */
#define BMP280_CTRL_MEAS_VALOR 0x2FU
/* CONFIG: standby 0.5ms, filtro IIR 4x (010<<2) -> 0x08 */
#define BMP280_CONFIG_VALOR    0x08U

int bmp280_inicializa(bmp280_t *baro, const imuc_transport_t *transporte)
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

int bmp280_verifica_id(bmp280_t *baro)
{
  uint8_t id = 0;

  if (baro == 0 || baro->transporte == 0)
  {
    return 0;
  }
  if (baro->transporte->ler(baro->transporte->ctx, BMP280_REG_ID,
                            &id, 1) != 0)
  {
    return 0;
  }
  return (id == BMP280_WHO_AM_I) ? 1 : 0;
}

static int bmp280_le_calibracao(bmp280_t *baro)
{
  uint8_t buf[24];
  uint8_t i;

  if (baro->transporte->ler(baro->transporte->ctx, BMP280_REG_CALIB,
                            buf, sizeof(buf)) != 0)
  {
    return -1;
  }
  baro->dig[0] = (int16_t)(buf[0] | (uint16_t)(buf[1] << 8));
  for (i = 1; i < 3u; ++i)
  {
    baro->dig[i] = (int16_t)(buf[i * 2] | (uint16_t)(buf[i * 2 + 1] << 8));
  }
  baro->dig[3] = (int16_t)(buf[6] | (uint16_t)(buf[7] << 8));
  for (i = 4; i < BMP280_DIG_TAMANHO; ++i)
  {
    baro->dig[i] = (int16_t)(buf[i * 2] | (uint16_t)(buf[i * 2 + 1] << 8));
  }
  return 0;
}

int bmp280_configura(bmp280_t *baro)
{
  uint8_t reset = 0xB6U; /* soft reset do datasheet */

  if (baro == 0 || baro->transporte == 0)
  {
    return -1;
  }
  if (baro->transporte->escrever(baro->transporte->ctx, BMP280_REG_RESET,
                                 &reset, 1) != 0)
  {
    return -2;
  }
  /* aguarda o reset interno terminar */
  {
    volatile uint32_t i;
    for (i = 0; i < 2000u; ++i)
    {
    }
  }
  if (bmp280_le_calibracao(baro) != 0)
  {
    return -3;
  }
  {
    uint8_t v = BMP280_CTRL_MEAS_VALOR;
    if (baro->transporte->escrever(baro->transporte->ctx,
                                   BMP280_REG_CTRL_MEAS, &v, 1) != 0)
    {
      return -4;
    }
  }
  {
    uint8_t v = BMP280_CONFIG_VALOR;
    if (baro->transporte->escrever(baro->transporte->ctx,
                                   BMP280_REG_CONFIG, &v, 1) != 0)
    {
      return -5;
    }
  }
  baro->ok = 1;
  return 0;
}

void bmp280_compensa(const int16_t *dig, int32_t t_raw, int32_t p_raw,
                     float *temperatura_c, float *pressao_pa)
{
  float var1, var2;
  float t, p;
  int32_t t_fine;

  /* dig[0] = T1 (u16), dig[1] = T2, dig[2] = T3 */
  var1 = (((float)t_raw) / 16384.0f - (float)(uint16_t)dig[0] / 1024.0f) *
         ((float)dig[1]);
  var2 = ((((float)t_raw) / 131072.0f - (float)(uint16_t)dig[0] / 8192.0f) *
          (((float)t_raw) / 131072.0f - (float)(uint16_t)dig[0] / 8192.0f)) *
         ((float)dig[2]);
  t_fine = (int32_t)(var1 + var2);
  t = (var1 + var2) / 5120.0f;

  /* dig[3] = P1 (u16), dig[4] = P2, dig[5] = P3, dig[6] = P4,
   * dig[7] = P5, dig[8] = P6, dig[9] = P7, dig[10] = P8, dig[11] = P9 */
  var1 = ((float)t_fine) * 0.5f - 64000.0f;
  var2 = var1 * var1 * ((float)dig[8]) / 32768.0f;
  var2 = var2 + var1 * ((float)dig[7]) * 2.0f;
  var2 = (var2 / 4.0f) + (((float)dig[6]) * 65536.0f);
  var1 = (((float)dig[5]) * var1 * var1 / 524288.0f +
          ((float)dig[4]) * var1) / 524288.0f;
  var1 = (1.0f + var1 / 32768.0f) * (float)(uint16_t)dig[3];
  p = 1048576.0f - (float)p_raw;
  p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
  var1 = ((float)dig[11]) * p * p / 2147483648.0f;
  var2 = p * ((float)dig[10]) / 32768.0f;
  p = p + (var1 + var2 + ((float)dig[9])) / 16.0f;

  if (temperatura_c != 0)
  {
    *temperatura_c = t;
  }
  if (pressao_pa != 0)
  {
    *pressao_pa = p;
  }
}

float bmp280_altitude(float pressao_pa, float p0_pa)
{
  float razao = pressao_pa / p0_pa;

  if (razao <= 0.0f)
  {
    return 0.0f;
  }
  return 44330.0f * (1.0f - powf(razao, 0.19029495f));
}

uint8_t bmp280_processa(bmp280_t *baro)
{
  uint8_t dados[6];
  int32_t p_raw, t_raw;

  if (baro == 0 || baro->transporte == 0 || !baro->ok)
  {
    return 0;
  }
  if (baro->transporte->ler(baro->transporte->ctx, BMP280_REG_PRESS_MSB,
                            dados, sizeof(dados)) != 0)
  {
    return 0;
  }
  p_raw = ((int32_t)dados[0] << 12) | ((int32_t)dados[1] << 4) |
          ((int32_t)dados[2] >> 4);
  t_raw = ((int32_t)dados[3] << 12) | ((int32_t)dados[4] << 4) |
          ((int32_t)dados[5] >> 4);

  bmp280_compensa(baro->dig, t_raw, p_raw, &baro->medida.temperatura_c,
                  &baro->medida.pressao_pa);
  baro->medida.altitude_m = bmp280_altitude(baro->medida.pressao_pa,
                                            baro->p0_pa);
  baro->contador_leituras++;
  return 1;
}

const baro_medida_t *bmp280_medida(const bmp280_t *baro)
{
  return (baro != 0 && baro->contador_leituras > 0u) ? &baro->medida : 0;
}
