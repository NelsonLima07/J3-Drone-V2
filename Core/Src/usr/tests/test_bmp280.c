/**
 * @file    test_bmp280.c
 * @brief   Testes nativos do driver do barometro BMP280.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Usa o exemplo de calculo do datasheet da Bosch:
 *   dig_T1=27504, T2=26435, T3=-1000
 *   dig_P1=36477, P2=-10685, P3=3024, P4=2855, P5=140, P6=-7,
 *   P7=15500, P8=-14600, P9=6000
 *   adc_T=519888 -> 25,08 C ; adc_P=415148 -> 100656,26 Pa
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sensors/bmp280.h"

static int falhas = 0;

#define VERIFICA(cond, nome)                                        \
  do {                                                              \
    if (cond) {                                                     \
      printf("  ok: %s\n", nome);                                   \
    } else {                                                        \
      printf("  FALHOU: %s (linha %d)\n", nome, __LINE__);          \
      ++falhas;                                                     \
    }                                                               \
  } while (0)

#define APROX(a, b, tol) (fabsf((a) - (b)) <= (tol))

/* --- Transporte mock: memoria de 256 bytes ----------------------------- */
static uint8_t mock_mem[256];

static int mock_ler(void *ctx, uint8_t reg, uint8_t *dados, uint16_t n)
{
  (void)ctx;
  memcpy(dados, &mock_mem[reg], n);
  return 0;
}

static int mock_escrever(void *ctx, uint8_t reg, const uint8_t *dados,
                         uint16_t n)
{
  int i;
  (void)ctx;
  for (i = 0; i < (int)n; ++i)
  {
    mock_mem[reg + (uint8_t)i] = dados[i];
  }
  return 0;
}

static void mock_atraso(void *ctx, uint32_t us)
{
  (void)ctx;
  (void)us;
}

static void calib_datasheet(void)
{
  static const uint8_t cal[24] = {
    0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC, /* T1, T2, T3 */
    0x7D, 0x8E, 0x43, 0xD6, 0xD0, 0x0B, /* P1, P2, P3 */
    0x27, 0x0B, 0x8C, 0x00, 0xF9, 0xFF, /* P4, P5, P6 */
    0x8C, 0x3C, 0xF8, 0xC6, 0x70, 0x17  /* P7, P8, P9 */
  };
  memcpy(&mock_mem[BMP280_REG_CALIB], cal, sizeof(cal));
}

int testes_bmp280(void)
{
  imuc_transport_t t;
  bmp280_t baro;
  const baro_medida_t *m;
  float temperatura, pressao;
  int16_t dig[BMP280_DIG_TAMANHO];
  float alt;

  memset(&t, 0, sizeof(t));
  t.ctx = 0;
  t.ler = mock_ler;
  t.escrever = mock_escrever;
  t.atraso_us = mock_atraso;

  memset(mock_mem, 0, sizeof(mock_mem));
  mock_mem[BMP280_REG_ID] = BMP280_WHO_AM_I;

  VERIFICA(bmp280_inicializa(&baro, &t) == 0, "inicializa vincula transporte");
  VERIFICA(bmp280_verifica_id(&baro) == 1, "ID = 0x58 reconhecido");
  mock_mem[BMP280_REG_ID] = 0x00;
  VERIFICA(bmp280_verifica_id(&baro) == 0, "ID errado rejeitado");
  mock_mem[BMP280_REG_ID] = BMP280_WHO_AM_I;

  calib_datasheet();
  VERIFICA(bmp280_configura(&baro) == 0, "configura ok (soft reset + calib)");
  VERIFICA(baro.ok == 1u, "driver marcado ok apos configurar");
  VERIFICA(mock_mem[BMP280_REG_RESET] == 0xB6u, "soft reset 0xB6 escrito");
  VERIFICA(mock_mem[BMP280_REG_CTRL_MEAS] == 0x2Fu, "CTRL_MEAS modo normal");
  VERIFICA(mock_mem[BMP280_REG_CONFIG] == 0x08u, "CONFIG filtro 4x");

  /* Compensacao pura com o exemplo do datasheet. */
  dig[0] = 27504;
  dig[1] = 26435;
  dig[2] = -1000;
  dig[3] = 36477;   /* P1: bit padrao preservado via int16 */
  dig[4] = -10685;
  dig[5] = 3024;
  dig[6] = 2855;
  dig[7] = 140;
  dig[8] = -7;
  dig[9] = 15500;
  dig[10] = -14600;
  dig[11] = 6000;
  bmp280_compensa(dig, 519888, 415148, &temperatura, &pressao);
  VERIFICA(APROX(temperatura, 25.08f, 0.3f),
           "exemplo datasheet -> 25,08 C");
  VERIFICA(APROX(pressao, 100656.26f, 20.0f),
           "exemplo datasheet -> 100656 Pa");

  /* Formula barometrica. */
  VERIFICA(APROX(bmp280_altitude(101325.0f, 101325.0f), 0.0f, 0.1f),
           "ao nivel do mar -> 0 m");
  alt = bmp280_altitude(90000.0f, 101325.0f);
  VERIFICA(APROX(alt, 988.5f, 15.0f), "90 kPa -> ~988 m");

  /* Processa via barramento mock: raw de 20 bits em PRESS_MSB..TEMP_MSB. */
  mock_mem[BMP280_REG_PRESS_MSB + 0] = 0x65; /* p_raw = 415148 */
  mock_mem[BMP280_REG_PRESS_MSB + 1] = 0x5A;
  mock_mem[BMP280_REG_PRESS_MSB + 2] = 0xC0;
  mock_mem[BMP280_REG_TEMP_MSB + 0] = 0x7E;  /* t_raw = 519888 (0x7EED0) */
  mock_mem[BMP280_REG_TEMP_MSB + 1] = 0xED;
  mock_mem[BMP280_REG_TEMP_MSB + 2] = 0x00;
  VERIFICA(bmp280_processa(&baro) == 1, "processa produz amostra");
  m = bmp280_medida(&baro);
  VERIFICA(m != 0, "medida disponivel");
  if (m != 0)
  {
    VERIFICA(APROX(m->temperatura_c, 25.08f, 0.3f), "T compensada ~25 C");
    VERIFICA(APROX(m->pressao_pa, 100656.26f, 20.0f),
             "P compensada ~100656 Pa");
    /* Com P0 igual a pressao medida, altitude ~0. */
    baro.p0_pa = m->pressao_pa;
    VERIFICA(bmp280_processa(&baro) == 1, "processa segunda amostra");
    m = bmp280_medida(&baro);
    VERIFICA(APROX(m->altitude_m, 0.0f, 2.0f),
             "altitude ~0 com P0 = pressao medida");
  }

  printf("testes_bmp280: %d falha(s)\n", falhas);
  return falhas;
}
