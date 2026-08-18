/**
 * @file    test_bmp581.c
 * @brief   Testes nativos do driver do barometro BMP581 (I3C, on-chip compensation).
 * @date    2026-08-17
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Compensacao on-chip: valores ja linearizados no sensor.
 * Temperatura: 24-bit signed Q16.16 [degC] -> / 65536.0
 * Pressao:     24-bit signed Q17.6  [Pa]   -> / 64.0
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sensors/bmp581.h"

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

/**
 * @brief  Prepara dados de 6 bytes em mock_mem[0x1D..0x22] para
 *         temperatura e pressao conhecidas.
 *
 * temp = 25.0 C  -> raw = 25.0 * 65536 = 1638400 = 0x190000
 *   XLSB=0x00, LSB=0x00, MSB=0x19
 * press = 101325.0 Pa -> raw = 101325 * 64 = 6484800 = 0x62F340
 *   XLSB=0x40, LSB=0xF3, MSB=0x62
 */
static void dados_25c_101325pa(void)
{
  mock_mem[BMP581_REG_TEMP_XLSB + 0] = 0x00; /* temp XLSB */
  mock_mem[BMP581_REG_TEMP_XLSB + 1] = 0x00; /* temp LSB  */
  mock_mem[BMP581_REG_TEMP_XLSB + 2] = 0x19; /* temp MSB  */
  mock_mem[BMP581_REG_TEMP_XLSB + 3] = 0x40; /* press XLSB */
  mock_mem[BMP581_REG_TEMP_XLSB + 4] = 0xF3; /* press LSB  */
  mock_mem[BMP581_REG_TEMP_XLSB + 5] = 0x62; /* press MSB  */
}

int testes_bmp581(void)
{
  imuc_transport_t t;
  bmp581_t baro;
  const baro_medida_t *m;
  float alt;

  memset(&t, 0, sizeof(t));
  t.ctx = 0;
  t.ler = mock_ler;
  t.escrever = mock_escrever;
  t.atraso_us = mock_atraso;

  memset(mock_mem, 0, sizeof(mock_mem));
  mock_mem[BMP581_REG_CHIP_ID] = BMP581_WHO_AM_I;

  VERIFICA(bmp581_inicializa(&baro, &t) == 0, "inicializa vincula transporte");
  VERIFICA(bmp581_verifica_id(&baro) == 1, "ID = 0x50 reconhecido");
  mock_mem[BMP581_REG_CHIP_ID] = 0x00;
  VERIFICA(bmp581_verifica_id(&baro) == 0, "ID errado rejeitado");
  mock_mem[BMP581_REG_CHIP_ID] = BMP581_WHO_AM_I;

  /* configura escreve todos os registradores. */
  VERIFICA(bmp581_configura(&baro) == 0, "configura ok (soft reset + cfg)");
  VERIFICA(baro.ok == 1u, "driver marcado ok apos configurar");
  VERIFICA(mock_mem[BMP581_REG_CMD] == 0xB6u, "soft reset 0xB6 escrito");
  VERIFICA(mock_mem[BMP581_REG_DSP_CONFIG] == 0x03u, "DSP_CONFIG compensacao P+T");
  VERIFICA(mock_mem[BMP581_REG_OSR_CONFIG] == 0x0Fu, "OSR_CONFIG T=1x P=8x");
  VERIFICA(mock_mem[BMP581_REG_INT_SOURCE] == 0x01u, "INT_SOURCE drdy habilitado");
  VERIFICA(mock_mem[BMP581_REG_ODR_CONFIG] == 0x2Du, "ODR_CONFIG 25Hz NORMAL");

  /* Formula barometrica. */
  VERIFICA(APROX(bmp581_altitude(101325.0f, 101325.0f), 0.0f, 0.1f),
           "ao nivel do mar -> 0 m");
  alt = bmp581_altitude(90000.0f, 101325.0f);
  VERIFICA(APROX(alt, 988.5f, 15.0f), "90 kPa -> ~988 m");

  /* Processa via barramento mock: dados compensados on-chip. */
  dados_25c_101325pa();
  VERIFICA(bmp581_processa(&baro) == 1, "processa produz amostra");
  m = bmp581_medida(&baro);
  VERIFICA(m != 0, "medida disponivel");
  if (m != 0)
  {
    VERIFICA(APROX(m->temperatura_c, 25.0f, 0.01f), "T = 25.0 C");
    VERIFICA(APROX(m->pressao_pa, 101325.0f, 1.0f), "P = 101325 Pa");
    /* Com P0 igual a pressao medida, altitude ~0. */
    baro.p0_pa = m->pressao_pa;
    VERIFICA(bmp581_processa(&baro) == 1, "processa segunda amostra");
    m = bmp581_medida(&baro);
    VERIFICA(APROX(m->altitude_m, 0.0f, 0.1f),
             "altitude ~0 com P0 = pressao medida");
  }

  /* Teste com valor negativo: temp = -10.0 C -> raw = -655360 = 0xFFF60000
   * signed 24-bit: 0xF60000 = -655360 */
  mock_mem[BMP581_REG_TEMP_XLSB + 0] = 0x00;
  mock_mem[BMP581_REG_TEMP_XLSB + 1] = 0x00;
  mock_mem[BMP581_REG_TEMP_XLSB + 2] = 0xF6;
  VERIFICA(bmp581_processa(&baro) == 1, "processa temp negativa");
  m = bmp581_medida(&baro);
  if (m != 0)
  {
    VERIFICA(APROX(m->temperatura_c, -10.0f, 0.01f), "T = -10.0 C");
  }

  printf("testes_bmp581: %d falha(s)\n", falhas);
  return falhas;
}
