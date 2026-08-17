/**
 * @file    test_lis3mdl.c
 * @brief   Testes nativos do driver do magnetometro LIS3MDL.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "sensors/lis3mdl.h"

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
static uint8_t mock_registra[8][3];
static int mock_nwrites;

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
    if (mock_nwrites < 8)
    {
      mock_registra[mock_nwrites][0] = reg + (uint8_t)i;
      mock_registra[mock_nwrites][1] = dados[i];
      mock_nwrites++;
    }
  }
  return 0;
}

static void mock_atraso(void *ctx, uint32_t us)
{
  (void)ctx;
  (void)us;
}

int testes_lis3mdl(void)
{
  imuc_transport_t t;
  lis3mdl_t mag;
  const mag_medida_t *m;

  memset(&t, 0, sizeof(t));
  t.ctx = 0;
  t.ler = mock_ler;
  t.escrever = mock_escrever;
  t.atraso_us = mock_atraso;

  memset(mock_mem, 0, sizeof(mock_mem));
  mock_nwrites = 0;
  mock_mem[LIS3MDL_REG_WHO_AM_I] = LIS3MDL_WHO_AM_I;

  VERIFICA(lis3mdl_inicializa(&mag, &t) == 0, "inicializa vincula transporte");
  VERIFICA(lis3mdl_verifica_id(&mag) == 1, "WHO_AM_I = 0x3D reconhecido");
  mock_mem[LIS3MDL_REG_WHO_AM_I] = 0x00;
  VERIFICA(lis3mdl_verifica_id(&mag) == 0, "WHO_AM_I errado rejeitado");
  mock_mem[LIS3MDL_REG_WHO_AM_I] = LIS3MDL_WHO_AM_I;

  VERIFICA(lis3mdl_configura(&mag) == 0, "configura ok");
  VERIFICA(mag.ok == 1u, "driver marcado ok apos configurar");

  /* Registradores de configuracao escritos com os valores fixos. */
  {
    static const uint8_t esperado[5] = {0x10, 0x00, 0x00, 0x80, 0x00};
    int i;
    for (i = 0; i < 5; ++i)
    {
      if (mock_registra[i][0] != (uint8_t)(LIS3MDL_REG_CTRL1 + i) ||
          mock_registra[i][1] != esperado[i])
      {
        VERIFICA(0, "CTRL1..5 escritos corretamente");
      }
    }
  }
  VERIFICA(mock_mem[LIS3MDL_REG_CTRL4] == 0x80u, "BDU ativo (CTRL4)");

  /* Sem dados novos (ZYXDA limpo) -> processa nao produz medida. */
  mock_mem[LIS3MDL_REG_STATUS] = 0x00;
  VERIFICA(lis3mdl_processa(&mag) == 0, "sem ZYXDA -> sem amostra");

  /* X = +6842 LSB -> +1 gauss; Y = -6842 -> -1 gauss; Z = 0. */
  mock_mem[LIS3MDL_REG_STATUS] = 0x08;
  mock_mem[LIS3MDL_REG_OUT_X_L + 0] = 0xBA; /* 6842 = 0x1ABA */
  mock_mem[LIS3MDL_REG_OUT_X_L + 1] = 0x1A;
  mock_mem[LIS3MDL_REG_OUT_X_L + 2] = 0x46; /* -6842 = 0xE546 */
  mock_mem[LIS3MDL_REG_OUT_X_L + 3] = 0xE5;
  mock_mem[LIS3MDL_REG_OUT_X_L + 4] = 0x00;
  mock_mem[LIS3MDL_REG_OUT_X_L + 5] = 0x00;

  VERIFICA(lis3mdl_processa(&mag) == 1, "amostra nova produzida");
  m = lis3mdl_medida(&mag);
  VERIFICA(m != 0, "medida disponivel");
  if (m != 0)
  {
    VERIFICA(APROX(m->x, 1.0f, 0.01f), "x = +1 gauss");
    VERIFICA(APROX(m->y, -1.0f, 0.01f), "y = -1 gauss");
    VERIFICA(APROX(m->z, 0.0f, 0.01f), "z = 0 gauss");
  }

  printf("testes_lis3mdl: %d falha(s)\n", falhas);
  return falhas;
}
