/**
 * @file    test_sensors.c
 * @brief   Testes nativos do driver do ICM-42688-P (transporte mock)
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "sensors/imuc42688.h"

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

/* --- Transporte mock ---------------------------------------------------- */

static int mock_falha;
static uint8_t mock_id;

static uint8_t mock_enderecos[16];
static uint8_t mock_valores[16];
static int mock_n_escritas;

static int mock_ler(void *ctx, uint8_t endereco, uint8_t *dados, uint16_t n)
{
  (void)ctx;
  if (mock_falha)
  {
    return -1;
  }
  if (endereco == ICM_WHO_AM_I && n == 1)
  {
    dados[0] = mock_id;
    return 0;
  }
  return -1;
}

static int mock_escrever(void *ctx, uint8_t endereco, const uint8_t *dados, uint16_t n)
{
  int i;
  (void)ctx;
  if (mock_falha)
  {
    return -1;
  }
  for (i = 0; i < n; ++i)
  {
    if (mock_n_escritas < (int)(sizeof(mock_enderecos)))
    {
      mock_enderecos[mock_n_escritas] = endereco;
      mock_valores[mock_n_escritas] = dados[i];
      ++mock_n_escritas;
    }
  }
  return 0;
}

static void mock_atraso(void *ctx, uint32_t us)
{
  (void)ctx;
  (void)us;
}

static imuc_transport_t mock_transporte;

static void mock_reseta(void)
{
  mock_falha = 0;
  mock_id = ICM_WHO_AM_I_VALOR;
  mock_n_escritas = 0;
  mock_transporte.ctx = 0;
  mock_transporte.ler = mock_ler;
  mock_transporte.escrever = mock_escrever;
  mock_transporte.atraso_us = mock_atraso;
}

/* --- Testes -------------------------------------------------------------- */

static void teste_inicializa(void)
{
  imuc42688_t imu;

  printf("  inicializa\n");
  VERIFICA(imuc42688_inicializa(0, &mock_transporte) < 0, "NULL imu");
  VERIFICA(imuc42688_inicializa(&imu, 0) < 0, "NULL transporte");
  VERIFICA(imuc42688_inicializa(&imu, &mock_transporte) == 0, "transporte valido");
  VERIFICA(imu.contador_leituras == 0, "contador zerado");
  VERIFICA(imu.pronta == 0, "pronta zerada");
}

static void teste_verifica_id(void)
{
  imuc42688_t imu;

  printf("  verifica_id\n");
  imuc42688_inicializa(&imu, &mock_transporte);

  mock_id = ICM_WHO_AM_I_VALOR;
  VERIFICA(imuc42688_verifica_id(&imu) == 1, "ID correto reconhecido");

  mock_id = 0x00;
  VERIFICA(imuc42688_verifica_id(&imu) == 0, "ID errado rejeitado");

  mock_falha = 1;
  VERIFICA(imuc42688_verifica_id(&imu) == 0, "falha de comunicacao rejeitada");
}

static void teste_configura(void)
{
  imuc42688_t imu;
  int i;

  printf("  configura\n");
  mock_reseta();
  imuc42688_inicializa(&imu, &mock_transporte);

  VERIFICA(imuc42688_configura(&imu) == 0, "configuracao OK");

  VERIFICA(mock_n_escritas == 5, "5 escritas esperadas");
  VERIFICA(mock_enderecos[0] == ICM_DEVICE_CONFIG && mock_valores[0] == 0x01,
           "soft reset em DEVICE_CONFIG");
  VERIFICA(mock_enderecos[1] == ICM_PWR_MGMT0 && mock_valores[1] == 0x0F,
           "PWR_MGMT0 low noise 6 eixos");
  VERIFICA(mock_enderecos[2] == ICM_GYRO_CONFIG0 && mock_valores[2] == 0x03,
           "GYRO_CONFIG0 +-2000dps 8kHz");
  VERIFICA(mock_enderecos[3] == ICM_ACCEL_CONFIG0 && mock_valores[3] == 0x03,
           "ACCEL_CONFIG0 +-16g 8kHz");
  VERIFICA(mock_enderecos[4] == ICM_INT_SOURCE0 && mock_valores[4] == 0x80,
           "INT_SOURCE0 DRDY->INT1");

  /* Falha de escrita na 2a operacao deve abortar */
  mock_reseta();
  mock_falha = 1;
  imuc42688_inicializa(&imu, &mock_transporte);
  VERIFICA(imuc42688_configura(&imu) < 0, "falha de escrita detectada");

  /* Ordem de escritas nao deve repetir endereco (burst de 1 byte cada) */
  mock_reseta();
  imuc42688_inicializa(&imu, &mock_transporte);
  imuc42688_configura(&imu);
  for (i = 1; i < mock_n_escritas; ++i)
  {
    VERIFICA(mock_enderecos[i] != mock_enderecos[i - 1], "enderecos distintos");
  }
}

static void teste_processa_burst(void)
{
  imuc42688_t imu;
  const imu_medida_t *m;
  static const uint8_t burst[ICM_BURST_TAMANHO] = {
    0x05, 0x2D, /* TEMP = 1325 -> 35.0 C            */
    0x08, 0x00, /* ACCEL_X = 2048  -> +1 g          */
    0xF8, 0x00, /* ACCEL_Y = -2048 -> -1 g          */
    0x00, 0x00, /* ACCEL_Z = 0     -> 0 g           */
    0x00, 0xA4, /* GYRO_X = 164    -> +10 dps        */
    0xFF, 0x5C, /* GYRO_Y = -164   -> -10 dps        */
    0x00, 0x00  /* GYRO_Z = 0      -> 0 dps          */
  };

  printf("  processa_burst\n");
  imuc42688_inicializa(&imu, &mock_transporte);

  VERIFICA(imuc42688_medida(&imu) == 0, "sem medida antes do burst");

  imuc42688_processa_burst(&imu, burst);
  m = imuc42688_medida(&imu);

  VERIFICA(m != 0, "medida disponivel apos burst");
  VERIFICA(imu.contador_leituras == 1, "contador incrementado");
  VERIFICA(APROX(m->temp_c, 35.0f, 0.01f), "temperatura decodificada");
  VERIFICA(APROX(m->aceleracao_m_s2[0], 9.80665f, 0.001f), "accel X +1 g");
  VERIFICA(APROX(m->aceleracao_m_s2[1], -9.80665f, 0.001f), "accel Y -1 g");
  VERIFICA(APROX(m->aceleracao_m_s2[2], 0.0f, 0.001f), "accel Z 0 g");
  VERIFICA(APROX(m->giro_rad_s[0], 10.0f * 3.14159265f / 180.0f, 0.001f), "gyro X +10 dps");
  VERIFICA(APROX(m->giro_rad_s[1], -10.0f * 3.14159265f / 180.0f, 0.001f), "gyro Y -10 dps");
  VERIFICA(APROX(m->giro_rad_s[2], 0.0f, 0.001f), "gyro Z 0 dps");

  imuc42688_processa_burst(&imu, burst);
  VERIFICA(imu.contador_leituras == 2, "contador acumula amostras");
}

int testes_sensors(void)
{
  printf("=== Testes de sensores (ICM-42688-P) ===\n");

  mock_reseta();
  teste_inicializa();
  teste_verifica_id();
  teste_configura();
  teste_processa_burst();

  return falhas;
}
