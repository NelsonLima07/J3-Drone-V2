/**
 * @file    imuc42688.c
 * @brief   Implementacao do driver do ICM-42688-P (C puro)
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "sensors/imuc42688.h"

#include <string.h>

/* Constantes de conversao */
#define ICM_TEMP_ESCALA    132.48f  /* LSB por grau Celsius */
#define ICM_TEMP_OFFSET    25.0f    /* °C com conta zerada  */
#define ICM_GYRO_ESCALA    16.4f    /* LSB por dps (+-2000) */
#define ICM_ACCEL_ESCALA   2048.0f  /* LSB por g (+-16)     */
#define ICM_GRAVIDADE      9.80665f /* m/s^2                */
#define ICM_DPS_PARA_RADS  0.017453292519943295f /* pi/180 */

#define ICM_SOFT_RESET          0x01U
#define ICM_ATRASO_RESET_US     10000U

/* Valor de PWR_MGMT0: gyro LN (bits[3:2]=11) e accel LN (bits[1:0]=11) */
#define ICM_PWR_MGMT0_LN_6AXIS  0x0FU
/* GYRO_CONFIG0: FS +-2000 dps (bits[7:5]=0), ODR 8 kHz (bits[3:0]=3) */
#define ICM_GYRO_CONFIG0_PADRAO 0x03U
/* ACCEL_CONFIG0: FS +-16 g (bits[6:5]=0), ODR 8 kHz (bits[3:0]=3) */
#define ICM_ACCEL_CONFIG0_PADRAO 0x03U
/* INT_SOURCE0: UI_DRDY_INT1_EN (bit 7) */
#define ICM_INT_SOURCE0_DRDY_INT1 0x80U

static int16_t le_int16(const uint8_t *p)
{
  return (int16_t)(((uint16_t)p[0] << 8U) | (uint16_t)p[1]);
}

int imuc42688_inicializa(imuc42688_t *imu, const imuc_transport_t *transporte)
{
  if (imu == 0 || transporte == 0 ||
      transporte->ler == 0 || transporte->escrever == 0)
  {
    return -1;
  }

  imu->transporte = transporte;
  imu->contador_leituras = 0;
  imu->pronta = 0;
  memset(imu->registros, 0, sizeof(imu->registros));
  memset(&imu->medida, 0, sizeof(imu->medida));

  return 0;
}

int imuc42688_verifica_id(imuc42688_t *imu)
{
  uint8_t id = 0;

  if (imu == 0 || imu->transporte == 0)
  {
    return 0;
  }
  if (imu->transporte->ler(imu->transporte->ctx, ICM_WHO_AM_I, &id, 1) != 0)
  {
    return 0;
  }
  return (id == ICM_WHO_AM_I_VALOR) ? 1 : 0;
}

int imuc42688_configura(imuc42688_t *imu)
{
  uint8_t reg;

  if (imu == 0 || imu->transporte == 0)
  {
    return -1;
  }

  /* Soft reset: volta aos valores padrao e reinicia a interface */
  reg = ICM_SOFT_RESET;
  if (imu->transporte->escrever(imu->transporte->ctx, ICM_DEVICE_CONFIG, &reg, 1) != 0)
  {
    return -1;
  }
  if (imu->transporte->atraso_us != 0)
  {
    imu->transporte->atraso_us(imu->transporte->ctx, ICM_ATRASO_RESET_US);
  }

  /* Gyro e accel em modo low noise (6 eixos ativos) */
  reg = ICM_PWR_MGMT0_LN_6AXIS;
  if (imu->transporte->escrever(imu->transporte->ctx, ICM_PWR_MGMT0, &reg, 1) != 0)
  {
    return -1;
  }

  /* Gyro +-2000 dps, ODR 8 kHz */
  reg = ICM_GYRO_CONFIG0_PADRAO;
  if (imu->transporte->escrever(imu->transporte->ctx, ICM_GYRO_CONFIG0, &reg, 1) != 0)
  {
    return -1;
  }

  /* Accel +-16 g, ODR 8 kHz */
  reg = ICM_ACCEL_CONFIG0_PADRAO;
  if (imu->transporte->escrever(imu->transporte->ctx, ICM_ACCEL_CONFIG0, &reg, 1) != 0)
  {
    return -1;
  }

  /* Data ready -> pino INT1 */
  reg = ICM_INT_SOURCE0_DRDY_INT1;
  if (imu->transporte->escrever(imu->transporte->ctx, ICM_INT_SOURCE0, &reg, 1) != 0)
  {
    return -1;
  }

  return 0;
}

void imuc42688_processa_burst(imuc42688_t *imu, const uint8_t *dados)
{
  float temp_lsb;
  int16_t acel[3];
  int16_t giro[3];
  int i;

  if (imu == 0 || dados == 0)
  {
    return;
  }

  memcpy(imu->registros, dados, ICM_BURST_TAMANHO);

  /* Ordem do burst (0x1D): TEMP, ACCEL_X/Y/Z, GYRO_X/Y/Z (big endian) */
  temp_lsb = (float)le_int16(&dados[0]);
  acel[0]  = le_int16(&dados[2]);
  acel[1]  = le_int16(&dados[4]);
  acel[2]  = le_int16(&dados[6]);
  giro[0]  = le_int16(&dados[8]);
  giro[1]  = le_int16(&dados[10]);
  giro[2]  = le_int16(&dados[12]);

  imu->medida.temp_c = temp_lsb / ICM_TEMP_ESCALA + ICM_TEMP_OFFSET;

  for (i = 0; i < 3; ++i)
  {
    imu->medida.aceleracao_m_s2[i] = (float)acel[i] / ICM_ACCEL_ESCALA * ICM_GRAVIDADE;
    imu->medida.giro_rad_s[i] = (float)giro[i] / ICM_GYRO_ESCALA * ICM_DPS_PARA_RADS;
  }

  imu->pronta = 1;
  ++imu->contador_leituras;
}

const imu_medida_t *imuc42688_medida(const imuc42688_t *imu)
{
  if (imu == 0 || imu->pronta == 0)
  {
    return 0;
  }
  return &imu->medida;
}
