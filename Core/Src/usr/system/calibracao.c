/**
 * @file    calibracao.c
 * @brief   Calibracao do IMU em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "system/calibracao.h"

void calibracao_inicializa(calibracao_t *cal)
{
  uint8_t i;

  if (cal == 0)
  {
    return;
  }
  for (i = 0; i < 3; ++i)
  {
    cal->giro_soma[i] = 0.0;
    cal->accel_soma[i] = 0.0;
    cal->giro_bias[i] = 0.0f;
    cal->accel_ref[i] = 0.0f;
  }
  cal->n = 0;
  cal->meta = 0;
  cal->ativa = 0;
  cal->concluida = 0;
}

void calibracao_inicia(calibracao_t *cal, uint32_t meta_amostras)
{
  uint8_t i;

  if (cal == 0)
  {
    return;
  }
  for (i = 0; i < 3; ++i)
  {
    cal->giro_soma[i] = 0.0;
    cal->accel_soma[i] = 0.0;
  }
  cal->n = 0;
  cal->meta = meta_amostras;
  cal->ativa = 1;
  cal->concluida = 0;
}

int calibracao_processa(calibracao_t *cal, const imu_medida_t *m)
{
  uint8_t i;

  if (cal == 0 || m == 0 || !cal->ativa)
  {
    return -1;
  }

  for (i = 0; i < 3; ++i)
  {
    cal->giro_soma[i] += m->giro_rad_s[i];
    cal->accel_soma[i] += m->aceleracao_m_s2[i];
  }
  cal->n++;

  if (cal->meta > 0 && cal->n >= cal->meta)
  {
    return (calibracao_finaliza(cal) == 0) ? 1 : -2;
  }
  return 0;
}

int calibracao_finaliza(calibracao_t *cal)
{
  uint8_t i;

  if (cal == 0 || !cal->ativa || cal->n == 0)
  {
    return -1;
  }

  for (i = 0; i < 3; ++i)
  {
    cal->giro_bias[i] = (float)(cal->giro_soma[i] / (double)cal->n);
    cal->accel_ref[i] = (float)(cal->accel_soma[i] / (double)cal->n);
  }
  cal->ativa = 0;
  cal->concluida = 1;
  return 0;
}

uint8_t calibracao_ativa(const calibracao_t *cal)
{
  return (cal != 0) ? cal->ativa : 0;
}

uint8_t calibracao_concluida(const calibracao_t *cal)
{
  return (cal != 0) ? cal->concluida : 0;
}

void calibracao_aplica_bias(const calibracao_t *cal,
                            const imu_medida_t *entrada,
                            imu_medida_t *saida)
{
  uint8_t i;

  if (cal == 0 || entrada == 0 || saida == 0)
  {
    return;
  }

  *saida = *entrada;
  if (!cal->concluida)
  {
    return;
  }

  for (i = 0; i < 3; ++i)
  {
    saida->giro_rad_s[i] -= cal->giro_bias[i];
  }
}
