/**
 * @file    test_calibracao.c
 * @brief   Testes nativos da calibracao do IMU
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>
#include "system/calibracao.h"

static float quase_igual(float a, float b, float tol)
{
  return fabsf(a - b) <= tol;
}

int testes_calibracao(void)
{
  int falhas = 0;
  calibracao_t cal;
  imu_medida_t m;
  imu_medida_t saida;
  uint32_t i;

  /* Processar sem estar ativa nao acumula (retorna < 0). */
  calibracao_inicializa(&cal);
  m.temp_c = 25.0f;
  m.aceleracao_m_s2[0] = 0.0f;
  m.aceleracao_m_s2[1] = 0.0f;
  m.aceleracao_m_s2[2] = 9.81f;
  m.giro_rad_s[0] = 0.1f;
  m.giro_rad_s[1] = -0.2f;
  m.giro_rad_s[2] = 0.05f;
  if (calibracao_processa(&cal, &m) >= 0)
  {
    printf("FALHA: processa sem estar ativa\n");
    falhas++;
  }

  /* Coleta de 10 amostras: bias = media do gyro, ref = media do accel. */
  calibracao_inicia(&cal, 10);
  for (i = 0; i < 10; ++i)
  {
    m.giro_rad_s[0] = 0.1f + (float)i * 0.01f;
    m.giro_rad_s[1] = -0.2f;
    m.giro_rad_s[2] = 0.05f;
    m.aceleracao_m_s2[0] = 0.0f;
    m.aceleracao_m_s2[1] = 0.0f;
    m.aceleracao_m_s2[2] = 9.81f;
    (void)calibracao_processa(&cal, &m);
  }

  if (!calibracao_concluida(&cal))
  {
    printf("FALHA: coleta nao concluiu ao atingir a meta\n");
    falhas++;
  }
  if (calibracao_ativa(&cal))
  {
    printf("FALHA: calibracao ainda ativa apos concluir\n");
    falhas++;
  }

  /* giro_rad_s[0]: media de 0.1..0.19 = 0.145 */
  if (!quase_igual(cal.giro_bias[0], 0.145f, 1e-4f))
  {
    printf("FALHA: bias X = %.4f, esperado 0.145\n", cal.giro_bias[0]);
    falhas++;
  }
  if (!quase_igual(cal.giro_bias[1], -0.2f, 1e-4f))
  {
    printf("FALHA: bias Y = %.4f, esperado -0.2\n", cal.giro_bias[1]);
    falhas++;
  }
  if (!quase_igual(cal.giro_bias[2], 0.05f, 1e-4f))
  {
    printf("FALHA: bias Z = %.4f, esperado 0.05\n", cal.giro_bias[2]);
    falhas++;
  }
  if (!quase_igual(cal.accel_ref[2], 9.81f, 1e-4f))
  {
    printf("FALHA: ref Z = %.4f, esperado 9.81\n", cal.accel_ref[2]);
    falhas++;
  }

  /* Aplicar o bias remove a media do gyro e preserva o accel. */
  m.giro_rad_s[0] = 0.645f;
  m.giro_rad_s[1] = 0.0f;
  m.giro_rad_s[2] = 0.15f;
  m.aceleracao_m_s2[2] = 9.81f;
  calibracao_aplica_bias(&cal, &m, &saida);
  if (!quase_igual(saida.giro_rad_s[0], 0.5f, 1e-4f))
  {
    printf("FALHA: giro corrigido X = %.4f, esperado 0.5\n", saida.giro_rad_s[0]);
    falhas++;
  }
  if (!quase_igual(saida.giro_rad_s[1], 0.2f, 1e-4f))
  {
    printf("FALHA: giro corrigido Y = %.4f, esperado 0.2\n", saida.giro_rad_s[1]);
    falhas++;
  }
  if (!quase_igual(saida.giro_rad_s[2], 0.1f, 1e-4f))
  {
    printf("FALHA: giro corrigido Z = %.4f, esperado 0.1\n", saida.giro_rad_s[2]);
    falhas++;
  }
  if (!quase_igual(saida.aceleracao_m_s2[2], 9.81f, 1e-4f))
  {
    printf("FALHA: accel Z alterado indevidamente\n");
    falhas++;
  }

  /* Finalizar sem coleta ativa retorna erro. */
  if (calibracao_finaliza(&cal) >= 0)
  {
    printf("FALHA: finalizou sem coleta ativa\n");
    falhas++;
  }

  printf("testes_calibracao: %d falha(s)\n", falhas);
  return falhas;
}
