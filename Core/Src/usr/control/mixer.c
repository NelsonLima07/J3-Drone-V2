/**
 * @file    mixer.c
 * @brief   Mixagem (alocação) de torque+coletivo para motores em frame X
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "control/mixer.h"
#include "math/math_utils.h"

void misturador_aplica(float thrust, const vetor3_t *torque, saida_misturador_t *saida)
{
  float rol = torque->x;
  float arf = torque->y;
  float guinada = torque->z;

  saida->motores[0] = thrust + rol + arf - guinada; /* motor 1: dianteiro direito */
  saida->motores[1] = thrust + rol - arf + guinada; /* motor 2: traseiro esquerdo */
  saida->motores[2] = thrust - rol + arf + guinada; /* motor 3: dianteiro esquerdo */
  saida->motores[3] = thrust - rol - arf - guinada; /* motor 4: traseiro direito */

  misturador_normaliza(saida);
}

void misturador_normaliza(saida_misturador_t *saida)
{
  float minimo = saida->motores[0];
  float maximo = saida->motores[0];
  int i;

  for (i = 1; i < (int)MIXER_NUM_MOTORES; ++i)
  {
    if (saida->motores[i] < minimo)
      minimo = saida->motores[i];
    if (saida->motores[i] > maximo)
      maximo = saida->motores[i];
  }

  if (minimo < 0.0f)
  {
    for (i = 0; i < (int)MIXER_NUM_MOTORES; ++i)
      saida->motores[i] -= minimo;
    maximo -= minimo;
  }

  if (maximo > 1.0f)
  {
    for (i = 0; i < (int)MIXER_NUM_MOTORES; ++i)
      saida->motores[i] /= maximo;
  }
}
