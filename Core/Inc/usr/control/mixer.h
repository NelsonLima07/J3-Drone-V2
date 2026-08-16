/**
 * @file    mixer.h
 * @brief   Mixagem (alocação) de torque+coletivo para motores em frame X
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_CONTROL_MIXER_H
#define USR_CONTROL_MIXER_H

#include "math/math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MIXER_NUM_MOTORES 4u

typedef struct {
  float motores[MIXER_NUM_MOTORES]; /* comandos normalizados 0..1 */
} saida_misturador_t;

/**
 * @brief  Converte thrust (0..1) e torque {rol, arfagem, guinada} (-1..1)
 *         em comandos de motor para frame X (ordem Betaflight).
 */
void misturador_aplica(float thrust, const vetor3_t *torque, saida_misturador_t *saida);

/**
 * @brief  Normaliza os comandos para o intervalo [0, 1] preservando a relação
 *         entre motores (desloca se mínimo negativo, escala se máximo > 1).
 */
void misturador_normaliza(saida_misturador_t *saida);

#ifdef __cplusplus
}
#endif

#endif /* USR_CONTROL_MIXER_H */
