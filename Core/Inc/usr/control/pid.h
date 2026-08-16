/**
 * @file    pid.h
 * @brief   Núcleo de controle PID com anti-windup e D na medição
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_CONTROL_PID_H
#define USR_CONTROL_PID_H

#include "math/filters.h"
#include "control/control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  ganhos_pid_t ganhos;
  filtro_lpf1_t d_filtro;
  float erro_anterior;
  float medicao_anterior;
  float integral;
  float saida;
  float dt;
} controlador_pid_t;

/**
 * @brief  Inicializa o PID com ganhos e passo de tempo fixo.
 */
void pid_inicializa(controlador_pid_t *pid, const ganhos_pid_t *ganhos, float dt);

/**
 * @brief  Executa uma iteração de controle.
 * @return Saída saturada conforme limite_saida.
 */
float pid_atualiza(controlador_pid_t *pid, float setpoint, float medicao);

/** Zera o estado interno do PID. */
void pid_reseta(controlador_pid_t *pid);

/** Define o valor do integrador (útil para inicialização bumpless). */
void pid_define_integral(controlador_pid_t *pid, float valor);

/** Retorna o valor atual do integrador. */
float pid_obtem_integral(const controlador_pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* USR_CONTROL_PID_H */
