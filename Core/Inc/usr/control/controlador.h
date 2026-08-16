/**
 * @file    controlador.h
 * @brief   Interface do controlador de voo (permite trocar o algoritmo interno)
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_CONTROL_CONTROLADOR_H
#define USR_CONTROL_CONTROLADOR_H

#include "control/control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Implementações concretas: pid_cascade agora, LADRC no futuro */
struct controlador;

typedef struct {
  const char *nome;

  void (*inicializa)(void *self, const void *config);
  void (*atualiza)(void *self,
                   const atitude_t *atitude,
                   const velocidade_angular_t *taxa,
                   const setpoint_t *setpoint,
                   float dt,
                   saida_controle_t *saida);
  void (*reseta)(void *self);
} ops_controlador_t;

typedef struct controlador {
  const ops_controlador_t *ops;
  void *estado;
  const void *config;
} controlador_t;

/**
 * @brief  Vincula o controlador a uma implementação (ex.: ops_cascata).
 */
void controlador_inicializa(controlador_t *controlador,
                            const ops_controlador_t *ops,
                            void *estado,
                            const void *config);

/** Executa uma iteração do controlador. */
void controlador_atualiza(controlador_t *controlador,
                          const atitude_t *atitude,
                          const velocidade_angular_t *taxa,
                          const setpoint_t *setpoint,
                          float dt,
                          saida_controle_t *saida);

/** Zera o estado do controlador. */
void controlador_reseta(controlador_t *controlador);

/** Nome da implementação em uso (ex.: "cascata"). */
const char *controlador_nome(const controlador_t *controlador);

#ifdef __cplusplus
}
#endif

#endif /* USR_CONTROL_CONTROLADOR_H */
