/**
 * @file    controlador.c
 * @brief   Interface do controlador de voo
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "control/controlador.h"

void controlador_inicializa(controlador_t *controlador,
                            const ops_controlador_t *ops,
                            void *estado,
                            const void *config)
{
  controlador->ops = ops;
  controlador->estado = estado;
  controlador->config = config;

  if (ops->inicializa != 0)
    ops->inicializa(estado, config);
}

void controlador_atualiza(controlador_t *controlador,
                          const atitude_t *atitude,
                          const velocidade_angular_t *taxa,
                          const setpoint_t *setpoint,
                          float dt,
                          saida_controle_t *saida)
{
  if (controlador->ops->atualiza != 0)
    controlador->ops->atualiza(controlador->estado, atitude, taxa, setpoint, dt, saida);
}

void controlador_reseta(controlador_t *controlador)
{
  if (controlador->ops->reseta != 0)
    controlador->ops->reseta(controlador->estado);
}

const char *controlador_nome(const controlador_t *controlador)
{
  return controlador->ops->nome;
}
