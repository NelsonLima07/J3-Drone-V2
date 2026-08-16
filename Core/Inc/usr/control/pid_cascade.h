/**
 * @file    pid_cascade.h
 * @brief   Controlador em cascata: ângulo -> taxa -> torque
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_CONTROL_PID_CASCADE_H
#define USR_CONTROL_PID_CASCADE_H

#include "control/controlador.h"
#include "control/pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuração do controlador em cascata */
typedef struct {
  controlador_modo_t modo;      /* ANGLE ou ACRO */
  ganhos_pid_t ganhos_angulo[3];/* {rol, arfagem, guinada} - loop externo */
  ganhos_pid_t ganhos_taxa[3];  /* {rol, arfagem, guinada} - loop interno */
  float dt_angulo;              /* período do loop externo (ex.: 1/1000 s) */
  float dt_taxa;                /* período do loop interno (ex.: 1/8000 s) */
  float limite_comando_taxa;    /* limite do comando de taxa (rad/s) */
} config_cascata_t;

/* Estado do controlador em cascata (implementa ops_controlador_t) */
typedef struct {
  const ops_controlador_t *ops;
  controlador_pid_t pid_angulo[3];
  controlador_pid_t pid_taxa[3];
  config_cascata_t config;
  uint32_t passo;
  uint32_t divisor;             /* divisões do loop interno por iteração do externo */
  float comando_taxa[3];        /* setpoints de taxa produzidos pelo loop externo */
} controlador_cascata_t;

/** Preenche config com valores iniciais seguros (ajustar em bancada). */
void cascata_config_padrao(config_cascata_t *config);

/**
 * @brief  Inicializa o controlador em cascata.
 */
void controlador_cascata_inicializa(controlador_cascata_t *c, const config_cascata_t *config);

/**
 * @brief  Executa uma iteração do loop interno (dt = dt_taxa).
 * @note   O loop externo roda a cada 'divisor' chamadas.
 */
void controlador_cascata_atualiza(controlador_cascata_t *c,
                                  const atitude_t *atitude,
                                  const velocidade_angular_t *taxa,
                                  const setpoint_t *setpoint,
                                  float dt,
                                  saida_controle_t *saida);

/** Zera o estado de todos os PIDs. */
void controlador_cascata_reseta(controlador_cascata_t *c);

/* Implementação concreta da interface: use com controlador_t */
extern const ops_controlador_t ops_cascata;

#ifdef __cplusplus
}
#endif

#endif /* USR_CONTROL_PID_CASCADE_H */
