/**
 * @file    radio_comandos.h
 * @brief   Mapeamento dos canais do receptor (iBus) para o setpoint de voo
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Puro (sem HAL). Convencao AETR:
 *   CH1 rol    -> setpoint.atitude.euler.x (rad, modo ANGLE)
 *   CH2 pitch  -> setpoint.atitude.euler.y (rad)
 *   CH3 throttle -> setpoint.throttle (0..1)
 *   CH4 yaw    -> setpoint.taxa.angvel.z (rad/s; guinada nao se auto-nivela)
 *
 * Tambem preenche setpoint.taxa.angvel.x/y (reserva para o modo ACRO).
 */

#ifndef USR_SERIAL_RADIO_COMANDOS_H
#define USR_SERIAL_RADIO_COMANDOS_H

#include <stdint.h>

#include "control/control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t canal_rol;        /* indice do canal de rol   (CH1 -> 0) */
  uint16_t canal_arfagem;    /* indice do canal de arfagem (CH2 -> 1) */
  uint16_t canal_throttle;   /* indice do canal de throttle (CH3 -> 2) */
  uint16_t canal_guinada;    /* indice do canal de guinada (CH4 -> 3) */
  uint16_t centro;           /* centro dos sticks (1500 us)            */
  uint16_t meio_curso;       /* metade do curso (500 us: 1000..2000)   */
  uint16_t deadband;         /* zona morta (us)                        */
  float    max_angulo;       /* angulo maximo de rol/arfagem (rad)     */
  float    max_taxa;         /* taxa maxima de rol/arfagem (rad/s)     */
  float    max_taxa_guinada; /* taxa maxima de guinada (rad/s)         */
} config_radio_t;

/** Configuracao padrao AETR: centro 1500, deadband 12, 30 deg e 200 deg/s. */
void radio_comandos_config_padrao(config_radio_t *cfg);

/**
 * @brief  Converte os canais do receptor em um setpoint de voo.
 * @param  cfg    configuracao
 * @param  canais canais do iBus (1000..2000)
 * @param  armado 1 com o controle habilitado; 0 zera o throttle
 * @param  sp     setpoint de saida (nao pode ser NULL)
 */
void radio_comandos_processa(const config_radio_t *cfg,
                             const uint16_t *canais, uint8_t armado,
                             setpoint_t *sp);

#ifdef __cplusplus
}
#endif

#endif /* USR_SERIAL_RADIO_COMANDOS_H */
