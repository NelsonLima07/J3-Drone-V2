/**
 * @file    control_types.h
 * @brief   Tipos compartilhados do sistema de controle
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_CONTROL_TYPES_H
#define USR_CONTROL_TYPES_H

#include "math/math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ganhos e limites de um controlador PID */
typedef struct {
  float kp;
  float ki;
  float kd;
  float d_fc;            /* frequência de corte do LPF do termo D (Hz) */
  float limite_integral; /* saturação do integrador */
  float limite_saida;    /* saturação da saída */
  float peso_setpoint;   /* ponderação do setpoint na ação proporcional [0..1] */
} ganhos_pid_t;

/* Modos de operação do controlador de voo */
typedef enum {
  CONTROLADOR_MODO_ANGLE = 0, /* auto-nível: atitude -> taxa -> torque */
  CONTROLADOR_MODO_ACRO       /* taxa direta do rádio -> torque */
} controlador_modo_t;

/* Atitude estimada (rad) */
typedef struct {
  vetor3_t euler; /* {rol, arfagem, guinada} */
} atitude_t;

/* Velocidade angular do corpo (rad/s) */
typedef struct {
  vetor3_t angvel; /* {p, q, r} */
} velocidade_angular_t;

/* Comandos do piloto */
typedef struct {
  atitude_t atitude;          /* setpoint de atitude (modo ANGLE) */
  velocidade_angular_t taxa;  /* setpoint de taxa (modo ACRO / loop interno) */
  float throttle;             /* coletivo 0..1 */
} setpoint_t;

/* Saída do controlador (pré-mixagem) */
typedef struct {
  vetor3_t torque; /* comandos de torque {rol, arfagem, guinada}, normalizados -1..1 */
  float thrust;    /* coletivo 0..1 */
} saida_controle_t;

#ifdef __cplusplus
}
#endif

#endif /* USR_CONTROL_TYPES_H */
