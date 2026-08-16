/**
 * @file    pid.c
 * @brief   Núcleo de controle PID com anti-windup e D na medição
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "control/pid.h"
#include "math/math_utils.h"

void pid_inicializa(controlador_pid_t *pid, const ganhos_pid_t *ganhos, float dt)
{
  pid->ganhos = *ganhos;
  pid->dt = dt;
  pid->erro_anterior = 0.0f;
  pid->medicao_anterior = 0.0f;
  pid->integral = 0.0f;
  pid->saida = 0.0f;

  if ((ganhos->d_fc > 0.0f) && (dt > 0.0f))
    filtro_lpf1_inicializa(&pid->d_filtro, ganhos->d_fc, dt);
  else
    filtro_lpf1_inicializa(&pid->d_filtro, 1.0e3f, dt);
}

float pid_atualiza(controlador_pid_t *pid, float setpoint, float medicao)
{
  float erro = setpoint - medicao;
  float erro_p = pid->ganhos.peso_setpoint * setpoint - medicao;
  float derivada;
  float saida_bruta;
  float dt = pid->dt;

  if (dt <= 0.0f)
    return pid->saida;

  /* Derivada na medição para evitar derivative-kick */
  derivada = -(medicao - pid->medicao_anterior) / dt;
  if (pid->ganhos.d_fc > 0.0f)
    derivada = filtro_lpf1_aplica(&pid->d_filtro, derivada);
  pid->medicao_anterior = medicao;

  saida_bruta = pid->ganhos.kp * erro_p
              + pid->ganhos.ki * pid->integral
              + pid->ganhos.kd * derivada;

  /* Anti-windup: integração condicional, integra apenas fora da saturação */
  if ((saida_bruta <= pid->ganhos.limite_saida) &&
      (saida_bruta >= -pid->ganhos.limite_saida))
  {
    pid->integral += erro * dt;
    pid->integral = limita(pid->integral,
                           -pid->ganhos.limite_integral,
                           pid->ganhos.limite_integral);
  }

  pid->saida = limita(saida_bruta,
                      -pid->ganhos.limite_saida,
                      pid->ganhos.limite_saida);
  pid->erro_anterior = erro;
  return pid->saida;
}

void pid_reseta(controlador_pid_t *pid)
{
  pid->erro_anterior = 0.0f;
  pid->medicao_anterior = 0.0f;
  pid->integral = 0.0f;
  pid->saida = 0.0f;
  filtro_lpf1_reseta(&pid->d_filtro);
}

void pid_define_integral(controlador_pid_t *pid, float valor)
{
  pid->integral = limita(valor,
                         -pid->ganhos.limite_integral,
                         pid->ganhos.limite_integral);
}

float pid_obtem_integral(const controlador_pid_t *pid)
{
  return pid->integral;
}
