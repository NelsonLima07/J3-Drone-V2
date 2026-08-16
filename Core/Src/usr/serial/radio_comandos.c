/**
 * @file    radio_comandos.c
 * @brief   Mapeamento dos canais do receptor para o setpoint de voo
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "serial/radio_comandos.h"
#include "math/math_utils.h"

#define DEG_TO_RAD 0.01745329251994329577f

void radio_comandos_config_padrao(config_radio_t *cfg)
{
  cfg->canal_rol = 0u;        /* CH1 */
  cfg->canal_arfagem = 1u;    /* CH2 */
  cfg->canal_throttle = 2u;   /* CH3 */
  cfg->canal_guinada = 3u;    /* CH4 */
  cfg->centro = 1500u;
  cfg->meio_curso = 500u;
  cfg->deadband = 12u;
  cfg->max_angulo = 30.0f * DEG_TO_RAD;
  cfg->max_taxa = 200.0f * DEG_TO_RAD;
  cfg->max_taxa_guinada = 200.0f * DEG_TO_RAD;
}

void radio_comandos_processa(const config_radio_t *cfg,
                             const uint16_t *canais, uint8_t armado,
                             setpoint_t *sp)
{
  float rol;
  float arfagem;
  float guinada;

  if (cfg == 0 || canais == 0 || sp == 0 || cfg->meio_curso == 0u)
  {
    return;
  }

  /* Delta normalizado em [-1, 1] com zona morta (curso cheio = 500 us). */
  {
    int16_t d = (int16_t)((int)canais[cfg->canal_rol] - (int)cfg->centro);
    if (d < 0)
    {
      if (-d < (int16_t)cfg->deadband) d = 0;
    }
    else if (d < (int16_t)cfg->deadband)
    {
      d = 0;
    }
    rol = (float)d / (float)cfg->meio_curso;
  }
  {
    int16_t d = (int16_t)((int)canais[cfg->canal_arfagem] - (int)cfg->centro);
    if (d < 0)
    {
      if (-d < (int16_t)cfg->deadband) d = 0;
    }
    else if (d < (int16_t)cfg->deadband)
    {
      d = 0;
    }
    arfagem = (float)d / (float)cfg->meio_curso;
  }
  {
    int16_t d = (int16_t)((int)canais[cfg->canal_guinada] - (int)cfg->centro);
    if (d < 0)
    {
      if (-d < (int16_t)cfg->deadband) d = 0;
    }
    else if (d < (int16_t)cfg->deadband)
    {
      d = 0;
    }
    guinada = (float)d / (float)cfg->meio_curso;
  }

  rol = limita(rol, -1.0f, 1.0f);
  arfagem = limita(arfagem, -1.0f, 1.0f);
  guinada = limita(guinada, -1.0f, 1.0f);

  /* Modo ANGLE: rol/arfagem viram setpoint de atitude; guinada vira taxa. */
  sp->atitude.euler.x = rol * cfg->max_angulo;
  sp->atitude.euler.y = arfagem * cfg->max_angulo;
  sp->atitude.euler.z = 0.0f;

  /* Reserva ACRO: taxas de rol/arfagem/guinada. */
  sp->taxa.angvel.x = rol * cfg->max_taxa;
  sp->taxa.angvel.y = arfagem * cfg->max_taxa;
  sp->taxa.angvel.z = guinada * cfg->max_taxa_guinada;

  /* Throttle: 1000..2000 -> 0..1; zerado quando desarmado. */
  if (armado)
  {
    float th = (float)((int)canais[cfg->canal_throttle] - 1000) / 1000.0f;
    sp->throttle = limita(th, 0.0f, 1.0f);
  }
  else
  {
    sp->throttle = 0.0f;
  }
}
