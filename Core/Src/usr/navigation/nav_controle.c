/**
 * @file    nav_controle.c
 * @brief   Controlador de navegacao (posicao/altura/rumo) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <math.h>
#include <string.h>
#include "navigation/nav_controle.h"

#define NAV_PI 3.14159265358979323846f

void config_nav_controle_padrao(config_nav_controle_t *cfg)
{
  cfg->kp_posicao = 0.30f;
  cfg->kd_velocidade = 0.25f;
  cfg->kp_guinada = 3.0f;
  cfg->max_angulo = 0.35f;
  cfg->kp_altura = 0.15f;
  cfg->kd_altura = 0.20f;
  cfg->thrust_hover = 0.52f;
  cfg->raio_chegada = 1.5f;
  cfg->vel_chegada = 0.5f;
  cfg->alinhar_rad = 0.15f;
}

void nav_controle_inicializa(nav_controle_t *nav,
                             const config_nav_controle_t *cfg)
{
  memset(nav, 0, sizeof(*nav));
  if (cfg != 0)
  {
    nav->config = *cfg;
  }
  else
  {
    config_nav_controle_padrao(&nav->config);
  }
}

float nav_controle_envolve_pi(float angulo)
{
  float a = fmodf(angulo, 2.0f * NAV_PI);
  if (a >= NAV_PI)
  {
    a -= 2.0f * NAV_PI;
  }
  else if (a < -NAV_PI)
  {
    a += 2.0f * NAV_PI;
  }
  return a;
}

static float limitar(float v, float max_abs)
{
  if (v > max_abs)
  {
    return max_abs;
  }
  if (v < -max_abs)
  {
    return -max_abs;
  }
  return v;
}

void nav_controle_atualiza(nav_controle_t *nav, nav_comando_t *cmd,
                           float guinada_rad,
                           float erro_norte_m, float erro_leste_m,
                           float vel_norte_m_s, float vel_leste_m_s,
                           float alt_erro_m, float vel_v_m_s,
                           float dist_m, float vel_solo_m_s)
{
  const config_nav_controle_t *cfg = &nav->config;
  float rumo_des;
  float erro_rumo;
  float c, s;
  float erro_x, erro_y;
  float vel_x, vel_y;
  float arfagem, rol;

  /* Rumo desejado: aponta para o alvo. Chegando (dist e vel abaixo
   * dos limites) e ja quase alinhado, congela a guinada para evitar
   * giro desnecessario. */
  rumo_des = atan2f(erro_leste_m, erro_norte_m);
  erro_rumo = nav_controle_envolve_pi(rumo_des - guinada_rad);
  if (dist_m <= cfg->raio_chegada && vel_solo_m_s <= cfg->vel_chegada &&
      fabsf(erro_rumo) <= cfg->alinhar_rad)
  {
    rumo_des = guinada_rad;
    erro_rumo = 0.0f;
  }

  /* Erro e velocidade no corpo da aeronave (rotaciona por -guinada). */
  c = cosf(guinada_rad);
  s = sinf(guinada_rad);
  erro_x = c * erro_norte_m + s * erro_leste_m;
  erro_y = -s * erro_norte_m + c * erro_leste_m;
  vel_x = c * vel_norte_m_s + s * vel_leste_m_s;
  vel_y = -s * vel_norte_m_s + c * vel_leste_m_s;

  /* Inclina para mover: frente = arfagem negativa, direita = rol+.
   * Amortecimento: +kd*vel (reduz a aceleracao quando aproxima). */
  arfagem = -cfg->kp_posicao * erro_x + cfg->kd_velocidade * vel_x;
  rol = cfg->kp_posicao * erro_y - cfg->kd_velocidade * vel_y;

  cmd->arfagem_des_rad = limitar(arfagem, cfg->max_angulo);
  cmd->rol_des_rad = limitar(rol, cfg->max_angulo);
  cmd->taxa_guinada_rad_s = cfg->kp_guinada * erro_rumo;

  cmd->throttle = cfg->thrust_hover + cfg->kp_altura * alt_erro_m -
                  cfg->kd_altura * vel_v_m_s;
  if (cmd->throttle < 0.0f)
  {
    cmd->throttle = 0.0f;
  }
  if (cmd->throttle > 1.0f)
  {
    cmd->throttle = 1.0f;
  }
}
