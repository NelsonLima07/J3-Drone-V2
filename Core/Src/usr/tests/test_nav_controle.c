/**
 * @file    test_nav_controle.c
 * @brief   Testes nativos do controlador de navegacao (nav_controle).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "navigation/nav_controle.h"

static int falhas = 0;

#define VERIFICA(cond, nome)                                        \
  do {                                                              \
    if (cond) {                                                     \
      printf("  ok: %s\n", nome);                                   \
    } else {                                                        \
      printf("  FALHOU: %s (linha %d)\n", nome, __LINE__);          \
      ++falhas;                                                     \
    }                                                               \
  } while (0)

#define APROX(a, b, tol) (fabsf((a) - (b)) <= (tol))

#define G 9.80665f
#define PI 3.14159265358979323846f

int testes_nav_controle(void)
{
  config_nav_controle_t cfg;
  nav_controle_t nav;
  nav_comando_t cmd;

  config_nav_controle_padrao(&cfg);
  nav_controle_inicializa(&nav, &cfg);

  /* ---------- Sinais: alvo ao norte, drone rumo norte ---------- */
  nav_controle_atualiza(&nav, &cmd, 0.0f, 10.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 10.0f, 0.0f);
  VERIFICA(cmd.arfagem_des_rad < 0.0f, "alvo ao norte -> arfagem negativa");
  VERIFICA(APROX(cmd.rol_des_rad, 0.0f, 1.0e-6f), "alvo ao norte -> rol ~0");
  VERIFICA(APROX(cmd.taxa_guinada_rad_s, 0.0f, 1.0e-6f),
           "rumo ja correto -> guinada ~0");

  /* ---------- Sinais: alvo ao leste, drone rumo norte ---------- */
  nav_controle_atualiza(&nav, &cmd, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 10.0f, 0.0f);
  VERIFICA(cmd.rol_des_rad > 0.0f, "alvo ao leste -> rol positivo");
  VERIFICA(cmd.taxa_guinada_rad_s > 0.0f,
           "alvo ao leste -> guina para a direita");

  /* ---------- Saturacoes: erro grande limita o angulo ---------- */
  nav_controle_atualiza(&nav, &cmd, 0.0f, 100.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 100.0f, 0.0f);
  VERIFICA(fabsf(cmd.arfagem_des_rad) <= cfg.max_angulo + 1.0e-6f,
           "arfagem satura no max_angulo");
  VERIFICA(APROX(cmd.arfagem_des_rad, -cfg.max_angulo, 1.0e-6f),
           "arfagem negativa saturada");

  /* ---------- Altura: throttle proporcional ao erro ---------- */
  nav_controle_atualiza(&nav, &cmd, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        1.0f, 0.0f, 0.0f, 0.0f);
  VERIFICA(cmd.throttle > cfg.thrust_hover, "subir -> throttle acima do hover");
  nav_controle_atualiza(&nav, &cmd, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        -1.0f, 0.0f, 0.0f, 0.0f);
  VERIFICA(cmd.throttle < cfg.thrust_hover, "descer -> throttle abaixo do hover");
  nav_controle_atualiza(&nav, &cmd, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0f);
  VERIFICA(APROX(cmd.throttle, cfg.thrust_hover, 1.0e-6f),
           "nivelado -> throttle = hover");

  /* ---------- Chegou e alinhado: segura o rumo ---------- */
  nav_controle_atualiza(&nav, &cmd, 0.1f, 1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, 0.1f);
  VERIFICA(APROX(cmd.taxa_guinada_rad_s, 0.0f, 1.0e-6f),
           "perto do alvo e alinhado -> guinada nula");

  /* ---------- Chegou mas desalinhado: gira ate alinhar ---------- */
  nav_controle_atualiza(&nav, &cmd, 0.5f, 1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, 0.1f);
  VERIFICA(cmd.taxa_guinada_rad_s < 0.0f,
           "perto do alvo desalinhado -> gira para alinhar");

  /* ---------- Envolve PI ---------- */
  VERIFICA(APROX(nav_controle_envolve_pi(PI), -PI, 1.0e-4f), "PI -> -PI");
  VERIFICA(APROX(nav_controle_envolve_pi(-PI), -PI, 1.0e-4f), "-PI mantem");
  VERIFICA(APROX(nav_controle_envolve_pi(4.0f), 4.0f - 2.0f * PI, 1.0e-4f),
           "4 rad envolve para ~ -2,283");

  /* ---------- Malha fechada: convergir para o alvo ao norte ---------- */
  {
    float yaw = 0.0f;
    float pn = 0.0f, pe = 0.0f;
    float vn = 0.0f, ve = 0.0f;
    float dt = 0.01f;
    int k, passos = 5000;
    float an, ae;

    for (k = 0; k < passos; ++k)
    {
      float en = 10.0f - pn;
      float ee = 0.0f - pe;
      float dist = sqrtf(en * en + ee * ee);
      float vel = sqrtf(vn * vn + ve * ve);

      nav_controle_atualiza(&nav, &cmd, yaw, en, ee, vn, ve,
                            0.0f, 0.0f, dist, vel);

      yaw += cmd.taxa_guinada_rad_s * dt;
      {
        float ax = -G * sinf(cmd.arfagem_des_rad);
        float ay = G * sinf(cmd.rol_des_rad);
        an = cosf(yaw) * ax - sinf(yaw) * ay;
        ae = sinf(yaw) * ax + cosf(yaw) * ay;
      }
      vn += an * dt;
      ve += ae * dt;
      vn *= 1.0f - 0.05f * dt;
      ve *= 1.0f - 0.05f * dt;
      pn += vn * dt;
      pe += ve * dt;
    }

    VERIFICA(fabsf(pn - 10.0f) < 1.5f, "malha fechada: convergiu a 10 m norte");
    VERIFICA(fabsf(pe) < 1.5f, "malha fechada: leste permanece ~0");
    VERIFICA(sqrtf(vn * vn + ve * ve) < 1.0f, "malha fechada: parado no alvo");
    VERIFICA(fabsf(nav_controle_envolve_pi(yaw)) < 0.2f,
             "malha fechada: nariz aponta ao norte");
  }

  printf("testes_nav_controle: %d falha(s)\n", falhas);
  return falhas;
}
