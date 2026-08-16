/**
 * @file    test_radio.c
 * @brief   Testes nativos do mapeamento canais do radio -> setpoint
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "serial/radio_comandos.h"
#include "control/control_types.h"

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

int testes_radio(void)
{
  config_radio_t cfg;
  uint16_t canais[4] = {1500u, 1500u, 1500u, 1500u};
  setpoint_t sp;

  radio_comandos_config_padrao(&cfg);

  /* ---------- Sticks centrados ---------- */
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.atitude.euler.x, 0.0f, 1.0e-6f) &&
           APROX(sp.atitude.euler.y, 0.0f, 1.0e-6f) &&
           APROX(sp.taxa.angvel.z, 0.0f, 1.0e-6f),
           "centro -> rol/arfagem/guinada nulos");
  VERIFICA(APROX(sp.throttle, 0.5f, 1.0e-6f), "throttle no centro = 0,5");

  /* ---------- Rol cheio para a direita ---------- */
  canais[0] = 2000u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.atitude.euler.x, cfg.max_angulo, 1.0e-3f),
           "rol cheio -> max_angulo");
  VERIFICA(APROX(sp.taxa.angvel.x, cfg.max_taxa, 1.0e-3f),
           "rol cheio -> max_taxa (reserva ACRO)");

  /* ---------- Zona morta ---------- */
  canais[0] = 1500u + 10u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.atitude.euler.x, 0.0f, 1.0e-6f),
           "dentro da zona morta -> sem comando");
  canais[0] = 1500u - 10u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.atitude.euler.x, 0.0f, 1.0e-6f),
           "zona morta simetrica (esquerda)");

  /* ---------- Arfagem cheia para trás ---------- */
  canais[0] = 1500u;
  canais[1] = 1000u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.atitude.euler.y, -cfg.max_angulo, 1.0e-3f),
           "arfagem cheia -> -max_angulo");

  /* ---------- Guinada cheia ---------- */
  canais[1] = 1500u;
  canais[3] = 2000u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.taxa.angvel.z, cfg.max_taxa_guinada, 1.0e-3f),
           "guinada cheia -> max_taxa_guinada");

  /* ---------- Throttle 0% e 100% ---------- */
  canais[3] = 1500u;
  canais[2] = 1000u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.throttle, 0.0f, 1.0e-6f), "throttle minimo -> 0");

  canais[2] = 2000u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.throttle, 1.0f, 1.0e-6f), "throttle maximo -> 1");

  /* ---------- Desarmado zera o throttle ---------- */
  radio_comandos_processa(&cfg, canais, 0u, &sp);
  VERIFICA(APROX(sp.throttle, 0.0f, 1.0e-6f), "desarmado -> throttle 0");

  /* ---------- Clamp de valores fora da faixa ---------- */
  canais[0] = 2500u;
  canais[2] = 1500u;
  radio_comandos_processa(&cfg, canais, 1u, &sp);
  VERIFICA(APROX(sp.atitude.euler.x, cfg.max_angulo, 1.0e-3f),
           "rol 2500 -> limitado em max_angulo");

  /* ---------- Argumentos nulos nao travam ---------- */
  radio_comandos_processa(&cfg, canais, 1u, 0);
  radio_comandos_processa(0, canais, 1u, &sp);

  printf("testes_radio: %d falha(s)\n", falhas);
  return falhas;
}
