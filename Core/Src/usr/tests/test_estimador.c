/**
 * @file    test_estimador.c
 * @brief   Testes nativos do estimador de atitude (fusao giro + accel)
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "math/math_types.h"
#include "math/quaternion.h"
#include "control/estimador_atitude.h"

#define GRAVIDADE_M_S2 9.80665f
#define DT 0.000125f

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

/* Acelerometro medido em repouso com a atitude q (gravidade -> corpo). */
static void accel_de_atitude(const quat_t q, float a[3])
{
  vetor3_t g = quat_rotaciona_vetor(quat_conjuga(q),
                                    (vetor3_t){0.0f, 0.0f, 1.0f});
  a[0] = g.x * GRAVIDADE_M_S2;
  a[1] = g.y * GRAVIDADE_M_S2;
  a[2] = g.z * GRAVIDADE_M_S2;
}

static void roda_estatico(estimador_atitude_t *est, const quat_t alvo, int passos)
{
  float a[3];
  int k;

  accel_de_atitude(alvo, a);
  for (k = 0; k < passos; ++k)
  {
    float giro[3] = {0.0f, 0.0f, 0.0f};
    estimador_atitude_atualiza(est, giro, a, DT);
  }
}

int testes_estimador(void)
{
  config_estimador_t cfg;
  estimador_atitude_t est;
  vetor3_t euler;
  quat_t alvo;
  float a[3];
  float giro[3];
  int k;

  /* ---------- Nivelado em repouso ---------- */
  {
    estimador_config_padrao(&cfg);
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    roda_estatico(&est, quat_identidade(), 2000);
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.x, 0.0f, 1.0e-3f) &&
             APROX(euler.y, 0.0f, 1.0e-3f) &&
             APROX(euler.z, 0.0f, 1.0e-3f),
             "nivelado em repouso -> euler nulo");
  }

  /* ---------- Convergencia para arfagem de 30 deg ---------- */
  {
    cfg.ganho = 5.0f;
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    alvo = quat_de_euler(0.0f, 30.0f * 0.0174532925f, 0.0f);
    roda_estatico(&est, alvo, 8000); /* 1 s */
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.y, 30.0f * 0.0174532925f, 0.02f),
             "converge para arfagem de 30 deg");
    VERIFICA(APROX(euler.x, 0.0f, 0.02f), "rol permanece ~0 com arfagem");
  }

  /* ---------- Convergencia para rol de -30 deg ---------- */
  {
    cfg.ganho = 5.0f;
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    alvo = quat_de_euler(-30.0f * 0.0174532925f, 0.0f, 0.0f);
    roda_estatico(&est, alvo, 8000);
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.x, -30.0f * 0.0174532925f, 0.02f),
             "converge para rol de -30 deg");
  }

  /* ---------- Integracao pura de guinada (gyro z) ---------- */
  {
    estimador_config_padrao(&cfg);
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    a[0] = 0.0f;
    a[1] = 0.0f;
    a[2] = GRAVIDADE_M_S2;
    giro[0] = 0.0f;
    giro[1] = 0.0f;
    giro[2] = 90.0f * 0.0174532925f; /* 90 deg/s */
    for (k = 0; k < 4000; ++k)       /* 0.5 s -> 45 deg */
    {
      estimador_atitude_atualiza(&est, giro, a, DT);
    }
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.z, 45.0f * 0.0174532925f, 0.01f),
             "guinada integra 90 deg/s por 0,5 s -> 45 deg");
    VERIFICA(APROX(euler.x, 0.0f, 0.02f) && APROX(euler.y, 0.0f, 0.02f),
             "rol/arfagem permanecem ~0 na guinada pura");
  }

  /* ---------- Convergencia a partir de atitude errada ---------- */
  {
    cfg.ganho = 5.0f;
    estimador_atitude_inicializa(&est, &cfg,
                                 30.0f * 0.0174532925f,
                                 20.0f * 0.0174532925f,
                                 0.0f);
    roda_estatico(&est, quat_identidade(), 8000);
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.x, 0.0f, 0.02f) && APROX(euler.y, 0.0f, 0.02f),
             "partindo de atitude errada converge para nivelado");
  }

  printf("testes_estimador: %d falha(s)\n", falhas);
  return falhas;
}
