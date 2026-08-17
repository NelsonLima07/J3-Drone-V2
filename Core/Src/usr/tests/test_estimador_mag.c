/**
 * @file    test_estimador_mag.c
 * @brief   Testes nativos da correcao de guinada do estimador (LIS3MDL).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "math/math_types.h"
#include "math/quaternion.h"
#include "control/estimador_atitude.h"

#define DT 0.000125f
#define PI 3.14159265358979323846f

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

static void roda_mag(estimador_atitude_t *est, const float mag[3], int passos)
{
  int k;
  for (k = 0; k < passos; ++k)
  {
    estimador_atitude_atualiza_mag(est, mag, DT);
  }
}

int testes_estimador_mag(void)
{
  config_estimador_t cfg;
  estimador_atitude_t est;
  vetor3_t euler;
  float mag[3];
  int k;

  /* ---------- Correto: campo no X do corpo, yaw 0 ---------- */
  {
    estimador_config_padrao(&cfg);
    cfg.ganho_mag = 5.0f;
    cfg.declinacao_rad = 0.0f;
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    mag[0] = 1.0f;
    mag[1] = 0.0f;
    mag[2] = 0.0f;
    roda_mag(&est, mag, 4000);
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.z, 0.0f, 0.01f), "norte alinhado -> yaw 0 mantem");
    VERIFICA(APROX(euler.x, 0.0f, 0.01f) && APROX(euler.y, 0.0f, 0.01f),
             "rol/arfagem nao mudam na correcao de yaw");
  }

  /* ---------- Drone de fato guinado +45 deg ---------- */
  {
    estimador_config_padrao(&cfg);
    cfg.ganho_mag = 5.0f;
    cfg.declinacao_rad = 0.0f;
    /* Estimativa errada (yaw 0); campo medido no corpo de um drone
     * guinado +45: corpo = (cos45, -sin45, 0) (campo ao norte). */
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    mag[0] = 0.70710678118f;
    mag[1] = -0.70710678118f;
    mag[2] = 0.0f;
    roda_mag(&est, mag, 40000); /* 5 s a 8 kHz de passo mag */
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.z, 45.0f * PI / 180.0f, 0.05f),
             "converge para yaw +45 deg com campo no corpo");
  }

  /* ---------- Declinacao magnetica aplicada ---------- */
  {
    estimador_config_padrao(&cfg);
    cfg.ganho_mag = 5.0f;
    cfg.declinacao_rad = 20.0f * PI / 180.0f;
    /* Campo medido ao longo do X do corpo (norte magnetico).
     * Com declinacao +20, o norte verdadeiro esta +20 a leste: o
     * estimador gira o yaw para +20. */
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    mag[0] = 1.0f;
    mag[1] = 0.0f;
    mag[2] = 0.0f;
    roda_mag(&est, mag, 40000);
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.z, 20.0f * PI / 180.0f, 0.05f),
             "declinacao +20 -> yaw converge para +20");
  }

  /* ---------- Estabilidade: com yaw correto nao deriva ---------- */
  {
    estimador_config_padrao(&cfg);
    cfg.ganho_mag = 10.0f;
    cfg.declinacao_rad = 0.0f;
    estimador_atitude_inicializa(&est, &cfg, 0.0f, 0.0f, 0.0f);
    /* Simula deriva de gyro de -2 deg/s por 4 s corrigida pela mag.
     * Ganho efetivo 10/16 (mag a cada 16 passos) -> erro de regime
     * ~ asin(0,0349/0,625) ~ 3,2 deg, dentro da tolerancia. */
    mag[0] = 1.0f;
    mag[1] = 0.0f;
    mag[2] = 0.0f;
    for (k = 0; k < 32000; ++k)
    {
      float giro[3] = {0.0f, 0.0f, -2.0f * PI / 180.0f};
      float accel[3] = {0.0f, 0.0f, 9.80665f};
      estimador_atitude_atualiza(&est, giro, accel, DT);
      if ((k & 15u) == 0u)
      {
        estimador_atitude_atualiza_mag(&est, mag, DT);
      }
    }
    euler = estimador_atitude_obtem_euler(&est);
    VERIFICA(APROX(euler.z, 0.0f, 0.1f),
             "mag segura o yaw contra deriva do giroscopio");
  }

  printf("testes_estimador_mag: %d falha(s)\n", falhas);
  return falhas;
}
