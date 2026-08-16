/**
 * @file    test_math.c
 * @brief   Testes nativos da biblioteca math
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include <math.h>

#include "math/math_types.h"
#include "math/math_utils.h"
#include "math/vector3.h"
#include "math/quaternion.h"
#include "math/matrix3.h"
#include "math/filters.h"

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

typedef float (*aplica_lpf1_fn)(void *, float);
typedef float (*aplica_biquad_fn)(void *, float);

static float aplica_lpf1(void *f, float am)
{
  return filtro_lpf1_aplica((filtro_lpf1_t *)f, am);
}

static float aplica_biquad(void *f, float am)
{
  return filtro_biquad_aplica((filtro_biquad_t *)f, am);
}

static float amplitude_sinal(void *filtro, aplica_lpf1_fn fn_lpf1, aplica_biquad_fn fn_biquad,
                             float fc_sinal, float fs, int n_total, int n_medida)
{
  float t = 0.0f;
  float minimo = 1.0e30f;
  float maximo = -1.0e30f;
  int i;

  for (i = 0; i < n_total; ++i, t += 1.0f / fs)
  {
    float amostra = sinf(TWO_PI_F * fc_sinal * t);
    float saida = (fn_lpf1 != 0) ? fn_lpf1(filtro, amostra) : fn_biquad(filtro, amostra);
    if (i >= (n_total - n_medida))
    {
      if (saida < minimo)
        minimo = saida;
      if (saida > maximo)
        maximo = saida;
    }
  }
  return (maximo - minimo) * 0.5f;
}

int testes_math(void)
{
  int i;

  /* ---------- math_utils ---------- */
  VERIFICA(APROX(limita(5.0f, 0.0f, 1.0f), 1.0f, 1.0e-6f), "limita acima");
  VERIFICA(APROX(limita(-5.0f, 0.0f, 1.0f), 0.0f, 1.0e-6f), "limita abaixo");
  VERIFICA(APROX(limita(0.5f, 0.0f, 1.0f), 0.5f, 1.0e-6f), "limita interno");
  VERIFICA(APROX(envolve_pi(3.5f), -2.78318530718f, 1.0e-5f), "envolve_pi acima");
  VERIFICA(APROX(envolve_pi(-3.5f), 2.78318530718f, 1.0e-5f), "envolve_pi abaixo");
  VERIFICA(zona_morta(0.05f, 0.1f) == 0.0f, "zona_morta interno");
  VERIFICA(APROX(zona_morta(0.5f, 0.1f), 0.5f, 1.0e-6f), "zona_morta externo");
  VERIFICA(APROX(lerp(0.0f, 10.0f, 0.25f), 2.5f, 1.0e-6f), "lerp");
  VERIFICA(APROX(raiz_inversa_rapida(4.0f), 0.5f, 1.0e-6f), "raiz_inversa_rapida");

  /* ---------- atan2_rapido ---------- */
  {
    float erro_max = 0.0f;
    for (i = 0; i < 720; ++i)
    {
      float ang = (float)i * DEG_TO_RAD_F * 0.5f;
      float x = cosf(ang);
      float y = sinf(ang);
      float ref = atan2f(y, x);
      float obtido = atan2_rapido(y, x);
      float erro = fabsf(obtido - ref);
      if (erro > erro_max)
        erro_max = erro;
    }
    printf("  atan2_rapido: erro maximo medido = %.3e rad\n", (double)erro_max);
    VERIFICA(erro_max < 1.0e-3f, "atan2_rapido precisao < 1e-3 rad");
  }
  VERIFICA(APROX(atan2_rapido(1.0f, 0.0f), PI_F * 0.5f, 1.0e-3f), "atan2(1,0) = pi/2");
  VERIFICA(APROX(atan2_rapido(0.0f, 1.0f), 0.0f, 1.0e-3f), "atan2(0,1) = 0");
  VERIFICA(APROX(atan2_rapido(-1.0f, 0.0f), -PI_F * 0.5f, 1.0e-3f), "atan2(-1,0) = -pi/2");
  VERIFICA(APROX(atan2_rapido(0.0f, -1.0f), PI_F, 1.0e-3f), "atan2(0,-1) = pi");
  VERIFICA(atan2_rapido(0.0f, 0.0f) == 0.0f, "atan2(0,0) = 0");

  /* ---------- vector3 ---------- */
  {
    vetor3_t a = {1.0f, 2.0f, 3.0f};
    vetor3_t b = {4.0f, 5.0f, 6.0f};
    vetor3_t s = vetor3_soma(a, b);
    vetor3_t d = vetor3_subtrai(a, b);
    vetor3_t c = vetor3_produto_vetorial(a, b);
    vetor3_t n = vetor3_normaliza(a);

    VERIFICA(APROX(s.x, 5.0f, 1.0e-6f) && APROX(s.y, 7.0f, 1.0e-6f) && APROX(s.z, 9.0f, 1.0e-6f), "vetor3_soma");
    VERIFICA(APROX(d.x, -3.0f, 1.0e-6f) && APROX(d.y, -3.0f, 1.0e-6f) && APROX(d.z, -3.0f, 1.0e-6f), "vetor3_subtrai");
    VERIFICA(APROX(vetor3_produto_escalar(a, b), 32.0f, 1.0e-6f), "vetor3_produto_escalar");
    VERIFICA(APROX(c.x, -3.0f, 1.0e-6f) && APROX(c.y, 6.0f, 1.0e-6f) && APROX(c.z, -3.0f, 1.0e-6f), "vetor3_produto_vetorial");
    VERIFICA(APROX(vetor3_norma(n), 1.0f, 1.0e-5f), "vetor3_normaliza norma 1");
  }

  /* ---------- quaternion ---------- */
  {
    quat_t qi = quat_identidade();
    quat_t qz = quat_de_eixo_angulo((vetor3_t){0.0f, 0.0f, 1.0f}, PI_F * 0.5f);
    vetor3_t v = {1.0f, 0.0f, 0.0f};
    vetor3_t r = quat_rotaciona_vetor(qz, v);
    vetor3_t r2 = vetor3_rotaciona_por_quat(v, qz);

    VERIFICA(qi.w == 1.0f, "quat_identidade");
    VERIFICA(APROX(r.x, 0.0f, 1.0e-5f) && APROX(r.y, 1.0f, 1.0e-5f) && APROX(r.z, 0.0f, 1.0e-5f), "rotaciona X por 90 graus em Z");
    VERIFICA(APROX(r2.x, r.x, 1.0e-6f) && APROX(r2.y, r.y, 1.0e-6f) && APROX(r2.z, r.z, 1.0e-6f), "vetor3_rotaciona_por_quat consistente");
  }

  /* roundtrip euler -> quat -> euler */
  {
    const float rps[][3] = {{0.1f, -0.2f, 0.3f},
                            {-0.5f, 0.5f, -0.5f},
                            {1.4f, -0.3f, 2.0f},
                            {0.0f, 0.0f, 0.0f}};
    int k;
    for (k = 0; k < 4; ++k)
    {
      quat_t q = quat_de_euler(rps[k][0], rps[k][1], rps[k][2]);
      vetor3_t e = quat_para_euler(q);
      char nome[64];
      float d0 = envolve_pi(e.x - rps[k][0]);
      float d1 = envolve_pi(e.y - rps[k][1]);
      float d2 = envolve_pi(e.z - rps[k][2]);
      sprintf(nome, "roundtrip euler/quat caso %d", k);
      VERIFICA(APROX(d0, 0.0f, 1.0e-3f) && APROX(d1, 0.0f, 1.0e-3f) && APROX(d2, 0.0f, 1.0e-3f), nome);
    }
  }

  /* quat_de_dois_vetores */
  {
    vetor3_t de = {1.0f, 0.0f, 0.0f};
    vetor3_t para = {0.0f, 1.0f, 0.0f};
    quat_t q = quat_de_dois_vetores(de, para);
    vetor3_t r = quat_rotaciona_vetor(q, de);
    VERIFICA(APROX(r.x, 0.0f, 1.0e-4f) && APROX(r.y, 1.0f, 1.0e-4f) && APROX(r.z, 0.0f, 1.0e-4f), "quat_de_dois_vetores eixo X->Y");
  }

  /* slerp */
  {
    quat_t q0 = quat_identidade();
    quat_t q1 = quat_de_eixo_angulo((vetor3_t){0.0f, 0.0f, 1.0f}, PI_F * 0.5f);
    quat_t qa = quat_slerp(q0, q1, 0.0f);
    quat_t qb = quat_slerp(q0, q1, 1.0f);
    quat_t qm = quat_slerp(q0, q1, 0.5f);

    VERIFICA(APROX(qa.w, 1.0f, 1.0e-6f), "slerp t=0 -> q0");
    VERIFICA(APROX(qb.x, q1.x, 1.0e-6f) && APROX(qb.z, q1.z, 1.0e-6f), "slerp t=1 -> q1");
    VERIFICA(APROX(quat_norma(qm), 1.0f, 1.0e-5f), "slerp ponto medio normalizado");
    VERIFICA(APROX(qm.w, cosf(PI_F * 0.125f), 1.0e-4f), "slerp ponto medio angulo pi/8");
  }

  /* ---------- matrix3 ---------- */
  {
    matriz3_t id = matriz3_identidade();
    matriz3_t a = {{{4.0f, 7.0f, 2.0f}, {3.0f, 6.0f, 1.0f}, {2.0f, 5.0f, 1.0f}}};
    matriz3_t inv = matriz3_inversa(a);
    matriz3_t prod = matriz3_multiplica(a, inv);
    vetor3_t v = {1.0f, 2.0f, 3.0f};
    vetor3_t mv = matriz3_multiplica_vetor(id, v);

    VERIFICA(APROX(mv.x, 1.0f, 1.0e-6f) && APROX(mv.y, 2.0f, 1.0e-6f) && APROX(mv.z, 3.0f, 1.0e-6f), "matriz3_identidade * vetor");
    VERIFICA(APROX(prod.m[0][0], 1.0f, 1.0e-5f) && APROX(prod.m[1][1], 1.0f, 1.0e-5f) && APROX(prod.m[2][2], 1.0f, 1.0e-5f),
             "matriz3_inversa: A*inv(A)=I");
    VERIFICA(APROX(matriz3_determinante(a), 3.0f, 1.0e-5f), "matriz3_determinante");
  }

  /* quat <-> matriz roundtrip */
  {
    quat_t q = quat_de_eixo_angulo((vetor3_t){1.0f, 2.0f, 3.0f}, 0.8f);
    matriz3_t m = matriz3_de_quat(q);
    quat_t q2 = matriz3_para_quat(m);
    VERIFICA(APROX(q.w, q2.w, 1.0e-4f) && APROX(q.x, q2.x, 1.0e-4f) &&
             APROX(q.y, q2.y, 1.0e-4f) && APROX(q.z, q2.z, 1.0e-4f),
             "roundtrip quat -> mat -> quat");
  }

  /* ---------- filtros ---------- */
  {
    filtro_lpf1_t lpf1;
    float saida = 0.0f;
    int k;
    filtro_lpf1_inicializa(&lpf1, 100.0f, 1.0f / 8000.0f);
    for (k = 0; k < 8000; ++k)
      saida = filtro_lpf1_aplica(&lpf1, 1.0f);
    VERIFICA(APROX(saida, 1.0f, 1.0e-3f), "lpf1 converge ao degrau");
  }

  {
    filtro_biquad_t lpf2;
    filtro_biquad_inicializa_lpf2(&lpf2, 500.0f, 0.7071f, 1.0f / 8000.0f);
    {
      float baixo = amplitude_sinal(&lpf2, 0, aplica_biquad, 50.0f, 8000.0f, 16000, 8000);
      float alto = amplitude_sinal(&lpf2, 0, aplica_biquad, 3000.0f, 8000.0f, 16000, 8000);
      VERIFICA(baixo > 0.9f, "biquad lpf2 passa frequencia baixa");
      VERIFICA(alto < 0.5f, "biquad lpf2 atenua frequencia alta");
    }
  }

  {
    filtro_biquad_t notch;
    filtro_biquad_inicializa_notch(&notch, 1000.0f, 8.0f, 1.0f / 8000.0f);
    {
      float na_fc = amplitude_sinal(&notch, 0, aplica_biquad, 1000.0f, 8000.0f, 16000, 8000);
      float fora = amplitude_sinal(&notch, 0, aplica_biquad, 200.0f, 8000.0f, 16000, 8000);
      VERIFICA(na_fc < 0.2f, "notch rejeita a frequencia central");
      VERIFICA(fora > 0.8f, "notch nao afeta frequencia distante");
    }
  }

  {
    filtro_media_movel_t mv;
    float saida = 0.0f;
    int k;
    filtro_media_movel_inicializa(&mv, 4u);
    for (k = 1; k <= 5; ++k)
      saida = filtro_media_movel_aplica(&mv, (float)k);
    VERIFICA(APROX(saida, (2.0f + 3.0f + 4.0f + 5.0f) * 0.25f, 1.0e-5f), "media movel ultima janela");
  }

  {
    filtro_complementar_t comp;
    float saida;
    filtro_complementar_inicializa(&comp, 0.9f);
    saida = filtro_complementar_aplica(&comp, 1.0f, 0.0f);
    VERIFICA(APROX(saida, 0.9f, 1.0e-6f), "complementar mistura 0.9/0.1");
  }

  return falhas;
}
