/**
 * @file    vector3.c
 * @brief   Operações com vetores 3D
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "math/vector3.h"
#include "math/math_utils.h"

#include <math.h>

vetor3_t vetor3_zero(void)
{
  vetor3_t v = {0.0f, 0.0f, 0.0f};
  return v;
}

vetor3_t vetor3_soma(vetor3_t a, vetor3_t b)
{
  vetor3_t r = {a.x + b.x, a.y + b.y, a.z + b.z};
  return r;
}

vetor3_t vetor3_subtrai(vetor3_t a, vetor3_t b)
{
  vetor3_t r = {a.x - b.x, a.y - b.y, a.z - b.z};
  return r;
}

vetor3_t vetor3_escala(vetor3_t v, float escala)
{
  vetor3_t r = {v.x * escala, v.y * escala, v.z * escala};
  return r;
}

float vetor3_produto_escalar(vetor3_t a, vetor3_t b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

vetor3_t vetor3_produto_vetorial(vetor3_t a, vetor3_t b)
{
  vetor3_t r = {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
  return r;
}

float vetor3_norma(vetor3_t v)
{
  return sqrtf(vetor3_produto_escalar(v, v));
}

vetor3_t vetor3_normaliza(vetor3_t v)
{
  float norma = vetor3_norma(v);
  if (norma < EPS_F)
    return vetor3_zero();
  return vetor3_escala(v, 1.0f / norma);
}

vetor3_t vetor3_rotaciona_por_quat(vetor3_t v, quat_t q)
{
  /* v' = q * v * q^-1  (q deve estar normalizado) */
  vetor3_t u = {q.x, q.y, q.z};
  float s = q.w;
  vetor3_t t = vetor3_escala(vetor3_produto_vetorial(u, v), 2.0f);
  vetor3_t r;

  r = vetor3_soma(v, vetor3_escala(t, s));
  r = vetor3_soma(r, vetor3_escala(vetor3_produto_vetorial(u, t), 1.0f));
  return r;
}
