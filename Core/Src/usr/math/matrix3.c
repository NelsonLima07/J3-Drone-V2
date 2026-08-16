/**
 * @file    matrix3.c
 * @brief   Operações com matrizes 3x3 (linha-major)
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "math/matrix3.h"
#include "math/quaternion.h"
#include "math/math_utils.h"

#include <math.h>

matriz3_t matriz3_identidade(void)
{
  matriz3_t a = {{{1.0f, 0.0f, 0.0f},
                  {0.0f, 1.0f, 0.0f},
                  {0.0f, 0.0f, 1.0f}}};
  return a;
}

matriz3_t matriz3_multiplica(matriz3_t a, matriz3_t b)
{
  matriz3_t r;
  int i, j, k;

  for (i = 0; i < 3; ++i)
  {
    for (j = 0; j < 3; ++j)
    {
      float soma = 0.0f;
      for (k = 0; k < 3; ++k)
        soma += a.m[i][k] * b.m[k][j];
      r.m[i][j] = soma;
    }
  }
  return r;
}

vetor3_t matriz3_multiplica_vetor(matriz3_t a, vetor3_t v)
{
  vetor3_t r;

  r.x = a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z;
  r.y = a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z;
  r.z = a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z;
  return r;
}

matriz3_t matriz3_transposta(matriz3_t a)
{
  matriz3_t r;
  int i, j;

  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j)
      r.m[i][j] = a.m[j][i];
  return r;
}

float matriz3_determinante(matriz3_t a)
{
  return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1])
       - a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0])
       + a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}

matriz3_t matriz3_inversa(matriz3_t a)
{
  matriz3_t r = matriz3_identidade();
  float det = matriz3_determinante(a);
  float inv;
  int i, j;

  if (fabsf(det) < EPS_F)
    return r;

  inv = 1.0f / det;
  r.m[0][0] = (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) * inv;
  r.m[0][1] = (a.m[0][2] * a.m[2][1] - a.m[0][1] * a.m[2][2]) * inv;
  r.m[0][2] = (a.m[0][1] * a.m[1][2] - a.m[0][2] * a.m[1][1]) * inv;
  r.m[1][0] = (a.m[1][2] * a.m[2][0] - a.m[1][0] * a.m[2][2]) * inv;
  r.m[1][1] = (a.m[0][0] * a.m[2][2] - a.m[0][2] * a.m[2][0]) * inv;
  r.m[1][2] = (a.m[0][2] * a.m[1][0] - a.m[0][0] * a.m[1][2]) * inv;
  r.m[2][0] = (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]) * inv;
  r.m[2][1] = (a.m[0][1] * a.m[2][0] - a.m[0][0] * a.m[2][1]) * inv;
  r.m[2][2] = (a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0]) * inv;

  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j)
      if (isnan(r.m[i][j]))
        return matriz3_identidade();
  return r;
}

matriz3_t matriz3_de_quat(quat_t q)
{
  matriz3_t a;
  float w = q.w, x = q.x, y = q.y, z = q.z;
  float xx = x * x, yy = y * y, zz = z * z;
  float xy = x * y, xz = x * z, yz = y * z;
  float wx = w * x, wy = w * y, wz = w * z;

  a.m[0][0] = 1.0f - 2.0f * (yy + zz);
  a.m[0][1] = 2.0f * (xy - wz);
  a.m[0][2] = 2.0f * (xz + wy);
  a.m[1][0] = 2.0f * (xy + wz);
  a.m[1][1] = 1.0f - 2.0f * (xx + zz);
  a.m[1][2] = 2.0f * (yz - wx);
  a.m[2][0] = 2.0f * (xz - wy);
  a.m[2][1] = 2.0f * (yz + wx);
  a.m[2][2] = 1.0f - 2.0f * (xx + yy);
  return a;
}

quat_t matriz3_para_quat(matriz3_t a)
{
  quat_t q;
  float traco = a.m[0][0] + a.m[1][1] + a.m[2][2];

  if (traco > 0.0f)
  {
    float s = sqrtf(traco + 1.0f) * 2.0f;
    q.w = 0.25f * s;
    q.x = (a.m[2][1] - a.m[1][2]) / s;
    q.y = (a.m[0][2] - a.m[2][0]) / s;
    q.z = (a.m[1][0] - a.m[0][1]) / s;
  }
  else if ((a.m[0][0] > a.m[1][1]) && (a.m[0][0] > a.m[2][2]))
  {
    float s = sqrtf(1.0f + a.m[0][0] - a.m[1][1] - a.m[2][2]) * 2.0f;
    q.w = (a.m[2][1] - a.m[1][2]) / s;
    q.x = 0.25f * s;
    q.y = (a.m[0][1] + a.m[1][0]) / s;
    q.z = (a.m[0][2] + a.m[2][0]) / s;
  }
  else if (a.m[1][1] > a.m[2][2])
  {
    float s = sqrtf(1.0f + a.m[1][1] - a.m[0][0] - a.m[2][2]) * 2.0f;
    q.w = (a.m[0][2] - a.m[2][0]) / s;
    q.x = (a.m[0][1] + a.m[1][0]) / s;
    q.y = 0.25f * s;
    q.z = (a.m[1][2] + a.m[2][1]) / s;
  }
  else
  {
    float s = sqrtf(1.0f + a.m[2][2] - a.m[0][0] - a.m[1][1]) * 2.0f;
    q.w = (a.m[1][0] - a.m[0][1]) / s;
    q.x = (a.m[0][2] + a.m[2][0]) / s;
    q.y = (a.m[1][2] + a.m[2][1]) / s;
    q.z = 0.25f * s;
  }
  return quat_normaliza(q);
}
