/**
 * @file    quaternion.c
 * @brief   Operações com quaternions para atitude
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "math/quaternion.h"
#include "math/vector3.h"
#include "math/math_utils.h"

#include <math.h>

quat_t quat_identidade(void)
{
  quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
  return q;
}

quat_t quat_conjuga(quat_t q)
{
  quat_t r = {q.w, -q.x, -q.y, -q.z};
  return r;
}

float quat_norma(quat_t q)
{
  return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

quat_t quat_normaliza(quat_t q)
{
  float norma = quat_norma(q);
  float inv;
  quat_t r = q;

  if (norma < EPS_F)
    return quat_identidade();

  inv = 1.0f / norma;
  r.w *= inv;
  r.x *= inv;
  r.y *= inv;
  r.z *= inv;
  return r;
}

quat_t quat_multiplica(quat_t p, quat_t q)
{
  quat_t r;

  r.w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
  r.x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
  r.y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x;
  r.z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w;
  return r;
}

vetor3_t quat_rotaciona_vetor(quat_t q, vetor3_t v)
{
  quat_t q_conj = quat_conjuga(q);
  quat_t qv = {0.0f, v.x, v.y, v.z};
  quat_t res = quat_multiplica(quat_multiplica(q, qv), q_conj);
  vetor3_t r = {res.x, res.y, res.z};
  return r;
}

quat_t quat_de_euler(float rol, float arfagem, float guinada)
{
  quat_t q;
  float cy = cosf(guinada * 0.5f);
  float sy = sinf(guinada * 0.5f);
  float cp = cosf(arfagem * 0.5f);
  float sp = sinf(arfagem * 0.5f);
  float cr = cosf(rol * 0.5f);
  float sr = sinf(rol * 0.5f);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

vetor3_t quat_para_euler(quat_t q)
{
  vetor3_t r;
  float w = q.w, x = q.x, y = q.y, z = q.z;
  float m02 = 2.0f * (x * z - w * y); /* R[2][0] */
  float m12 = 2.0f * (y * z + w * x); /* R[2][1] */
  float m22 = 1.0f - 2.0f * (x * x + y * y); /* R[2][2] */
  float m10 = 2.0f * (x * y + w * z); /* R[1][0] */
  float m00 = 1.0f - 2.0f * (y * y + z * z); /* R[0][0] */

  r.x = atan2_rapido(m12, m22);
  r.y = -asin_rapido(limita(m02, -1.0f, 1.0f));
  r.z = atan2_rapido(m10, m00);
  return r;
}

quat_t quat_de_eixo_angulo(vetor3_t eixo, float angulo)
{
  quat_t q;
  float metade = angulo * 0.5f;
  float s = sinf(metade);
  vetor3_t n = vetor3_normaliza(eixo);

  q.w = cosf(metade);
  q.x = n.x * s;
  q.y = n.y * s;
  q.z = n.z * s;
  return q;
}

quat_t quat_de_dois_vetores(vetor3_t de, vetor3_t para)
{
  vetor3_t n_de = vetor3_normaliza(de);
  vetor3_t n_para = vetor3_normaliza(para);
  vetor3_t eixo = vetor3_produto_vetorial(n_de, n_para);
  float produto = vetor3_produto_escalar(n_de, n_para);
  quat_t q;

  if (vetor3_norma(eixo) < EPS_F)
  {
    if (produto < 0.0f)
    {
      vetor3_t ortogonal = {1.0f, 0.0f, 0.0f};
      if (fabsf(n_de.x) > 0.9f)
      {
        ortogonal.x = 0.0f;
        ortogonal.y = 1.0f;
      }
      eixo = vetor3_produto_vetorial(n_de, ortogonal);
      q = quat_de_eixo_angulo(vetor3_normaliza(eixo), PI_F);
      return q;
    }
    return quat_identidade();
  }

  q.w = vetor3_norma(n_de) * vetor3_norma(n_para) + produto;
  q.x = eixo.x;
  q.y = eixo.y;
  q.z = eixo.z;
  return quat_normaliza(q);
}

quat_t quat_slerp(quat_t q0, quat_t q1, float t)
{
  float cos_theta = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
  float s0, s1;
  quat_t q1_ajustado = q1;

  if (cos_theta < 0.0f)
  {
    cos_theta = -cos_theta;
    q1_ajustado.w = -q1.w;
    q1_ajustado.x = -q1.x;
    q1_ajustado.y = -q1.y;
    q1_ajustado.z = -q1.z;
  }

  if (cos_theta > 0.9995f)
  {
    return quat_normaliza((quat_t){q0.w + (q1_ajustado.w - q0.w) * t,
                                   q0.x + (q1_ajustado.x - q0.x) * t,
                                   q0.y + (q1_ajustado.y - q0.y) * t,
                                   q0.z + (q1_ajustado.z - q0.z) * t});
  }

  {
    float theta = acosf(cos_theta);
    float seno_theta = sinf(theta);
    s0 = sinf((1.0f - t) * theta) / seno_theta;
    s1 = sinf(t * theta) / seno_theta;
  }

  {
    quat_t r;
    r.w = q0.w * s0 + q1_ajustado.w * s1;
    r.x = q0.x * s0 + q1_ajustado.x * s1;
    r.y = q0.y * s0 + q1_ajustado.y * s1;
    r.z = q0.z * s0 + q1_ajustado.z * s1;
    return r;
  }
}

quat_t quat_diferenca(quat_t q0, quat_t q1)
{
  return quat_normaliza(quat_multiplica(quat_conjuga(q0), q1));
}
