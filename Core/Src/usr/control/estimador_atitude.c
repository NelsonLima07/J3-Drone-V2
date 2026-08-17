/**
 * @file    estimador_atitude.c
 * @brief   Estimador de atitude por fusao complementar (giro + accel)
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Formulacao (Mahony-style):
 *   v   = direcao estimada da gravidade no frame do corpo  (q* . (0,0,1) . q)
 *   err = a_medido_normalizado x v
 *   w   = giro + Kp * err            (correcao no frame do corpo)
 *   q'  = 0.5 * q (x) (0, w)         (derivada do quaternion)
 */

#include "control/estimador_atitude.h"
#include "math/vector3.h"
#include "math/math_utils.h"

#include <math.h>

#define GRAVIDADE_M_S2 9.80665f

void estimador_config_padrao(config_estimador_t *cfg)
{
  cfg->ganho = 0.3f;
  cfg->g_min = 0.5f;
  cfg->g_max = 2.0f;
  cfg->ganho_mag = 0.5f;
  cfg->declinacao_rad = 0.0f;
}

void estimador_atitude_inicializa(estimador_atitude_t *est,
                                  const config_estimador_t *cfg,
                                  float rol0, float arfagem0, float guinada0)
{
  est->config = *cfg;
  est->q = quat_de_euler(rol0, arfagem0, guinada0);
  est->contador = 0u;
}

void estimador_atitude_atualiza(estimador_atitude_t *est,
                                const float giro[3], const float accel[3],
                                float dt)
{
  vetor3_t a = {accel[0], accel[1], accel[2]};
  vetor3_t v;   /* gravidade estimada no frame do corpo (normalizada) */
  vetor3_t erro;
  quat_t q_conj = quat_conjuga(est->q);
  quat_t omega = {0.0f, 0.0f, 0.0f, 0.0f};
  quat_t qd;
  quat_t q_novo;
  float norma = vetor3_norma(a);
  float em_g = (norma > 0.0f) ? (norma / GRAVIDADE_M_S2) : 0.0f;

  v = quat_rotaciona_vetor(q_conj, (vetor3_t){0.0f, 0.0f, 1.0f});

  omega.x = giro[0];
  omega.y = giro[1];
  omega.z = giro[2];

  /* Correcao apenas quando |a| ~ 1 g (sem aceleracao linear forte). */
  if ((em_g >= est->config.g_min) && (em_g <= est->config.g_max) &&
      (norma > EPS_F))
  {
    vetor3_t am = {a.x / norma, a.y / norma, a.z / norma};

    erro = vetor3_produto_vetorial(am, v);
    omega.x += est->config.ganho * erro.x;
    omega.y += est->config.ganho * erro.y;
    omega.z += est->config.ganho * erro.z;
  }

  /* q' = 0.5 * q (x) (0, w)  ->  q += q' * dt  ->  normaliza */
  qd = quat_multiplica(est->q, omega);
  qd.w *= 0.5f;
  qd.x *= 0.5f;
  qd.y *= 0.5f;
  qd.z *= 0.5f;

  q_novo.w = est->q.w + qd.w * dt;
  q_novo.x = est->q.x + qd.x * dt;
  q_novo.y = est->q.y + qd.y * dt;
  q_novo.z = est->q.z + qd.z * dt;

  est->q = quat_normaliza(q_novo);
  est->contador++;
}

void estimador_atitude_atualiza_mag(estimador_atitude_t *est,
                                    const float mag[3], float dt)
{
  vetor3_t m = {mag[0], mag[1], mag[2]};
  vetor3_t m_mundo;      /* campo no frame do mundo (NED)          */
  vetor3_t ref_mundo;    /* referencia no mundo apos declinacao    */
  vetor3_t w;            /* campo predito no frame do corpo        */
  vetor3_t erro;
  quat_t q_conj = quat_conjuga(est->q);
  quat_t omega = {0.0f, 0.0f, 0.0f, 0.0f};
  quat_t qd;
  quat_t q_novo;
  float norma = vetor3_norma(m);
  float horizontal;

  if (norma < EPS_F)
  {
    return;
  }
  m.x /= norma;
  m.y /= norma;
  m.z /= norma;

  /* Campo medido levado ao mundo -> referenciado por declinacao. */
  m_mundo = quat_rotaciona_vetor(est->q, m);
  horizontal = sqrtf(m_mundo.x * m_mundo.x + m_mundo.y * m_mundo.y);
  ref_mundo.x = cosf(est->config.declinacao_rad) * horizontal;
  ref_mundo.y = sinf(est->config.declinacao_rad) * horizontal;
  ref_mundo.z = m_mundo.z;

  /* Volta ao corpo e corrige o mesmo eixo que o erro aponta. */
  w = quat_rotaciona_vetor(q_conj, ref_mundo);
  erro = vetor3_produto_vetorial(m, w);
  omega.x = est->config.ganho_mag * erro.x;
  omega.y = est->config.ganho_mag * erro.y;
  omega.z = est->config.ganho_mag * erro.z;

  qd = quat_multiplica(est->q, omega);
  qd.w *= 0.5f;
  qd.x *= 0.5f;
  qd.y *= 0.5f;
  qd.z *= 0.5f;

  q_novo.w = est->q.w + qd.w * dt;
  q_novo.x = est->q.x + qd.x * dt;
  q_novo.y = est->q.y + qd.y * dt;
  q_novo.z = est->q.z + qd.z * dt;

  est->q = quat_normaliza(q_novo);
  est->contador++;
}

vetor3_t estimador_atitude_obtem_euler(const estimador_atitude_t *est)
{
  return quat_para_euler(est->q);
}

quat_t estimador_atitude_obtem_quat(const estimador_atitude_t *est)
{
  return est->q;
}
