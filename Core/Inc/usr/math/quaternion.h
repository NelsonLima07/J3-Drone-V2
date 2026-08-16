/**
 * @file    quaternion.h
 * @brief   Operações com quaternions para atitude
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_MATH_QUATERNION_H
#define USR_MATH_QUATERNION_H

#include "math/math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Quaternion identidade (rotação nula). */
quat_t quat_identidade(void);

/** Conjugado: q* = (w, -x, -y, -z). Para quaternions unitários equivale à inversa. */
quat_t quat_conjuga(quat_t q);

/** Norma do quaternion. */
float quat_norma(quat_t q);

/** Retorna o quaternion normalizado (magnitude 1). */
quat_t quat_normaliza(quat_t q);

/** Produto de Hamilton p * q (aplica q primeiro e depois p). */
quat_t quat_multiplica(quat_t p, quat_t q);

/** Rotaciona o vetor v pela rotação q (v' = q * v * q*). */
vetor3_t quat_rotaciona_vetor(quat_t q, vetor3_t v);

/**
 * @brief  Cria quaternion a partir de ângulos de Euler (rad), ordem ZYX
 *         (guinada, arfagem, rol).
 */
quat_t quat_de_euler(float rol, float arfagem, float guinada);

/**
 * @brief  Extrai ângulos de Euler (rad) do quaternion.
 * @return vetor3_t com {rol, arfagem, guinada}.
 */
vetor3_t quat_para_euler(quat_t q);

/**
 * @brief  Cria quaternion a partir de eixo de rotação (normalizado) e ângulo (rad).
 */
quat_t quat_de_eixo_angulo(vetor3_t eixo, float angulo);

/** Rota o vetor v (não normalizado) pela rotação dada por eixo/ângulo. */
quat_t quat_de_dois_vetores(vetor3_t de, vetor3_t para);

/**
 * @brief  Interpolação esférica entre q0 e q1 com fator t em [0, 1].
 */
quat_t quat_slerp(quat_t q0, quat_t q1, float t);

/** Diferença relativa entre duas atitudes (q0^-1 * q1). */
quat_t quat_diferenca(quat_t q0, quat_t q1);

#ifdef __cplusplus
}
#endif

#endif /* USR_MATH_QUATERNION_H */
