/**
 * @file    vector3.h
 * @brief   Operações com vetores 3D
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_MATH_VECTOR3_H
#define USR_MATH_VECTOR3_H

#include "math/math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

vetor3_t vetor3_zero(void);

vetor3_t vetor3_soma(vetor3_t a, vetor3_t b);

vetor3_t vetor3_subtrai(vetor3_t a, vetor3_t b);

vetor3_t vetor3_escala(vetor3_t v, float escala);

/** Produto escalar (dot product). */
float vetor3_produto_escalar(vetor3_t a, vetor3_t b);

/** Produto vetorial (cross product). */
vetor3_t vetor3_produto_vetorial(vetor3_t a, vetor3_t b);

/** Norma (magnitude) do vetor. */
float vetor3_norma(vetor3_t v);

/**
 * @brief  Retorna o vetor normalizado (magnitude 1).
 * @note   Retorna vetor zero se a norma for nula.
 */
vetor3_t vetor3_normaliza(vetor3_t v);

/**
 * @brief  Rotaciona o vetor v pela rotação definida pelo quaternion q.
 */
vetor3_t vetor3_rotaciona_por_quat(vetor3_t v, quat_t q);

#ifdef __cplusplus
}
#endif

#endif /* USR_MATH_VECTOR3_H */
