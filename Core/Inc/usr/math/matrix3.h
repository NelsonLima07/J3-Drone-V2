/**
 * @file    matrix3.h
 * @brief   Operações com matrizes 3x3 (linha-major)
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_MATH_MATRIX3_H
#define USR_MATH_MATRIX3_H

#include "math/math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

matriz3_t matriz3_identidade(void);

/** Multiplicação de matrizes A * B. */
matriz3_t matriz3_multiplica(matriz3_t a, matriz3_t b);

/** Multiplicação de matriz por vetor (vetor coluna). */
vetor3_t matriz3_multiplica_vetor(matriz3_t a, vetor3_t v);

matriz3_t matriz3_transposta(matriz3_t a);

float matriz3_determinante(matriz3_t a);

/**
 * @brief  Inversa geral via cofatores.
 * @return Matriz identidade se o determinante for nulo.
 */
matriz3_t matriz3_inversa(matriz3_t a);

/** Matriz de rotação equivalente ao quaternion q. */
matriz3_t matriz3_de_quat(quat_t q);

/** Quaternion equivalente à matriz de rotação a. */
quat_t matriz3_para_quat(matriz3_t a);

#ifdef __cplusplus
}
#endif

#endif /* USR_MATH_MATRIX3_H */
