/**
 * @file    math_utils.h
 * @brief   Utilidades matemáticas gerais (limites, envoltória angular,
 *          zona morta, interpolação, aproximações rápidas)
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_MATH_UTILS_H
#define USR_MATH_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Limita um valor em ponto flutuante ao intervalo [min, max].
 */
float limita(float valor, float minimo, float maximo);

/**
 * @brief  Limita um valor inteiro ao intervalo [min, max].
 */
int limita_int(int valor, int minimo, int maximo);

/**
 * @brief  Envolve um ângulo (rad) ao intervalo [-pi, pi].
 */
float envolve_pi(float angulo);

/**
 * @brief  Envolve um ângulo (rad) ao intervalo [0, 2*pi).
 */
float envolve_2pi(float angulo);

/**
 * @brief  Zona morta simétrica: retorna 0 se |valor| < limite.
 */
float zona_morta(float valor, float limite);

/**
 * @brief  Interpolação linear entre a e b com fator t em [0, 1].
 */
float lerp(float a, float b, float t);

/**
 * @brief  Retorna o menor entre dois valores.
 */
float menor_f(float a, float b);

/**
 * @brief  Retorna o maior entre dois valores.
 */
float maior_f(float a, float b);

/**
 * @brief  Arredonda para o inteiro mais próximo.
 */
int arredonda(float valor);

/**
 * @brief  Raiz inversa rápida (1/sqrt(x)). No Cortex-M33 com FPU o sqrtf
 *         é implementado em hardware, mantendo precisão total.
 */
float raiz_inversa_rapida(float x);

/**
 * @brief  Aproximação rápida de arco tangente de dois argumentos atan2(y, x)
 *         com erro típico < 1e-4 rad. Usada em caminhos quentes (8 kHz).
 */
float atan2_rapido(float y, float x);

/**
 * @brief  Aproximação rápida de arco seno com erro típico < 1e-4 rad.
 */
float asin_rapido(float x);

#ifdef __cplusplus
}
#endif

#endif /* USR_MATH_UTILS_H */
