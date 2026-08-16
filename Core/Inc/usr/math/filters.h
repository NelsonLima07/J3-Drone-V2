/**
 * @file    filters.h
 * @brief   Filtros digitais para o caminho de sensores e controle
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_MATH_FILTERS_H
#define USR_MATH_FILTERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FILTRO_MEDIA_MOVEL_MAX 64u

/* Filtro passa-baixa de 1ª ordem (EMA discretizado por bilinear) */
typedef struct {
  float alfa;
  float saida;
} filtro_lpf1_t;

/* Biquad (2ª ordem) em forma direta 1: H(z) = (b0 + b1 z^-1 + b2 z^-2)/(1 + a1 z^-1 + a2 z^-2) */
typedef struct {
  float b0, b1, b2;
  float a1, a2;
  float z1, z2;
} filtro_biquad_t;

/* Média móvel com buffer fixo (sem alocação dinâmica) */
typedef struct {
  float soma;
  float amostras[FILTRO_MEDIA_MOVEL_MAX];
  uint32_t n;
  uint32_t indice;
} filtro_media_movel_t;

/* Filtro complementar: mistura fonte rápida e fonte lenta */
typedef struct {
  float coeficiente;
  float saida;
} filtro_complementar_t;

void filtro_lpf1_inicializa(filtro_lpf1_t *f, float fc, float dt);

float filtro_lpf1_aplica(filtro_lpf1_t *f, float amostra);

void filtro_lpf1_reseta(filtro_lpf1_t *f);

/** Passa-baixa de 2ª ordem (Butterworth, q tipicamente 0.7071). */
void filtro_biquad_inicializa_lpf2(filtro_biquad_t *f, float fc, float q, float dt);

/** Passa-alta de 2ª ordem. */
void filtro_biquad_inicializa_hpf(filtro_biquad_t *f, float fc, float q, float dt);

/** Rejeita banda (notch) para eliminar vibração de hélices/motores. */
void filtro_biquad_inicializa_notch(filtro_biquad_t *f, float fc, float q, float dt);

float filtro_biquad_aplica(filtro_biquad_t *f, float amostra);

void filtro_biquad_reseta(filtro_biquad_t *f);

void filtro_media_movel_inicializa(filtro_media_movel_t *f, uint32_t n);

float filtro_media_movel_aplica(filtro_media_movel_t *f, float amostra);

void filtro_complementar_inicializa(filtro_complementar_t *f, float coeficiente);

float filtro_complementar_aplica(filtro_complementar_t *f, float fonte_rapida, float fonte_lenta);

void filtro_complementar_reseta(filtro_complementar_t *f);

#ifdef __cplusplus
}
#endif

#endif /* USR_MATH_FILTERS_H */
