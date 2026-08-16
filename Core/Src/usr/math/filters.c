/**
 * @file    filters.c
 * @brief   Filtros digitais para o caminho de sensores e controle
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "math/filters.h"
#include "math/math_types.h"

#include <math.h>

void filtro_lpf1_inicializa(filtro_lpf1_t *f, float fc, float dt)
{
  float x = TWO_PI_F * fc * dt;

  f->alfa = x / (1.0f + x);
  f->saida = 0.0f;
}

float filtro_lpf1_aplica(filtro_lpf1_t *f, float amostra)
{
  f->saida += f->alfa * (amostra - f->saida);
  return f->saida;
}

void filtro_lpf1_reseta(filtro_lpf1_t *f)
{
  f->saida = 0.0f;
}

static void biquad_prepara(filtro_biquad_t *f, float b0, float b1, float b2,
                           float a0, float a1, float a2)
{
  f->b0 = b0 / a0;
  f->b1 = b1 / a0;
  f->b2 = b2 / a0;
  f->a1 = a1 / a0;
  f->a2 = a2 / a0;
  f->z1 = 0.0f;
  f->z2 = 0.0f;
}

void filtro_biquad_inicializa_lpf2(filtro_biquad_t *f, float fc, float q, float dt)
{
  float omega = TWO_PI_F * fc * dt;
  float c = cosf(omega);
  float alfa = sinf(omega) / (2.0f * q);
  float b0 = (1.0f - c) * 0.5f;
  float b1 = 1.0f - c;
  float b2 = (1.0f - c) * 0.5f;

  biquad_prepara(f, b0, b1, b2, 1.0f + alfa, -2.0f * c, 1.0f - alfa);
}

void filtro_biquad_inicializa_hpf(filtro_biquad_t *f, float fc, float q, float dt)
{
  float omega = TWO_PI_F * fc * dt;
  float c = cosf(omega);
  float alfa = sinf(omega) / (2.0f * q);
  float b0 = (1.0f + c) * 0.5f;
  float b1 = -(1.0f + c);
  float b2 = (1.0f + c) * 0.5f;

  biquad_prepara(f, b0, b1, b2, 1.0f + alfa, -2.0f * c, 1.0f - alfa);
}

void filtro_biquad_inicializa_notch(filtro_biquad_t *f, float fc, float q, float dt)
{
  float omega = TWO_PI_F * fc * dt;
  float c = cosf(omega);
  float alfa = sinf(omega) / (2.0f * q);

  biquad_prepara(f, 1.0f, -2.0f * c, 1.0f, 1.0f + alfa, -2.0f * c, 1.0f - alfa);
}

float filtro_biquad_aplica(filtro_biquad_t *f, float amostra)
{
  float saida = f->b0 * amostra + f->z1;
  f->z1 = f->b1 * amostra - f->a1 * saida + f->z2;
  f->z2 = f->b2 * amostra - f->a2 * saida;
  return saida;
}

void filtro_biquad_reseta(filtro_biquad_t *f)
{
  f->z1 = 0.0f;
  f->z2 = 0.0f;
}

void filtro_media_movel_inicializa(filtro_media_movel_t *f, uint32_t n)
{
  uint32_t i;

  f->n = (n > FILTRO_MEDIA_MOVEL_MAX) ? FILTRO_MEDIA_MOVEL_MAX : n;
  if (f->n == 0u)
    f->n = 1u;
  f->indice = 0u;
  f->soma = 0.0f;
  for (i = 0u; i < f->n; ++i)
    f->amostras[i] = 0.0f;
}

float filtro_media_movel_aplica(filtro_media_movel_t *f, float amostra)
{
  f->soma -= f->amostras[f->indice];
  f->soma += amostra;
  f->amostras[f->indice] = amostra;
  f->indice = (f->indice + 1u) % f->n;
  return f->soma / (float)f->n;
}

void filtro_complementar_inicializa(filtro_complementar_t *f, float coeficiente)
{
  f->coeficiente = coeficiente;
  f->saida = 0.0f;
}

float filtro_complementar_aplica(filtro_complementar_t *f, float fonte_rapida, float fonte_lenta)
{
  f->saida = f->coeficiente * fonte_rapida + (1.0f - f->coeficiente) * fonte_lenta;
  return f->saida;
}

void filtro_complementar_reseta(filtro_complementar_t *f)
{
  f->saida = 0.0f;
}
