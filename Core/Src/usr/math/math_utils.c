/**
 * @file    math_utils.c
 * @brief   Utilidades matemáticas gerais
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "math/math_utils.h"
#include "math/math_types.h"

#include <math.h>

float limita(float valor, float minimo, float maximo)
{
  if (valor < minimo)
    return minimo;
  if (valor > maximo)
    return maximo;
  return valor;
}

int limita_int(int valor, int minimo, int maximo)
{
  if (valor < minimo)
    return minimo;
  if (valor > maximo)
    return maximo;
  return valor;
}

float envolve_pi(float angulo)
{
  while (angulo > PI_F)
    angulo -= TWO_PI_F;
  while (angulo < -PI_F)
    angulo += TWO_PI_F;
  return angulo;
}

float envolve_2pi(float angulo)
{
  while (angulo >= TWO_PI_F)
    angulo -= TWO_PI_F;
  while (angulo < 0.0f)
    angulo += TWO_PI_F;
  return angulo;
}

float zona_morta(float valor, float limite)
{
  if ((valor > -limite) && (valor < limite))
    return 0.0f;
  return valor;
}

float lerp(float a, float b, float t)
{
  return a + (b - a) * t;
}

float menor_f(float a, float b)
{
  return (a < b) ? a : b;
}

float maior_f(float a, float b)
{
  return (a > b) ? a : b;
}

int arredonda(float valor)
{
  return (int)floorf(valor + 0.5f);
}

float raiz_inversa_rapida(float x)
{
  return 1.0f / sqrtf(x);
}

/* Aproximação minimax de atan(a) para a em [0, 1], erro máximo < 1e-5 rad. */
static float atan_aprox(float a)
{
  const float c1 = 0.99997726f;
  const float c3 = -0.33262347f;
  const float c5 = 0.19354346f;
  const float c7 = -0.11643287f;
  const float c9 = 0.05265332f;
  const float c11 = -0.01172120f;
  float a2 = a * a;
  return a * (c1 + a2 * (c3 + a2 * (c5 + a2 * (c7 + a2 * (c9 + a2 * c11)))));
}

float atan2_rapido(float y, float x)
{
  float ax = fabsf(x);
  float ay = fabsf(y);
  float angulo;

  if ((ax == 0.0f) && (ay == 0.0f))
    return 0.0f;

  if (ax >= ay)
  {
    angulo = atan_aprox(ay / ax);
    if (x < 0.0f)
      angulo = PI_F - angulo;
  }
  else
  {
    angulo = (PI_F * 0.5f) - atan_aprox(ax / ay);
    if (x < 0.0f)
      angulo = PI_F - angulo;
  }

  if (y < 0.0f)
    angulo = -angulo;

  return angulo;
}

float asin_rapido(float x)
{
  float y = limita(x, -1.0f, 1.0f);
  return atan2_rapido(y, sqrtf((1.0f - y) * (1.0f + y)));
}
