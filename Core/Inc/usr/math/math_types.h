/**
 * @file    math_types.h
 * @brief   Tipos fundamentais e constantes da biblioteca matemática
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#ifndef USR_MATH_TYPES_H
#define USR_MATH_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI_F            3.14159265358979323846f
#define TWO_PI_F        6.28318530717958647692f
#define DEG_TO_RAD_F    0.01745329251994329577f
#define RAD_TO_DEG_F    57.29577951308232087680f
#define EPS_F           1.0e-6f

/* Vetor 3D em ponto flutuante (float32) */
typedef struct {
  float x;
  float y;
  float z;
} vetor3_t;

/* Quaternion de atitude: q = w + x*i + y*j + z*k (w é o escalar) */
typedef struct {
  float w;
  float x;
  float y;
  float z;
} quat_t;

/* Matriz 3x3 em linha-major: m[linha][coluna]; multiplica vetor coluna */
typedef struct {
  float m[3][3];
} matriz3_t;

#ifdef __cplusplus
}
#endif

#endif /* USR_MATH_TYPES_H */
