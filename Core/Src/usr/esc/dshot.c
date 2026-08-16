/**
 * @file    dshot.c
 * @brief   Implementacao da codificacao DShot (puro).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "esc/dshot.h"

uint16_t dshot_crc4(uint16_t valor)
{
  uint16_t pacote = (uint16_t)(valor << 1);

  return (uint16_t)(((pacote >> 8U) ^ (pacote >> 4U) ^ pacote) & 0x0Fu);
}

void dshot_quadro(uint16_t valor, uint16_t bits[DSHOT_FRAME_BITS])
{
  uint16_t crc;
  uint16_t i;

  crc = dshot_crc4(valor);

  for (i = 0; i < 11u; ++i)
  {
    bits[i] = (uint16_t)((valor >> (10u - i)) & 1u);
  }
  bits[11] = 0u; /* stop bit */
  bits[12] = (uint16_t)((crc >> 3u) & 1u);
  bits[13] = (uint16_t)((crc >> 2u) & 1u);
  bits[14] = (uint16_t)((crc >> 1u) & 1u);
  bits[15] = (uint16_t)(crc & 1u);
}

void dshot_buffer_ccr(const uint16_t bits[DSHOT_FRAME_BITS],
                      uint32_t ccr_zero, uint32_t ccr_um,
                      uint32_t buffer[DSHOT_BUFFER_TAMANHO])
{
  uint16_t i;

  for (i = 0; i < DSHOT_FRAME_BITS; ++i)
  {
    buffer[i] = bits[i] ? ccr_um : ccr_zero;
  }
  for (i = DSHOT_FRAME_BITS; i < DSHOT_BUFFER_TAMANHO; ++i)
  {
    buffer[i] = 0u; /* pausa: CCR 0 mantem a linha em nivel baixo */
  }
}

uint16_t dshot_valor_throttle(float motor)
{
  float v;

  if (motor <= 0.0f)
  {
    v = 0.0f;
  }
  else if (motor >= 1.0f)
  {
    v = 1.0f;
  }
  else
  {
    v = motor;
  }

  return (uint16_t)(DSHOT_MIN_THROTTLE +
                    (uint16_t)(v * (float)(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) + 0.5f));
}

void dshot_buffer_motor(float motor, uint32_t ccr_zero, uint32_t ccr_um,
                        uint32_t buffer[DSHOT_BUFFER_TAMANHO])
{
  uint16_t valor = dshot_valor_throttle(motor);
  uint16_t bits[DSHOT_FRAME_BITS];

  dshot_quadro(valor, bits);
  dshot_buffer_ccr(bits, ccr_zero, ccr_um, buffer);
}
