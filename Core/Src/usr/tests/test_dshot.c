/**
 * @file    test_dshot.c
 * @brief   Testes nativos da codificacao DShot
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include "esc/dshot.h"

int testes_dshot(void)
{
  int falhas = 0;
  uint16_t bits[DSHOT_FRAME_BITS];
  uint32_t buffer[DSHOT_BUFFER_TAMANHO];
  uint16_t esperado[DSHOT_FRAME_BITS];
  const uint32_t ccr_zero = 125u;
  const uint32_t ccr_um = 250u;
  uint16_t valor;
  int i;
  int r;

  /* CRC4: valor 11 -> 7 (vetor canonico), 0x7FF -> 0xE, 0x400 -> 8, 0 -> 0. */
  if (dshot_crc4(11u) != 0x7u)
  {
    printf("FALHA: crc4(11) = %u, esperado 7\n", (unsigned)dshot_crc4(11u));
    falhas++;
  }
  if (dshot_crc4(0x7FFu) != 0xEu)
  {
    printf("FALHA: crc4(0x7FF) = %u, esperado 14\n", (unsigned)dshot_crc4(0x7FFu));
    falhas++;
  }
  if (dshot_crc4(0x400u) != 0x8u)
  {
    printf("FALHA: crc4(0x400) = %u, esperado 8\n", (unsigned)dshot_crc4(0x400u));
    falhas++;
  }
  if (dshot_crc4(0x000u) != 0x0u)
  {
    printf("FALHA: crc4(0) = %u, esperado 0\n", (unsigned)dshot_crc4(0u));
    falhas++;
  }

  /* Quadro de valor 11: 0000000101100111 (exemplo da spec). */
  {
    const uint16_t v11[DSHOT_FRAME_BITS] = {0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1};
    dshot_quadro(11u, bits);
    r = 0;
    for (i = 0; i < DSHOT_FRAME_BITS; ++i)
    {
      if (bits[i] != v11[i])
      {
        r = 1;
      }
    }
    if (r != 0)
    {
      printf("FALHA: quadro(11) nao bate com o vetor da spec\n");
      falhas++;
    }
  }

  /* Quadro de valor 0: todos os bits zero (desarmado). */
  dshot_quadro(0u, bits);
  r = 0;
  for (i = 0; i < DSHOT_FRAME_BITS; ++i)
  {
    if (bits[i] != 0u)
    {
      r = 1;
    }
  }
  if (r != 0)
  {
    printf("FALHA: quadro(0) deve ser todo zero\n");
    falhas++;
  }

  /* Quadro de 0x7FF: 11 uns + stop 0 + crc 0xE (1110). */
  dshot_quadro(0x7FFu, bits);
  r = 0;
  for (i = 0; i < 11; ++i)
  {
    if (bits[i] != 1u)
    {
      r = 1;
    }
  }
  if (bits[11] != 0u || bits[12] != 1u || bits[13] != 1u || bits[14] != 1u || bits[15] != 0u)
  {
    r = 1;
  }
  if (r != 0)
  {
    printf("FALHA: quadro(0x7FF) incorreto\n");
    falhas++;
  }

  /* Buffer CCR: quadro todo zero -> 16 x ccr_zero, depois 4 x 0 (pausa). */
  dshot_quadro(0u, bits);
  dshot_buffer_ccr(bits, ccr_zero, ccr_um, buffer);
  r = 0;
  for (i = 0; i < DSHOT_FRAME_BITS; ++i)
  {
    if (buffer[i] != ccr_zero)
    {
      r = 1;
    }
  }
  for (i = DSHOT_FRAME_BITS; i < DSHOT_BUFFER_TAMANHO; ++i)
  {
    if (buffer[i] != 0u)
    {
      r = 1;
    }
  }
  if (r != 0)
  {
    printf("FALHA: buffer_ccr(quadro 0) incorreto\n");
    falhas++;
  }

  /* Buffer CCR espelha os bits (um -> ccr_um, zero -> ccr_zero). */
  for (i = 0; i < DSHOT_FRAME_BITS; ++i)
  {
    esperado[i] = (uint16_t)(i % 2u);
  }
  dshot_buffer_ccr(esperado, ccr_zero, ccr_um, buffer);
  r = 0;
  for (i = 0; i < DSHOT_FRAME_BITS; ++i)
  {
    if (buffer[i] != (esperado[i] ? ccr_um : ccr_zero))
    {
      r = 1;
    }
  }
  if (r != 0)
  {
    printf("FALHA: buffer_ccr nao espelha os bits\n");
    falhas++;
  }

  /* Mapeamento throttle: 0 -> 48, 1 -> 2047, 0.5 -> 1048. */
  if (dshot_valor_throttle(0.0f) != DSHOT_MIN_THROTTLE)
  {
    printf("FALHA: throttle(0) = %u, esperado 48\n", (unsigned)dshot_valor_throttle(0.0f));
    falhas++;
  }
  if (dshot_valor_throttle(1.0f) != DSHOT_MAX_THROTTLE)
  {
    printf("FALHA: throttle(1) = %u, esperado 2047\n", (unsigned)dshot_valor_throttle(1.0f));
    falhas++;
  }
  if (dshot_valor_throttle(0.5f) != 1048u)
  {
    printf("FALHA: throttle(0.5) = %u, esperado 1048\n", (unsigned)dshot_valor_throttle(0.5f));
    falhas++;
  }
  if (dshot_valor_throttle(2.0f) != DSHOT_MAX_THROTTLE ||
      dshot_valor_throttle(-1.0f) != DSHOT_MIN_THROTTLE)
  {
    printf("FALHA: throttle fora da faixa nao foi limitado\n");
    falhas++;
  }

  /* buffer_motor(0.5) deve gerar o quadro de 1048 = 0x418. */
  dshot_buffer_motor(0.5f, ccr_zero, ccr_um, buffer);
  valor = 1048u; /* 0x418 */
  dshot_quadro(valor, bits);
  r = 0;
  for (i = 0; i < DSHOT_FRAME_BITS; ++i)
  {
    if (buffer[i] != (bits[i] ? ccr_um : ccr_zero))
    {
      r = 1;
    }
  }
  for (i = DSHOT_FRAME_BITS; i < DSHOT_BUFFER_TAMANHO; ++i)
  {
    if (buffer[i] != 0u)
    {
      r = 1;
    }
  }
  if (r != 0)
  {
    printf("FALHA: buffer_motor(0.5) nao corresponde ao valor 1048\n");
    falhas++;
  }

  printf("testes_dshot: %d falha(s)\n", falhas);
  return falhas;
}
