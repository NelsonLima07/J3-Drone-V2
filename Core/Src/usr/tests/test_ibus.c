/**
 * @file    test_ibus.c
 * @brief   Testes nativos do decodificador iBus
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>
#include "serial/ibus.h"

int testes_ibus(void)
{
  int falhas = 0;
  uint8_t frame[IBUS_FRAME_TAMANHO];
  uint16_t canais[IBUS_NUM_CANAIS];
  uint16_t valor;
  uint16_t soma;
  uint16_t checksum;
  int i;
  int r;

  /* Quadro valido: sync + 14 canais (LE) + checksum (0xFFFF - soma). */
  frame[0] = IBUS_SYNC_BYTE;
  frame[1] = IBUS_SYNC2_BYTE;
  for (i = 0; i < IBUS_NUM_CANAIS; ++i)
  {
    valor = (uint16_t)(1000 + i * 57);
    frame[2 + 2 * i] = (uint8_t)(valor & 0xFF);
    frame[3 + 2 * i] = (uint8_t)(valor >> 8);
  }
  soma = 0;
  for (i = 0; i < IBUS_FRAME_TAMANHO - 2; ++i)
  {
    soma += frame[i];
  }
  checksum = (uint16_t)(0xFFFFU - soma);
  frame[IBUS_FRAME_TAMANHO - 2] = (uint8_t)(checksum & 0xFF);
  frame[IBUS_FRAME_TAMANHO - 1] = (uint8_t)(checksum >> 8);

  r = ibus_decodifica(frame, canais);
  if (r != 0)
  {
    printf("FALHA: quadro valido rejeitado (r=%d)\n", r);
    falhas++;
  }
  for (i = 0; i < IBUS_NUM_CANAIS; ++i)
  {
    if (canais[i] != (uint16_t)(1000 + i * 57))
    {
      printf("FALHA: canal %d = %u, esperado %u\n",
             i, (unsigned)canais[i], (unsigned)(1000 + i * 57));
      falhas++;
    }
  }

  /* Checksum errado deve ser rejeitado. */
  frame[IBUS_FRAME_TAMANHO - 1] ^= 0x01;
  if (ibus_decodifica(frame, canais) == 0)
  {
    printf("FALHA: checksum invalido aceito\n");
    falhas++;
  }
  frame[IBUS_FRAME_TAMANHO - 1] ^= 0x01;

  /* Sync errado deve ser rejeitado. */
  frame[0] = 0x99;
  if (ibus_decodifica(frame, canais) == 0)
  {
    printf("FALHA: sync invalido aceito\n");
    falhas++;
  }
  frame[0] = IBUS_SYNC_BYTE;

  /* Argumentos nulos. */
  if (ibus_decodifica(0, canais) == 0)
  {
    printf("FALHA: bytes nulo aceito\n");
    falhas++;
  }
  if (ibus_decodifica(frame, 0) == 0)
  {
    printf("FALHA: canais nulo aceito\n");
    falhas++;
  }

  printf("testes_ibus: %d falha(s)\n", falhas);
  return falhas;
}
