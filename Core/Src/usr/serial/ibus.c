/**
 * @file    ibus.c
 * @brief   Decodificador do protocolo iBus (FlySky) em C puro.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "serial/ibus.h"

int ibus_decodifica(const uint8_t *bytes, uint16_t *canais)
{
  uint16_t soma = 0;
  uint16_t checksum;
  uint16_t i;

  if (bytes == 0 || canais == 0)
  {
    return -1;
  }

  if (bytes[0] != IBUS_SYNC_BYTE || bytes[1] != IBUS_SYNC2_BYTE)
  {
    return -2;
  }

  for (i = 0; i < IBUS_NUM_CANAIS; ++i)
  {
    canais[i] = (uint16_t)(bytes[2 + 2 * i] |
                           ((uint16_t)bytes[3 + 2 * i] << 8));
  }

  for (i = 0; i < (uint16_t)(IBUS_FRAME_TAMANHO - 2); ++i)
  {
    soma += bytes[i];
  }

  checksum = (uint16_t)(bytes[IBUS_FRAME_TAMANHO - 2] |
                        ((uint16_t)bytes[IBUS_FRAME_TAMANHO - 1] << 8));

  if ((uint16_t)(0xFFFFU - soma) != checksum)
  {
    return -3;
  }

  return 0;
}
