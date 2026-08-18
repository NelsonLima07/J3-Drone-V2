/**
 * @file    i3c_bmp581_hal.c
 * @brief   Transporte I3C1 (interrupt) para BMP581 com IBI.
 * @date    2026-08-17
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include "sensors/i3c_bmp581_hal.h"

#include "i3c.h"
#include "stm32h5xx_hal.h"

#define I3C_BMP581_TIMEOUT_MS  50U

static volatile uint8_t ibi_flag;

/**
 * @brief  Implementa imuc_transport_t.ler via I3C private read (IT).
 *
 * Protocolo I3C: envia o endereco do registrador (1 byte write) em
 * modo restart e le @p n bytes (HAL_I3C_DIRECTION_BOTH).
 */
static int i3c_bmp581_ler(void *ctx, uint8_t reg, uint8_t *dados, uint16_t n)
{
  I3C_PrivateTypeDef desc;
  I3C_XferTypeDef xfer;
  uint32_t ctrl_buf[4];
  uint32_t status_buf[1];
  HAL_StatusTypeDef st;
  uint32_t tick;

  (void)ctx;

  /* Preenche descritor privado: write reg + read n bytes. */
  desc.TargetAddr = BMP581_I3C_ADDR;
  desc.TxBuf.pBuffer = &reg;
  desc.TxBuf.Size = 1U;
  desc.RxBuf.pBuffer = dados;
  desc.RxBuf.Size = (uint32_t)n;
  desc.Direction = HAL_I3C_DIRECTION_BOTH;

  /* Preenche buffers de controle/estatus no XferData. */
  xfer.CtrlBuf.pBuffer = ctrl_buf;
  xfer.CtrlBuf.Size = sizeof(ctrl_buf) / sizeof(ctrl_buf[0]);
  xfer.StatusBuf.pBuffer = status_buf;
  xfer.StatusBuf.Size = sizeof(status_buf) / sizeof(status_buf[0]);
  xfer.TxBuf.pBuffer = 0;
  xfer.TxBuf.Size = 0;
  xfer.RxBuf.pBuffer = 0;
  xfer.RxBuf.Size = 0;

  /* Monta o frame I3C (restart entre write e read). */
  st = HAL_I3C_AddDescToFrame(&hi3c1, 0, &desc, &xfer, 1U,
                               I3C_PRIVATE_WITH_ARB_RESTART);
  if (st != HAL_OK)
  {
    return -1;
  }

  /* Inicia a transferencia (fase de leitura). */
  st = HAL_I3C_Ctrl_Receive_IT(&hi3c1, &xfer);
  if (st != HAL_OK)
  {
    return -2;
  }

  /* Aguarda conclusao. */
  tick = HAL_GetTick();
  while (hi3c1.State != HAL_I3C_STATE_READY)
  {
    if ((HAL_GetTick() - tick) > I3C_BMP581_TIMEOUT_MS)
    {
      return -3;
    }
  }

  return 0;
}

/**
 * @brief  Implementa imuc_transport_t.escrever via I3C private write (IT).
 *
 * Envia reg + dados em modo stop.
 */
static int i3c_bmp581_escrever(void *ctx, uint8_t reg, const uint8_t *dados,
                               uint16_t n)
{
  I3C_PrivateTypeDef desc;
  I3C_XferTypeDef xfer;
  uint32_t ctrl_buf[4];
  uint32_t status_buf[1];
  uint8_t buf[8];
  uint16_t i;
  HAL_StatusTypeDef st;
  uint32_t tick;

  (void)ctx;

  /* Monta: reg + dados (max 7 bytes). */
  if (n > 7u)
  {
    return -1;
  }
  buf[0] = reg;
  for (i = 0; i < n; ++i)
  {
    buf[i + 1u] = dados[i];
  }

  desc.TargetAddr = BMP581_I3C_ADDR;
  desc.TxBuf.pBuffer = buf;
  desc.TxBuf.Size = (uint32_t)(n + 1u);
  desc.RxBuf.pBuffer = 0;
  desc.RxBuf.Size = 0;
  desc.Direction = HAL_I3C_DIRECTION_WRITE;

  xfer.CtrlBuf.pBuffer = ctrl_buf;
  xfer.CtrlBuf.Size = sizeof(ctrl_buf) / sizeof(ctrl_buf[0]);
  xfer.StatusBuf.pBuffer = status_buf;
  xfer.StatusBuf.Size = sizeof(status_buf) / sizeof(status_buf[0]);
  xfer.TxBuf.pBuffer = 0;
  xfer.TxBuf.Size = 0;
  xfer.RxBuf.pBuffer = 0;
  xfer.RxBuf.Size = 0;

  st = HAL_I3C_AddDescToFrame(&hi3c1, 0, &desc, &xfer, 1U,
                               I3C_PRIVATE_WITH_ARB_STOP);
  if (st != HAL_OK)
  {
    return -2;
  }

  st = HAL_I3C_Ctrl_Transmit_IT(&hi3c1, &xfer);
  if (st != HAL_OK)
  {
    return -3;
  }

  tick = HAL_GetTick();
  while (hi3c1.State != HAL_I3C_STATE_READY)
  {
    if ((HAL_GetTick() - tick) > I3C_BMP581_TIMEOUT_MS)
    {
      return -4;
    }
  }

  return 0;
}

static void i3c_bmp581_atraso_us(void *ctx, uint32_t us)
{
  volatile uint32_t i;
  (void)ctx;
  for (i = 0; i < us; ++i)
  {
  }
}

int i3c_bmp581_hal_vincula(imuc_transport_t *t)
{
  uint8_t enec_payload;
  I3C_CCCTypeDef ccc;
  I3C_XferTypeDef xfer;
  uint32_t ctrl_buf[4];
  uint32_t status_buf[1];
  HAL_StatusTypeDef st;
  uint32_t tick;

  if (t == 0)
  {
    return -1;
  }

  ibi_flag = 0;

  /* ENEC broadcast: habilita IBI (defining byte = 0x02, bit 1 = IBI). */
  enec_payload = 0x02U;
  ccc.TargetAddr = 0x00U;
  ccc.CCC = 0x00U; /* ENEC */
  ccc.CCCBuf.pBuffer = &enec_payload;
  ccc.CCCBuf.Size = 1U;
  ccc.Direction = HAL_I3C_DIRECTION_WRITE;

  xfer.CtrlBuf.pBuffer = ctrl_buf;
  xfer.CtrlBuf.Size = sizeof(ctrl_buf) / sizeof(ctrl_buf[0]);
  xfer.StatusBuf.pBuffer = status_buf;
  xfer.StatusBuf.Size = sizeof(status_buf) / sizeof(status_buf[0]);
  xfer.TxBuf.pBuffer = 0;
  xfer.TxBuf.Size = 0;
  xfer.RxBuf.pBuffer = 0;
  xfer.RxBuf.Size = 0;

  st = HAL_I3C_AddDescToFrame(&hi3c1, &ccc, 0, &xfer, 1U,
                               I3C_BROADCAST_WITH_DEFBYTE_STOP);
  if (st != HAL_OK)
  {
    return -2;
  }

  st = HAL_I3C_Ctrl_TransmitCCC_IT(&hi3c1, &xfer);
  if (st != HAL_OK)
  {
    return -3;
  }
  tick = HAL_GetTick();
  while (hi3c1.State != HAL_I3C_STATE_READY)
  {
    if ((HAL_GetTick() - tick) > I3C_BMP581_TIMEOUT_MS)
    {
      return -4;
    }
  }

  /* Vincula o transporte. */
  t->ctx = (void *)0;
  t->ler = i3c_bmp581_ler;
  t->escrever = i3c_bmp581_escrever;
  t->atraso_us = i3c_bmp581_atraso_us;
  return 0;
}

uint8_t i3c_bmp581_hal_ibi_pronto(void)
{
  uint8_t r = ibi_flag;
  ibi_flag = 0;
  return r;
}

/* Callback de notificacao I3C: chamado em contexto de interrupcao. */
void HAL_I3C_NotifyCallback(I3C_HandleTypeDef *hi3c, uint32_t eventId)
{
  if (hi3c->Instance == I3C1 && (eventId & EVENT_ID_IBI) != 0U)
  {
    ibi_flag = 1;
  }
}
