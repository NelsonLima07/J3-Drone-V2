/**
 * @file    dshot_timer_hal.c
 * @brief   Saida DShot600 pelos 4 ESCs (TIM1 PA8..PA11 + GPDMA1 CH3..CH6).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Arquitetura:
 *   - TIM1 em PWM Mode 1, ARR = 332 (333 ticks @ 200 MHz = 600,6 kHz).
 *   - Cada canal usa um DMA GPDMA1 em NORMAL: 20 palavras (16 bits + 4 de
 *     pausa) escritas em CCRx a cada comparacao (1 request por periodo).
 *   - Ao terminar (fim do quadro), HAL_TIM_PWM_PulseFinishedCallback re-arma
 *     o canal apontando para o mesmo buffer (dshot_ativo), que foi copiado
 *     do dshot_sombra dentro da janela de pausa.
 *   - dshot_escreve roda no contexto do controle (8 kHz, prio 5) e o callback
 *     de fim de quadro tambem e prio 5: sem preempcao entre eles, a copia
 *     sombra -> ativo e atomica (e ocorre na pausa de 4 bits, sem leitura DMA).
 *
 * Nota H5: os callbacks de DMA do HAL TIM resolvem htim via hdma->Parent
 * (o HAL nunca o atribui), portanto o Parent de cada canal e setado aqui.
 */

#include "esc/dshot_timer_hal.h"

#include "esc/dshot.h"
#include "stm32h5xx_hal.h"

#define DSHOT_BIT_PERIODO  333u /* 200 MHz / 600 kHz = 333 ticks -> ARR = 332 */
#define DSHOT_CCR_ZERO     125u /* bit "0" = 625 ns  */
#define DSHOT_CCR_UM       250u /* bit "1" = 1250 ns */
#define DSHOT_FRAMES_ARM   48u  /* frames zero na borda de armamento (BLHeli) */

static TIM_HandleTypeDef htim1;

static DMA_HandleTypeDef hdma_tim1_ch1;
static DMA_HandleTypeDef hdma_tim1_ch2;
static DMA_HandleTypeDef hdma_tim1_ch3;
static DMA_HandleTypeDef hdma_tim1_ch4;

static uint32_t dshot_sombra[MIXER_NUM_MOTORES][DSHOT_BUFFER_TAMANHO];
static uint32_t dshot_ativo[MIXER_NUM_MOTORES][DSHOT_BUFFER_TAMANHO];

/* Escritos apenas no contexto da ISR do controle (prio 5). */
static volatile uint8_t dshot_sombra_pronta;
static volatile uint8_t dshot_armado;
static volatile uint8_t dshot_frames_zero;

/* --- Quadro DShot pronto para o DMA (valor -> buffer de CCR) --------------- */

static void dshot_quadro_ccr(uint16_t valor, uint32_t buffer[DSHOT_BUFFER_TAMANHO])
{
  uint16_t bits[DSHOT_FRAME_BITS];

  dshot_quadro(valor, bits);
  dshot_buffer_ccr(bits, DSHOT_CCR_ZERO, DSHOT_CCR_UM, buffer);
}

/* --- GPIO: PA8..PA11 em AF1 (TIM1) ----------------------------------------- */

static void dshot_gpio_configura(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &gpio);
}

/* --- TIM1: time base + PWM Mode 1 nos 4 canais ------------------------------ */

static int dshot_timer_configura(void)
{
  TIM_OC_InitTypeDef oc = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = DSHOT_BIT_PERIODO - 1u;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    return -1;
  }

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/* --- DMA: GPDMA1 CH3..CH6, memoria -> CCRx --------------------------------- */

static int dshot_dma_configura(DMA_HandleTypeDef *hdma,
                               DMA_Channel_TypeDef *canal,
                               uint32_t request)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  hdma->Instance = canal;
  hdma->Init.Request = request;
  hdma->Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma->Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma->Init.DestInc = DMA_DINC_FIXED;
  hdma->Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
  hdma->Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
  hdma->Init.Priority = DMA_HIGH_PRIORITY;
  hdma->Init.SrcBurstLength = 1;
  hdma->Init.DestBurstLength = 1;
  hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
  hdma->Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma->Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(hdma) != HAL_OK)
  {
    return -1;
  }
  if (HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/* --- API publica ------------------------------------------------------------ */

int dshot_timer_inicializa(void)
{
  uint16_t i;
  uint16_t j;

  dshot_gpio_configura();
  if (dshot_timer_configura() != 0)
  {
    return -1;
  }

  if (dshot_dma_configura(&hdma_tim1_ch1, GPDMA1_Channel3, GPDMA1_REQUEST_TIM1_CH1) != 0 ||
      dshot_dma_configura(&hdma_tim1_ch2, GPDMA1_Channel4, GPDMA1_REQUEST_TIM1_CH2) != 0 ||
      dshot_dma_configura(&hdma_tim1_ch3, GPDMA1_Channel5, GPDMA1_REQUEST_TIM1_CH3) != 0 ||
      dshot_dma_configura(&hdma_tim1_ch4, GPDMA1_Channel6, GPDMA1_REQUEST_TIM1_CH4) != 0)
  {
    return -1;
  }

  /* Parent: callbacks do HAL TIM resolvem htim via hdma->Parent. */
  hdma_tim1_ch1.Parent = &htim1;
  hdma_tim1_ch2.Parent = &htim1;
  hdma_tim1_ch3.Parent = &htim1;
  hdma_tim1_ch4.Parent = &htim1;

  htim1.hdma[TIM_DMA_ID_CC1] = &hdma_tim1_ch1;
  htim1.hdma[TIM_DMA_ID_CC2] = &hdma_tim1_ch2;
  htim1.hdma[TIM_DMA_ID_CC3] = &hdma_tim1_ch3;
  htim1.hdma[TIM_DMA_ID_CC4] = &hdma_tim1_ch4;

  HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);
  HAL_NVIC_SetPriority(GPDMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel6_IRQn);

  /* Frames zero (desarmado) antes de ligar o DMA. */
  for (i = 0; i < MIXER_NUM_MOTORES; ++i)
  {
    dshot_quadro_ccr(0u, dshot_ativo[i]);
    dshot_quadro_ccr(0u, dshot_sombra[i]);
    for (j = 0; j < DSHOT_BUFFER_TAMANHO; ++j)
    {
      dshot_ativo[i][j] = dshot_sombra[i][j];
    }
  }
  dshot_sombra_pronta = 0;
  dshot_armado = 0;
  dshot_frames_zero = 0;

  if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, dshot_ativo[0], DSHOT_BUFFER_TAMANHO) != HAL_OK ||
      HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, dshot_ativo[1], DSHOT_BUFFER_TAMANHO) != HAL_OK ||
      HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, dshot_ativo[2], DSHOT_BUFFER_TAMANHO) != HAL_OK ||
      HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, dshot_ativo[3], DSHOT_BUFFER_TAMANHO) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

void dshot_escreve(const saida_misturador_t *saida, uint8_t armado)
{
  uint16_t i;

  if (armado && !dshot_armado)
  {
    dshot_frames_zero = DSHOT_FRAMES_ARM;
  }
  dshot_armado = (uint8_t)(armado ? 1u : 0u);

  for (i = 0; i < MIXER_NUM_MOTORES; ++i)
  {
    if (armado && dshot_frames_zero == 0u)
    {
      dshot_buffer_motor(saida->motores[i], DSHOT_CCR_ZERO, DSHOT_CCR_UM,
                         dshot_sombra[i]);
    }
    else
    {
      dshot_quadro_ccr(0u, dshot_sombra[i]);
    }
  }

  if (dshot_frames_zero > 0u)
  {
    --dshot_frames_zero;
  }
  dshot_sombra_pronta = 1;
}

/* --- Callbacks HAL ----------------------------------------------------------- */

static uint32_t dshot_canal_para_indice(uint32_t canal)
{
  switch (canal)
  {
    case TIM_CHANNEL_2: return 1u;
    case TIM_CHANNEL_3: return 2u;
    case TIM_CHANNEL_4: return 3u;
    case TIM_CHANNEL_1:
    default:            return 0u;
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t canal = htim->Channel;
  uint32_t idx = dshot_canal_para_indice(canal);
  uint16_t i;
  uint16_t j;

  /* Copia o ultimo quadro escrito pelo controle durante a pausa (4 bits),
     quando todos os canais estao entre frames (sem DMA lendo o ativo). */
  if (dshot_sombra_pronta)
  {
    for (i = 0; i < MIXER_NUM_MOTORES; ++i)
    {
      for (j = 0; j < DSHOT_BUFFER_TAMANHO; ++j)
      {
        dshot_ativo[i][j] = dshot_sombra[i][j];
      }
    }
    dshot_sombra_pronta = 0;
  }

  /* O HAL TIM deixa o canal em BUSY ao iniciar; o fim de quadro nao volta
     para READY (so o erro faz isso). Re-arma explicitamente. */
  TIM_CHANNEL_STATE_SET(htim, canal, HAL_TIM_CHANNEL_STATE_READY);
  (void)HAL_TIM_PWM_Start_DMA(htim, canal, dshot_ativo[idx], DSHOT_BUFFER_TAMANHO);
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  uint32_t canal = htim->Channel;
  uint32_t idx = dshot_canal_para_indice(canal);

  /* TIM_DMAError ja devolveu o canal para READY; re-arma com o ultimo quadro. */
  (void)HAL_TIM_PWM_Start_DMA(htim, canal, dshot_ativo[idx], DSHOT_BUFFER_TAMANHO);
}

/* --- IRQ handlers (weak no startup; ausentes no stm32h5xx_it.c) ------------- */

void GPDMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}

void GPDMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch2);
}

void GPDMA1_Channel5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch3);
}

void GPDMA1_Channel6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim1_ch4);
}
