/**
 * @file    gps_led.c
 * @brief   LED dedicado de status do GPS (PB2).
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * PORTAS (ver docs/pinout_map.md - secao LED GPS):
 *   - LED GPS em PB2 (GPIO livre), ativo alto. Acesso direto ao GPIO.
 */

#include "system/gps_led.h"

#include "main.h"

#define GPS_LED_PERIODO_MS 500U  /* 2 Hz sem fix */
#define GPS_LED_MEIO_MS    250U

void gps_led_inicializa(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void gps_led_atualiza(uint8_t gps_ok, uint32_t agora_ms)
{
  uint32_t fase;
  uint8_t aceso;

  if (gps_ok)
  {
    aceso = 1;
  }
  else
  {
    fase = agora_ms % GPS_LED_PERIODO_MS;
    aceso = (fase < GPS_LED_MEIO_MS) ? 1 : 0;
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, aceso ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
