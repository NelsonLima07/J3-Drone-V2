/**
 * @file    hardware_glue.h
 * @brief   Reparo em codigo da regressao do CubeMX no barramento do IMU e
 *          no iBus: PLL2 (kernel SPI1), DMA do SPI1 e da USART2.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Este modulo existe porque o regen do CubeMX (a partir do .ioc
 * serializado a mao) apagou: a configuracao do PLL2, o kernel SPI1=PLL2P,
 * os handles de DMA do SPI1 (GPDMA1 CH0/CH1), o DMA da USART2 (CH2) e os
 * IRQ handlers de SPI1/GPDMA1. Todo o reparo fica aqui, em codigo, e roda
 * antes de qualquer uso do IMU ou do iBus.
 */

#ifndef USR_SYSTEM_HARDWARE_GLUE_H
#define USR_SYSTEM_HARDWARE_GLUE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Habilita o PLL2, reconfigure o SPI1 (prescaler /2 com kernel
 *         PLL2P = 24 MHz), vincula o DMA do SPI1 (GPDMA1 CH0/CH1) e da
 *         USART2 (GPDMA1 CH2) e programa as prioridades do NVIC.
 * @note   Deve ser chamado antes de imuc_hal_vincular e ibus_uart_inicializa.
 */
void hardware_glue_inicializa(void);

#ifdef __cplusplus
}
#endif

#endif /* USR_SYSTEM_HARDWARE_GLUE_H */
