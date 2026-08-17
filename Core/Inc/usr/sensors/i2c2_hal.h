/**
 * @file    i2c2_hal.h
 * @brief   Transporte I2C2 (polling) para LIS3MDL e BMP280.
 * @date    2026-08-16
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 *
 * Glue somente firmware (usa HAL). Implementa imuc_transport_t sobre
 * a I2C2 em modo polling, com timeout curto e recuperacao (DeInit +
 * Init) em caso de NACK/busy.
 */

#ifndef USR_SENSORS_I2C2_HAL_H
#define USR_SENSORS_I2C2_HAL_H

#include "sensors/imuc42688_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Vincula @p t ao barramento I2C2 para um dispositivo de
 *         endereco @p endereco (7 bits).
 * @param  endereco  endereco I2C 7 bits (ex.: LIS3MDL_ENDERECO).
 * @return 0 em caso de sucesso, < 0 em caso de argumento invalido.
 */
int i2c2_hal_vincula(imuc_transport_t *t, uint8_t endereco);

#ifdef __cplusplus
}
#endif

#endif /* USR_SENSORS_I2C2_HAL_H */
