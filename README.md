# J3_DroneV2

Placa controladora de voo (flight controller) para drone FPV modo X, com firmware em C organizado em camadas sobre o STM32Cube HAL. O código de usuário é C puro, independente de hardware e testável no host.

## Hardware

| Componente | Especificação |
|---|---|
| MCU | STM32H562RGT6 (Cortex-M33, 200 MHz) |
| IMU | ICM-42688-P via SPI1 @24 MHz + GPDMA1 (ODR 8 kHz) |
| Receptor | FlySky iBus via USART2 + DMA (115200 bps) |
| ESCs | DShot600 via TIM1 CH1–CH4 + GPDMA1 |
| GPS | BN-220 (NMEA 9600 8N1) via USART3 + GPDMA1 CH7 |
| Magnetômetro | LIS3MDL (I2C2 @0x1C, ±4 gauss, 100 Hz) |
| Barômetro | BMP581 (I3C1 @0x46, pressão + temperatura, IBI) |
| LEDs | Modo (PB5), GPS fix (PB2) |
| Frame | Modo X, hélices 9x4.5, motores Sunnysky, bateria 3S |

## Arquitetura de software

Cadeia de controle de voo:

```
iBus (USART2+DMA) + GPS (USART3+DMA, 9600 NMEA)
   → radio_comandos (AETR → setpoint)
   → estimador_atitude (quaternion + mag → yaw, Mahony @8 kHz)
   → controlador (PID em cascata; interface p/ LADRC)
   → mixer (frame X)
   → dshot (DShot600, TIM1)

GPS + BMP581 (I3C1):
   → home_ponto (ponto de retorno)
   → nav_posicao/controle (navegação entre waypoints)
```

- Máquina de estados: `ESPERA` / `VOO` / `CALIBRAÇÃO`.
- Porta de armar: CH6 (modo GPS) + fix válido + home gravado.
- Código de usuário em `Core/Inc/usr` e `Core/Src/usr`, separado por responsabilidade:

| Pasta | Módulos |
|---|---|
| `control/` | controlador, pid (cascata), mixer, estimador de atitude |
| `math/` | quaternion, vector3, matrix3, filters, math_utils |
| `sensors/` | ICM-42688-P, LIS3MDL, BMP581, GPS NMEA |
| `navigation/` | waypoints (lista manual), home_ponto, nav_posicao, nav_controle |
| `serial/` | iBus, radio_comandos |
| `system/` | estados, calibracao, hardware_glue |
| `esc/` | DShot600 |

O gerenciamento de hardware (DMA, NVIC, clock, callbacks) fica isolado nas camadas `*_hal.c` e no `hardware_glue.c`; a lógica restante não depende do HAL e é testável no host.

## Build

Requisitos: CMake ≥ 3.22, Ninja, toolchain `arm-none-eabi-gcc`.

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Presets disponíveis: `Debug` e `Release` (ver `CMakePresets.json`).

## Testes host

Testes nativos da biblioteca `usr` (math, controle, sensores, GPS, mag, baro, navegação), sem toolchain ARM:

```sh
cmake -S Core/Src/usr/tests -B build/tests
cmake --build build/tests
build/tests/usr_tests
```

## Estrutura de pastas

```
Core/            Código CubeMX + código de usuário (usr/)
Drivers/         HAL STM32H5 e CMSIS
cmake/           Toolchains e arquivos gerados pelo CubeMX
docs/            Documentação (pinagem, contexto p/ IA)
J3-Drone.ioc     Projeto STM32CubeMX
```

## Documentação

- `docs/pinout_map.md` — pinagem, timing (SPI/DShot/iBus), GPS, I2C2, LEDs e notas de regeneração no CubeMX.
- `docs/ia/contexto.md` — contexto do projeto para desenvolvimento assistido por IA.

## Estado atual

Integração GPS concluída: DMA Normal + ReceiveToIdle no H5 (GPDMA1 CH7, 9600 baud), parser NMEA completo (GGA/RMC), LED de fix em PB2. Magnetômetro LIS3MDL via I2C2 polling e barômetro BMP581 via I3C1 com IBI (interrupt-driven). Navegação entre waypoints, ponto de retorno e porta de armar (CH6 + fix + home) funcionando. Todos os módulos puros validados por testes host; firmware compilando e linkando limpo.
