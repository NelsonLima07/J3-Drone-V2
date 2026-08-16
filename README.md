# J3_DroneV2

Placa controladora de voo (flight controller) para drone FPV modo X, com firmware em C organizado em camadas sobre o STM32Cube HAL. O código de usuário é C puro, independente de hardware e testável no host.

## Hardware

| Componente | Especificação |
|---|---|
| MCU | STM32H562RGT6 (Cortex-M33, 200 MHz) |
| IMU | ICM-42688-P via SPI1 @24 MHz + GPDMA1 (ODR 8 kHz) |
| Receptor | FlySky iBus via USART2 + DMA (115200 bps) |
| ESCs | DShot600 via TIM1 CH1–CH4 + GPDMA1 |
| Status | LED de modo em PB5 |
| Frame | Modo X, hélices 9x4.5, motores Sunnysky, bateria 3S |

## Arquitetura de software

Cadeia de controle de voo:

```
iBus (USART2+DMA)
   → radio_comandos (AETR → setpoint)
   → estimador_atitude (quaternion, Mahony @8 kHz)
   → controlador (PID em cascata; interface p/ LADRC)
   → mixer (frame X)
   → dshot (DShot600, TIM1)
```

- Máquina de estados: `ESPERA` / `VOO` / `CALIBRAÇÃO`.
- Código de usuário em `Core/Inc/usr` e `Core/Src/usr`, separado por responsabilidade:

| Pasta | Módulos |
|---|---|
| `control/` | controlador, pid (cascata), mixer, estimador de atitude |
| `math/` | quaternion, vector3, matrix3, filters, math_utils |
| `sensors/` | driver ICM-42688-P (transporte, SPI HAL) |
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

Testes nativos da biblioteca `usr` (math, controle, sensores), sem toolchain ARM:

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

- `docs/pinout_map.md` — pinagem, timing (SPI/DShot/iBus) e notas de regeneração no CubeMX.
- `docs/ia/contexto.md` — contexto do projeto para desenvolvimento assistido por IA.

## Estado atual

Firmware em desenvolvimento. A lógica de voo (estimador, controle, mixer, DShot, iBus, máquina de estados) é validada por testes no host; a integração com o hardware está em andamento.
