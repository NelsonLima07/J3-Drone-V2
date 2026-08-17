# Mapa de pinos - ICM-42688-P (IMU)

> Decisão de pinagem do driver do IMU ICM-42688-P no J3_DroneV2.
> Atualizado em 2026-08-16 (opencode/big-pickle).

## 1. Conexão do sensor

| Função      | Pino MCU | Sinal       | Obs.                                                        |
|-------------|----------|-------------|-------------------------------------------------------------|
| SPI1 SCK    | PA5      | SPI1_SCK    | AF5, speed HIGH                                             |
| SPI1 MISO   | PA6      | SPI1_MISO   | AF5, speed HIGH                                             |
| SPI1 MOSI   | PA7      | SPI1_MOSI   | AF5, speed HIGH                                             |
| Chip Select | PA4      | GPIO_Output | **CS do sensor, ativo em nível baixo** (NSS soft)           |
| INT1        | PB0      | EXTI0       | Data ready do sensor (rising); dispara leitura via DMA      |
| RESET       | —        | —           | Não ligado ao MCU (o soft reset é feito por registro 0x11)  |
| VDD / VDDIO | —        | —           | 1,71..3,6 V (3,3 V da placa); decoupling 100 nF + 1 uF      |
| AP / AS     | —        | —           | Pinos de seleção de endereço SPI/I2C: deixar acessíveis     |
| FSYNC       | —        | —           | Não usado                                                   |

## 2. Configuração do SPI1

| Parâmetro        | Valor                              | Motivo                                        |
|------------------|------------------------------------|-----------------------------------------------|
| Modo             | Master, full duplex (2 lines)      | SPI 4 fios do sensor                          |
| Clock            | 24 MHz (exato)                     | Máximo do ICM-42688-P (24 MHz)                |
| Fonte do kernel  | PLL2P = 48 MHz                      | PLL2: CSI 4 MHz, M=1, N=96, VCO 384 MHz, P=8  |
| Prescaler        | BR = ÷2                             | 48 / 2 = 24 MHz (H5 não tem prescaler ÷1)     |
| Data size        | 8 bits                              | Registradores de 8 bits                       |
| CPOL / CPHA      | 0 / 1 (Modo 0)                     | Datasheet ICM-42688-P                         |
| NSS              | Soft + CS manual (PA4)              | Pulso NSS desabilitado (NSSPMode off)         |
| FirstBit         | MSB                                 | Datasheet                                     |
| GPIO speed       | HIGH                                | 24 MHz com bordas limpas em ~10 cm de trilha  |

## 3. DMA (GPDMA1)

| Canal     | Periférico   | Request | Direção       | Uso                          |
|-----------|--------------|---------|---------------|------------------------------|
| Channel 0 | GPDMA1       | SPI1_RX (6) | perif→mem | Leitura do burst (14 bytes)  |
| Channel 1 | GPDMA1       | SPI1_TX (7) | mem→perif  | Endereço/clock do burst      |
| Channel 2 | GPDMA1       | USART2_RX (23) | perif→mem | Recepção iBus (32 bytes) |
| Channel 3 | GPDMA1       | TIM1_CH1 (58) | mem→perif | DShot600 motor 1 (PA8)   |
| Channel 4 | GPDMA1       | TIM1_CH2 (59) | mem→perif | DShot600 motor 2 (PA9)   |
| Channel 5 | GPDMA1       | TIM1_CH3 (60) | mem→perif | DShot600 motor 3 (PA10)  |
| Channel 6 | GPDMA1       | TIM1_CH4 (61) | mem→perif | DShot600 motor 4 (PA11)  |
| Channel 7 | GPDMA1       | USART3_RX (25) | perif→mem | Recepção GPS BN-220 (9600) |

- Handles: `hdma_spi1_rx` / `hdma_spi1_tx` / `hdma_usart2_rx`, vinculados a `hspi1` e `huart2` no `Core/Src/usr/system/hardware_glue.c` (reparo em código do regen).
- Handle GPS: `hdma_usart3_rx`, vinculado a `huart3` no `Core/Src/usr/sensors/gps_uart_hal.c`.
- Handles DShot: `hdma_tim1_ch1..ch4`, vinculados a `htim1` no `Core/Src/usr/esc/dshot_timer_hal.c`.
- IRQs: `SPI1_IRQn`, `GPDMA1_Channel0/1/2_IRQn`, `USART2_IRQn` e `EXTI0_IRQn` (preempt 5, sub 0) — os handlers de SPI1/GPDMA1 ficam no `hardware_glue.c`.
- IRQs GPS: `GPDMA1_Channel7_IRQn` e `USART3_IRQn` (preempt 5, sub 0) — handlers no `gps_uart_hal.c`.
- IRQs DShot: `GPDMA1_Channel3/4/5/6_IRQn` (preempt 5, sub 0) — handlers no `dshot_timer_hal.c`.
- O fim da leitura do IMU dispara `HAL_SPI_TxRxCpltCallback` → decodifica o burst → `controlador_atualiza`.

## 4. Registradores usados (SPI)

| Registrador     | Endereço | Valor | Descrição                          |
|-----------------|----------|-------|------------------------------------|
| WHO_AM_I        | 0x75     | 0x47  | Identificação do dispositivo       |
| DEVICE_CONFIG   | 0x11     | 0x01  | Soft reset (aguardar ~10 ms)       |
| PWR_MGMT0       | 0x4E     | 0x0F  | Gyro LN + Accel LN (6 eixos)       |
| GYRO_CONFIG0    | 0x4F     | 0x03  | ±2000 dps, ODR 8 kHz               |
| ACCEL_CONFIG0   | 0x50     | 0x03  | ±16 g, ODR 8 kHz                   |
| INT_SOURCE0     | 0x65     | 0x80  | UI_DRDY_INT1_EN (DRDY → INT1)      |
| Burst (0x1D)    | —        | 14 B  | Temp + Accel X/Y/Z + Gyro X/Y/Z    |

Leitura SPI: endereço | 0x80. Escrita SPI: endereço & 0x7F. Little: big-endian.

## 4b. Receptor iBus (FlySky) - USART2

| Função   | Pino MCU | Sinal       | Obs.                                                  |
|----------|----------|-------------|-------------------------------------------------------|
| USART2 TX| PA2      | USART2_TX   | AF7 (não usado pelo iBus, reservado)                  |
| USART2 RX| PA3      | USART2_RX   | AF7, **FT (5 V tolerant)**; sinal iBus invertido       |

- 115200 8N1, RX com inversão de nível via `USART_CR2_RXINV` (iBus: idle em nível baixo).
- Recepção por `HAL_UARTEx_ReceiveToIdle_DMA` (buffer 32 bytes, DMA Normal).
  Quadro completo → TC; quadro parcial → IDLE; ambos chamam `HAL_UARTEx_RxEventCallback` (rearm no callback).
- Decodificação: `Core/Src/usr/serial/ibus.c` (puro) — sync `0x20 0x40`, 14 canais little-endian,
  checksum `0xFFFF - soma[0..29]`.
- Mapeamento AETR: CH3 = throttle (canais[2]), CH4 = yaw (canais[3]), CH5 = calibração (canais[4]).

## 4c. LED de status

| Função  | Pino MCU | Sinal        | Obs.                              |
|---------|----------|--------------|-----------------------------------|
| LED_MODO| PB5      | GPIO_Output  | Ativo alto (`LED_MODO_Pin` em main.h) |

- ESPERA: pisca 5 Hz · VOO: aceso · CALIBRAÇÃO: pisca 1 Hz · sem sinal: apagado.

## 4d. Saída DShot600 — ESCs (TIM1)

| Função    | Pino MCU | Sinal      | Obs.                                  |
|-----------|----------|------------|---------------------------------------|
| Motor 1   | PA8      | TIM1_CH1   | AF1 (GPIO_AF1_TIM1), speed VERY_HIGH  |
| Motor 2   | PA9      | TIM1_CH2   | AF1                                   |
| Motor 3   | PA10     | TIM1_CH3   | AF1                                   |
| Motor 4   | PA11     | TIM1_CH4   | AF1                                   |

- **Timing (200 MHz, APB2 sem divisor)**: bit period = 333 ticks → `ARR = 332` (600,6 kHz,
  erro 0,1%); bit "0" = `CCR 125` (625 ns); bit "1" = `CCR 250` (1250 ns); pausa = 4 bits
  com `CCR 0` (≥ 2 µs da spec). Quadro = 20 bits → ~30 kHz de atualização.
- **Frame 16 bits** (MSB primeiro): valor 11 bits (bit 10 = telemetria `0x400`) + stop `0` +
  CRC4 = XOR dos nibbles de `(valor << 1)` — `Core/Src/usr/esc/dshot.c` (puro, testável no host).
- **Mapeamento**: desarmado → frame 0; armado → `48 + round(motor·1999)` [48..2047];
  na borda de armamento envia 48 frames zero antes do primeiro valor real (BLHeli).
- **DMA**: cada canal usa um GPDMA1 em NORMAL (memória → `CCRx`, palavra 32 bits).
  Ao terminar o quadro, `HAL_TIM_PWM_PulseFinishedCallback` (preempt 5) copia `sombra → ativo`
  durante a pausa (janela sem leitura DMA) e re-arma o canal — `TIM_CHANNEL_STATE_SET` volta para
  READY antes do re-arm, pois o HAL TIM não limpa o estado BUSY no fim de quadro.
- **Parent**: os callbacks do HAL TIM resolvem `htim` via `hdma->Parent` (o HAL nunca o atribui) —
  o `Parent = &htim1` de cada canal é setado no `dshot_timer_hal.c`.
- **Integração**: `dshot_escreve` é chamada no `ao_medir` (8 kHz) após
  `misturador_aplica` + `misturador_normaliza`, com `armado = controle_habilitado` (VOO).
- Implementado **100% em código** (TIM1 não configurado no `.ioc`; HAL TIM copiado do pacote
  `STM32Cube_FW_H5_V1.7.0` e `HAL_TIM_MODULE_ENABLED` habilitado no `stm32h5xx_hal_conf.h`).

## 4f. GPS BN-220 - USART3

| Função     | Pino MCU | Sinal      | Obs.                                        |
|------------|----------|------------|---------------------------------------------|
| USART3 TX  | PC10     | USART3_TX  | AF7 (não usado pelo GPS, reservado)         |
| USART3 RX  | PC4      | USART3_RX  | AF7; NMEA 9600 8N1                          |

- O CubeMX gera a USART3 a **115200**; o glue re-inicializa para **9600 8N1**
  (`GPS_BAUD_RATE` em `gps_uart_hal.c`) — padrão de fábrica do BN-220.
- **DMA GPDMA1 CH7** (`GPDMA1_REQUEST_USART3_RX = 25`): o GPDMA do H5 **não tem modo
  circular** (`DMA_CIRCULAR` não existe no HAL H5); usa-se **DMA Normal +
  `HAL_UARTEx_ReceiveToIdle_DMA`** com re-arme no callback (mesmo padrão do iBus).
- Evento TC (buffer 512 cheio) ou IDLE (folga de 1 frame) → `gps_uart_rx_event`
  copia os bytes para um ring de 1024 → o `main_j3dronev2.c` esvazia no parser
  `gps_nmea.c` (puro, testável no host).
- `gps_led.c` controla o LED de status em PB2 (fix válido e recente).

## 4g. LIS3MDL + BMP280 - I2C2

| Função  | Pino MCU | Sinal    | Obs.                      |
|---------|----------|----------|---------------------------|
| I2C2 SCL| PB10     | I2C2_SCL | AF4, ~400 kHz (fast mode) |
| I2C2 SDA| PB12     | I2C2_SDA | AF4                        |

| Sensor  | Endereço | WHO_AM_I | Função                          |
|---------|----------|----------|---------------------------------|
| LIS3MDL | 0x1C     | 0x3D     | Magnetômetro → correção de yaw no estimador |
| BMP280  | 0x76     | 0x58     | Barômetro → altitude (home point / retorno)  |

- Leitura por **polling** (~100 Hz) no loop principal via `i2c2_hal.c`
  (HAL I2C bloqueante; a I2C2 é lenta e o tráfego é pequeno).
- Config LIS3MDL: modo contínuo, ±4 gauss, ODR 100 Hz (`CTRL1 = 0x10`).

## 4h. LED de status do GPS (PB2)

| Função  | Pino MCU | Sinal       | Obs.                                    |
|---------|----------|-------------|-----------------------------------------|
| LED_GPS | PB2      | GPIO_Output | Ativo alto (`GPS_LED_Pin` em main.h)    |

- Aceso com fix válido (HDOP ok) e recente (dentro de `GPS_FIX_JANELA_MS`); apagado sem fix.

## 4e. Cadeia de controle de voo (entrada → processamento → saída)

- **Entrada**: iBus (USART2+DMA+IDLE) → `radio_comandos.c` mapeia os canais AETR para o `setpoint`
  (rol/arfagem ±30° em ANGLE, guinada por taxa, throttle 0..1; desarmado zera o throttle).
- **Processamento**: `estimador_atitude.c` integra o gyro a 8 kHz com correção de gravidade do accel
  (quaternion corpo→mundo, Mahony) → `controlador_atualiza` (cascata; no ANGLE a guinada é por taxa,
  `pid_cascade.c`) → `mixer.c` (frame X).
- **Saída**: `dshot_escreve` (DShot600, TIM1) com arm/desarm.
- O setpoint é escrito pelo loop principal em shadow e copiado na ISR (prio 5), mesmo padrão do DShot.
- `libm` agora é vinculada ao firmware (o estimador usa `cosf`/`sinf` de `quaternion.c`).

## 5. Escalas de conversão

| Grandeza            | Fórmula                                |
|---------------------|----------------------------------------|
| Temperatura         | raw / 132.48 + 25 °C                   |
| Aceleração (m/s²)   | raw / 2048 · 9,80665 (FS ±16 g)        |
| Velocidade angular  | raw / 16,4 · π/180 rad/s (FS ±2000 dps)|

## 6. Notas de regeneração no CubeMX

> A partir de 2026-08-16 o projeto **não será mais regenerado pelo CubeMX**:
> o regen apagou PLL2/kernel SPI1/DMA (regressão documentada na sessão).
> Os reparos ficaram em código no `Core/Src/usr/system/hardware_glue.c`
> (PLL2, prescaler `/2`, kernel `SPI1=PLL2P`, DMA SPI1 CH0/CH1, DMA USART2 CH2,
> NVIC prio 5 e IRQ handlers `SPI1_IRQHandler`/`GPDMA1_Channel0/1/2_IRQHandler`).

O `.ioc` já contém SPI1 com prescaler `_2` e `SPI1Freq_Value = 24 MHz`. Se um dia for regenerado, rever:

1. **GPDMA1**: recria `dma.c`/`dma.h` e pode apagar os canais SPI1/USART2 — conferir os canais 0/1 (SPI1) e 2 (USART2) e as IRQs como acima.
2. **Clocks**: conferir `RCC` → PLL2 habilitado (M=1, N=96, P=8, VCO WIDE) e `SPI1Freq_Value = 24 MHz`.
3. **SPI1**: Data Size 8 bits, prescaler ÷2, NSS soft, NSSP off, modo 0.
4. **PB0**: EXTI0 rising; **PA4**: GPIO output HIGH com label `IMU_CS`; **PB5**: `LED_MODO`.
5. **USART2**: 115200 8N1, pinos PA2/PA3 AF7 (RX FT). Não ligar DMA no CubeMX (feito no hardware_glue).
6. **USART3 (GPS)**: pinos PC10/PC4 AF7; o regen gera 115200 e o glue volta para **9600 8N1**.
   DMA CH7 não é criado pelo CubeMX (feito no `gps_uart_hal.c`, DMA Normal + ReceiveToIdle —
   o GPDMA do H5 não tem circular).
7. **I2C2**: SCL PB10 / SDA PB12 AF4; **PB2**: GPIO output `GPS_LED`. Sem DMA (polling no `i2c2_hal.c`).
8. **TIM1/ESC**: PA8..PA11 AF1, ARR 332, PWM Mode 1 — **não** usar GPIO/DMA do CubeMX (feito no `dshot_timer_hal.c`). O HAL TIM copiado e o `HAL_TIM_MODULE_ENABLED` podem ser revertidos por um regen: recopiar `stm32h5xx_hal_tim*` e re-habilitar o define.
9. Evitar `HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0)` duplicado no `stm32h5xx_it.c` (já está no bloco USER CODE EXTI0) e os IRQ handlers de SPI1/GPDMA1 CH0..CH6/CH7 (ficam no `hardware_glue.c`/`dshot_timer_hal.c`/`gps_uart_hal.c`).
