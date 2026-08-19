# Protocolo j3Serial v0.1 — Link serial de waypoints, telemetria e teste (USART1)

> Documento de referência para a implementação do **programa de solo (host)** em
> Linux/Windows e da **interface no firmware** do J3_DroneV2.
> Criado em 2026-08-18 por Nelson Lima com opencode (deepseek-v4-flash-free).

---

## 1. Visão geral

O link serial **j3Serial** conecta um computador (host: Linux/Windows) à placa de
controle de voo (FC/escravo) através da **USART1** do STM32H562RGT6.

O FC **fica aguardando comandos**: não envia nada espontaneamente, exceto a
**telemetria periódica** quando o host a habilita explicitamente. Todo comando
válido recebe uma **resposta** (ACK e/ou mensagem de dados).

| Item            | Valor                                     |
|-----------------|-------------------------------------------|
| Periférico      | USART1 (STM32H562)                        |
| Pinos           | **PB14 = TX** (placa → host), **PB15 = RX** (host → placa) |
| Baud rate       | **115200**                                |
| Formato         | 8 bits, sem paridade, 1 stop bit (8N1)    |
| Controle de fluxo| Nenhum                                   |
| Nível elétrico  | 3,3 V TTL (usar conversor USB-TTL 3,3 V)  |
| Byte order      | **Little-endian** (o mesmo em x86/ARM)    |
| Float           | IEEE 754, 32 bits (float C)               |
| Topologia       | Host = mestre; FC = escravo               |

---

## 2. Frame

Toda mensagem tem o mesmo formato fixo:

```
+--------+--------+--------+--------------------+--------+
| MAGIC  | LEN    | TIPO   | PAYLOAD            | CRC8   |
| 0xAA   | 1 byte | 1 byte | LEN bytes          | 1 byte |
+--------+--------+--------+--------------------+--------+
```

| Campo   | Tamanho | Descrição                                              |
|---------|---------|--------------------------------------------------------|
| MAGIC   | 1       | Sempre `0xAA`. Sem escape: o comprimento é explícito. |
| LEN     | 1       | Tamanho do PAYLOAD (0..200).                          |
| TIPO    | 1       | Identificador da mensagem (tabela na seção 4).        |
| PAYLOAD | LEN     | Conteúdo específico da mensagem.                      |
| CRC8    | 1       | CRC-8 sobre **TIPO + PAYLOAD** (sem MAGIC, sem LEN).  |

Tamanho máximo do frame: `1 + 1 + 1 + 200 + 1 = 204` bytes.

### 2.1 CRC8

| Parâmetro        | Valor                                      |
|------------------|--------------------------------------------|
| Polinômio        | `0x31` (x⁸ + x⁵ + x⁴ + 1, "Dallas/MAXIM") |
| Valor inicial    | `0x00`                                     |
| Reflexão        | **Não** (MSB primeiro)                     |
| XOR de saída    | `0x00`                                     |
| Escopo           | `TIPO` + `PAYLOAD` (na ordem do frame)     |

Implementação de referência em C (sem tabela):

```c
uint8_t j3serial_crc8(const uint8_t *dados, uint32_t n)
{
  uint8_t crc = 0x00;
  for (uint32_t i = 0; i < n; ++i)
  {
    crc ^= dados[i];
    for (int b = 0; b < 8; ++b)
    {
      if (crc & 0x80u)
      {
        crc = (uint8_t)((crc << 1) ^ 0x31u);
      }
      else
      {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}
```

Vetores de teste (o CRC é calculado sobre TIPO + PAYLOAD):

| Bytes calculados (TIPO+PAYLOAD)                  | CRC8  |
|--------------------------------------------------|-------|
| `81 01 00` (ACK de UPLOAD_ROTA, status OK)       | `67`  |
| `02` (LIMPAR_ROTA sem payload)                   | `62`  |
| `03` (INICIAR_MISSAO sem payload)                | `53`  |
| `05` (CONSULTAR_ROTA sem payload)                | `F5`  |
| `08` (PEDIR_TELEMETRIA sem payload)              | `B9`  |
| `01 01 77 67 BC C1 82 88 3A C2 00 00 F0 42` (UPLOAD_ROTA com 1 waypoint)| `C5` |
| `09 02 50 D0 07 B8 0B D0 07` (TESTE_MOTOR, ver exemplo) | `81` |
| (vazio)                                         | `00`  |

---

## 3. Modelo de comunicação

- O FC processa **um comando por vez**, na ordem de chegada.
- **Comandos que alteram estado** (UPLOAD_ROTA, LIMPAR_ROTA, INICIAR_MISSAO,
  PAUSAR_MISSAO, CONFIG_TELEMETRIA, TESTE_MOTOR, PARAR_TESTE) respondem com
  **ACK** (`0x81`).
- **Comandos de consulta** (CONSULTAR_ROTA, PEDIR_TELEMETRIA, PEDIR_IMU,
  PEDIR_MAG, PEDIR_BARO, PEDIR_GPS, PEDIR_ATITUDE, PEDIR_ESTADO) respondem
  **diretamente com a mensagem de dados** (a própria resposta confirma). Se o
  dado não estiver disponível (ex.: sensor inativo), o FC responde com ACK e
  status `SENSOR_INATIVO` (5).
- O FC também envia mensagens **EVENTO** (`0x87`) espontaneamente quando algo
  assíncrono acontece (rota concluída, chegada em waypoint, fim de teste de
  motor) e a **TELEMETRIA** periódica quando configurada.

### 3.1 Timeout e retransmissão (regra do host)

| Parâmetro            | Valor  |
|----------------------|--------|
| Timeout de resposta  | 500 ms |
| Máx. retransmissões  | 3      |

O host deve esperar a resposta após enviar um comando. Se o tempo esgotar,
reenvia o mesmo frame (mesmo conteúdo). Após 3 tentativas sem resposta, assume
perda de link e informa o usuário.

> Nota: enquanto um TESTE_MOTOR está em andamento, o FC continua respondendo aos
> demais comandos normalmente (exceto outro TESTE_MOTOR → ACK `MOTOR_OCUPADO`).

---

## 4. Catálogo de mensagens

### 4.1 Host → FC (comandos)

| TIPO | Nome              | LEN | Resposta                          |
|------|-------------------|-----|-----------------------------------|
| 0x01 | UPLOAD_ROTA       | 1+n·12 | ACK                          |
| 0x02 | LIMPAR_ROTA       | 0   | ACK                               |
| 0x03 | INICIAR_MISSAO    | 0   | ACK                               |
| 0x04 | PAUSAR_MISSAO     | 0   | ACK                               |
| 0x05 | CONSULTAR_ROTA    | 0   | ROTA_RESPOSTA (0x85)              |
| 0x06 | CONFIG_TELEMETRIA | 2   | ACK                               |
| 0x08 | PEDIR_TELEMETRIA  | 0   | TELEMETRIA (0x86)                 |
| 0x09 | TESTE_MOTOR       | 8   | ACK                               |
| 0x0A | PARAR_TESTE       | 0   | ACK                               |
| 0x0B | PEDIR_IMU         | 0   | RESPOSTA_IMU (0x8B)               |
| 0x0C | PEDIR_MAG         | 0   | RESPOSTA_MAG (0x8C)               |
| 0x0D | PEDIR_BARO        | 0   | RESPOSTA_BARO (0x8D)              |
| 0x0E | PEDIR_GPS         | 0   | RESPOSTA_GPS (0x8E)               |
| 0x0F | PEDIR_ATITUDE     | 0   | RESPOSTA_ATITUDE (0x8F)           |
| 0x10 | PEDIR_ESTADO      | 0   | RESPOSTA_ESTADO (0x90)            |

### 4.2 FC → Host (respostas e eventos)

| TIPO | Nome              | LEN   |
|------|-------------------|-------|
| 0x81 | ACK               | 2     |
| 0x85 | ROTA_RESPOSTA     | 1+n·12|
| 0x86 | TELEMETRIA        | 61    |
| 0x87 | EVENTO            | 1..4  |
| 0x8B | RESPOSTA_IMU      | 28    |
| 0x8C | RESPOSTA_MAG      | 12    |
| 0x8D | RESPOSTA_BARO     | 12    |
| 0x8E | RESPOSTA_GPS      | 33    |
| 0x8F | RESPOSTA_ATITUDE  | 12    |
| 0x90 | RESPOSTA_ESTADO   | 7+n·12|

---

## 5. Detalhamento das mensagens

Notação: `u8` = uint8, `u16` = uint16 (little-endian), `f32` = float 32 bits
(little-endian, IEEE 754). Todos os campos na ordem listada.

### 5.1 Waypoint

A estrutura de waypoint é idêntica à do firmware (`waypoint_t` em
`Core/Inc/usr/navigation/waypoints.h`):

| Campo   | Tipo | Descrição              |
|---------|------|------------------------|
| lat_deg | f32  | Latitude em graus (-90..+90) |
| lon_deg | f32  | Longitude em graus (-180..+180) |
| alt_m   | f32  | Altitude alvo em metros (usada na altura barométrica) |

Tamanho: **12 bytes**. Máximo de waypoints por rota: **16** (limite do frame:
16·12 + 1 = 193 ≤ 200 bytes de payload).

### 5.2 UPLOAD_ROTA (0x01)

```
u8  n              quantidade de waypoints (1..16)
n × waypoint       lat f32, lon f32, alt f32 (12 bytes cada)
```

- Substitui **toda** a rota anterior (upload atômico: ou a rota toda entra, ou nada).
- n = 0 é inválido (use LIMPAR_ROTA).
- ACK: status 0 (OK), 3 (ROTA_CHEIA) se n > 16, 6 (ARGUMENTO_INVALIDO) se n = 0
  ou payload malformado.
- Ao carregar com sucesso emite EVENTO `ROTA_CARREGADA` (n). Se houver missão em
  andamento, ela é abortada silenciosamente (a rota mudou).

### 5.3 LIMPAR_ROTA (0x02)

Sem payload. Apaga a rota e aborta a missão em andamento (se houver).
Emite EVENTO `ROTA_LIMPA` (1).

### 5.4 INICIAR_MISSAO (0x03)

Sem payload. Inicia a missão a partir do waypoint 0.

- Exige: FC **armado (VOO)**, rota com ≥ 1 ponto, fix GPS válido e home válido.
- ACK status: 0 (OK), 2 (COMANDO_NAO_PERMITIDO — desarmado ou sem fix/home),
  1 (ERRO — rota vazia), 4 (MOTOR_OCUPADO — teste em andamento).
- Se a missão já está ativa, reinicia do waypoint 0.
- Envia EVENTO `MISSAO_INICIADA` (2) ao iniciar.

### 5.5 PAUSAR_MISSAO (0x04)

Sem payload. Congela a navegação no waypoint alvo atual (a aeronave segura a
posição atual via position hold). Um novo INICIAR_MISSAO reinicia do waypoint 0.
Envia EVENTO `MISSAO_PAUSADA` (5). ACK 0 mesmo sem missão ativa.

### 5.6 CONSULTAR_ROTA (0x05)

Sem payload. O FC responde com **ROTA_RESPOSTA** (0x85):

```
u8  n              quantidade de waypoints (0 = rota vazia)
n × waypoint       lat f32, lon f32, alt f32
```

### 5.7 CONFIG_TELEMETRIA (0x06)

```
u16 periodo_ms     período da telemetria periódica (0 = desliga)
```

- `periodo_ms = 0` desliga a telemetria periódica (padrão de fábrica).
- Qualquer valor ≥ 50 ms liga. Valores < 50 são aceitos como 50.
- A primeira transmissão ocorre `periodo_ms` após o ACK.
- PEDIR_TELEMETRIA (one-shot) funciona mesmo com a periódica desligada.

### 5.8 PEDIR_TELEMETRIA (0x08)

Sem payload. Resposta: **TELEMETRIA** (0x86).

### 5.9 TELEMETRIA (0x86) — 61 bytes

| Campo          | Tipo | Descrição                                         |
|----------------|------|---------------------------------------------------|
| estado         | u8   | 0=ESPERA, 1=VOO, 2=CALIBRACAO                     |
| armado         | u8   | 1 quando o controle está habilitado (VOO)         |
| roll_rad       | f32  | Rolagem (rad)                                     |
| pitch_rad      | f32  | Arfagem (rad)                                     |
| yaw_rad        | f32  | Guinada (rad, -π..π)                              |
| lat_deg        | f32  | Latitude GPS (graus)                              |
| lon_deg        | f32  | Longitude GPS (graus)                             |
| alt_msl_m      | f32  | Altitude GPS MSL (m)                              |
| vel_m_s        | f32  | Velocidade sobre o solo (m/s)                     |
| curso_deg      | f32  | Rumo sobre o solo (0..360°)                       |
| sats           | u8   | Satélites usados no fix                           |
| hdop           | f32  | HDOP do GPS                                      |
| home_lat_deg   | f32  | Latitude do home point (travado no armar)         |
| home_lon_deg   | f32  | Longitude do home point                           |
| home_alt_m     | f32  | Altitude do home point (m)                        |
| home_valido    | u8   | 1 = home válido                                   |
| wp_indice_alvo | u16  | Índice do waypoint alvo atual (0xFFFF = nenhum)   |
| wp_total       | u16  | Total de waypoints da rota (0 = sem rota)         |
| dist_alvo_m    | f32  | Distância horizontal até o alvo (m)               |
| baro_alt_m     | f32  | Altitude barométrica filtrada (m)                 |

### 5.10 EVENTO (0x87)

```
u8  evento    código do evento
    [payload variável, conforme a tabela]
```

| evento | Nome              | Payload                          |
|--------|-------------------|----------------------------------|
| 0      | ROTA_CARREGADA    | n u8 (waypoints recebidos)       |
| 1      | ROTA_LIMPA        | —                                |
| 2      | MISSAO_INICIADA   | —                                |
| 3      | CHEGOU_WAYPOINT   | indice u8 (waypoint alcançado)   |
| 4      | ROTA_CONCLUIDA    | — (último waypoint alcançado)    |
| 5      | MISSAO_PAUSADA    | —                                |
| 6      | MISSAO_ABORTADA   | motivo u8 (1=perdeu fix GPS, 2=desarmou) |
| 7      | TESTE_MOTOR_FIM   | motor u8, status u8 (0=concluído, 1=abortado) |

### 5.11 TESTE_MOTOR (0x09)

```
u8   motor          1..4 (somente individual; 0 inválido)
u8   pico_pct       pico da rampa em % (1..100; 0 = inválido)
u16  subida_ms      tempo de 0% até o pico (100..30000)
u16  pico_ms        tempo mantendo o pico (padrão 3000)
u16  descida_ms     tempo do pico até 0% (100..30000)
```

**Perfil automático da rampa** (gerenciado pelo FC):

```
   pico_pct ────────────┐ pico_ms ──┐
        │ subida_ms     │           │ descida_ms
        │               │           │
   0 ───┴───────────────┴───────────┴───────────► tempo
```

1. **SUBIDA**: motor sobe linearmente de 0% até `pico_pct` em `subida_ms`.
2. **PICO**: mantém `pico_pct` por `pico_ms` (default 3000 ms).
3. **DESCIDA**: desce linearmente até 0% em `descida_ms`.
4. **FIM**: motor em 0%; EVENTO `TESTE_MOTOR_FIM` (motor, status 0).

Regras de segurança:

- **Somente com o FC desarmado (ESPERA)**. Armado → ACK `COMANDO_NAO_PERMITIDO`.
- **Somente um motor por vez**. Com outro teste ativo → ACK `MOTOR_OCUPADO`.
- O percentual é sempre relativo ao **throttle máximo do DShot** (48..2047).
- Enquanto o teste roda, o motor testado recebe a rampa e os outros 3 ficam em 0.
- **PARAR_TESTE (0x0A)**: aborta imediatamente (motor → 0%);
  EVENTO `TESTE_MOTOR_FIM` com status 1.
- Desarmar/armar o FC durante o teste também aborta (EVENTO status 1).

### 5.12 PARAR_TESTE (0x0A)

Sem payload. Aborta qualquer teste de motor em andamento (sem efeito se não
houver teste). ACK com status 0.

### 5.13 PEDIR_IMU (0x0B) → RESPOSTA_IMU (0x8B) — 28 bytes

| Campo      | Tipo | Descrição               |
|------------|------|-------------------------|
| temp_c     | f32  | Temperatura (°C)        |
| accel_x    | f32  | Aceleração X (m/s²)     |
| accel_y    | f32  | Aceleração Y (m/s²)     |
| accel_z    | f32  | Aceleração Z (m/s²)     |
| giro_x     | f32  | Velocidade angular X (rad/s) |
| giro_y     | f32  | Velocidade angular Y (rad/s) |
| giro_z     | f32  | Velocidade angular Z (rad/s) |

### 5.14 PEDIR_MAG (0x0C) → RESPOSTA_MAG (0x8C) — 12 bytes

| Campo | Tipo | Descrição              |
|-------|------|------------------------|
| x     | f32  | Campo magnético X (gauss) |
| y     | f32  | Campo magnético Y (gauss) |
| z     | f32  | Campo magnético Z (gauss) |

### 5.15 PEDIR_BARO (0x0D) → RESPOSTA_BARO (0x8D) — 12 bytes

| Campo       | Tipo | Descrição                    |
|-------------|------|------------------------------|
| temp_c      | f32  | Temperatura (°C)             |
| pressao_pa  | f32  | Pressão absoluta (Pa)        |
| altitude_m  | f32  | Altitude em relação ao P0 (m)|

### 5.16 PEDIR_GPS (0x0E) → RESPOSTA_GPS (0x8E) — 33 bytes

| Campo           | Tipo | Descrição                             |
|-----------------|------|---------------------------------------|
| fix_valido      | u8   | 1 = fix válido                        |
| num_sats        | u8   | Satélites usados                      |
| lat_deg         | f32  | Latitude (graus)                      |
| lon_deg         | f32  | Longitude (graus)                     |
| alt_m           | f32  | Altitude MSL (m)                      |
| vel_m_s         | f32  | Velocidade sobre o solo (m/s)         |
| curso_deg       | f32  | Rumo sobre o solo (0..360°)           |
| hdop            | f32  | HDOP                                  |
| erro_estimado_m | f32  | HDOP × UERE (erro horizontal est., m) |

### 5.17 PEDIR_ATITUDE (0x0F) → RESPOSTA_ATITUDE (0x8F) — 12 bytes

| Campo     | Tipo | Descrição          |
|-----------|------|--------------------|
| roll_rad  | f32  | Rolagem (rad)      |
| pitch_rad | f32  | Arfagem (rad)      |
| yaw_rad   | f32  | Guinada (rad)      |

### 5.18 PEDIR_ESTADO (0x10) → RESPOSTA_ESTADO (0x90)

```
u8   estado           0=ESPERA, 1=VOO, 2=CALIBRACAO
u8   armado           1 = controle habilitado
u8   motor_1_pct      saída atual do motor 1 (0..100 %)
u8   motor_2_pct      saída atual do motor 2 (0..100 %)
u8   motor_3_pct      saída atual do motor 3 (0..100 %)
u8   motor_4_pct      saída atual do motor 4 (0..100 %)
u8   rota_n           quantidade de waypoints (0 = vazia)
n × waypoint         lat f32, lon f32, alt f32
```

### 5.19 ACK (0x81)

```
u8  tipo_origem   TIPO do comando que gerou o ACK
u8  status        código de status (tabela abaixo)
```

| status | Nome                  | Significado                                   |
|--------|-----------------------|-----------------------------------------------|
| 0      | OK                    | Comando aceito/executado                      |
| 1      | ERRO                  | Falha interna ou rota vazia ao iniciar missão |
| 2      | COMANDO_NAO_PERMITIDO | Estado atual não permite (ex.: teste armado, missão desarmada) |
| 3      | ROTA_CHEIA            | Mais de 16 waypoints no upload                |
| 4      | MOTOR_OCUPADO         | Outro teste de motor em andamento             |
| 5      | SENSOR_INATIVO        | Sensor solicitado não está operacional        |
| 6      | ARGUMENTO_INVALIDO    | Payload malformado (n=0, motor=0, pico=0, etc.) |

---

## 6. Comportamento do FC (regras que o host deve conhecer)

### 6.1 Rota

- A rota vive em RAM no FC (máx. 16 pontos) e é apagada no boot.
- UPLOAD_ROTA substitui a rota inteira e emite EVENTO `ROTA_CARREGADA` (n).

### 6.2 Missão

- INICIAR_MISSAO só é aceito **armado (VOO)** e com **fix GPS válido** e home
  travado.
- A aeronave navega para o waypoint `i`; ao entrar no **raio de chegada
  (1,5 m)** emite EVENTO `CHEGOU_WAYPOINT` (i) e avança para `i+1`.
- Ao alcançar o último: EVENTO `ROTA_CONCLUIDA` e o FC segura a posição final
  (hold). A missão termina (wp_indice_alvo volta a 0xFFFF na telemetria).
- **Perda do fix GPS durante a missão**: EVENTO `MISSAO_ABORTADA` (motivo 1) e
  a navegação é desativada (volta ao controle manual/assistido).
- **Desarmar durante a missão**: EVENTO `MISSAO_ABORTADA` (motivo 2).
- LIMPAR_ROTA ou novo UPLOAD_ROTA durante a missão também aborta.
- A altitude do waypoint é seguida pela altura **barométrica** (BMP581).

### 6.3 Telemetria

- Desligada por padrão. `CONFIG_TELEMETRIA periodo_ms` liga (mín. 50 ms).
- A mensagem TELEMETRIA é idêntica na periódica e no one-shot (PEDIR_TELEMETRIA).

---

## 7. Máquina de estados do host (orientação para o programa)

```
        ┌─────────┐  envio de comando   ┌─────────┐
        │  OCIOSO │────────────────────►│ AGUARDA │
        └─────────┘                     └─────────┘
           ▲    ▲                            │  resposta válida (CRC ok, tipo esperado)
           │    │                            ▼
           │    └── timeout 500 ms ──► retransmite (máx. 3) ──► ERRO_LINK
           │
           └── resposta recebida ──► trata resposta / evento ──► OCIOSO
```

- Frames de EVENTO podem chegar a qualquer momento (mesmo sem comando pendente)
  e não interferem no estado AGUARDA.
- O host deve **validar CRC8 em todo frame** recebido e descartar frames
  inválidos (buscando o próximo `0xAA`).
- Frames de resposta com TIPO diferente do esperado podem ser ignorados com
  segurança (ou armazenados como "resposta atrasada").
- Ao abrir a porta serial: enviar `PEDIR_ESTADO` para confirmar o link antes de
  qualquer outra operação.

---

## 8. Exemplos hex

### 8.1 PEDIR_TELEMETRIA (host → FC)

```
AA 00 08 B9
```

### 8.2 UPLOAD_ROTA com 1 waypoint (-23.550520, -46.633309, 120.0 m)

Payload = `01` (n=1) + lat `77 67 BC C1` + lon `82 88 3A C2` + alt `00 00 F0 42`
(13 bytes). CRC8 sobre TIPO+PAYLOAD = `C5`.

```
AA 0D 01 01 77 67 BC C1 82 88 3A C2 00 00 F0 42 C5
```

### 8.3 ACK do FC (UPLOAD_ROTA, OK)

Payload = `01 00` (tipo de origem, status 0). CRC8 = `67`.

```
AA 02 81 01 00 67
```

### 8.4 LIMPAR_ROTA (host → FC)

```
AA 00 02 62
```

### 8.5 TESTE_MOTOR — motor 2, pico 80 %, subida 2000 ms, pico 3000 ms, descida 2000 ms

Payload = `02 50 D0 07 B8 0B D0 07` (motor 2, 80 % = 0x50, 2000 = 0x07D0,
3000 = 0x0BB8, 2000 = 0x07D0). CRC8 = `81`.

```
AA 08 09 02 50 D0 07 B8 0B D0 07 81
```

### 8.6 CONSULTAR_ROTA (host → FC)

```
AA 00 05 F5
```

### 8.7 RESPOSTA_IMU (exemplo de valores)

temp 25,0 °C (`00 00 C8 41`), accel {0,0,9,80665} (`00 00 00 00`, `00 00 00 00`,
`CD CC 1C 41`), giro {0,0,0}. Payload = 28 bytes; CRC8 calculado pelo programa
de referência (seção 2.1).

```
AA 1C 8B 00 00 C8 41 00 00 00 00 00 00 00 00 CD CC 1C 41
   00 00 00 00 00 00 00 00 00 00 00 00 <CRC8>
```

> Os valores f32 dos exemplos podem ser conferidos com o código de referência
> do CRC8 e a conversão `[BitConverter]::GetBytes([single]valor)` no .NET.

---

## 9. Limitações e pendências (v0.1)

- Não há retransmissão no lado do FC (o host cuida disso).
- Não há controle de fluxo nem handshake de conexão (PEDIR_ESTADO serve de
  "ping").
- A rota não é persistida na flash (apagada no boot).
- Não há comando de escrita/leitura de parâmetros (PID etc.) — futura versão.

---

## 10. Referências no firmware (para a implementação da placa)

| Arquivo                              | Conteúdo                          |
|--------------------------------------|-----------------------------------|
| `Core/Src/usart.c`                   | MX_USART1_UART_Init (115200 8N1)  |
| `Core/Inc/usr/navigation/waypoints.h`| `waypoint_t` (lat/lon/alt f32)    |
| `Core/Inc/usr/serial/ibus_uart_hal.c`| Padrão de RX UART usado no projeto|
| `Core/Inc/usr/system/estados.h`      | `estado_modo_t` (ESPERA/VOO/CALIBRACAO) |
| `Core/Inc/usr/serial/gps_nmea.h`     | `gps_medida_t` (campos da RESPOSTA_GPS) |
| `Core/Inc/usr/sensors/imuc42688.h`   | `imu_medida_t` (RESPOSTA_IMU)     |
| `Core/Inc/usr/sensors/lis3mdl.h`     | `mag_medida_t` (gauss)            |
| `Core/Inc/usr/sensors/bmp581.h`      | `baro_medida_t` (RESPOSTA_BARO)   |
| `Core/Inc/usr/control/mixer.h`       | `saida_misturador_t` (4 motores)  |
| `Core/Inc/usr/esc/dshot.h`           | Conversão % → frame DShot         |

---

## Histórico

| Versão | Data       | Mudanças                                        |
|--------|------------|-------------------------------------------------|
| 0.1    | 2026-08-18 | Versão inicial: waypoints, missão, telemetria, sensores e teste de motor |