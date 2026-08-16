# Contextualização do Projeto: J3_DroneV2

Este documento serve como guia de contexto para a Inteligência Artificial (IA) auxiliar no desenvolvimento, refatoração e depuração do firmware deste projeto. Ele detalha a arquitetura do hardware, a pilha de software escolhida, o sistema de build e a lógica do sistema.

---

## 1. Visão Geral do Projeto

O objetivo principal é desenvolver uma **Placa de circuito integrado para controle de voo de drone com estabilização de voo extrema** com um firmware moderno organizado e otimizado com divisoes de responsabilidades bem definidas, operando diretamente sobre a camada **HAL (Hardware Abstraction Layer)** da STMicroelectronics.

### Stack Tecnológica e Componentes

* **Microcontrolador:** Família **STM32H562RGT6** (desenvolvimento focado em performance de barramento e DMA).
* **IMU:** Módulo controle inercial **ICM-42688-P** (alto desempenho ODR de gyro 8khz, comunicação SPI).
* **Interface de Comunicação:** **SPI (Serial Peripheral Interface)** configurado em modo mestre, otimizado via **DMA (Direct Memory Access)** enviar e receber dados.
* **Camada de Abstração:** **STM32Cube HAL**.
* **Sistema de Build:** **CMake** moderno integrado com a toolchain `arm-none-eabi-gcc`.

---

## 2. Arquitetura do Sistema e Conectividade

* **Entrada de comandos**
Interface serial, compativel com IBUS que vem do receptor RX da FlySky

* **Controle Inercial / IMU**
O módulo ICM-42688-P tem a comunicação através de um barramento SPI de 4 fios (Clock, MOSI, DC, CS) mais pinos de controle de hardware e INTERRUPÇÃO.

* **Algoritmo base**
 Implementar PID em cascata primeiro (estável, testável), mas isolar o "controlador" atrás de uma interface no código — assim o inner loop pode ser trocado por LADRC depois, sem refazer o projeto. É o caminho que os FCs de ponta seguem.

* **Interface Motores**
 Saida para os ESC é por DSHOT600

## 3. Estrutura fisica do Drone e frame

 Drone voo modo X, helices 9x4,5; Motores Sunnysky brushless, Bateria 3S

## 4. Linguagem codificao e padrao

* **estrutura de pastas**
Criar caso não exista a pasta usr dentro de Core\Src e Core\Inc e colocar os arquivos criados pelo projeto dentro das pastas usr. criar um main_j3dronev2.c e main_j3dronev2.h nas respecticas pastas e iniciar o sistema por ai na funcao main_j3dronev2() que deve ser chamanda pela main() em main.c do arquivo Core\Src\main.c

* **Cabeçalho**
Acima dos arquivos sempre colocar o comentario quando foi criado com data e autor e qual IA ajudou;

* **Nome funções**
Sempre usar nome das funcoes em portugues. main.c na funcao main direcionar main_j3dronev2.c
é onde de fato começa o nosso sistema

* **Padrao de codigo**
Bare matel

* **Linguagem**
C moderno que seja suportado pela HAL
