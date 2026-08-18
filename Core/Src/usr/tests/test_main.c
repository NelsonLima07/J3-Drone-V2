/**
 * @file    test_main.c
 * @brief   Executável dos testes nativos das bibliotecas usr
 * @date    2026-08-15
 * @author  Nelson Lima
 * @ai      opencode (big-pickle)
 */

#include <stdio.h>

int testes_math(void);
int testes_control(void);
int testes_sensors(void);
int testes_ibus(void);
int testes_estados(void);
int testes_calibracao(void);
int testes_dshot(void);
int testes_estimador(void);
int testes_radio(void);
int testes_gps_nmea(void);
int testes_home_ponto(void);
int testes_nav_posicao(void);
int testes_nav_controle(void);
int testes_lis3mdl(void);
int testes_bmp581(void);
int testes_estimador_mag(void);

int main(void)
{
  int falhas = 0;

  printf("=== Testes de math ===\n");
  falhas += testes_math();

  printf("\n=== Testes de controle ===\n");
  falhas += testes_control();

  printf("\n=== Testes de sensores ===\n");
  falhas += testes_sensors();

  printf("\n=== Testes de iBus ===\n");
  falhas += testes_ibus();

  printf("\n=== Testes de estados ===\n");
  falhas += testes_estados();

  printf("\n=== Testes de calibracao ===\n");
  falhas += testes_calibracao();

  printf("\n=== Testes de DShot ===\n");
  falhas += testes_dshot();

  printf("\n=== Testes do estimador de atitude ===\n");
  falhas += testes_estimador();

  printf("\n=== Testes do radio (canais -> setpoint) ===\n");
  falhas += testes_radio();

  printf("\n=== Testes do parser NMEA (GPS BN-220) ===\n");
  falhas += testes_gps_nmea();

  printf("\n=== Testes do home point ===\n");
  falhas += testes_home_ponto();

  printf("\n=== Testes da geometria de navegacao ===\n");
  falhas += testes_nav_posicao();

  printf("\n=== Testes do controlador de navegacao ===\n");
  falhas += testes_nav_controle();

  printf("\n=== Testes do magnetometro LIS3MDL ===\n");
  falhas += testes_lis3mdl();

  printf("\n=== Testes do barometro BMP581 ===\n");
  falhas += testes_bmp581();

  printf("\n=== Testes do estimador com magnetometro ===\n");
  falhas += testes_estimador_mag();

  printf("\n=== Resumo ===\n");
  if (falhas == 0)
    printf("TODOS OS TESTES PASSARAM\n");
  else
    printf("%d TESTE(S) FALHARAM\n", falhas);

  return (falhas > 0) ? 1 : 0;
}
