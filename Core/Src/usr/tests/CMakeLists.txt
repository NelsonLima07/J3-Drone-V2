cmake_minimum_required(VERSION 3.22)

# Testes nativos (host) das bibliotecas usr de math, controle e sensores.
# Build separado: nao usa a toolchain ARM.
project(usr_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(usr_tests
    ../math/math_utils.c
    ../math/vector3.c
    ../math/quaternion.c
    ../math/matrix3.c
    ../math/filters.c
    ../control/controlador.c
    ../control/pid.c
    ../control/pid_cascade.c
    ../control/mixer.c
    ../control/estimador_atitude.c
    ../sensors/imuc42688.c
    ../serial/ibus.c
    ../serial/radio_comandos.c
    ../serial/gps_nmea.c
    ../system/estados.c
    ../system/calibracao.c
    ../navigation/home_ponto.c
    ../navigation/nav_posicao.c
    ../navigation/nav_controle.c
    ../sensors/lis3mdl.c
    ../sensors/bmp280.c
    ../esc/dshot.c
    test_main.c
    test_math.c
    test_control.c
    test_sensors.c
    test_ibus.c
    test_estados.c
    test_calibracao.c
    test_dshot.c
    test_estimador.c
    test_radio.c
    test_gps_nmea.c
    test_home_ponto.c
    test_nav_posicao.c
    test_nav_controle.c
    test_lis3mdl.c
    test_bmp280.c
    test_estimador_mag.c
)

target_include_directories(usr_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Inc/usr
)

if(NOT MSVC)
    target_link_libraries(usr_tests m)
endif()
