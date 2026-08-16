cmake_minimum_required(VERSION 3.22)

# Biblioteca com o codigo de usuario do J3_DroneV2
# Nao depende do HAL: C puro, testavel no host.
add_library(usr STATIC
    math/math_utils.c
    math/vector3.c
    math/quaternion.c
    math/matrix3.c
    math/filters.c
    control/controlador.c
    control/pid.c
    control/pid_cascade.c
    control/mixer.c
    control/estimador_atitude.c
    sensors/imuc42688.c
    serial/ibus.c
    serial/radio_comandos.c
    system/estados.c
    system/calibracao.c
    esc/dshot.c
)

target_include_directories(usr PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Inc/usr
)

target_compile_features(usr PUBLIC c_std_11)
