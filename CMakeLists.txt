cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME J3-Drone)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add user libraries (J3_DroneV2)
add_subdirectory(Core/Src/usr)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Glue HAL do ICM-42688-P (SPI1 + GPDMA1): somente firmware
    Core/Src/usr/sensors/imuc42688_spi_hal.c
    # Reparo em codigo da regressao do CubeMX: PLL2, DMA SPI1/USART2, NVIC e IRQ
    Core/Src/usr/system/hardware_glue.c
    # Recepcao iBus na USART2 (DMA + IDLE)
    Core/Src/usr/serial/ibus_uart_hal.c
    # Saida DShot600 para os 4 ESCs (TIM1 + GPDMA1 CH3..CH6)
    Core/Src/usr/esc/dshot_timer_hal.c
    # Recepcao GPS BN-220 na USART3 (DMA circular GPDMA1 CH7)
    Core/Src/usr/sensors/gps_uart_hal.c
    # Transporte I2C2 (polling) para LIS3MDL e BMP280
    Core/Src/usr/sensors/i2c2_hal.c
    # LED dedicado de status do GPS (PB2)
    Core/Src/usr/system/gps_led.c
    # Supervisor do sistema de controle de voo (usa main.h e HAL_GetTick)
    Core/Src/usr/main_j3dronev2.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    usr
    m

    # Add user defined libraries
)
