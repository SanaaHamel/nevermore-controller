cmake_minimum_required(VERSION 3.13)

include(cmake/pico_sdk_import.cmake)

project(nevermore-ble C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w)

pico_sdk_init()

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
file(GLOB_RECURSE SRC_H ${SRC_DIR}/*.h)
file(GLOB_RECURSE SRC_C ${SRC_DIR}/*.c)
file(GLOB_RECURSE SRC_HPP ${SRC_DIR}/*.hpp)
file(GLOB_RECURSE SRC_CPP ${SRC_DIR}/*.cpp)
file(GLOB_RECURSE SRC_GATT ${SRC_DIR}/*.gatt)

# suppress pointless warning
add_compile_options(-Wno-psabi)
add_compile_definitions(PARAM_ASSERTIONS_ENABLE_ALL=1)

add_executable(nevermore-ble
  ${SRC_H} ${SRC_C} ${SRC_HPP} ${SRC_CPP}
)
target_link_libraries(nevermore-ble
  pico_stdlib
  pico_cyw43_arch_none
  pico_btstack_ble
  pico_btstack_cyw43
  hardware_adc
  hardware_i2c
  hardware_pio
  hardware_pwm
)

target_include_directories(nevermore-ble PRIVATE
  ${SRC_DIR}
)

pico_enable_stdio_usb(nevermore-ble 1)
pico_enable_stdio_uart(nevermore-ble 1)
pico_add_extra_outputs(nevermore-ble)
pico_btstack_make_gatt_header(nevermore-ble PRIVATE ${SRC_DIR}/nevermore.gatt)

# TODO: implement and test
# pico_generate_pio_header(nevermore-ble ${SRC_DIR}/ws2812.pio)
