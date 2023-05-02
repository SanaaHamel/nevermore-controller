cmake_minimum_required(VERSION 3.13)

include(cmake/GetGitRevisionDescription.cmake)
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

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S UTC" UTC)
git_describe_working_tree(GIT_DESCRIPTION --always)
configure_file("${SRC_DIR}/binary_info.cpp.in" "${SRC_DIR}/binary_info.cpp" @ONLY)
list(APPEND SRC_CPP "${SRC_DIR}/binary_info.cpp")

set(SRC_FILES ${SRC_H} ${SRC_C} ${SRC_HPP} ${SRC_CPP})

# `psabi` is a useless warning for our purposes
add_compile_options(-Wall -Wno-psabi)
add_compile_definitions(PARAM_ASSERTIONS_ENABLE_ALL=1)

add_executable(nevermore-ble
  ${SRC_FILES}
)

target_include_directories(nevermore-ble PRIVATE
  ${SRC_DIR}
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

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Specify source files explicitly b/c some SDK sources trigger warnings.
  # target_compile_options(nevermore-ble PRIVATE -Werror)
  set_source_files_properties(${SRC_FILES} PROPERTIES COMPILE_FLAGS -Werror)
endif()

pico_enable_stdio_usb(nevermore-ble 1)
pico_enable_stdio_uart(nevermore-ble 1)
pico_add_extra_outputs(nevermore-ble)
pico_btstack_make_gatt_header(nevermore-ble PRIVATE ${SRC_DIR}/nevermore.gatt)

# TODO: implement and test
# pico_generate_pio_header(nevermore-ble ${SRC_DIR}/ws2812.pio)
