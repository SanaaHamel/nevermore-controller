cmake_minimum_required(VERSION 3.13)

include(FetchContent)
include(cmake/GetGitRevisionDescription.cmake)
include(cmake/pico_sdk_import.cmake)

FetchContent_Declare(lvgl
  GIT_REPOSITORY git@github.com:lvgl/lvgl.git
  GIT_TAG release/v8.3
  GIT_SHALLOW ON
)
FetchContent_Declare(lv_drivers
  GIT_REPOSITORY https://github.com/lvgl/lv_drivers
  GIT_TAG release/v8.3
  GIT_SHALLOW ON
)

FetchContent_MakeAvailable(lvgl lv_drivers)

project(nevermore-controller C CXX ASM)

option(BLUETOOTH_DEBUG "enable bluetooth debug logging (noisy)")
option(BLUETOOTH_LOW_LEVEL_DEBUG "enable bluetooth low level debug logging (very noisy)")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF) # -std=c++ instead of -std=gnu++
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
pico_sdk_init()

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(SRC_CONFIG_DIR ${SRC_DIR}/config) # HACK: libs often include a config header (e.g. lvgl, btstack, etc)

# modify lvgl to add config header path & sdk dependencies
target_include_directories(lvgl PRIVATE ${SRC_CONFIG_DIR})
target_include_directories(lv_drivers PRIVATE ${SRC_CONFIG_DIR})
target_link_libraries(lvgl pico_time) # add `pico_time` dependency so lvgl can get a millisecond tick

# HACK: Also modify `lvgl_demos` & `lvgl_examples` for config include to suppress warnings/errors.
# TODO: Find a way to completely exclude `lvgl_demos` & `lvgl_examples` from being built.
# Even if they're not linked it's annoying.
target_include_directories(lvgl_demos PRIVATE ${SRC_CONFIG_DIR})
target_include_directories(lvgl_examples PRIVATE ${SRC_CONFIG_DIR})

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
add_compile_options(-Wall -Wno-psabi -Wno-trigraphs)
add_compile_definitions(PARAM_ASSERTIONS_ENABLE_ALL=1)

# default is 4.
# Current users: tinyusb (1), stdio w/ USB (1), CYW43 driver (1), WS2812 (1), CST816S (1)
add_compile_definitions(PICO_MAX_SHARED_IRQ_HANDLERS=5)

if(BLUETOOTH_DEBUG)
  add_compile_definitions(CMAKE_BLUETOOTH_DEBUG=1)
endif()

if(BLUETOOTH_LOW_LEVEL_DEBUG)
  add_compile_definitions(CMAKE_BLUETOOTH_LOW_LEVEL_DEBUG=1)
endif()

add_executable(nevermore-controller
  ${SRC_FILES}
)

target_include_directories(nevermore-controller PRIVATE
  ${SRC_DIR}
  ${SRC_CONFIG_DIR}
)

target_link_libraries(nevermore-controller
  PRIVATE
  lvgl::lvgl
  lvgl::drivers
  pico_stdlib
  pico_cyw43_arch_none
  pico_btstack_ble
  pico_btstack_cyw43
  hardware_adc
  hardware_i2c
  hardware_pio
  hardware_pwm
  hardware_spi
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Specify source files explicitly b/c some SDK sources trigger warnings.
  # target_compile_options(nevermore-controller PRIVATE -Werror)
  set_source_files_properties(${SRC_FILES} PROPERTIES COMPILE_FLAGS -Werror)
endif()

pico_enable_stdio_usb(nevermore-controller 1)
pico_enable_stdio_uart(nevermore-controller 1)
pico_add_extra_outputs(nevermore-controller)
pico_btstack_make_gatt_header(nevermore-controller PRIVATE ${SRC_DIR}/nevermore.gatt)

pico_generate_pio_header(nevermore-controller ${SRC_DIR}/ws2812.pio)
