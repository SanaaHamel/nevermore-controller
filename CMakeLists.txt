cmake_minimum_required(VERSION 3.20)

include(FetchContent)
include(cmake/pico_sdk_import.cmake)

FetchContent_Declare(
  freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG smp # V202110.00-SMP
  GIT_SHALLOW ON
  SOURCE_SUBDIR "this path doesn't exist. it is here to disable the auto build inclusion"
)
FetchContent_Declare(
  lvgl
  GIT_REPOSITORY git@github.com:lvgl/lvgl.git
  GIT_TAG release/v8.3
  GIT_SHALLOW ON
)

FetchContent_MakeAvailable(lvgl)

set(FREERTOS_HEAP
    "4"
    CACHE STRING "" FORCE
)
set(FREERTOS_PORT
    "GCC_POSIX"
    CACHE STRING "" FORCE
) # Select the native compile PORT
set(FREERTOS_KERNEL_PATH "${CMAKE_BINARY_DIR}/_deps/freertos_kernel-src")

if(CMAKE_CROSSCOMPILING) # Select the cross-compile PORT
  set(FREERTOS_PORT
      "GCC_ARM_CA9"
      CACHE STRING "" FORCE
  )
endif()

FetchContent_MakeAvailable(freertos_kernel)
include(cmake/FreeRTOS_Kernel_import.cmake)

project(nevermore-controller C CXX ASM)

# debug logging options
option(NEVERMORE_BLUETOOTH_DEBUG "bluetooth debug logging (noisy)" OFF)
option(NEVERMORE_BLUETOOTH_LOW_LEVEL_DEBUG "bluetooth low level debug logging (very noisy)" OFF)
option(NEVERMORE_I2C_DEBUG "I2C debug logging" OFF)
# feature options
option(NEVERMORE_SETTINGS_PERSISTENCE "settings are saved/restored from flash" ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF) # -std=c++ instead of -std=gnu++
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_DIR_BOARDS "${CMAKE_CURRENT_LIST_DIR}/src/config/boards")
if(NOT ("${SRC_DIR_BOARDS}" IN_LIST PICO_BOARD_HEADER_DIRS))
  list(APPEND PICO_BOARD_HEADER_DIRS "${SRC_DIR_BOARDS}")
endif()

set(PICO_CXX_ENABLE_EXCEPTIONS 1)
pico_sdk_init()

add_subdirectory(picowota)
add_picowota(picowota)
set_property(TARGET picowota PROPERTY PICOWOTA_BT_SPP "${NEVERMORE_PICO_W_BT}")
set_property(TARGET picowota PROPERTY PICOWOTA_USB_CDC true)
# each slot is 4 KiB (== erase sector size), want 4 slots to cycle
set_property(TARGET picowota PROPERTY PICOWOTA_APP_STORE_SIZE 16)

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
# HACK: libs often include a config header (e.g. lvgl, btstack, etc)
set(SRC_CONFIG_DIR ${SRC_DIR}/config/lib)

# modify lvgl to add config header path & sdk dependencies
target_include_directories(lvgl PRIVATE ${SRC_CONFIG_DIR})
# add `pico_time` dependency so lvgl can get a millisecond tick
target_link_libraries(lvgl pico_time)

# Completely exclude `lvgl_demos` & `lvgl_examples` from being built.
# Even if they're not linked it's annoying.
set_target_properties(lvgl_demos lvgl_examples PROPERTIES EXCLUDE_FROM_ALL TRUE)

# Touch outputs of config files so the rest of config knows there'll be associated outputs
file(GLOB_RECURSE SRC_IN ${SRC_DIR}/*.in)

foreach(in_file ${SRC_IN})
  cmake_path(REMOVE_EXTENSION in_file LAST_ONLY)

  if(NOT EXISTS ${in_file})
    file(TOUCH "${in_file}")
  endif()
endforeach()

file(GLOB_RECURSE SRC_H ${SRC_DIR}/*.h)
file(GLOB_RECURSE SRC_C ${SRC_DIR}/*.c)
file(GLOB_RECURSE SRC_HPP ${SRC_DIR}/*.hpp)
file(GLOB_RECURSE SRC_CPP ${SRC_DIR}/*.cpp)
set(SRC_FILES ${SRC_H} ${SRC_C} ${SRC_HPP} ${SRC_CPP})

add_library(nevermore-controller ${SRC_FILES})
target_include_directories(nevermore-controller PUBLIC
  "${SRC_DIR}"
  "${SRC_CONFIG_DIR}"
)

target_compile_options(nevermore-controller PUBLIC
  -Wall
  # disable useless warnings
  -Wno-psabi -Wno-trigraphs
)

target_compile_definitions(nevermore-controller PUBLIC
  # board config/defs
  NEVERMORE_BOARD="${NEVERMORE_BOARD}"
  NEVERMORE_BOARD_HEADER="config/pins/${NEVERMORE_BOARD}.hpp"
  NEVERMORE_PICO_W_BT=${NEVERMORE_PICO_W_BT}
  # logging options
  NEVERMORE_BLUETOOTH_DEBUG=$<BOOL:${NEVERMORE_BLUETOOTH_DEBUG}>
  NEVERMORE_BLUETOOTH_LOW_LEVEL_DEBUG=$<BOOL:${NEVERMORE_BLUETOOTH_LOW_LEVEL_DEBUG}>
  NEVERMORE_I2C_DEBUG=$<BOOL:${NEVERMORE_I2C_DEBUG}>
  # feature options
  NEVERMORE_SETTINGS_PERSISTENCE=$<BOOL:${NEVERMORE_SETTINGS_PERSISTENCE}>

  # UGLY: Also defines the value of `nevermore::Priority::Communication`.
  # We want a priority above almost anything else.
  # Leaves 3 slots above (Max, USBD, Startup)
  CYW43_TASK_PRIORITY=28
  CYW43_HOST_NAME="Nevermore"
  CYW43_LWIP=0

  # default is 4.
  # Current users: tinyusb w/ DCD (1), stdio w/ USB (1), CYW43 driver (1), WS2812 (1), CST816S (1), GC9A01 (1)
  PICO_MAX_SHARED_IRQ_HANDLERS=6
  PARAM_ASSERTIONS_ENABLE_ALL=1
  PICO_USE_MALLOC_MUTEX=1

  WANT_HCI_DUMP=1

  CFG_TUSB_CONFIG_FILE="config/lib/tusb_config.h"
)

target_link_libraries(
  nevermore-controller
  PUBLIC # Pico SDK
         hardware_adc
         hardware_dma
         hardware_flash
         hardware_i2c
         hardware_pio
         hardware_pwm
         hardware_spi
         hardware_watchdog
         pico_btstack_ble
         pico_stdio_uart
        #  pico_stdio_usb
         pico_stdlib
         pico_time
         pico_flash
         pico_unique_id
         tinyusb_board
         tinyusb_device
         # Others
         FreeRTOS-Kernel
         FreeRTOS-Kernel-Heap4
         lvgl::lvgl
         picowota_client
)

if(${PICO_BOARD} STREQUAL "pico_w")
  target_link_libraries(nevermore-controller PUBLIC pico_cyw43_arch_sys_freertos)
endif()

if(${NEVERMORE_PICO_W_BT})
  target_link_libraries(nevermore-controller PUBLIC pico_btstack_cyw43)
endif()

target_compile_definitions(nevermore-controller PUBLIC
  USBD_MANUFACTURER="Nevermore"
  USBD_PRODUCT="Filter ${NEVERMORE_BOARD}"
)

target_compile_definitions(picowota PUBLIC
  USBD_MANUFACTURER="Nevermore"
  USBD_PRODUCT="Filter ${NEVERMORE_BOARD}"
)

list(TRANSFORM SRC_IN REPLACE "(.*)\.in$" "\\1" OUTPUT_VARIABLE SRC_IN_BYPRODUCTS)
# cmake-format: off
add_custom_target(
  nevermore-controller-generate-build-info
  COMMAND ${CMAKE_COMMAND}
          -D EXTRA_MODULES_DIR=${CMAKE_CURRENT_SOURCE_DIR}/cmake
          -D NEVERMORE_BOARD=${NEVERMORE_BOARD}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/configure_files.cmake
          -- ${SRC_IN}
  BYPRODUCTS ${SRC_IN_BYPRODUCTS}
)
# cmake-format: on

add_dependencies(nevermore-controller nevermore-controller-generate-build-info)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Specify source files explicitly b/c some SDK sources trigger warnings.
  # target_compile_options(nevermore-controller PRIVATE -Werror)
  set_source_files_properties(${SRC_FILES} PROPERTIES COMPILE_FLAGS -Werror)
  add_compile_definitions(PICO_DEBUG_MALLOC=0)
endif()

pico_btstack_make_gatt_header(nevermore-controller PRIVATE ${SRC_DIR}/nevermore.gatt)
pico_generate_pio_header(nevermore-controller ${SRC_DIR}/lib/pio_i2c.pio)
pico_generate_pio_header(nevermore-controller ${SRC_DIR}/ws2812.pio)
picowota_app_store_declare(nevermore-controller picowota)
picowota_share_cyw43_firmware(nevermore-controller)

add_executable(nevermore-controller-no-bootloader)
set_target_properties(nevermore-controller-no-bootloader PROPERTIES OUTPUT_NAME "nevermore-controller_${NEVERMORE_BOARD}")
target_link_libraries(nevermore-controller-no-bootloader PRIVATE nevermore-controller)
pico_add_extra_outputs(nevermore-controller-no-bootloader)
picowota_build_no_bootloader_with_app_store(nevermore-controller-no-bootloader picowota)

add_executable(nevermore-controller-ota)
set_target_properties(nevermore-controller-ota PROPERTIES OUTPUT_NAME "ota-nevermore-controller_${NEVERMORE_BOARD}")
target_link_libraries(nevermore-controller-ota PRIVATE nevermore-controller)
pico_add_extra_outputs(nevermore-controller-ota)
picowota_build_combined(nevermore-controller-ota picowota)
