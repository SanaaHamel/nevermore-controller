cmake_minimum_required(VERSION 3.20)

# vars pulled from parent context:
#   SRC_DIR_BOARDS
#   PICO_BOARD_HEADER_DIRS

set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)

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
set(SRC_FILES ${SRC_H} ${SRC_C} ${SRC_HPP} ${SRC_CPP}
  "${PROJECT_ROOT_DIR}/src/sdk/i2c_hw.cpp"
  "${PROJECT_ROOT_DIR}/src/utility/i2c.cpp"
)

add_library(nevermore-sensorium ${SRC_FILES})
target_include_directories(nevermore-sensorium PUBLIC
  "${SRC_DIR}"
  # for stuff we're stealing from nevermore-controller (e.g board configs & utils)
  "${PROJECT_ROOT_DIR}/src"
)

target_compile_options(nevermore-sensorium PUBLIC
  -Wall
  # disable useless warnings
  -Wno-psabi -Wno-trigraphs
)

target_compile_definitions(nevermore-sensorium PUBLIC
  # board config/defs
  NEVERMORE_BOARD="${NEVERMORE_BOARD}"
  NEVERMORE_BOARD_HEADER="config/pins/${NEVERMORE_BOARD}.hpp"
  NEVERMORE_PICO_W_BT=${NEVERMORE_PICO_W_BT}
  # logging options
  NEVERMORE_BLUETOOTH_DEBUG=$<BOOL:${NEVERMORE_BLUETOOTH_DEBUG}>
  NEVERMORE_BLUETOOTH_LOW_LEVEL_DEBUG=$<BOOL:${NEVERMORE_BLUETOOTH_LOW_LEVEL_DEBUG}>
  NEVERMORE_I2C_DEBUG=$<BOOL:${NEVERMORE_I2C_DEBUG}>

  PARAM_ASSERTIONS_ENABLE_ALL=1
  PICO_USE_MALLOC_MUTEX=1

  WANT_HCI_DUMP=1
)

target_link_libraries(
  nevermore-sensorium
  PUBLIC # Pico SDK
         hardware_i2c
         hardware_pwm
         hardware_spi
         hardware_watchdog
         pico_btstack_ble
         pico_stdio_uart
         pico_stdio_usb
         pico_stdlib
         pico_time
         # Others
         picowota_client
)

if(${PICO_BOARD} STREQUAL "pico_w")
  target_link_libraries(nevermore-sensorium PUBLIC pico_cyw43_arch_poll)
endif()

if(${NEVERMORE_PICO_W_BT})
  target_link_libraries(nevermore-sensorium PUBLIC pico_btstack_cyw43)
endif()

target_compile_definitions(nevermore-sensorium PUBLIC
  USBD_MANUFACTURER="Nevermore"
  USBD_PRODUCT="Sensorium ${NEVERMORE_BOARD}"
  CYW43_LWIP=0
)

list(TRANSFORM SRC_IN REPLACE "(.*)\.in$" "\\1" OUTPUT_VARIABLE SRC_IN_BYPRODUCTS)
# cmake-format: off
add_custom_target(
  nevermore-sensorium-generate-build-info
  COMMAND ${CMAKE_COMMAND}
          -D EXTRA_MODULES_DIR=${PROJECT_ROOT_DIR}/cmake
          -D NEVERMORE_BOARD=${NEVERMORE_BOARD}
          -P ${PROJECT_ROOT_DIR}/cmake/configure_files.cmake
          -- ${SRC_IN}
  BYPRODUCTS ${SRC_IN_BYPRODUCTS}
)
# cmake-format: on

add_dependencies(nevermore-sensorium nevermore-sensorium-generate-build-info)

pico_btstack_make_gatt_header(nevermore-sensorium PRIVATE ${SRC_DIR}/sensorium.gatt)
picowota_app_store_declare(nevermore-sensorium picowota)
picowota_share_cyw43_firmware(nevermore-sensorium)

add_executable(nevermore-sensorium-no-bootloader)
set_target_properties(nevermore-sensorium-no-bootloader PROPERTIES OUTPUT_NAME "nevermore-sensorium_${NEVERMORE_BOARD}")
target_link_libraries(nevermore-sensorium-no-bootloader PRIVATE nevermore-sensorium)
pico_add_extra_outputs(nevermore-sensorium-no-bootloader)
picowota_build_no_bootloader_with_app_store(nevermore-sensorium-no-bootloader picowota)

add_executable(nevermore-sensorium-ota)
set_target_properties(nevermore-sensorium-ota PROPERTIES OUTPUT_NAME "ota-nevermore-sensorium_${NEVERMORE_BOARD}")
target_link_libraries(nevermore-sensorium-ota PRIVATE nevermore-sensorium)
pico_add_extra_outputs(nevermore-sensorium-ota)
picowota_build_combined(nevermore-sensorium-ota picowota)
