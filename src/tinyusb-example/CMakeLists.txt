cmake_minimum_required(VERSION 3.18)

add_library(TinyUsbCdc
  STATIC
    tusb_config.h
)

target_include_directories(TinyUsbCdc
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(TinyUsbCdc
  PUBLIC
    TinyUsb
)

set(TARGET "tinyusb-example")

add_executable(${TARGET})
target_sources(${TARGET}
  PRIVATE
    msc-disk.c
    usb-descriptors.c
    CdcExample.cpp
)

target_compile_features(${TARGET}
  PRIVATE
    cxx_std_17
)

target_link_libraries(${TARGET}
  PUBLIC
    stm32f4x1-devices
    TinyUsbCdc
)

stm32_generate_binary_file(${TARGET})
if(STM32F4X1_GENERATE_FLASH_TARGETS)
  include(${PROJECT_SOURCE_DIR}/cmake/stm32/flash_targets.cmake)
  generate_flash_targets(${TARGET})
endif()
