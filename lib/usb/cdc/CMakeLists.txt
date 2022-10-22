cmake_minimum_required(VERSION 3.18)

add_library(TinyUsbCdc STATIC)

target_include_directories(TinyUsbCdc
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_sources(TinyUsbCdc
  PUBLIC
    "tusb_config.h"
  PRIVATE
    "usb-descriptors.c"
)

target_link_libraries(TinyUsbCdc
  PUBLIC
    TinyUsb
)