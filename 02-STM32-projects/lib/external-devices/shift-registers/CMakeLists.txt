cmake_minimum_required(VERSION 3.18)

set(TARGET "shift-registers")

add_library(${TARGET} INTERFACE)
target_sources(${TARGET}
  INTERFACE
    "include/Piso.hpp"
    "include/Sipo.hpp"
)

target_include_directories(${TARGET}
  INTERFACE
    include
)

target_compile_features(${TARGET}
  INTERFACE
    cxx_std_17)

target_link_libraries(${TARGET}
  INTERFACE
    stm32f401-devices
)