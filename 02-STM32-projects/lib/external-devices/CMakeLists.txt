cmake_minimum_required(VERSION 3.18)

set(TARGET "external-devices")

add_library(${TARGET})
target_sources(${TARGET}
  PRIVATE
    "include/Piso.hpp"
    "include/Sipo.hpp"
    "include/SevenSegmentDisplay.hpp"
)

target_include_directories(${TARGET}
  PRIVATE
    src
  PUBLIC
    include
)

target_compile_features(${TARGET}
  PRIVATE
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    stm32f401-devices
)