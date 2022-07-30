cmake_minimum_required(VERSION 3.18)

set(TARGET "ultrasonic-measure")

add_executable(${TARGET})
target_sources(${TARGET}
    PRIVATE
      UltrasonicMeasure.cpp
)

target_compile_features(${TARGET}
  PUBLIC
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    stm32f401-devices
    external-devices
    CMSIS::STM32::F401CC
    STM32::NoSys)

stm32_print_size_of_target(${TARGET})
stm32_generate_binary_file(${TARGET})

include(${PROJECT_SOURCE_DIR}/cmake/stm32/stlink.cmake)
st_link_flash(${TARGET})
include(${PROJECT_SOURCE_DIR}/cmake/stm32/jlink.cmake)
jlink_flash(${TARGET})
