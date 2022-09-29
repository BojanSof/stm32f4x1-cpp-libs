cmake_minimum_required(VERSION 3.18)

set(TARGET "timer-interface-test")

add_executable(${TARGET})
target_sources(${TARGET}
  PRIVATE
    TimerTest.cpp
)

target_compile_features(${TARGET}
  PRIVATE
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    stm32f401-devices
)

stm32_generate_binary_file(${TARGET})

include(${PROJECT_SOURCE_DIR}/cmake/stm32/stlink.cmake)
st_link_flash(${TARGET})
include(${PROJECT_SOURCE_DIR}/cmake/stm32/jlink.cmake)
jlink_flash(${TARGET})
