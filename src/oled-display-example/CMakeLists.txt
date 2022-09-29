cmake_minimum_required(VERSION 3.18)

set(TARGET "oled-display-example")

add_executable(${TARGET})
target_sources(${TARGET}
  PRIVATE
    OledDisplay.cpp
)

target_compile_features(${TARGET}
  PRIVATE
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    stm32f401-devices
    oled-display
    embedded-gfx
)

stm32_print_size_of_target(${TARGET})
stm32_generate_binary_file(${TARGET})

include(${PROJECT_SOURCE_DIR}/cmake/stm32/stlink.cmake)
st_link_flash(${TARGET})
include(${PROJECT_SOURCE_DIR}/cmake/stm32/jlink.cmake)
jlink_flash(${TARGET})