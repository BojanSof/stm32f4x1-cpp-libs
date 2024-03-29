cmake_minimum_required(VERSION 3.18)

set(TARGET "blink-first-example")

add_executable(${TARGET})
target_sources(${TARGET}
    PRIVATE
      blink.cpp
)

target_compile_features(${TARGET}
  PRIVATE
    cxx_std_17)

target_link_libraries(${TARGET}
  PUBLIC
    ${CMSIS_LIBRARY}
    STM32::NoSys
    STM32::Nano)

stm32_generate_binary_file(${TARGET})
if(STM32F4X1_GENERATE_FLASH_TARGETS)
  include(${PROJECT_SOURCE_DIR}/cmake/stm32/flash_targets.cmake)
  generate_flash_targets(${TARGET})
endif()