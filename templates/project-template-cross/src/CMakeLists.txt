cmake_minimum_required(VERSION 3.16)

# fetch CMSIS and HAL
stm32_fetch_cmsis(F4)
#stm32_fetch_hal(F4)

find_package(CMSIS COMPONENTS STM32F401CC REQUIRED)
#find_package(HAL COMPONENTS STM32F4 REQUIRED)

add_executable(stm32f4-blink)
target_sources(stm32f4-blink
    PRIVATE
      blink.cpp
)

target_compile_features(stm32f4-blink
  PUBLIC
    cxx_std_17)

target_link_libraries(stm32f4-blink
  PUBLIC
    CMSIS::STM32::F401CC
    STM32::NoSys)

stm32_generate_binary_file(stm32f4-blink)

include(${PROJECT_SOURCE_DIR}/cmake/stm32/stlink.cmake)
st_link_flash(stm32f4-blink)