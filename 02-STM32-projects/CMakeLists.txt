cmake_minimum_required(VERSION 3.18)

project(
  stm32-projects
  VERSION 0.1
  DESCRIPTION "Projects using STM32F401CC MCU"
  LANGUAGES C CXX ASM)

list(APPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(DEVICE STM32F401CC)

# fetch CMSIS and HAL
stm32_fetch_cmsis(F4)
#stm32_fetch_hal(F4)

find_package(CMSIS COMPONENTS ${DEVICE} REQUIRED)
#find_package(HAL COMPONENTS STM32F4 REQUIRED)

add_subdirectory(src)
