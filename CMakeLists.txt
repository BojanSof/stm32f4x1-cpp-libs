cmake_minimum_required(VERSION 3.18)

project(
  stm32f4x1-projects
  VERSION 0.1
  DESCRIPTION "CPP library for STM32F4 BlackPill boards and example projects"
  LANGUAGES C CXX ASM
)

list(APPEND CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_LIST_DIR}/cmake"
)

option(STM32F4X1_GENERATE_FLASH_TARGETS "Add JLink and ST-link flash targets" ON)

# add preprocessor definitions
add_compile_definitions(
  DEVICE=${DEVICE}
)

# fetch CMSIS
stm32_fetch_cmsis(F4)
# fetch HAL
# stm32_fetch_hal(F4)

find_package(CMSIS COMPONENTS ${DEVICE} REQUIRED)
# find_package(HAL COMPONENTS ${DEVICE} REQUIRED)
find_package(FreeRTOS COMPONENTS ARM_CM4F REQUIRED)

# set cmsis and hal library for linking based on device
if(DEVICE STREQUAL "STM32F401CC")
  set(CMSIS_LIBRARY CMSIS::STM32::F401CC)
elseif(DEVICE STREQUAL "STM32F411CE")
  set(CMSIS_LIBRARY CMSIS::STM32::F411CE)
endif()
# set(HAL_LIBRARY HAL::STM32::F4)

# fetch additional libraries
find_package(TinyUsb REQUIRED)
find_package(FsmGenerator REQUIRED)
find_package(EmbeddedGfx REQUIRED)

add_subdirectory(lib)
add_subdirectory(src)
