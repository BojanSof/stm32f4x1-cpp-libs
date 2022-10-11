include(FetchContent)

FetchContent_Declare(
  Tinyusb
  GIT_REPOSITORY https://github.com/hathach/tinyusb.git
  GIT_TAG        "0.14.0"
  GIT_SUBMODULES ""# "hw/mcu/st/cmsis_device_f4" "hw/mcu/st/stm32f4xx_hal_driver"
  GIT_SHALLOW    TRUE
  GIT_PROGRESS   TRUE
)

FetchContent_GetProperties(Tinyusb)
if(NOT Tinyusb_POPULATED)
  FetchContent_Populate(Tinyusb)
  write_file(${tinyusb_SOURCE_DIR}/CMakeLists.txt "include(${CMAKE_CURRENT_LIST_DIR}/tinyusb.cmake)")
  add_subdirectory(${tinyusb_SOURCE_DIR} ${tinyusb_BINARY_DIR})
endif()