project(Tinyusb LANGUAGES C ASM)

add_library(Tinyusb)

set_target_properties(Tinyusb
  PROPERTIES
    C_STANDARD 11
)

target_include_directories(Tinyusb
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(Tinyusb
  PRIVATE
    "src/tusb.c"
    "src/class/cdc/cdc_device.c"
    "src/portable/synopsys/dwc2/dcd_dwc2.c"
)

target_link_libraries(Tinyusb
  PRIVATE
    ${CMSIS_LIBRARY}
    ${HAL_LIBRARY}
)