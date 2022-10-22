project(TinyUsb LANGUAGES C ASM)

add_library(TinyUsb
  INTERFACE  
)

set_target_properties(TinyUsb
  PROPERTIES
    C_STANDARD 11
)

target_include_directories(TinyUsb
  INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_LIST_DIR}"
)

target_sources(TinyUsb
  INTERFACE
    "src/tusb.c"
    "src/device/usbd.c"
    "src/device/usbd_control.c"
    "src/common/tusb_fifo.c"
    "src/class/cdc/cdc_device.c"
    "src/class/msc/msc_device.c"
    "src/portable/synopsys/dwc2/dcd_dwc2.c"
)

target_link_libraries(TinyUsb
  INTERFACE
    ${CMSIS_LIBRARY}
)