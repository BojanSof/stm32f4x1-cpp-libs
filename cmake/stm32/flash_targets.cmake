include(${PROJECT_SOURCE_DIR}/cmake/stm32/stlink.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/stm32/jlink.cmake)

function(generate_flash_targets TARGET)
  st_link_flash(${TARGET})
  jlink_flash(${TARGET})
endfunction()