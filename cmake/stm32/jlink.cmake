# jlink functions

function(jlink_flash TARGET)
	set(BINARY ${TARGET}.bin)
	set(FLASH_START "0x08000000")
  configure_file(${PROJECT_SOURCE_DIR}/cmake/stm32/jlink-flash.in ${CMAKE_CURRENT_BINARY_DIR}/flash.jlink)
	add_custom_target(${TARGET}-jlink-flash 
		COMMAND JLinkExe -device ${DEVICE} -speed 4000 -if SWD -CommanderScript ${CMAKE_CURRENT_BINARY_DIR}/flash.jlink
		DEPENDS "$<TARGET_FILE:${TARGET}>"
	)
endfunction()