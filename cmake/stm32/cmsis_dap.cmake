function(cmsis_dap_flash TARGET)
	set(BINARY ${TARGET}.bin)
	set(FLASH_START "0x08000000")
	add_custom_target(${TARGET}-cmsis_dap-flash 
		COMMAND openocd -f ${PROJECT_SOURCE_DIR}/cmake/stm32/openocd.cfg
							-c "init"
              -c "reset halt"
              -c "program ${BINARY} ${FLASH_START}"
              -c "reset run"
              -c "shutdown"
		DEPENDS "$<TARGET_FILE:${TARGET}>"
	)
endfunction()
