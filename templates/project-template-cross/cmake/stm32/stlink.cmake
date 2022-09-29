# ST_Link functions

function(st_link_flash TARGET)
	set(BINARY ${TARGET}.bin)
	set(FLASH_START "0x08000000")
	add_custom_target(${TARGET}-flash 
		COMMAND st-flash --reset write ${BINARY} ${FLASH_START}
		DEPENDS "$<TARGET_FILE:${TARGET}>"
	)
endfunction()