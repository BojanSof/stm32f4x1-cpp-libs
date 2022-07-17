# ST_Link functions

set(BINARY ${TARGET}.bin)
set(FLASH_START "0x08000000")
add_custom_target(st-link-flash 
	COMMAND st-flash --reset write ${BINARY} ${FLASH_START}
	DEPENDS ${TARGET}.elf
	)

add_custom_target(st-link-erase 
	COMMAND st-flash erase
	)