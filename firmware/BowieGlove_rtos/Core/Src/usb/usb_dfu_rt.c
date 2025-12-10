/*
 * usb_dfu_rt.c
 *
 *      Author: Mike Lambeta
 */
#include "main.h"

void tud_dfu_runtime_reboot_to_dfu_cb(void) {
	uint32_t boot_key = 0x157F32D4;
	uint32_t volatile *const sram_map = (uint32_t volatile*) 0x2001fffc;
	*sram_map = boot_key;
	NVIC_SystemReset();
}
