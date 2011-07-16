/* linux/arch/arm/mach-lpc313x/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2008 NXP semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include "hardware.h"

		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst     \rx, #1                 @ MMU enabled?
		mov     \rx, #0x15000000	    @ physical base
		movne   \rx, \rx, lsr #4	    @ virtual base
		orrne   \rx, \rx, #0xf0000000   @ virtual base
		orr     \rx, \rx, #0x00001000   @ offset from base
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>

