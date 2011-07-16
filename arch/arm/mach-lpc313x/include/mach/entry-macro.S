/*
 * linux/arch/arm/mach-lpc313x/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for LPC313x-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>

		.macro	disable_fiq
		.endm

		.macro	get_irqnr_preamble, base, tmp
		.endm

		.macro	arch_ret_to_user, tmp1, tmp2
		.endm

#define IRQ_VEC_OFF     0x100      /* Offset to IRQ number  */
#define IRQ_STAT_OFF    0x200      /* Offset to IRQ status  */

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr	\base, =io_p2v(INTC_PHYS)
		@ Load offset & priority of the highest priority
		@ interrupt pending.
		ldr	\irqnr, [\base, #IRQ_VEC_OFF]
		mov	\irqnr, \irqnr, lsr #3
		/* Assuming INTC_IRQ_VEC_BASE is set 0 during init. 
		   If not then add masking instruction below. uncomment the following.
		and	\irqnr, \irqnr, #0x1F
		*/
		movs	\irqstat, \irqnr
		.endm
