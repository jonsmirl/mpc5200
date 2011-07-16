/* linux/arch/arm/mach-lpc313x/include/mach/system.h
 *  
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <mach/hardware.h>
#include <mach/cgu.h>


static inline void arch_idle(void)
{
	cpu_do_idle ();
}

static inline void arch_reset(char mode, const char *cmd)
{
	printk("arch_reset: attempting watchdog reset\n");

	/* enable WDT clock */
	cgu_clk_en_dis(CGU_SB_WDOG_PCLK_ID, 1);

	/* Disable watchdog */
	WDT_TCR = 0;
	WDT_MCR = WDT_MCR_STOP_MR1 | WDT_MCR_INT_MR1;

	/*  If TC and MR1 are equal a reset is generated. */
	WDT_PR  = 0x00000002;
	WDT_TC  = 0x00000FF0;
	WDT_MR0 = 0x0000F000;
	WDT_MR1 = 0x00001000;
	WDT_EMR = WDT_EMR_CTRL1(0x3);
	/* Enable watchdog timer; assert reset at timer timeout */
	WDT_TCR = WDT_TCR_CNT_EN;
	cpu_reset (0);/* loop forever and wait for reset to happen */

	/*NOTREACHED*/
}
