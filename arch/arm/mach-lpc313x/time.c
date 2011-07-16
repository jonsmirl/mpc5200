/*  arch/arm/mach-lpc313x/time.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  Timer driver for LPC313x & LPC315x.
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/leds.h>

#include <asm/mach/time.h>
#include <mach/gpio.h>
#include <mach/board.h>
//#include <mach/cgu.h>


static irqreturn_t lpc313x_timer_interrupt(int irq, void *dev_id)
{
	TIMER_CLEAR(TIMER0_PHYS) = 0;
	timer_tick();
	return IRQ_HANDLED;
}

static struct irqaction lpc313x_timer_irq = {
	.name		= "LPC313x Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= lpc313x_timer_interrupt,
};

static void __init lpc313x_timer_init (void)
{
	/* Switch on needed Timer clocks & switch off others*/
	cgu_clk_en_dis(CGU_SB_TIMER0_PCLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_TIMER1_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_TIMER2_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_TIMER3_PCLK_ID, 0);

	/* Stop/disable all timers */
	TIMER_CONTROL(TIMER0_PHYS) = 0;

	TIMER_LOAD(TIMER0_PHYS) = LATCH;
	TIMER_CONTROL(TIMER0_PHYS) = (TM_CTRL_ENABLE | TM_CTRL_PERIODIC);
	TIMER_CLEAR(TIMER0_PHYS) = 0;
	setup_irq (IRQ_TIMER0, &lpc313x_timer_irq);
}

/*!
 * Returns number of us since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
static unsigned long lpc313x_gettimeoffset(void)
{
	u32 elapsed = LATCH - TIMER_VALUE(TIMER0_PHYS);
	return ((elapsed * 100) / (XTAL_CLOCK / 20000));
}

static void lpc313x_timer_suspend(void)
{
	TIMER_CONTROL(TIMER0_PHYS) &= ~TM_CTRL_ENABLE;	/* disable timers */
}

static void lpc313x_timer_resume(void)
{
	TIMER_CONTROL(TIMER0_PHYS) |= TM_CTRL_ENABLE;	/* enable timers */
}


struct sys_timer lpc313x_timer = {
	.init = lpc313x_timer_init,
	.offset = lpc313x_gettimeoffset,
	.suspend = lpc313x_timer_suspend,
	.resume = lpc313x_timer_resume,
};
