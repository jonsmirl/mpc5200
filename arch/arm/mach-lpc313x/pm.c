/*  linux/arch/arm/mach-lpc313x/leds.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * LPC313x/4x/5x power management.
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

#include <linux/pm.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/cacheflush.h>


#define LPC313x_ISRAM_VA io_p2v(ISRAM0_PHYS)

/*
 * Pointers used for sizing and copying suspend function data
 */
extern int lpc313x_suspend_mem(void);
extern int lpc313x_suspend_mem_sz;


/* Enable/Disable external refresh controller used by
 * auto clock scaling feature of CGU.
 */
static void lpc313x_ext_refresh_en(int enable)
{
	if (enable)
		SYS_MPMC_TESTMODE0 |= _BIT(12);
	else
		SYS_MPMC_TESTMODE0 &= ~_BIT(12);

}
static int lpc313x_pm_valid_state(suspend_state_t state)
{
	switch (state) {
		case PM_SUSPEND_ON:
		case PM_SUSPEND_STANDBY:
		case PM_SUSPEND_MEM:
			return 1;

		default:
			return 0;
	}
}


static suspend_state_t target_state;

/*
 * Called after processes are frozen, but before we shutdown devices.
 */
static int lpc313x_pm_begin(suspend_state_t state)
{
	target_state = state;
	return 0;
}

static void lpc313x_clk_debug(void)
{
#ifdef CONFIG_PM_DEBUG
	u32 i, clk_pcr;
	/* print all clocks which are not auto_wake enabled and are still
	 * running before entering low-power states.
	 */
	for ( i = 0; i < CGU_SB_NR_CLK; i++) {
		clk_pcr = CGU_SB->clk_pcr[i];
		if ( (clk_pcr & CGU_SB_PCR_RUN) && 
			((clk_pcr & CGU_SB_PCR_WAKE_EN) == 0) )
			printk("Opps Clk: %d is still enabled\n", i);
	}
#endif
}

static int lpc313x_enter_sleep(u32 standby)
{
	int (*lpc313x_suspend_ptr) (u32);
	int i;
	u32 base_clk_state = 0;

#if defined(BACKUP_ISRAM)
	void *iram_swap_area;
#endif

	/* print clocks which are still on */
	lpc313x_clk_debug();

	lpc313x_ext_refresh_en(0);
	/*
	 * To simplify stand-by routine, set FFAST as source clock for the
	 * non-active switch side of SYS, APB0, APB1, APB2 & APB3 domains.
	 */
	for (i = 0; i < 2; i++) {
		if (CGU_SB->base_ssr[i] & CGU_SB_SCR_EN1)
			CGU_SB->base_fs2[i] = CGU_FIN_SELECT_FFAST;
		else
			CGU_SB->base_fs1[i] = CGU_FIN_SELECT_FFAST;
	}
	if (standby == 0) {
		/* switch off remaining pheripheral clock domains */
		for (i = 2; i < CGU_SB_NR_BASE; i++) {
			if (CGU_SB->base_scr[i] & CGU_SB_SCR_STOP)
				base_clk_state |= _BIT(i);
			else
				CGU_SB->base_scr[i] |= CGU_SB_SCR_STOP;
		}
	} else {
		/* we need to have interrupt controller clock on for 
		 * internal events to wake us up.
		 */
		CGU_SB->clk_pcr[CGU_SB_INTC_CLK_ID] = CGU_SB_PCR_RUN;
	}



#if defined(BACKUP_ISRAM)
	/* Allocate some space for temporary IRAM storage */
	iram_swap_area = kmalloc(lpc313x_suspend_mem_sz, GFP_KERNEL);
	if (!iram_swap_area) {
		printk(KERN_ERR
		       "PM Suspend: cannot allocate memory to save portion "
			"of SRAM\n");
		return -ENOMEM;
	}
	/* Backup a small area of IRAM used for the suspend code */
	memcpy(iram_swap_area, (void *) LPC313x_ISRAM_VA,
			lpc313x_suspend_mem_sz);
#endif

	/*
	 * Copy code to suspend system into IRAM. The suspend code
	 * needs to run from IRAM as DRAM may no longer be available
	 * when the PLL is stopped.
	 */
	memcpy((void *) LPC313x_ISRAM_VA, &lpc313x_suspend_mem,
			lpc313x_suspend_mem_sz);
	flush_icache_range((unsigned long)LPC313x_ISRAM_VA,
		(unsigned long)(LPC313x_ISRAM_VA) + lpc313x_suspend_mem_sz);

	/* Transfer to suspend code in IRAM */
	lpc313x_suspend_ptr = (void *) LPC313x_ISRAM_VA;
	(void) lpc313x_suspend_ptr(standby);

#if defined(BACKUP_ISRAM)
	/* Restore original IRAM contents */
	memcpy((void *) LPC313x_ISRAM_VA, iram_swap_area,
			lpc313x_suspend_mem_sz);

	kfree(iram_swap_area);
#endif

	if (standby == 0) {
		/* switch on domains clocks which were switched off in this
		 * routine.
		 */
		for (i = 2; i < CGU_SB_NR_BASE; i++) {
			if ((base_clk_state & _BIT(i)) == 0)
				CGU_SB->base_scr[i] &= ~CGU_SB_SCR_STOP;
		}
	} else {

		/* resume the state of interrupt controller clock */
		CGU_SB->clk_pcr[CGU_SB_INTC_CLK_ID] = CGU_SB_PCR_WAKE_EN |
					CGU_SB_PCR_RUN | CGU_SB_PCR_AUTO;
	}

	lpc313x_ext_refresh_en(1);

	return 0;
}



static int lpc313x_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
		/*
		 * Suspend-to-RAM is like STANDBY plus slow clock mode, so
		 * drivers must suspend more deeply:  only the base domains
		 * switches SYS_BABSE, APB0, APB1 are clocked using the
		 * main oscillator. The system can be woken-up by EXTERNAL
		 * events ONLY. Ie., external interrupts handles through
		 * event router.
		 */
		case PM_SUSPEND_MEM:
			ret = lpc313x_enter_sleep(0);
			break;

		/*
		 * STANDBY mode has *all* drivers suspended; ignores irqs not
		 * marked as 'wakeup' event sources; and reduces DRAM power.
		 * System can be woken-up by internal events. The internal
		 * blocks which are set to wake should have their clocks
		 * running without WAK_EN bit set in CGU.
		 */
		case PM_SUSPEND_STANDBY:
			ret = lpc313x_enter_sleep(1);
			break;

		case PM_SUSPEND_ON:
			asm("mcr p15, 0, r0, c7, c0, 4");	/* wait for interrupt */
			break;

		default:
			pr_debug("LPC31: PM - bogus suspend state %d\n", state);
			goto error;
	}

	pr_debug("LPC31: PM - wakeup \n");

error:
	target_state = PM_SUSPEND_ON;
	return ret;
}

/*
 * Called right prior to thawing processes.
 */
static void lpc313x_pm_end(void)
{
	target_state = PM_SUSPEND_ON;
}

/*
 * Call this from platform driver suspend() to see how deeply to suspend.
 * For internal events to wake the chip we should not stop the module 
 * clocks. 
 */
int lpc313x_entering_suspend_mem(void)
{
	return (target_state == PM_SUSPEND_MEM);
}
EXPORT_SYMBOL(lpc313x_entering_suspend_mem);


static struct platform_suspend_ops lpc313x_pm_ops ={
	.valid	= lpc313x_pm_valid_state,
	.begin	= lpc313x_pm_begin,
	.enter	= lpc313x_pm_enter,
	.end	= lpc313x_pm_end,
};

static int __init lpc313x_pm_init(void)
{
	pr_info("LPC31: Power Management init.\n");

	/* Make sure all systems clocks are marked
	 * as wakeable.
	 */


	suspend_set_ops(&lpc313x_pm_ops);

	return 0;
}
arch_initcall(lpc313x_pm_init);
