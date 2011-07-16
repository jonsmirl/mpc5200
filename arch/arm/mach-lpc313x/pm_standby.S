/*  linux/arch/arm/mach-lpc313x/pm_standby.S
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2010 NXP semiconductors
 *
 * Power management code to put LPC313x in sleep state.
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

#include <linux/linkage.h>
#include <mach/hardware.h>

#define HPLL_LOCK_TIMEOUT	1000

/* CGU defines */
#define LPC313x_CGU_NR_BASE     12

/* CGU register offsets */
#define LPC313x_CGU_PWR_OFS      0xC00
#define LPC313x_CGU_HP0_MD_OFS   0xD00
#define LPC313x_CGU_HP0_ST_OFS   0xD04
#define LPC313x_CGU_HP1_MD_OFS   0xD38
#define LPC313x_CGU_HP1_ST_OFS   0xD3C

/* CGU bit defines */
#define LPC313x_PWR_NORM		  (1)
#define LPC313x_PWR_SLEEP		  (3)
#define LPC313x_HP_ST_LOCK      (1 << 0)
#define LPC313x_HP_MODE_PD      (1 << 2)
#define LPC313x_HP_MODE_EN      (1 << 0)

/* MPMC register offsets */
#define LPC313x_MPMC_CTL_OFS    0x000
#define LPC313x_MPMC_STAT_OFS   0x004
#define LPC313x_MPMC_DYNC_OFS   0x020

/* MPMC bit defines */
#define LPC313x_MPMC_LOW        (0x5)
#define LPC313x_MPMC_NORM       (0x1)
#define LPC313x_DYNC_SR         (1 << 2)
#define LPC313x_STAT_SR         (1 << 2)
#define LPC313x_STAT_WB         (1 << 1)
#define LPC313x_STAT_BS         (1 << 0)

/*
 * Wait until SYSTEM PLL (HPLL0) has locked.
 */
	.macro wait_hp0_lock
	tst r10, #LPC313x_HP_MODE_PD
	bne 2f
	mov	r4, #HPLL_LOCK_TIMEOUT
1:	sub	r4, r4, #1
	cmp	r4, #0
	beq	2f
	ldr	r3, [r1, #LPC313x_CGU_HP0_ST_OFS]
	tst	r3, #LPC313x_HP_ST_LOCK
	beq	1b
2:
	.endm

/*
 * Wait until SYSTEM PLL (HPLL1) has locked.
 */
	.macro wait_hp1_lock
	mov	r4, #HPLL_LOCK_TIMEOUT
	tst r11, #LPC313x_HP_MODE_PD
	bne 2f
1:	sub	r4, r4, #1
	cmp	r4, #0
	beq	2f
	ldr	r3, [r1, #LPC313x_CGU_HP1_ST_OFS]
	tst	r3, #LPC313x_HP_ST_LOCK
	beq	1b
2:
	.endm

	.text

ENTRY(lpc313x_suspend_mem)
	/* Save registers on stack */
	stmfd	sp!, {r0 - r12, lr}

	/*
	 * Register usage:
	 *  R1 = Base address of LPC31 CGU
	 *  R2 = Base address of LPC31 MPMC
	 *  R3 = temporary register
	 *  R4 = temporary register
	 *  R5 = temporary register
	 *  R6 = temporary register
	 *  R7 = temporary register
	 *  R8 = temporary register
	 *  R9 = temporary register
	 *  R10 = HPLL0 state
	 *  R11 = HPLL1 state
	 */
	ldr	r1, .lpc313x_va_base_cgu
	ldr	r2, .lpc313x_va_base_mpmc

	/* Drain write buffer */
	mcr	p15, 0, r0, c7, c10, 4

	/* Put SDRAM in self-refresh mode */

	/* Wait for SDRAM busy status to go busy and then idle
	 * This guarantees a small windows where DRAM isn't busy
	 */
1:	ldr r4, [r2, #LPC313x_MPMC_STAT_OFS]
	tst r4, #LPC313x_STAT_WB
	bne 1b
2:	ldr r4, [r2, #LPC313x_MPMC_STAT_OFS]
	tst r4, #LPC313x_STAT_BS
	beq 2b
3:	ldr r4, [r2, #LPC313x_MPMC_STAT_OFS]
	tst r4, #LPC313x_STAT_BS
	bne 3b
	mov r3, #LPC313x_DYNC_SR
	/* Enable SDRAM self-refresh mode */
	str	r3, [r2, #LPC313x_MPMC_DYNC_OFS]
	/* wait until SDRAM enters self-refresh */
4:	ldr r4, [r2, #LPC313x_MPMC_STAT_OFS]
	tst r4, #LPC313x_STAT_SR
	beq 4b
	/* put MPMC in low-power mode */
	mov r3, #LPC313x_MPMC_LOW
	/* Enable SDRAM self-refresh mode */
	str	r3, [r2, #LPC313x_MPMC_CTL_OFS]

	/* Skip PLL power-down for standby. */
	cmp r0, #0
	bne 5f
	/* Switch all bases domain clocks to FFAST.
	 * lpc313x_enter_suspend_mem() is called before this.
	 */
	ldr	r5, [r1]
	eor r5, r5, #3
	str	r5, [r1]
	ldr	r5, [r1, #4]
	eor r5, r5, #3
	str	r5, [r1, #4]

	/*
	 * power-down plls
	 */
	ldr	r10, [r1, #LPC313x_CGU_HP0_MD_OFS]
	orr r3, r12, #LPC313x_HP_MODE_PD
	str	r3, [r1, #LPC313x_CGU_HP0_MD_OFS]
	ldr	r11, [r1, #LPC313x_CGU_HP1_MD_OFS]
	orr r3, r12, #LPC313x_HP_MODE_PD
	str	r3, [r1, #LPC313x_CGU_HP1_MD_OFS]

	/* Go to sleep zzzzzzzzz */
5:	mov r3, #LPC313x_PWR_SLEEP
	str r3, [r1, #LPC313x_CGU_PWR_OFS]

	/* Skip PLL power-ups for standby. */
	cmp r0, #0
	bne 6f
	/* Turn on plls */
	str	r10, [r1, #LPC313x_CGU_HP0_MD_OFS]
	str	r11, [r1, #LPC313x_CGU_HP1_MD_OFS]

	wait_hp0_lock
	wait_hp1_lock

	/*
	 * Restore domain clocks
	 */
	ldr	r5, [r1]
	eor r5, r5, #3
	str	r5, [r1]
	ldr	r5, [r1, #4]
	eor r5, r5, #3
	str	r5, [r1, #4]

	/* restore MPMC from low-power mode */
6:	MOV r3, #LPC313x_MPMC_NORM
	/* Enable SDRAM self-refresh mode */
	str	r3, [r2, #LPC313x_MPMC_CTL_OFS]

	/* Restore dync_ctl. Remove self-refresh. */
	mov r3, #0
	str	r3, [r2, #LPC313x_MPMC_DYNC_OFS]
	/* wait until SDRAM exits self-refresh */
7:	ldr r4, [r2, #LPC313x_MPMC_STAT_OFS]
	tst r4, #LPC313x_STAT_SR
	bne 7b

	/* Restore registers, and return */
	ldmfd	sp!, {r0 - r12, pc}


.lpc313x_va_base_cgu:
	.word io_p2v(CGU_SB_PHYS)

.lpc313x_va_base_mpmc:
	.word io_p2v(MPMC_PHYS)

ENTRY(lpc313x_suspend_mem_sz)
	.word .-lpc313x_suspend_mem


