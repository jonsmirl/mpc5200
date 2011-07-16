/*  linux/arch/arm/mach-lpc313x/irq.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Interrupt controller and event router driver for LPC313x & LPC315x.
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

#include <linux/init.h>
#include <linux/list.h>
#include <linux/timer.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/irqs.h>
#include <mach/cgu.h>


static IRQ_EVENT_MAP_T irq_2_event[] = BOARD_IRQ_EVENT_MAP;

static void intc_mask_irq(unsigned int irq)
{
	INTC_REQ_REG(irq) = INTC_REQ_WE_ENABLE;
}

static void intc_unmask_irq(unsigned int irq)
{
	INTC_REQ_REG(irq) = INTC_REQ_ENABLE | INTC_REQ_WE_ENABLE;
}

static int intc_set_wake(unsigned irq, unsigned value)
{
	static u32 wake_ints = 0;

	if (value)
		/* save the irqs which wake */
		wake_ints |= _BIT(irq);
	else
		/* clear the irqs which don't wake */
		wake_ints &= ~_BIT(irq);

	/* Note: the clocks to corresponding blocks shouldn't be suspended
	 * by individual drivers for this logic to work.
	 */
	if (wake_ints) {
		/* enable ARM_IRQ routing to CGU_WAKEUP */
		EVRT_OUT_MASK_SET(4, EVT_GET_BANK(EVT_arm926_nirq)) = _BIT((EVT_arm926_nirq & 0x1F));
	} else {
		/* disable ARM_IRQ routing to CGU_WAKEUP */
		EVRT_OUT_MASK_CLR(4, EVT_GET_BANK(EVT_arm926_nirq)) = _BIT((EVT_arm926_nirq & 0x1F));
	}

	//printk("wake on irq=%d value=%d 0x%08x/0x%08x/0x%08x 0x%08x/0x%08x\r\n", irq, value, 
	//	EVRT_MASK(3), EVRT_APR(3), EVRT_ATR(3),
	//	EVRT_OUT_MASK(4,3), EVRT_OUT_PEND(4,3));

	return 0;
}

static struct irq_chip lpc313x_internal_chip = {
	.name = "INTC",
	.ack = intc_mask_irq,
	.mask = intc_mask_irq,
	.unmask = intc_unmask_irq,
	.set_wake = intc_set_wake,
};

static void evt_mask_irq(unsigned int irq)
{
	u32 bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;

	EVRT_MASK_CLR(bank) = _BIT(bit_pos);
}

static void evt_unmask_irq(unsigned int irq)
{
	u32 bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;

	EVRT_MASK_SET(bank) = _BIT(bit_pos);
}

static void evt_ack_irq(unsigned int irq)
{
	u32 bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;
	//EVRT_MASK_CLR(bank) = _BIT(bit_pos);
	EVRT_INT_CLR(bank) = _BIT(bit_pos);
}

static int evt_set_type(unsigned irq, unsigned type)
{
	u32 bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		EVRT_APR(bank) |= _BIT(bit_pos);
		EVRT_ATR(bank) |= _BIT(bit_pos);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		EVRT_APR(bank) &= ~_BIT(bit_pos);
		EVRT_ATR(bank) |= _BIT(bit_pos);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		EVRT_ATR(bank) |= _BIT(bit_pos);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		EVRT_APR(bank) |= _BIT(bit_pos);
		EVRT_ATR(bank) &= ~_BIT(bit_pos);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		EVRT_APR(bank) &= ~_BIT(bit_pos);
		EVRT_ATR(bank) &= ~_BIT(bit_pos);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int evt_set_wake(unsigned irq, unsigned value)
{
	u32 bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
	u32 bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;

	if (value)
		/* enable routing to CGU_WAKEUP */
		EVRT_OUT_MASK_SET(4, bank) = _BIT(bit_pos);
	else
		/* disable routing to CGU_WAKEUP */
		EVRT_OUT_MASK_CLR(4, bank) = _BIT(bit_pos);

	return 0;
}


static struct irq_chip lpc313x_evtr_chip = {
	.name = "EVENTROUTER",
	.ack = evt_ack_irq,
	.mask = evt_mask_irq,
	.unmask = evt_unmask_irq,
	.set_type = evt_set_type,
	.set_wake = evt_set_wake,
};


#define ROUTER_HDLR(n) \
	static void router##n##_handler (unsigned int irq, struct irq_desc *desc) { \
		u32 status, bank, bit_pos; \
		if (IRQ_EVTR##n##_START == IRQ_EVTR##n##_END) { \
			/* translate IRQ number */ \
			irq = IRQ_EVTR##n##_START; \
			generic_handle_irq(irq); \
		} else { \
			for (irq = IRQ_EVTR##n##_START; irq <= IRQ_EVTR##n##_END; irq++) {  \
				/* compute bank & bit position for the event_pin */ \
				bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin); \
				bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F; \
				status = EVRT_OUT_PEND(n, bank); \
				if (status & _BIT(bit_pos)) \
					generic_handle_irq(irq); \
			} \
		} \
	}


#if IRQ_EVTR0_END
ROUTER_HDLR(0)
#endif /* IRQ_EVTR0_END */

#if IRQ_EVTR1_END
ROUTER_HDLR(1)
#endif /* IRQ_EVTR1_END */

#if IRQ_EVTR2_END
ROUTER_HDLR(2)
#endif /* IRQ_EVTR2_END */

#if IRQ_EVTR3_END
ROUTER_HDLR(3)
#endif /* IRQ_EVTR3_END */


void __init lpc313x_init_irq(void)
{
	unsigned int irq;
	int i, j;
	u32 bank, bit_pos;

	/* enable clock to interrupt controller */
	cgu_clk_en_dis(CGU_SB_AHB2INTC_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_INTC_CLK_ID, 1);
	/* enable clock to Event router */
	cgu_clk_en_dis(CGU_SB_EVENT_ROUTER_PCLK_ID, 1);

	/* Set the vector base (we don't use direct vectoring, so this is 0) */
	INTC_IRQ_VEC_BASE = 0x00000000;
	INTC_FIQ_VEC_BASE = 0x00000000;

	/* mask all interrupt by setting high priority untill init is done*/
	INTC_IRQ_PRI_MASK = 0xFF;
	INTC_FIQ_PRI_MASK = 0xFF;

	/* mask all external events */
	for (i = 0; i < EVT_MAX_VALID_BANKS; i++)
	{
		/* mask all events */
		EVRT_MASK_CLR(i) = 0xFFFFFFFF;
		/* clear all pending events */
		EVRT_INT_CLR(i) = 0xFFFFFFFF;

		for (j = 0; j < EVT_MAX_VALID_INT_OUT; j++)
		{
			/* mask all events */
			EVRT_OUT_MASK_CLR(j,i) = 0xFFFFFFFF;
		}
	}

	/* Clear and disable all interrupts. Start from index 1 since 0 is unused.*/
	for (irq = 1; irq < NR_IRQ_CPU; irq++) {
		/* Set the initial control values */
		INTC_REQ_REG(irq) = INTC_REQ_WE_ENABLE;

		/* Initialize as high-active, Disable the interrupt,
		* Set target to IRQ , Set priority level to 1 (= lowest) for
		* all the interrupt lines */
		INTC_REQ_REG(irq) = INTC_REQ_WE_ACT_LOW |
			INTC_REQ_WE_ENABLE |
			INTC_REQ_TARGET_IRQ |
			INTC_REQ_PRIO_LVL(1) |
			INTC_REQ_WE_PRIO_LVL;

		set_irq_chip(irq, &lpc313x_internal_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	/* Now configure external/board interrupts using event router */
	for (irq = IRQ_EVT_START; irq < NR_IRQS; irq++) {
		/* compute bank & bit position for the event_pin */
		bank = EVT_GET_BANK(irq_2_event[irq - IRQ_EVT_START].event_pin);
		bit_pos = irq_2_event[irq - IRQ_EVT_START].event_pin & 0x1F;
		
		printk("irq=%d Event=0x%x bank:%d bit:%d type:%d\r\n", irq,
			irq_2_event[irq - IRQ_EVT_START].event_pin, bank,
			bit_pos, irq_2_event[irq - IRQ_EVT_START].type);

		set_irq_chip(irq, &lpc313x_evtr_chip);
		set_irq_flags(irq, IRQF_VALID);
		/* configure the interrupt senstivity */
		switch (irq_2_event[irq - IRQ_EVT_START].type) {
			case EVT_ACTIVE_LOW:
				EVRT_APR(bank) &= ~_BIT(bit_pos);
				EVRT_ATR(bank) &= ~_BIT(bit_pos);
				set_irq_handler(irq, handle_level_irq);
				break;
			case EVT_ACTIVE_HIGH:
				EVRT_APR(bank) |= _BIT(bit_pos);
				EVRT_ATR(bank) &= ~_BIT(bit_pos);
				set_irq_handler(irq, handle_level_irq);
				break;
			case EVT_FALLING_EDGE:
				EVRT_APR(bank) &= ~_BIT(bit_pos);
				EVRT_ATR(bank) |= _BIT(bit_pos);
				set_irq_handler(irq, handle_edge_irq);
				break;
			case EVT_RISING_EDGE:
				EVRT_APR(bank) |= _BIT(bit_pos);
				EVRT_ATR(bank) |= _BIT(bit_pos);
				set_irq_handler(irq, handle_edge_irq);
				break;
			case EVT_BOTH_EDGE:
				EVRT_ATR(bank) |= _BIT(bit_pos);
				set_irq_handler(irq, handle_edge_irq);
			default:
				printk("Invalid Event type.\r\n");
				break;
		}
		if ( (irq >= IRQ_EVTR0_START) && (irq <= IRQ_EVTR0_END) ) {
			/* enable routing to vector 0 */
			EVRT_OUT_MASK_SET(0, bank) = _BIT(bit_pos);
		} else if ( (irq >= IRQ_EVTR1_START) && (irq <= IRQ_EVTR1_END) ) {
			/* enable routing to vector 1 */
			EVRT_OUT_MASK_SET(1, bank) = _BIT(bit_pos);
		} else if ( (irq >= IRQ_EVTR2_START) && (irq <= IRQ_EVTR2_END) ) {
			/* enable routing to vector 2 */
			EVRT_OUT_MASK_SET(2, bank) = _BIT(bit_pos);
		} else if ( (irq >= IRQ_EVTR3_START) && (irq <= IRQ_EVTR3_END) ) {
			/* enable routing to vector 3 */
			EVRT_OUT_MASK_SET(3, bank) = _BIT(bit_pos);
		} else {
			printk("Invalid Event router setup.\r\n");
		}
	}
	/* for power management. Wake from internal irqs */
	EVRT_APR(3) &= ~_BIT(12);
	EVRT_ATR(3) &= ~_BIT(12);
	EVRT_MASK_SET(3) = _BIT(12);

	/* install IRQ_EVT_ROUTER0  chain handler */
#if IRQ_EVTR0_END
	/* install chain handler for IRQ_EVT_ROUTER0 */
	set_irq_chained_handler (IRQ_EVT_ROUTER0, router0_handler);
#endif

#if IRQ_EVTR1_END
	/* install chain handler for IRQ_EVT_ROUTER1 */
	set_irq_chained_handler (IRQ_EVT_ROUTER1, router1_handler);
#endif

#if IRQ_EVTR2_END
	/* install chain handler for IRQ_EVT_ROUTER2 */
	set_irq_chained_handler (IRQ_EVT_ROUTER2, router2_handler);
#endif

#if IRQ_EVTR3_END
	/* install chain handler for IRQ_EVT_ROUTER3 */
	set_irq_chained_handler (IRQ_EVT_ROUTER3, router3_handler);
#endif

	/* Set the priority treshold to 0, i.e. don't mask any interrupt */
	/* on the basis of priority level, for both targets (IRQ/FIQ)    */
	INTC_IRQ_PRI_MASK = 0;
	INTC_FIQ_PRI_MASK = 0;
}


