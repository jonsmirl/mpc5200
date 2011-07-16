/*  linux/arch/arm/mach-lpc313x/leds.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * LED driver for val3153-based boards.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <asm/leds.h>
#include <mach/gpio.h>


static inline void val3153_led_on(unsigned int led)
{
	gpio_set_value(led, 0);
}

static inline void val3153_led_off(unsigned int led)
{
	gpio_set_value(led, 1);
}

static inline void val3153_led_toggle(unsigned int led)
{
	unsigned long is_off = gpio_get_value(led);
	if (is_off)
		val3153_led_on(led);
	else
		val3153_led_off(led);
}


/*
 * Handle LED events.
 */
static void val3153_leds_event(led_event_t evt)
{
	unsigned long flags;

	local_irq_save(flags);

	switch(evt) {
	case led_start:		/* System startup */
		val3153_led_on(GPIO_GPIO0);
		break;

	case led_stop:		/* System stop / suspend */
		val3153_led_off(GPIO_GPIO0);
		break;

#ifdef CONFIG_LEDS_TIMER
	case led_timer:		/* Every 50 timer ticks */
		val3153_led_toggle(GPIO_GPIO2);
		break;
#endif

#ifdef CONFIG_LEDS_CPU
	case led_idle_start:	/* Entering idle state */
		val3153_led_off(GPIO_GPIO0);
		break;

	case led_idle_end:	/* Exit idle state */
		val3153_led_on(GPIO_GPIO0);
		break;
#endif

	default:
		break;
	}

	local_irq_restore(flags);
}


int __init leds_init(void)
{
	leds_event = val3153_leds_event;

	leds_event(led_start);
	return 0;
}

__initcall(leds_init);
