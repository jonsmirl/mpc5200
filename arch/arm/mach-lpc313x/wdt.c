/*
 * LPC313x Watchdog timer device registration
 *
 * drivers/watchdog/wdt_lpc313x.c
 *
 * Copyright (C) 2009 NXP Semiconductors
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
#include <linux/platform_device.h>
#include <mach/constants.h>
#include <mach/irqs.h>

static struct resource watchdog_resources[] = {
	{
		.start = WDT_PHYS,
		.end = WDT_PHYS + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device watchdog_device = {
	.name = "lpc313x-wdt",
	.id = -1,
	.num_resources = ARRAY_SIZE(watchdog_resources),
	.resource = watchdog_resources,
};

static int __init wdt_device_init(void)
{
	return platform_device_register(&watchdog_device);
}

arch_initcall(wdt_device_init);
