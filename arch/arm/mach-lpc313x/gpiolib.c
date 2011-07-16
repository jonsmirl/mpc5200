/* linux/arch/arm/mach-lpc313x/gpiolib.c
 *
 * Copyright (c) 2011 Jon Smirl <jonsmirl@gmail.com>
 *
 * LPC313X GPIOlib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>
#include <linux/ioport.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/gpio.h>


/**
 * struct lpc313x_gpio_chip - wrapper for specific implementation of gpio
 * @chip: The chip structure to be exported via gpiolib.
 * @base: The base pointer to the gpio configuration registers.
 * @config: special function and pull-resistor control information.
 * @pm_save: Save information for suspend/resume support.
 *
 * This wrapper provides the necessary information for the Samsung
 * specific gpios being registered with gpiolib.
 */
struct lpc313x_gpio_chip {
	struct gpio_chip	chip;
	struct lpc313x_gpio_cfg	*config;
	struct lpc313x_gpio_pm	*pm;
	int			base;
#ifdef CONFIG_PM
	u32			pm_save[4];
#endif
};

static inline struct lpc313x_gpio_chip *to_lpc313x_gpio(struct gpio_chip *gpc)
{
	return container_of(gpc, struct lpc313x_gpio_chip, chip);
}

struct lpc313x_gpio_chip lpc313x_gpios[] = {
	[0] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_EBI_MCI,
			.owner			= THIS_MODULE,
			.label			= "EBI_MCI",
			.ngpio			= 32,
		},
	},
	[1] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_EBI_I2STX_0,
			.owner			= THIS_MODULE,
			.label			= "EBI_I2STX_0",
			.ngpio			= 10,
		},
	},
	[2] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_CGU,
			.owner			= THIS_MODULE,
			.label			= "CGU",
			.ngpio			= 1,
		},
	},
	[3] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_I2SRX_0,
			.owner			= THIS_MODULE,
			.label			= "I2SRX_0",
			.ngpio			= 3,
		},
	},
	[4] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_I2SRX_1,
			.owner			= THIS_MODULE,
			.label			= "I2SRX_0",
			.ngpio			= 3,
		},
	},
	[5] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_I2STX_1,
			.owner			= THIS_MODULE,
			.label			= "I2STX_1",
			.ngpio			= 4,
		},
	},
	[6] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_EBI,
			.owner			= THIS_MODULE,
			.label			= "EBI",
			.ngpio			= 16,
		},
	},
	[7] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_GPIO,
			.owner			= THIS_MODULE,
			.label			= "GPIO",
			.ngpio			= 15,
		},
	},
	[8] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_I2C1,
			.owner			= THIS_MODULE,
			.label			= "I2C1",
			.ngpio			= 2,
		},
	},
	[9] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_SPI,
			.owner			= THIS_MODULE,
			.label			= "SPI",
			.ngpio			= 5,
		},
	},
	[10] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_NAND_CTRL,
			.owner			= THIS_MODULE,
			.label			= "NAND_CTRL",
			.ngpio			= 4,
		},
	},
	[11] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_PWM,
			.owner			= THIS_MODULE,
			.label			= "PWM",
			.ngpio			= 1,
		},
	},
	[12] = {
		.base	= GPIO_PHYS,
		.chip	= {
			.base			= IOCONF_UART,
			.owner			= THIS_MODULE,
			.label			= "UART",
			.ngpio			= 2,
		},
	},
};


static inline int lpc3131_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return lpc313x_gpio_direction_input(chip->base + offset);
}

static inline int lpc3131_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return lpc313x_gpio_direction_output(chip->base + offset, value);
}

static inline int lpc3131_gpio_get_value(struct gpio_chip *chip, unsigned offset)
{
	return lpc313x_gpio_get_value(chip->base + offset);
}

static inline void lpc3131_gpio_set_value(struct gpio_chip *chip, unsigned offset, int value)
{
	lpc313x_gpio_set_value(chip->base + offset, value);
}

__init void lpc313x_gpiolib_add(struct lpc313x_gpio_chip *chip)
{
	struct gpio_chip *gc = &chip->chip;
	int ret;

	BUG_ON(!chip->base);
	BUG_ON(!gc->label);
	BUG_ON(!gc->ngpio);

	if (!gc->direction_input)
		gc->direction_input = lpc3131_gpio_direction_input;
	if (!gc->direction_output)
		gc->direction_output = lpc3131_gpio_direction_output;
	if (!gc->set)
		gc->set = lpc3131_gpio_set_value;
	if (!gc->get)
		gc->get = lpc3131_gpio_get_value;

	/* gpiochip_add() prints own failure message on error. */
	ret = gpiochip_add(gc);
}

static __init int lpc313x_gpiolib_init(void)
{
	struct lpc313x_gpio_chip *chip = lpc313x_gpios;
	int gpn;

	for (gpn = 0; gpn < ARRAY_SIZE(lpc313x_gpios); gpn++, chip++)
		lpc313x_gpiolib_add(chip);

	return 0;
}

core_initcall(lpc313x_gpiolib_init);
