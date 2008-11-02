/*
 * OpenFirmware bindings for the MMC-over-SPI driver
 *
 * Copyright (c) MontaVista Software, Inc. 2008.
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/spi/spi.h>
#include <linux/spi/mmc_spi.h>
#include <linux/mmc/host.h>

/*
 * XXX: Place it somewhere in the generic MMC code?
 */
static int vdd_to_bitnum(int vdd)
{
	int bit;
	const int max_bit = ilog2(MMC_VDD_35_36);

	if (vdd < 1650 || vdd > 3600)
		return -EINVAL;

	if (vdd >= 1650 && vdd <= 1950)
		return ilog2(MMC_VDD_165_195);

	/* base 2000 mV, step 100 mV, bit's base 8 */
	bit = (vdd - 2000) / 100 + 8;
	if (bit > max_bit)
		return max_bit;
	return bit;
}

static int vddrange_to_ocrmask(int vdd_min, int vdd_max, unsigned int *mask)
{
	if (vdd_max < vdd_min)
		return -EINVAL;

	vdd_max = vdd_to_bitnum(vdd_max);
	if (vdd_max < 0)
		return -EINVAL;

	vdd_min = vdd_to_bitnum(vdd_min);
	if (vdd_min < 0)
		return -EINVAL;

	/* fill the mask, from max bit to min bit */
	while (vdd_max >= vdd_min)
		*mask |= 1 << vdd_max--;
	return 0;
}

struct of_mmc_spi {
	int wp_gpio;
	int cd_gpio;
	struct mmc_spi_platform_data mmc_pdata;
};

static struct of_mmc_spi *to_of_mmc_spi(struct device *dev)
{
	return container_of(dev->platform_data, struct of_mmc_spi, mmc_pdata);
}

static int mmc_get_ro(struct device *dev)
{
	struct of_mmc_spi *oms = to_of_mmc_spi(dev);

	return gpio_get_value(oms->wp_gpio);
}

static int mmc_get_cd(struct device *dev)
{
	struct of_mmc_spi *oms = to_of_mmc_spi(dev);

	return gpio_get_value(oms->cd_gpio);
}

static int of_mmc_spi_add(struct device *dev)
{
	int ret = -EINVAL;
	struct device_node *np = dev->archdata.of_node;
	struct of_mmc_spi *oms;
	const u32 *voltage_range;
	int size;

	if (!np || !of_device_is_compatible(np, "mmc-spi"))
		return NOTIFY_DONE;

	oms = kzalloc(sizeof(*oms), GFP_KERNEL);
	if (!oms)
		return notifier_to_errno(-ENOMEM);

	/* We don't support interrupts yet, let's poll. */
	oms->mmc_pdata.caps |= MMC_CAP_NEEDS_POLL;

	voltage_range = of_get_property(np, "voltage-range", &size);
	if (!voltage_range || size < sizeof(*voltage_range) * 2) {
		dev_err(dev, "OF: voltage-range unspecified\n");
		goto err_ocr;
	}

	ret = vddrange_to_ocrmask(voltage_range[0], voltage_range[1],
				  &oms->mmc_pdata.ocr_mask);
	if (ret) {
		dev_err(dev, "OF: specified voltage-range is invalid\n");
		goto err_ocr;
	}

	oms->wp_gpio = of_get_gpio(np, 0);
	if (gpio_is_valid(oms->wp_gpio)) {
		ret = gpio_request(oms->wp_gpio, dev->bus_id);
		if (ret < 0)
			goto err_wp_gpio;
		oms->mmc_pdata.get_ro = &mmc_get_ro;
	}

	oms->cd_gpio = of_get_gpio(np, 1);
	if (gpio_is_valid(oms->cd_gpio)) {
		ret = gpio_request(oms->cd_gpio, dev->bus_id);
		if (ret < 0)
			goto err_cd_gpio;
		oms->mmc_pdata.get_cd = &mmc_get_cd;
	}

	dev->platform_data = &oms->mmc_pdata;

	return NOTIFY_STOP;

err_cd_gpio:
	if (gpio_is_valid(oms->wp_gpio))
		gpio_free(oms->wp_gpio);
err_wp_gpio:
err_ocr:
	kfree(oms);
	return notifier_to_errno(ret);
}

static int of_mmc_spi_del(struct device *dev)
{
	struct device_node *np = dev->archdata.of_node;
	struct of_mmc_spi *oms;

	if (!np || !of_device_is_compatible(np, "mmc-spi") ||
			!dev->platform_data)
		return NOTIFY_DONE;

	oms = to_of_mmc_spi(dev);

	if (gpio_is_valid(oms->cd_gpio))
		gpio_free(oms->cd_gpio);
	if (gpio_is_valid(oms->wp_gpio))
		gpio_free(oms->wp_gpio);

	kfree(oms);
	return NOTIFY_STOP;
}

static int of_mmc_spi_notify(struct notifier_block *nb, unsigned long action,
			     void *_dev)
{
	struct device *dev = _dev;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		return of_mmc_spi_add(dev);
	case BUS_NOTIFY_DEL_DEVICE:
		return of_mmc_spi_del(dev);
	};
	return NOTIFY_DONE;
}

static struct notifier_block of_mmc_spi_notifier = {
	.notifier_call = of_mmc_spi_notify,
};

/*
 * Should be called early enough, but after SPI core initializes. So, we
 * use subsys_initcall (as in the SPI core), and link order guaranties
 * that we'll be called at the right time.
 */
static int __init of_mmc_spi_init(void)
{
	int ret;

	ret = bus_register_notifier(&spi_bus_type, &of_mmc_spi_notifier);
	if (ret) {
		pr_err("%s: unable to register notifier on the spi bus\n",
			__func__);
		return ret;
	}

	return 0;
}
subsys_initcall(of_mmc_spi_init);

MODULE_DESCRIPTION("OpenFirmware bindings for the MMC-over-SPI driver");
MODULE_AUTHOR("Anton Vorontsov <avorontsov@ru.mvista.com>");
MODULE_LICENSE("GPL");
