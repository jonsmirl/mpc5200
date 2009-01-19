/*
 * sound/soc/fsl/dspeak01_fabric.c -- The ALSA glue fabric for Digispeaker dspeak01
 *
 * Copyright 2008 Jon Smirl, Digispeaker
 * Author: Jon Smirl <jonsmirl@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_i2c.h>
#include <linux/i2c/max9485.h>

#include <sound/soc.h>
#include <sound/soc-of-simple.h>

#include "mpc5200_psc_i2s.h"

static struct dspeak01_fabric {
	struct i2c_client *clock;
} fabric;

static int dspeak01_fabric_startup(struct snd_pcm_substream *substream)
{
	printk("dspeak01_fabric_startup\n");
	return 0;
}

static void dspeak01_fabric_shutdown(struct snd_pcm_substream *substream)
{
	printk("dspeak01_fabric_shutdown\n");
}

static int dspeak01_fabric_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	uint rate, select;
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
    struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	printk("dspeak01_fabric_hw_params\n");

	switch (params_rate(params)) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		rate = 22579200;
		select = MAX9485_225792;
		break;
	default:
		rate = 24576000;
		select = MAX9485_245760;
		break;
	}
	max9485_set(fabric.clock, select | MAX9485_CLK_OUT_2);

	ret = cpu_dai->ops.set_sysclk(cpu_dai, MPC52xx_CLK_CELLSLAVE, rate, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static int dspeak01_fabric_hw_free(struct snd_pcm_substream *substream)
{
	printk("dspeak01_fabric_hw_free\n");
	return 0;
}

static int dspeak01_fabric_prepare(struct snd_pcm_substream *substream)
{
	printk("dspeak01_fabric_prepare\n");
	return 0;
}

static int dspeak01_fabric_trigger(struct snd_pcm_substream *substream, int trigger)
{
	printk("dspeak01_fabric_trigger\n");
	return 0;
}

static struct snd_soc_ops dspeak01_fabric_ops = {
	.startup = dspeak01_fabric_startup,
	.shutdown = dspeak01_fabric_shutdown,
	.hw_params = dspeak01_fabric_hw_params,
	.hw_free = dspeak01_fabric_hw_free,
	.prepare = dspeak01_fabric_prepare,
	.trigger = dspeak01_fabric_trigger,
};

static int __devinit dspeak01_fabric_probe(struct of_device *op,
				      const struct of_device_id *match)
{
	const phandle *handle;
	struct device_node *clock_node;
	unsigned int len;

	handle = of_get_property(op->node, "clock-handle", &len);
	if (!handle || len < sizeof(handle))
		return -ENODEV;

	clock_node = of_find_node_by_phandle(*handle);
	if (!clock_node)
		return -ENODEV;

	fabric.clock = of_find_i2c_device_by_node(clock_node);
	if (!fabric.clock)
		return -ENODEV;

	of_snd_soc_register_machine("DSPEAK01", &dspeak01_fabric_ops);
	return 0;
}

static int __exit dspeak01_fabric_remove(struct of_device *op)
{
	put_device(&fabric.clock->dev);
	return 0;
}

#ifdef CONFIG_PM

static int dspeak01_fabric_suspend(struct of_device *op,
	pm_message_t state)
{
	return 0;
}

static int dspeak01_fabric_resume(struct of_device *op)
{
	return 0;
}

#else
#define dspeak01_fabric_suspend NULL
#define dspeak01_fabric_resume  NULL
#endif

/* Match table for of_platform binding */
static struct of_device_id dspeak_fabric_match[] __devinitdata = {
	{ .compatible = "dspeak01-fabric", },
	{}
};
MODULE_DEVICE_TABLE(of, dspeak_fabric_match);

static struct of_platform_driver dspeak01_fabric_driver = {
	.match_table = dspeak_fabric_match,
	.probe		= dspeak01_fabric_probe,
	.remove		= __devexit_p(dspeak01_fabric_remove),
	.suspend	= dspeak01_fabric_suspend,
	.resume		= dspeak01_fabric_resume,
	.driver		= {
		.name		= "dspeak01-fabric",
		.owner		= THIS_MODULE,
	},
};

static int __init dspeak01_driver_init(void)
{
	return of_register_platform_driver(&dspeak01_fabric_driver);
}

static void __exit dspeak01_driver_exit(void)
{
	of_unregister_platform_driver(&dspeak01_fabric_driver);
}

module_init(dspeak01_driver_init);
module_exit(dspeak01_driver_exit);

/* Module information */
MODULE_AUTHOR("Jon Smirl");
MODULE_DESCRIPTION("ASOC Digispeaker fabric module");
MODULE_LICENSE("GPL");
