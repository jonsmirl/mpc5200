/*
 * sound/soc/lpc313x/lpc315x-codec.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "../codecs/lpc315x_codec.h"
#include "lpc313x-pcm.h"
#include "lpc313x-i2s.h"
#include "lpc313x-i2s-clocking.h"

#include <linux/io.h>
#include <mach/cgu.h>
#include <mach/board.h>
#include <mach/registers.h>

#define SND_MODNAME "lpc315x_machine"

static int ea315x_lpc315x_codec_hw_params(struct snd_pcm_substream
		*substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	const unsigned int fmt = (SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_CBS_CFS);
	int ret;

	/* Set the CPU I2S rate clock (first) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, params_rate(params),
					    SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set I2S clock (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	/* Set CPU and CODEC DAI format */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set CPU DAI format (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_warning("%s: "
			   "Failed to set CODEC DAI format (%d)\n",
			   SND_MODNAME, ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops ea315x_lpc315x_codec_ops = {
	.hw_params = ea315x_lpc315x_codec_hw_params,
};

static const struct snd_soc_dapm_widget ea315x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Headphone connected to HP_OUTR, HP_OUTL */
	{"Headphone Jack", NULL, "HP_OUTR"},
	{"Headphone Jack", NULL, "HP_OUTL"},

	/* Mic connected to ADC_VINM */
	{"ADC_MIC", NULL, "Mic Jack"},

	/* Line In connected to ADC_VINR, ADC_VINL */
	{"ADC_VINL", NULL, "Line In"},
	{"ADC_VINR", NULL, "Line In"},

	/* Line In connected to ADC_TINR, ADC_TINL */
	{"ADC_TINL", NULL, "Line In"},
	{"ADC_TINR", NULL, "Line In"},
};

static int ea315x_lpc315x_codec_init(struct snd_soc_codec *codec)
{
	/* Add widgets */
	snd_soc_dapm_new_controls(codec, ea315x_dapm_widgets,
				  ARRAY_SIZE(ea315x_dapm_widgets));

	/* Set up audio path audio_map */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	/* Always connected pins */
	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");
	snd_soc_dapm_enable_pin(codec, "Line In");

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link ea315x_lpc315x_codec_dai[] = {
	{
		.name = "LPC315X_CODEC",
		.stream_name = "LPC315X_CODEC",
		.cpu_dai = &lpc313x_i2s_dai,
		.codec_dai = &lpc315x_codec_dais[0],
		.init = ea315x_lpc315x_codec_init,
		.ops = &ea315x_lpc315x_codec_ops,
	},
};

static struct snd_soc_card snd_soc_machine_ea315x = {
	.name = "LPC315X_CODEC",
	.platform = &lpc313x_soc_platform,
	.dai_link = &ea315x_lpc315x_codec_dai[0],
	.num_links = ARRAY_SIZE(ea315x_lpc315x_codec_dai),
};

static struct snd_soc_device ea315x_lpc315x_codec_snd_dev = {
	.card = &snd_soc_machine_ea315x,
	.codec_dev = &soc_codec_lpc315x_dev,
};

static struct platform_device *ea315x_snd_device;

/*
 * EA315X Module init function
 * */
static int __init ea315x_asoc_init(void)
{
	int ret = 0;

	/* Enable CODEC clock first or I2C will fail to the CODEC */
	lpc313x_main_clk_rate(48000);

	/* Analog Die is added as I2C device in EA3131 Board file.
	 * So no need to add the I2C device again
	 */
	/* Create and register platform device */
	ea315x_snd_device = platform_device_alloc("soc-audio", -1);
	if (ea315x_snd_device == NULL) {
		printk(KERN_ERR "Unable to register Audio device \r\n");
		return -ENOMEM;
	}

	/* Store platform device info */
	platform_set_drvdata(ea315x_snd_device,
			&ea315x_lpc315x_codec_snd_dev);
	ea315x_lpc315x_codec_snd_dev.dev = &ea315x_snd_device->dev;

	/* Add Audio platform device */
	ret = platform_device_add(ea315x_snd_device);
	if (ret) {
		pr_warning("%s: platform_device_add failed (%d)\n",
			   SND_MODNAME, ret);
		goto err_device_add;
	}

	return 0;

err_device_add:
	if (ea315x_snd_device != NULL) {
		platform_device_put(ea315x_snd_device);
		lpc313x_main_clk_rate(0);
		ea315x_snd_device = NULL;
	}

	return ret;
}

static void __exit ea315x_asoc_exit(void)
{
	platform_device_unregister(ea315x_snd_device);
	lpc313x_main_clk_rate(0);
	ea315x_snd_device = NULL;
}

module_init(ea315x_asoc_init);
module_exit(ea315x_asoc_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("ASoC machine driver for LPC315X/Analog Die Audio CODEC");
MODULE_LICENSE("GPL");

