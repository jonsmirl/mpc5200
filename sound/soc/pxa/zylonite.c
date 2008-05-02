/*
 * zylonite.c  --  SoC audio for Zylonite
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 *
 * Authors: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>

#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mfp-pxa320.h>
#include <asm/arch/pxa-regs.h>

#include "pxa2xx-pcm.h"
#include "../codecs/wm9713.h"

static struct clk *mclk;

static const struct snd_soc_dapm_widget zylonite_dapm_widgets[] = {
SND_SOC_DAPM_HP("Audio Jack Headphones", NULL),
SND_SOC_DAPM_MIC("Audio Jack Microphone", NULL),
SND_SOC_DAPM_MIC("Handset Microphone", NULL),
SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {

        /* Audio jack has both microphone and headphones */
	{"Audio Jack Headphones", NULL, "HPL"},
	{"Audio Jack Headphones", NULL, "HPR"},
	{"Audio Jack Microphone", NULL, "MIC2A"}, /* ??? */
	{"Mic Bias", NULL, "Audio Jack Microphone"},

	/* Handset microphone */
	{"Handset Microphone", NULL, "MIC1"},
	{"Mic Bias", NULL, "Handset Microphone"},

	{"Speaker", NULL, "SPKL"},
	{"Speaker", NULL, "SPKR"},
};

static struct snd_soc_ops zylonite_hifi_ops = {
};

static struct snd_soc_ops zylonite_aux_ops = {
};

static struct snd_soc_pcm_config pcm_configs[] = {
	{
		.name		= "Aux",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_aux_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa_ac97_aux_dai_id,
		.playback	= 1,
		.ops		= &zylonite_aux_ops,
	},
	{
		.name		= "HiFi",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_hifi_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa_ac97_hifi_dai_id,
		.playback	= 1,
		.capture	= 1,
		.ops		= &zylonite_hifi_ops,
	},
};

static int zylonite_init(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dai *dai;
	struct snd_ac97_bus_ops *ac97_ops;
	int ret;

	codec = snd_soc_card_get_codec(card, wm9713_codec_id);
	if (codec == NULL) {
		printk(KERN_ERR "Unable to obtain WM9713 codec\n");
		return -ENODEV;
	}

	dai = snd_soc_card_get_dai(card, pxa_ac97_hifi_dai_id);
	if (dai == NULL) {
		printk(KERN_ERR "Unable to obtain WM9713 HiFi DAI\n");
		return -ENODEV;
	}

	ac97_ops = snd_soc_card_get_ac97_ops(card, pxa_ac97_hifi_dai_id);
	if (!ac97_ops) {
		printk(KERN_ERR "Unable to obtain AC97 operations\n");
		return -ENODEV;
	}

	/* register with AC97 bus for ad-hoc driver access */
	ret = snd_soc_new_ac97_codec(codec, ac97_ops, card->card, 0, 0);
	if (ret < 0) {
		printk(KERN_ERR "Unable to instantiate AC97 codec\n");
		return ret;
	}

	clk_enable(mclk);

	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	ac97_ops->reset(codec->ac97);
	ac97_ops->warm_reset(codec->ac97);

	if (ac97_ops->read(codec->ac97, AC97_VENDOR_ID1) != 0x574d) {
		printk(KERN_ERR "Unable to read codec vendor ID\n");

		return -ENODEV;
	}

	snd_soc_card_init_codec(codec, card);

	ret = snd_soc_dai_set_pll(dai, 0, clk_get_rate(mclk), 1);
	if (ret != 0) {
		dev_err(codec->dev, "Unable to configure PLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dapm_new_controls(card, codec,
			zylonite_dapm_widgets,
			ARRAY_SIZE(zylonite_dapm_widgets));
	if (ret < 0)
		return ret;

	ret = snd_soc_dapm_add_routes(card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	/* The on-board speaker can't be disconnected */
	snd_soc_dapm_enable_pin(card, "Speaker");

	/* Until we implement jack detect */
	snd_soc_dapm_enable_pin(card, "Audio Jack Headphones");

	snd_soc_dapm_sync(card);

	return 0;
}

static int zylonite_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	int ret;

	/* Most Zylonite based systems use POUT to provide MCLK to the
	 * WM9713 but the board has the option of using either that or
	 * AC97CLK based on the configuration of SW15.  The
	 * appropriate source should be selected here.
	 */
	mclk = clk_get(&pdev->dev, "CLK_POUT");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "Unable to obtain MCLK source\n");
		return -ENODEV;
	}

	dev_dbg(&pdev->dev, "MCLK rate: %luHz\n",
		clk_get_rate(mclk));

	card = snd_soc_card_create("zylonite", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (card == NULL)
		return -ENOMEM;

	card->longname = "Zylonite";
	card->init = zylonite_init,
	card->private_data = pdev;
	platform_set_drvdata(pdev, card);

	ret = snd_soc_card_create_pcms(card, pcm_configs,
				  ARRAY_SIZE(pcm_configs));
	if (ret < 0)
		goto err;

	ret = snd_soc_card_register(card);
	if (ret < 0)
		goto err;

	return ret;

err:
	dev_err(&pdev->dev, "probe() failed: %d\n", ret);
	snd_soc_card_free(card);
	return ret;
}

static int __exit zylonite_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_card_free(card);
	return 0;
}

static struct platform_driver zylonite_driver = {
	.probe		= zylonite_probe,
	.remove		= __devexit_p(zylonite_remove),
	.driver		= {
		.name		= "zylonite-audio",
		.owner		= THIS_MODULE,
	},
};

static int __init zylonite_asoc_init(void)
{
	return platform_driver_register(&zylonite_driver);
}

static void __exit zylonite_asoc_exit(void)
{
	platform_driver_unregister(&zylonite_driver);
}

module_init(zylonite_asoc_init);
module_exit(zylonite_asoc_exit);

/* Module information */
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfsonmicro.com>");
MODULE_DESCRIPTION("ALSA SoC Zylonite");
MODULE_LICENSE("GPL");
