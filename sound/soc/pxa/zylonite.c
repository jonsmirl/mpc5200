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

static const struct snd_soc_dapm_widget zylonite_dapm_widgets[] = {
SND_SOC_DAPM_HP("Audio Jack Headphones", NULL),
SND_SOC_DAPM_MIC("Audio Jack Microphone", NULL),
SND_SOC_DAPM_MIC("Handset Microphone", NULL),
SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const char *audio_map[][3] = {

        /* Audio jack has both microphone and headphones */
	{"Audio Jack Headphones", NULL, "HPOUTL"},
	{"Audio Jack Headphones", NULL, "HPOUTR"},
	{"Audio Jack Microphone", NULL, "COMP1"}, /* ??? */
	{"Mic Bias", NULL, "Audio Jack Microphone"},

	/* Handset microphone */
	{"Handset Microphone", NULL, "MIC1"},
	{"Mic Bias", NULL, "Handset Microphone"},

	{"Speaker", NULL, "SPKL"},
	{"Speaker", NULL, "SPKR"},

	{NULL, NULL, NULL},
};

static struct snd_soc_pcm_config pcm_configs[] = {
	{
		.name		= "HiFi",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_hifi_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa3xx_ac97_hifi_dai_id,
		.playback	= 1,
		.capture	= 1,
	},
	{
		.name		= "Aux",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_aux_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa3xx_ac97_aux_dai_id,
		.playback	= 1,
	},
};

static int zylonite_init(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_ac97_bus_ops *ac97_ops;
	int i, ret;

	codec = snd_soc_get_codec(machine, wm9713_codec_id);
	if (codec == NULL) {
		printk(KERN_ERR "Unable to obtain WM9713 codec\n");
		return -ENODEV;
	}
	
	ac97_ops = snd_soc_get_ac97_ops(machine, pxa3xx_ac97_hifi_dai_id);
	if (!ac97_ops) {
		printk(KERN_ERR "Unable to obtain AC97 operations\n");
		return -ENODEV;
	}

	/* register with AC97 bus for ad-hoc driver access */
	ret = snd_soc_new_ac97_codec(codec, ac97_ops, machine->card, 0, 0);
	if (ret < 0) {
		printk(KERN_ERR "Unable to instantiate AC97 codec\n");
		return ret;
	}

	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	ac97_ops->reset(codec->ac97);
	ac97_ops->warm_reset(codec->ac97);

	if (ac97_ops->read(codec->ac97, AC97_VENDOR_ID1) != 0x574d) {
		printk(KERN_ERR "Unable to read codec vendor ID\n");

		return -ENODEV;
	}

	snd_soc_codec_init(codec, machine);

	/* set up system-specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_add_route(machine, audio_map[i][0], 
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_enable_pin(machine, "Audio Jack Headphones");

	snd_soc_dapm_sync(machine);
	
	return 0;
}

static int zylonite_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	struct clk *mclk;
	int ret;

	/* Most Zylonite based systems use POUT to provide MCLK to the
	 * WM9713 but the board has the option of using either that or
	 * AC97_SYSCLK */
	mclk = clk_get(&pdev->dev, "CLK_POUT");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "Unable to obtain MCLK source\n");
		return -ENODEV;
	}
	clk_enable(mclk);

	machine = snd_soc_machine_create("zylonite", &pdev->dev, 
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (machine == NULL)
		return -ENOMEM;

	machine->longname = "Zylonite";
	machine->init = zylonite_init,
	machine->private_data = pdev;
	platform_set_drvdata(pdev, machine);

	ret = snd_soc_create_pcms(machine, &pcm_configs[0],
				  ARRAY_SIZE(pcm_configs));
	if (ret < 0)
		goto err;
	
	ret = snd_soc_machine_register(machine);
	if (ret < 0)
		goto err;

	return ret;

err:
	dev_err(&pdev->dev, "probe() failed: %d\n", ret);
	snd_soc_machine_free(machine);
	return ret;
}

static int __exit zylonite_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);

	snd_soc_machine_free(machine);
	return 0;
}

#ifdef CONFIG_PM

static int zylonite_suspend(struct platform_device *pdev, 
	pm_message_t state)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);
	return snd_soc_suspend(machine, state);
}

static int zylonite_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);
	return snd_soc_resume(machine);
}

#else
#define zylonite_suspend NULL
#define zylonite_resume  NULL
#endif

static struct platform_driver zylonite_driver = {
	.probe		= zylonite_probe,
	.remove		= __devexit_p(zylonite_remove),
	.suspend	= zylonite_suspend,
	.resume		= zylonite_resume,
	.driver		= {
		.name 		= "zylonite-audio",
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
