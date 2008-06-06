/*
 * mainstone.c  --  SoC audio for Mainstone
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  Mainstone audio amplifier code taken from arch/arm/mach-pxa/mainstone.c
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    29th Jan 2006   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mainstone.h>
#include <asm/arch/audio.h>

#include "../codecs/wm9712.h"
#include "pxa2xx-pcm.h"

/* mainstone soc_card dapm widgets */
static const struct snd_soc_dapm_widget mainstone_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic 1", NULL),
	SND_SOC_DAPM_MIC("Mic 2", NULL),
	SND_SOC_DAPM_MIC("Mic 3", NULL),
};

/* example soc_card audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

#if 0
	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 1"},
	/* mic is connected to mic2A - with bias */
	{"MIC2A", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 2"},
	/* mic is connected to mic2B - with bias */
	{"MIC2B", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 3"},
#endif
};

static int mainstone_wm9712_write(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	ac97->bus->ops->write(ac97, reg, val);
	return 0;
}

static int mainstone_wm9712_read(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	val = ac97->bus->ops->read(ac97, reg);
	return 0;
}

static int mainstone_wm9712_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	struct snd_ac97_bus_ops *ac97_ops;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm9712_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	snd_soc_card_config_codec(codec, mainstone_wm9712_read,
		mainstone_wm9712_write, codec->ac97);

	ac97_ops = snd_soc_card_get_ac97_ops(soc_card, pxa_ac97_hifi_dai_id);

	/* register with AC97 bus for ad-hoc driver access */
	ret = snd_soc_new_ac97_codec(codec, ac97_ops, soc_card->card, 0, 0);
	if (ret < 0)
		return ret;

	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	ac97_ops->reset(codec->ac97);
	ac97_ops->warm_reset(codec->ac97);
	if (ac97_ops->read(codec->ac97, AC97_VENDOR_ID1) == 0) {
		printk(KERN_ERR "AC97 link error\n");
		return ret;
	}

	snd_soc_card_init_codec(codec, soc_card);

	/* Add mainstone specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
		mainstone_dapm_widgets,
		ARRAY_SIZE(mainstone_dapm_widgets));
	if (ret < 0)
		return ret;

	/* set up mainstone specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	snd_soc_dapm_sync(soc_card);

	MST_MSCWR2 &= ~MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

static struct snd_soc_pcm_config pcm_config[] = {
{
	.name		= "HiFi",
	.codec		= wm9712_codec_id,
	.codec_dai	= wm9712_codec_hifi_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa2xx_i2s_dai_id,
	.playback	= 1,
	.capture	= 1,
},
{
	.name		= "Aux",
	.codec		= wm9712_codec_id,
	.codec_dai	= wm9712_codec_aux_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa_ac97_aux_dai_id,
	.playback	= 1,
},};

/*
 * This is an example soc_card initialisation for a wm9712 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int mainstone_wm9712_probe(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card;
	int ret;

	soc_card = snd_soc_card_create("mainstone_wm9712", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM9712";
	soc_card->init = mainstone_wm9712_init,
	soc_card->private_data = pdev;
	platform_set_drvdata(pdev, soc_card);

	ret = snd_soc_card_create_pcms(soc_card, pcm_config,
		ARRAY_SIZE(pcm_config));
	if (ret < 0)
		goto err;

	ret = snd_soc_card_register(soc_card);
	return ret;

err:
	snd_soc_card_free(soc_card);
	return ret;
}

static int __exit mainstone_wm9712_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	snd_soc_card_free(soc_card);

	/* disable speaker */
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

#ifdef CONFIG_PM
static long mst_audio_suspend_mask;

static int mainstone_wm9712_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	mst_audio_suspend_mask = MST_MSCWR2;
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_suspend_pcms(soc_card, state);
}

static int mainstone_wm9712_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	MST_MSCWR2 &= mst_audio_suspend_mask | ~MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_resume_pcms(soc_card);
}

#else
#define mainstone_wm9712_suspend NULL
#define mainstone_wm9712_resume  NULL
#endif

static struct platform_driver mainstone_wm9712_driver = {
	.probe		= mainstone_wm9712_probe,
	.remove		= __devexit_p(mainstone_wm9712_remove),
	.suspend	= mainstone_wm9712_suspend,
	.resume		= mainstone_wm9712_resume,
	.driver		= {
		.name		= "Mainstone-WM9712",
		.owner		= THIS_MODULE,
	},
};

static struct platform_device codec = {
	.name		= "wm9712-codec",
	.id		= -1,
};

static struct platform_device platform = {
	.name		= "Mainstone-WM9712",
	.id		= -1,
};

static struct platform_device *devices[] = {
	&codec,
	&platform,
};

static int __init mainstone_asoc_init(void)
{
	platform_add_devices(&devices[0], ARRAY_SIZE(devices));
	return platform_driver_register(&mainstone_wm9712_driver);
}

static void __exit mainstone_asoc_exit(void)
{
	platform_driver_unregister(&mainstone_wm9712_driver);
}

module_init(mainstone_asoc_init);
module_exit(mainstone_asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM9712 Mainstone");
MODULE_LICENSE("GPL");
