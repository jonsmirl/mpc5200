/*
 * e800-wm9712.c  --  SoC audio for e800
 *
 * Based on e800.c
 *
 * Copyright 2007 (c) Ian Molton <spyro@f2s.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation; version 2 ONLY.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>

#include <asm/mach-types.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/audio.h>

#include "../codecs/wm9712.h"
#include "pxa2xx-pcm.h"

static int e800_wm9712_write(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	ac97->bus->ops->write(ac97, reg, val);
	return 0;
}

static int e800_wm9712_read(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	val = ac97->bus->ops->read(ac97, reg);
	return 0;
}

static int e800_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	struct snd_ac97_bus_ops *ac97_ops;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm9712_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	snd_soc_card_config_codec(codec, e800_wm9712_read,
		e800_wm9712_write, codec->ac97);

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

	snd_soc_dapm_sync(soc_card);
	return 0;
}

static struct snd_soc_pcm_config pcm_config[] = {
{
	.name		= "HiFi",
	.codec		= wm9712_codec_id,
	.codec_dai	= wm9712_codec_hifi_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa_ac97_hifi_dai_id,
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
static int e800_wm9712_probe(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card;
	int ret;

	if (!machine_is_e800())
		return -ENODEV;

	soc_card = snd_soc_card_create("e800", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM9712";
	soc_card->init = e800_init,
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

static int __exit e800_wm9712_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	snd_soc_card_free(soc_card);
	return 0;
}

#ifdef CONFIG_PM

static int e800_wm9712_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	return snd_soc_suspend(soc_card, state);
}

static int e800_wm9712_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	return snd_soc_resume(soc_card);
}

#else
#define e800_wm9712_suspend NULL
#define e800_wm9712_resume  NULL
#endif

static struct platform_driver e800_wm9712_driver = {
	.probe		= e800_wm9712_probe,
	.remove		= __devexit_p(e800_wm9712_remove),
	.suspend	= e800_wm9712_suspend,
	.resume		= e800_wm9712_resume,
	.driver		= {
		.name		= "e800-WM9712",
		.owner		= THIS_MODULE,
	},
};

static int __init e800_asoc_init(void)
{
	return platform_driver_register(&e800_wm9712_driver);
}

static void __exit e800_asoc_exit(void)
{
	platform_driver_unregister(&e800_wm9712_driver);
}

module_init(e800_asoc_init);
module_exit(e800_asoc_exit);

/* Module information */
MODULE_AUTHOR("Ian Molton <spyro@f2s.com>");
MODULE_DESCRIPTION("ALSA SoC driver for e800");
MODULE_LICENSE("GPL");
