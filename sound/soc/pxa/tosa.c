/*
 * tosa.c  --  SoC audio for Tosa
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 *
 * Authors: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 *          Richard Purdie <richard@openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    30th Nov 2005   Initial version.
 *
 * GPIO's
 *  1 - Jack Insertion
 *  5 - Hookswitch (headset answer/hang up switch)
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
#include <asm/hardware/tmio.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/audio.h>
#include <asm/arch/tosa.h>

#include "../codecs/wm9712.h"
#include "pxa2xx-pcm.h"

#define TOSA_HP        0
#define TOSA_MIC_INT   1
#define TOSA_HEADSET   2
#define TOSA_HP_OFF    3
#define TOSA_SPK_ON    0
#define TOSA_SPK_OFF   1

static int tosa_jack_func;
static int tosa_spk_func;

static void tosa_ext_control(struct snd_soc_card *soc_card)
{
	/* set up jack connection */
	switch (tosa_jack_func) {
	case TOSA_HP:
		snd_soc_dapm_disable_pin(soc_card, "Headset Jack");
		snd_soc_dapm_disable_pin(soc_card, "Mic (Internal)");
		snd_soc_dapm_enable_pin(soc_card, "Headphone Jack");
		break;
	case TOSA_MIC_INT:
		snd_soc_dapm_disable_pin(soc_card, "Headset Jack");
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_enable_pin(soc_card, "Mic (Internal)");
		break;
	case TOSA_HEADSET:
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_disable_pin(soc_card, "Mic (Internal)");
		snd_soc_dapm_enable_pin(soc_card, "Headset Jack");
		break;
	}

	if (tosa_spk_func == TOSA_SPK_ON)
		snd_soc_dapm_enable_pin(soc_card, "Speaker");
	else
		snd_soc_dapm_disable_pin(soc_card, "Speaker");

	snd_soc_dapm_sync(soc_card);
}

static int tosa_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	/* check the jack status at stream startup */
	tosa_ext_control(soc_card);
	return 0;
}

static struct snd_soc_ops tosa_ops = {
	.startup = tosa_startup,
};

static int tosa_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tosa_jack_func;
	return 0;
}

static int tosa_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card =  snd_kcontrol_chip(kcontrol);

	if (tosa_jack_func == ucontrol->value.integer.value[0])
		return 0;

	tosa_jack_func = ucontrol->value.integer.value[0];
	tosa_ext_control(soc_card);
	return 1;
}

static int tosa_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tosa_spk_func;
	return 0;
}

static int tosa_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card =  snd_kcontrol_chip(kcontrol);

	if (tosa_spk_func == ucontrol->value.integer.value[0])
		return 0;

	tosa_spk_func = ucontrol->value.integer.value[0];
	tosa_ext_control(soc_card);
	return 1;
}

/* tosa dapm event handlers */
static int tosa_hp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *control, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		set_tc6393_gpio(&tc6393_device.dev,TOSA_TC6393_L_MUTE);
	else
		reset_tc6393_gpio(&tc6393_device.dev,TOSA_TC6393_L_MUTE);
	return 0;
}

/* tosa soc_card dapm widgets */
static const struct snd_soc_dapm_widget tosa_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", tosa_hp_event),
SND_SOC_DAPM_HP("Headset Jack", NULL),
SND_SOC_DAPM_MIC("Mic (Internal)", NULL),
SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* tosa audio map */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to HPOUTL, HPOUTR */
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Headphone Jack", NULL, "HPOUTR"},

	/* ext speaker connected to LOUT2, ROUT2 */
	{"Speaker", NULL, "LOUT2"},
	{"Speaker", NULL, "ROUT2"},

	/* internal mic is connected to mic1, mic2 differential - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC2", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic (Internal)"},

	/* headset is connected to HPOUTR, and LINEINR with bias */
	{"Headset Jack", NULL, "HPOUTR"},
	{"LINEINR", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Headset Jack"},
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
	"Off"};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum tosa_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new tosa_controls[] = {
	SOC_ENUM_EXT("Jack Function", tosa_enum[0], tosa_get_jack,
		tosa_set_jack),
	SOC_ENUM_EXT("Speaker Function", tosa_enum[1], tosa_get_spk,
		tosa_set_spk),
};

static int tosa_wm9712_write(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	ac97->bus->ops->write(ac97, reg, val);
	return 0;
}

static int tosa_wm9712_read(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	val = ac97->bus->ops->read(ac97, reg);
	return 0;
}

static int tosa_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	struct snd_ac97_bus_ops *ac97_ops;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm9712_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	snd_soc_card_config_codec(codec, tosa_wm9712_read,
		tosa_wm9712_write, codec->ac97);

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

	/* set up tosa codec pins */
	snd_soc_dapm_disable_pin(soc_card, "OUT3");
	snd_soc_dapm_disable_pin(soc_card, "MONOOUT");

	/* Add test specific controls */
	ret = snd_soc_add_new_controls(soc_card, tosa_controls,
		soc_card, ARRAY_SIZE(tosa_controls));
	if (ret < 0)
		return ret;

	/* Add tosa specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
			tosa_dapm_widgets, ARRAY_SIZE(tosa_dapm_widgets));
	if (ret < 0)
		return ret;

	/* set up tosa specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

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
static int tosa_wm9712_probe(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card;
	int ret;

	if (!machine_is_tosa())
		return -ENODEV;

	soc_card = snd_soc_card_create("tosa", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM9712";
	soc_card->init = tosa_init,
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

static int __exit tosa_wm9712_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	snd_soc_card_free(soc_card);
	return 0;
}

#ifdef CONFIG_PM

static int tosa_wm9712_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	return snd_soc_suspend(soc_card, state);
}

static int tosa_wm9712_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	return snd_soc_resume(soc_card);
}

#else
#define tosa_wm9712_suspend NULL
#define tosa_wm9712_resume  NULL
#endif

static struct platform_driver tosa_wm9712_driver = {
	.probe		= tosa_wm9712_probe,
	.remove		= __devexit_p(tosa_wm9712_remove),
	.suspend	= tosa_wm9712_suspend,
	.resume		= tosa_wm9712_resume,
	.driver		= {
		.name		= "Mainstone-WM9712",
		.owner		= THIS_MODULE,
	},
};

static int __init tosa_asoc_init(void)
{
	return platform_driver_register(&tosa_wm9712_driver);
}

static void __exit tosa_asoc_exit(void)
{
	platform_driver_unregister(&tosa_wm9712_driver);
}

module_init(tosa_asoc_init);
module_exit(tosa_asoc_exit);

/* Module information */
MODULE_AUTHOR("Richard Purdie");
MODULE_DESCRIPTION("ALSA SoC Tosa");
MODULE_LICENSE("GPL");
