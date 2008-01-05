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

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

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

static void tosa_ext_control(struct snd_soc_machine *machine)
{
	/* set up jack connection */
	switch (tosa_jack_func) {
	case TOSA_HP:
		snd_soc_dapm_disable_headset(machine, "Headset Jack");
		snd_soc_dapm_disable_mic(machine, "Mic (Internal)");
		snd_soc_dapm_enable_headphone(machine, "Headphone Jack");
		break;
	case TOSA_MIC_INT:
		snd_soc_dapm_disable_headset(machine, "Headset Jack");
		snd_soc_dapm_disable_headphone(machine, "Headphone Jack");
		snd_soc_dapm_enable_mic(machine, "Mic (Internal)");
		break;
	case TOSA_HEADSET:
		snd_soc_dapm_disable_headphone(machine, "Headphone Jack");
		snd_soc_dapm_disable_mic(machine, "Mic (Internal)");
		snd_soc_dapm_enable_headset(machine, "Headset Jack");
		break;
	}

	if (tosa_spk_func == TOSA_SPK_ON)
		snd_soc_dapm_enable_speaker(machine, "Speaker");
	else
		snd_soc_dapm_disable_speaker(machine, "Speaker");

	snd_soc_dapm_resync(machine);
}

static int tosa_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_link = substream->private_data;
	struct snd_soc_machine *machine = pcm_link->machine;

	/* check the jack status at stream startup */
	tosa_ext_control(machine);
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
	struct snd_soc_machine *machine =  snd_kcontrol_chip(kcontrol);

	if (tosa_jack_func == ucontrol->value.integer.value[0])
		return 0;

	tosa_jack_func = ucontrol->value.integer.value[0];
	tosa_ext_control(machine);
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
	struct snd_soc_machine *machine =  snd_kcontrol_chip(kcontrol);

	if (tosa_spk_func == ucontrol->value.integer.value[0])
		return 0;

	tosa_spk_func = ucontrol->value.integer.value[0];
	tosa_ext_control(machine);
	return 1;
}

/* tosa dapm event handlers */
static int tosa_hp_event(struct snd_soc_dapm_widget *w, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		set_tc6393_gpio(&tc6393_device.dev,TOSA_TC6393_L_MUTE);
	else
		reset_tc6393_gpio(&tc6393_device.dev,TOSA_TC6393_L_MUTE);
	return 0;
}

/* tosa machine dapm widgets */
static const struct snd_soc_dapm_widget tosa_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", tosa_hp_event),
SND_SOC_DAPM_HP("Headset Jack", NULL),
SND_SOC_DAPM_MIC("Mic (Internal)", NULL),
SND_SOC_DAPM_SPK("Speaker", NULL),
};

/* tosa audio map */
static const char *audio_map[][3] = {

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

	{NULL, NULL, NULL},
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

static int tosa_init(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_ac97_bus_ops *ac97_ops;
	int i, ret;

	codec = snd_soc_get_codec(machine, wm9712_codec_id);
	if (codec == NULL)
		return -ENODEV;
	
	snd_soc_codec_set_io(codec, tosa_wm9712_read, 
		tosa_wm9712_write, codec->ac97);
		
	ac97_ops = snd_soc_get_ac97_ops(machine, PXA2XX_DAI_AC97_HIFI);
	
	/* register with AC97 bus for ad-hoc driver access */
	ret = snd_soc_new_ac97_codec(codec, ac97_ops, machine->card, 0, 0);
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

	snd_soc_codec_init(codec, machine);

	/* set up tosa codec pins */
	snd_soc_dapm_disable_pin(machine, "OUT3");
	snd_soc_dapm_disable_pin(machine, "MONOOUT");

	/* Add test specific controls */
	for (i = 0; i < ARRAY_SIZE(tosa_controls); i++) {
		if ((ret = snd_ctl_add(machine->card,
				snd_soc_cnew(&tosa_controls[i], 
					codec, NULL))) < 0)
			return ret;
	}

	/* Add tosa specific widgets */
	for(i = 0; i < ARRAY_SIZE(tosa_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&tosa_dapm_widgets[i]);
	}

	/* set up tosa specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_add_route(machine, audio_map[i][0], 
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_resync(machine);
	
	return 0;
}

/*
 * This is an example machine initialisation for a wm9712 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int tosa_wm9712_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	int ret;

	if (!machine_is_tosa())
		return -ENODEV;
	
	machine = snd_soc_machine_create("tosa", &pdev->dev, 
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (machine == NULL)
		return -ENOMEM;

	machine->longname = "WM9712";
	machine->init = tosa_init,
	machine->private_data = pdev;
	platform_set_drvdata(pdev, machine);
	
	ret = snd_soc_codec_create(machine, wm9712_codec_id);
	if (ret < 0)
		goto err;

	ret = snd_soc_platform_create(machine, pxa_platform_id);
	if (ret < 0)
		goto err;

	ret = snd_soc_pcm_create(machine, "HiFi", NULL, 
		WM9712_DAI_HIFI, PXA2XX_DAI_AC97_HIFI, 1, 1);
	if (ret < 0)
		goto err;
	
	ret = snd_soc_pcm_create(machine, "Aux", NULL, 
		WM9712_DAI_AUX, PXA2XX_DAI_AC97_AUX, 1, 1);
	if (ret < 0)
		goto err;
	
	ret = snd_soc_machine_register(machine);
	return ret;
	
err:
	snd_soc_machine_free(machine);
	return ret;
}

static int __exit tosa_wm9712_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);

	snd_soc_machine_free(machine);
	return 0;
}

#ifdef CONFIG_PM

static int tosa_wm9712_suspend(struct platform_device *pdev, 
	pm_message_t state)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);
	return snd_soc_suspend(machine, state);
}

static int tosa_wm9712_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = platform_get_drvdata(pdev);
	return snd_soc_resume(machine);
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
		.name 		= "Mainstone-WM9712",
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
