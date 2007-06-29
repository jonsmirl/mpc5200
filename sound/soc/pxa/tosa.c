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
#include <linux/device.h>

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
#include "pxa2xx-ac97.h"

static struct snd_soc_machine tosa;

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
	int spk = 0, mic_int = 0, hp = 0, hs = 0;

	/* set up jack connection */
	switch (tosa_jack_func) {
	case TOSA_HP:
		hp = 1;
		break;
	case TOSA_MIC_INT:
		mic_int = 1;
		break;
	case TOSA_HEADSET:
		hs = 1;
		break;
	}

	if (tosa_spk_func == TOSA_SPK_ON)
		spk = 1;

	snd_soc_dapm_set_endpoint(machine, "Speaker", spk);
	snd_soc_dapm_set_endpoint(machine, "Mic (Internal)", mic_int);
	snd_soc_dapm_set_endpoint(machine, "Headphone Jack", hp);
	snd_soc_dapm_set_endpoint(machine, "Headset Jack", hs);
	snd_soc_dapm_sync_endpoints(machine);
}

static int tosa_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_machine *machine = pcm_link->machine;

	/* check the jack status at stream startup */
	tosa_ext_control(machine);
	return 0;
}

static struct snd_soc_ops tosa_ops = {
	.startup = tosa_startup,
};

static int tosa_hifi_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	pcm_link->audio_ops = &tosa_ops;
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

struct snd_soc_pcm_link_ops tosa_hifi_pcm = {
	.new	= tosa_hifi_pcm_new,
};

static int tosa_aux_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	pcm_link->audio_ops = &tosa_ops;
	return snd_soc_pcm_new(pcm_link, 1, 0);
}

struct snd_soc_pcm_link_ops tosa_aux_pcm = {
	.new	= tosa_aux_pcm_new,
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

static int tosa_mach_probe(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	struct snd_ac97_bus_ops *ac97_ops;
	int i, ret;

	pcm_link = list_first_entry(&machine->active_list, 
		struct snd_soc_pcm_link, active_list);
	codec = pcm_link->codec;
	
	codec->control_data = codec->ac97;
	codec->mach_write = tosa_wm9712_write;
	codec->mach_read = tosa_wm9712_read;
	ac97_ops = pcm_link->cpu_dai->ac97_ops;
	
	/* register with AC97 bus for ad-hoc driver access */
	ret = snd_soc_new_ac97_codec(pcm_link, ac97_ops, 0);
	if (ret < 0)
		return ret;
		
	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	ac97_ops->reset(codec->ac97);
	ac97_ops->warm_reset(codec->ac97);
	if (ac97_ops->read(codec->ac97, AC97_VENDOR_ID1) == 0) { //lg
		printk(KERN_ERR "AC97 link error\n");
		return ret;
	}	
	codec->ops->probe_codec(codec, machine);

	/* set up tosa codec pins */
	snd_soc_dapm_set_endpoint(machine, "OUT3", 0);
	snd_soc_dapm_set_endpoint(machine, "MONOOUT", 0);

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
		snd_soc_dapm_connect_input(machine, audio_map[i][0], 
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync_endpoints(machine);
	
	/* register card with ALSA upper layers */
	ret = snd_soc_register_card(machine);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register sound card\n",
			__FUNCTION__);
		return ret;
	}
	
	return 0;
}

struct snd_soc_machine_ops tosa_mach_ops = {
	.mach_probe = tosa_mach_probe,	
};

/*
 * This is an example machine initialisation for a wm9712 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int tosa_wm9712_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	struct snd_soc_pcm_link * hifi, *aux;
	int ret;

	if (!machine_is_tosa())
		return -ENODEV;

	machine = kzalloc(sizeof(struct snd_soc_machine), GFP_KERNEL);
	if (machine == NULL)
		return -ENOMEM;

	machine->owner = THIS_MODULE;
	machine->pdev = pdev;
	machine->name = "Mainstone";
	machine->longname = "WM9712";
	machine->ops = &tosa_mach_ops;
	pdev->dev.driver_data = machine;
	
	/* register card */
	ret = snd_soc_new_card(machine, 2, SNDRV_DEFAULT_IDX1, 
		SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create sound card\n", __func__);
		goto card_err;
	}

	/* tosa wm9712 hifi interface */
	hifi = snd_soc_pcm_link_new(machine, "tosa-hifi", &tosa_hifi_pcm,
		pxa2xx_pcm, wm9712_codec, wm9712_hifi_dai, pxa2xx_ac97_hifi);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(hifi);
	if (ret < 0) 
		goto link_err;
	
	/* tosa wm9712 aux interface */
	aux = snd_soc_pcm_link_new(machine, "tosa-aux", &tosa_aux_pcm,
		pxa2xx_pcm, wm9712_codec, wm9712_aux_dai, pxa2xx_ac97_aux);
	if (aux == NULL) {
		printk("failed to create AUX PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(aux);
	if (ret < 0) 
		goto link_err;
	
	return 0;

link_err:
	snd_soc_machine_free(machine);
card_err:
	kfree(machine);
	return ret;
}

static int __exit tosa_wm9712_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;

	snd_soc_machine_free(machine);
	kfree(machine);

	return 0;
}

#ifdef CONFIG_PM

static int tosa_wm9712_suspend(struct platform_device *pdev, 
	pm_message_t state)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	return snd_soc_suspend(machine, state);
}

static int tosa_wm9712_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	return snd_soc_resume(machine);
}

#else
#define tosa_machine_suspend NULL
#define tosa_machine_resume  NULL
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
