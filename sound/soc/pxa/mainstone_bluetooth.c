/*
 * mainstone_bluetooth.c
 * Mainstone Example Bluetooth  --  ALSA Soc Audio Layer
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    15th May 2006   Initial version.
 *
 * This is example code to demonstrate connecting a bluetooth codec to the PCM
 * DAI on the WM8753 codec on the Intel Mainstone platform. It is by no means
 * complete as it requires code to control the BT codec.
 *
 * The architecture consists of the WM8753 HIFI DAI connected to the PXA27x
 * I2S controller and the WM8753 PCM DAI connected to the bluetooth DAI. The
 * bluetooth codec and wm8753 are controlled via I2C. Audio is routed between
 * the PXA27x and the bluetooth via internal WM8753 analog paths.
 *
 * This example supports the following audio input/outputs.
 *
 *  o Board mounted Mic and Speaker (spk has amplifier)
 *  o Headphones via jack socket
 *  o BT source and sink
 *
 * This driver is not the bluetooth codec driver. This driver only calls
 * functions from the Bluetooth driver to set up it's PCM DAI.
 *
 * It's intended to use the driver as follows:-
 *
 *  1. open() WM8753 PCM audio device.
 *  2. configure PCM audio device (rate etc) - sets up WM8753 PCM DAI,
 *      this should also set up the BT codec DAI (via calling bt driver).
 *  3. configure codec audio mixer paths.
 *  4. open(), configure and read/write HIFI audio device - to Tx/Rx voice
 *
 * The PCM audio device is opened but IO is never performed on it as the IO is
 * directly between the codec and the BT codec (and not the CPU).
 *
 * TODO:
 *  o Implement callbacks
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>
#include <asm/arch/ssp.h>

#include "../codecs/wm8753.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-i2s.h"
#include "pxa2xx-ssp.h"

static struct snd_soc_card mainstone;

/* Do specific bluetooth PCM startup here */
static int bt_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

/* Do specific bluetooth PCM shutdown here */
static void bt_shutdown (struct snd_pcm_substream *substream)
{
}

/* Do pecific bluetooth PCM hw params init here */
static int bt_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

/* Do specific bluetooth PCM hw params free here */
static int bt_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

#define BT_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100)

/*
 * BT Codec DAI
 */
static struct snd_soc_cpu_dai bt_dai =
{	.name = "Bluetooth",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.playback = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = BT_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 1,
		.rates = BT_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = {
		.startup = bt_startup,
		.shutdown = bt_shutdown,
		.hw_params = bt_hw_params,
		.hw_free = bt_hw_free,
		},
};

/* PM */
static int mainstone_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int mainstone_resume(struct platform_device *pdev)
{
	return 0;
}

static int mainstone_probe(struct platform_device *pdev)
{
	return 0;
}

static int mainstone_remove(struct platform_device *pdev)
{
	return 0;
}

/*
 * Machine audio functions.
 *
 * The soc_card now has 3 extra audio controls.
 *
 * Jack function: Sets function (device plugged into Jack) to nothing (Off)
 *                or Headphones.
 *
 * Mic function: Set the on board Mic to On or Off
 * Spk function: Set the on board Spk to On or Off
 *
 * example: BT playback (of far end) and capture (of near end)
 *  Set Mic and Speaker to On, open BT alsa interface as above and set up
 *  internal audio paths.
 */

static int soc_card_jack_func = 0;
static int soc_card_spk_func = 0;
static int soc_card_mic_func = 0;

static int soc_card_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = soc_card_jack_func;
	return 0;
}

static int soc_card_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	soc_card_jack_func = ucontrol->value.integer.value[0];
	snd_soc_dapm_set_endpoint(codec, "Headphone Jack", soc_card_jack_func);
	return 0;
}

static int soc_card_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = soc_card_spk_func;
	return 0;
}

static int soc_card_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (soc_card_spk_func == ucontrol->value.integer.value[0])
		return 0;

	soc_card_spk_func = ucontrol->value.integer.value[0];
	snd_soc_dapm_set_endpoint(codec, "Spk", soc_card_spk_func);
	return 1;
}

static int soc_card_get_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = soc_card_spk_func;
	return 0;
}

static int soc_card_set_mic(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);

	if (soc_card_spk_func == ucontrol->value.integer.value[0])
		return 0;

	soc_card_spk_func = ucontrol->value.integer.value[0];
	snd_soc_dapm_set_endpoint(codec, "Mic", soc_card_mic_func);
	return 1;
}

/* turns on board speaker amp on/off */
static int soc_card_amp_event(struct snd_soc_dapm_widget *w, int event)
{
#if 0
	if (SND_SOC_DAPM_EVENT_ON(event))
		/* on */
	else
		/* off */
#endif
	return 0;
}

/* soc_card dapm widgets */
static const struct snd_soc_dapm_widget soc_card_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_SPK("Spk", soc_card_amp_event),
SND_SOC_DAPM_MIC("Mic", NULL),
};

/* soc_card connections to the codec pins */
static const char* audio_map[][3] = {

	/* headphone connected to LOUT1, ROUT1 */
	{"Headphone Jack", NULL, "LOUT"},
	{"Headphone Jack", NULL, "ROUT"},

	/* speaker connected to LOUT2, ROUT2 */
	{"Spk", NULL, "ROUT2"},
	{"Spk", NULL, "LOUT2"},

	/* mic is connected to MIC1 (via Mic Bias) */
	{"MIC1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic"},

	{NULL, NULL, NULL},
};

static const char* jack_function[] = {"Off", "Headphone"};
static const char* spk_function[] = {"Off", "On"};
static const char* mic_function[] = {"Off", "On"};
static const struct soc_enum soc_card_ctl_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, mic_function),
};

static const struct snd_kcontrol_new wm8753_soc_card_controls[] = {
	SOC_ENUM_EXT("Jack Function", soc_card_ctl_enum[0], soc_card_get_jack, soc_card_set_jack),
	SOC_ENUM_EXT("Speaker Function", soc_card_ctl_enum[1], soc_card_get_spk, soc_card_set_spk),
	SOC_ENUM_EXT("Mic Function", soc_card_ctl_enum[2], soc_card_get_mic, soc_card_set_mic),
};

static int mainstone_wm8753_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* not used on this soc_card - e.g. will never be powered up */
	snd_soc_dapm_set_endpoint(codec, "OUT3", 0);
	snd_soc_dapm_set_endpoint(codec, "OUT4", 0);
	snd_soc_dapm_set_endpoint(codec, "MONO2", 0);
	snd_soc_dapm_set_endpoint(codec, "MONO1", 0);
	snd_soc_dapm_set_endpoint(codec, "LINE1", 0);
	snd_soc_dapm_set_endpoint(codec, "LINE2", 0);
	snd_soc_dapm_set_endpoint(codec, "RXP", 0);
	snd_soc_dapm_set_endpoint(codec, "RXN", 0);
	snd_soc_dapm_set_endpoint(codec, "MIC2", 0);

	/* Add soc_card specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8753_soc_card_controls); i++) {
		if ((err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8753_soc_card_controls[i],codec, NULL))) < 0)
			return err;
	}

	/* Add soc_card specific widgets */
	for(i = 0; i < ARRAY_SIZE(soc_card_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &soc_card_dapm_widgets[i]);
	}

	/* Set up soc_card specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0], audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync_endpoints(codec);
	return 0;
}

static struct snd_soc_dai_link mainstone_dai[] = {
{ /* Hifi Playback - for similatious use with voice below */
	.name = "WM8753",
	.stream_name = "WM8753 HiFi",
	.cpu_dai = &pxa_i2s_dai,
	.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
	.init = mainstone_wm8753_init,
},
{ /* Voice via BT */
	.name = "Bluetooth",
	.stream_name = "Voice",
	.cpu_dai = &bt_dai,
	.codec_dai = &wm8753_dai[WM8753_DAI_VOICE],
},
};

static struct snd_soc_card mainstone = {
	.name = "Mainstone",
	.probe = mainstone_probe,
	.remove = mainstone_remove,
	.suspend_pre = mainstone_suspend,
	.resume_post = mainstone_resume,
	.dai_link = mainstone_dai,
	.num_links = ARRAY_SIZE(mainstone_dai),
};

static struct snd_soc_device mainstone_snd_wm8753_devdata = {
	.soc_card = &mainstone,
	.platform = &pxa2xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8753,
};

static struct platform_device *mainstone_snd_wm8753_device;

static int __init mainstone_init(void)
{
	int ret;

	mainstone_snd_wm8753_device = platform_device_alloc("soc-audio", -1);
	if (!mainstone_snd_wm8753_device)
		return -ENOMEM;

	platform_set_drvdata(mainstone_snd_wm8753_device, &mainstone_snd_wm8753_devdata);
	mainstone_snd_wm8753_devdata.dev = &mainstone_snd_wm8753_device->dev;

	if((ret = platform_device_add(mainstone_snd_wm8753_device)) != 0)
		platform_device_put(mainstone_snd_wm8753_device);

	return ret;
}

static void __exit mainstone_exit(void)
{
	platform_device_unregister(mainstone_snd_wm8753_device);
}

module_init(mainstone_init);
module_exit(mainstone_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("Mainstone Example Bluetooth PCM Interface");
MODULE_LICENSE("GPL");
