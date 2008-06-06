/*
 * spitz.c  --  SoC audio for Sharp SL-Cxx00 models Spitz, Borzoi and Akita
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/akita.h>
#include <asm/arch/spitz.h>
#include "../codecs/wm8750.h"
#include "pxa2xx-pcm.h"

#define SPITZ_HP        0
#define SPITZ_MIC       1
#define SPITZ_LINE      2
#define SPITZ_HEADSET   3
#define SPITZ_HP_OFF    4
#define SPITZ_SPK_ON    0
#define SPITZ_SPK_OFF   1

 /* audio clock in Hz - rounded from 12.235MHz */
#define SPITZ_AUDIO_CLOCK 12288000

static int spitz_jack_func;
static int spitz_spk_func;

static void spitz_ext_control(struct snd_soc_card *soc_card)
{
	if (spitz_spk_func == SPITZ_SPK_ON)
		snd_soc_dapm_enable_pin(soc_card, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(soc_card, "Ext Spk");

	/* set up jack connection */
	switch (spitz_jack_func) {
	case SPITZ_HP:
		/* enable and unmute hp jack, disable mic bias */
		snd_soc_dapm_disable_pin(soc_card, "Headset Jack");
		snd_soc_dapm_disable_pin(soc_card, "Mic Jack");
		snd_soc_dapm_disable_pin(soc_card, "Line Jack");
		snd_soc_dapm_enable_pin(soc_card, "Headphone Jack");
		set_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_L);
		set_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_R);
		break;
	case SPITZ_MIC:
		/* enable mic jack and bias, mute hp */
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_disable_pin(soc_card, "Headset Jack");
		snd_soc_dapm_disable_pin(soc_card, "Line Jack");
		snd_soc_dapm_enable_pin(soc_card, "Mic Jack");
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_L);
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_R);
		break;
	case SPITZ_LINE:
		/* enable line jack, disable mic bias and mute hp */
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_disable_pin(soc_card, "Headset Jack");
		snd_soc_dapm_disable_pin(soc_card, "Mic Jack");
		snd_soc_dapm_enable_pin(soc_card, "Line Jack");
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_L);
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_R);
		break;
	case SPITZ_HEADSET:
		/* enable and unmute headset jack enable mic bias, mute L hp */
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_disable_pin(soc_card, "Mic Jack");
		snd_soc_dapm_disable_pin(soc_card, "Line Jack");
		snd_soc_dapm_enable_pin(soc_card, "Headset Jack");
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_L);
		set_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_R);
		break;
	case SPITZ_HP_OFF:
		/* jack removed, everything off */
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_disable_pin(soc_card, "Headset Jack");
		snd_soc_dapm_disable_pin(soc_card, "Mic Jack");
		snd_soc_dapm_disable_pin(soc_card, "Line Jack");
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_L);
		reset_scoop_gpio(&spitzscoop_device.dev, SPITZ_SCP_MUTE_R);
		break;
	}
	snd_soc_dapm_sync(soc_card);
}

static int spitz_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	/* check the jack status at stream startup */
	spitz_ext_control(soc_card);
	return 0;
}

static int spitz_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	unsigned int clk = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		clk = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8750_SYSCLK, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops spitz_ops = {
	.startup = spitz_startup,
	.hw_params = spitz_hw_params,
};

static int spitz_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = spitz_jack_func;
	return 0;
}

static int spitz_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);

	if (spitz_jack_func == ucontrol->value.integer.value[0])
		return 0;

	spitz_jack_func = ucontrol->value.integer.value[0];
	spitz_ext_control(soc_card);
	return 1;
}

static int spitz_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = spitz_spk_func;
	return 0;
}

static int spitz_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card =  snd_kcontrol_chip(kcontrol);

	if (spitz_spk_func == ucontrol->value.integer.value[0])
		return 0;

	spitz_spk_func = ucontrol->value.integer.value[0];
	spitz_ext_control(soc_card);
	return 1;
}

static int spitz_mic_bias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *control, int event)
{
	if (machine_is_borzoi() || machine_is_spitz()) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			set_scoop_gpio(&spitzscoop2_device.dev,
				SPITZ_SCP2_MIC_BIAS);
		else
			reset_scoop_gpio(&spitzscoop2_device.dev,
				SPITZ_SCP2_MIC_BIAS);
	}

	if (machine_is_akita()) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			akita_set_ioexp(&akitaioexp_device.dev,
				AKITA_IOEXP_MIC_BIAS);
		else
			akita_reset_ioexp(&akitaioexp_device.dev,
				AKITA_IOEXP_MIC_BIAS);
	}
	return 0;
}

/* spitz soc_card dapm widgets */
static const struct snd_soc_dapm_widget wm8750_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", spitz_mic_bias),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_LINE("Line Jack", NULL),

	/* headset is a mic and mono headphone */
	SND_SOC_DAPM_HP("Headset Jack", NULL),
};

/* Spitz soc_card audio_map */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to LOUT1, ROUT1 */
	{"Headphone Jack", NULL, "LOUT1"},
	{"Headphone Jack", NULL, "ROUT1"},

	/* headset connected to ROUT1 and LINPUT1 with bias (def below) */
	{"Headset Jack", NULL, "ROUT1"},

	/* ext speaker connected to LOUT2, ROUT2  */
	{"Ext Spk", NULL , "ROUT2"},
	{"Ext Spk", NULL , "LOUT2"},

	/* mic is connected to input 1 - with bias */
	{"LINPUT1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic Jack"},

	/* line is connected to input 1 - no bias */
	{"LINPUT1", NULL, "Line Jack"},
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
	"Off"};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum spitz_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new wm8750_spitz_controls[] = {
	SOC_ENUM_EXT("Jack Function", spitz_enum[0], spitz_get_jack,
		spitz_set_jack),
	SOC_ENUM_EXT("Speaker Function", spitz_enum[1], spitz_get_spk,
		spitz_set_spk),
};

/*
 * WM8750 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
#define WM8750_I2C_ADDR	0x1b
static unsigned short normal_i2c[] = { WM8750_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8750_i2c_driver;
static struct i2c_client client_template;

static int spitz_wm8750_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

/*
 * Logic for a wm8750 as connected on a Sharp SL-Cxx00 Device
 */
static int spitz_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm8750_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	/* NC codec pins */
	snd_soc_dapm_disable_pin(soc_card, "RINPUT1");
	snd_soc_dapm_disable_pin(soc_card, "LINPUT2");
	snd_soc_dapm_disable_pin(soc_card, "RINPUT2");
	snd_soc_dapm_disable_pin(soc_card, "LINPUT3");
	snd_soc_dapm_disable_pin(soc_card, "RINPUT3");
	snd_soc_dapm_disable_pin(soc_card, "OUT3");
	snd_soc_dapm_disable_pin(soc_card, "MONO");

	/* add spitz specific controls */
	ret = snd_soc_add_new_controls(soc_card, wm8750_spitz_controls,
		soc_card, ARRAY_SIZE(wm8750_spitz_controls));
	if (ret < 0)
		return ret;

	/* Add spitz specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
			wm8750_dapm_widgets, ARRAY_SIZE(wm8750_dapm_widgets));
	if (ret < 0)
		return ret;

	/* Set up spitz specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	snd_soc_dapm_sync(soc_card);

	snd_soc_card_config_codec(codec, NULL, spitz_wm8750_write,
		soc_card->private_data);

	snd_soc_card_init_codec(codec, soc_card);

	return 0;
}

static struct snd_soc_pcm_config hifi_pcm_config = {
	.name		= "HiFi",
	.codec		= wm8750_codec_id,
	.codec_dai	= wm8750_codec_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa2xx_i2s_dai_id,
	.ops		= &spitz_ops,
	.playback	= 1,
	.capture	= 1,
};

static int wm8750_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *i2c;
	struct snd_soc_card *soc_card;
	int ret;

	if (addr != WM8750_I2C_ADDR)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL)
		return -ENOMEM;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		printk("failed to attach codec at addr %x\n", addr);
		goto attach_err;
	}

	soc_card = snd_soc_card_create("spitz", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM8750";
	soc_card->init = spitz_init;
	soc_card->private_data = i2c;
	i2c_set_clientdata(i2c, soc_card);

	ret = snd_soc_card_create_pcms(soc_card, &hifi_pcm_config, 1);
	if (ret < 0)
		goto err;

	ret = snd_soc_card_register(soc_card);
	return ret;

err:
	snd_soc_card_free(soc_card);
attach_err:
	i2c_detach_client(i2c);
	kfree(i2c);
	return ret;
}

static int wm8750_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_card *soc_card = i2c_get_clientdata(client);

	snd_soc_card_free(soc_card);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int wm8750_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8750_i2c_probe);
}

static struct i2c_driver wm8750_i2c_driver = {
	.driver = {
		.name = "WM8750 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8750,
	.attach_adapter = wm8750_i2c_attach,
	.detach_client =  wm8750_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8750",
	.driver = &wm8750_i2c_driver,
};

static int __init spitz_wm8750_probe(struct platform_device *pdev)
{
	int ret;

	if (!(machine_is_spitz() || machine_is_borzoi() || machine_is_akita()))
		return -ENODEV;

	/* register I2C driver for WM8750 codec control */
	ret = i2c_add_driver(&wm8750_i2c_driver);
	if (ret < 0)
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
	return ret;
}

static int __exit spitz_wm8750_remove(struct platform_device *pdev)
{
	i2c_del_driver(&wm8750_i2c_driver);
	return 0;
}

#ifdef CONFIG_PM
static int spitz_wm8750_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;
	return snd_soc_suspend(soc_card, state);
}

static int spitz_wm8750_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;
	return snd_soc_resume(soc_card);
}

#else
#define spitz_wm8750_suspend NULL
#define spitz_wm8750_resume  NULL
#endif

static struct platform_driver spitz_wm8750_driver = {
	.probe		= spitz_wm8750_probe,
	.remove		= __devexit_p(spitz_wm8750_remove),
	.suspend	= spitz_wm8750_suspend,
	.resume		= spitz_wm8750_resume,
	.driver		= {
		.name		= "spitz-wm8750",
		.owner		= THIS_MODULE,
	},
};

static int __init spitz_asoc_init(void)
{
	return platform_driver_register(&spitz_wm8750_driver);
}

static void __exit spitz_asoc_exit(void)
{
	platform_driver_unregister(&spitz_wm8750_driver);
}

module_init(spitz_asoc_init);
module_exit(spitz_asoc_exit);

MODULE_AUTHOR("Richard Purdie");
MODULE_DESCRIPTION("ALSA SoC Spitz");
MODULE_LICENSE("GPL");
