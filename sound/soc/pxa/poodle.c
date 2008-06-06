/*
 * poodle.c  --  SoC audio for Poodle
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
#include <asm/hardware/locomo.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/poodle.h>
#include <asm/arch/audio.h>

#include "../codecs/wm8731.h"
#include "pxa2xx-pcm.h"

#define POODLE_HP        1
#define POODLE_HP_OFF    0
#define POODLE_SPK_ON    1
#define POODLE_SPK_OFF   0

 /* audio clock in Hz - rounded from 12.235MHz */
#define POODLE_AUDIO_CLOCK 12288000

static int poodle_jack_func;
static int poodle_spk_func;

static void poodle_ext_control(struct snd_soc_card *soc_card)
{
	/* set up jack connection */
	if (poodle_jack_func == POODLE_HP) {
		/* set = unmute headphone */
		locomo_gpio_write(&poodle_locomo_device.dev,
			POODLE_LOCOMO_GPIO_MUTE_L, 1);
		locomo_gpio_write(&poodle_locomo_device.dev,
			POODLE_LOCOMO_GPIO_MUTE_R, 1);
		snd_soc_dapm_enable_pin(soc_card, "Headphone Jack");
	} else {
		locomo_gpio_write(&poodle_locomo_device.dev,
			POODLE_LOCOMO_GPIO_MUTE_L, 0);
		locomo_gpio_write(&poodle_locomo_device.dev,
			POODLE_LOCOMO_GPIO_MUTE_R, 0);
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
	}

	if (poodle_spk_func == POODLE_SPK_ON)
		snd_soc_dapm_enable_pin(soc_card, "Ext Spk");
	else
		snd_soc_dapm_disable_pin(soc_card, "Ext Spk");

	/* signal a DAPM event */
	snd_soc_dapm_sync(soc_card);
}

static int poodle_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	/* check the jack status at stream startup */
	poodle_ext_control(soc_card);
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on poodle */
static void poodle_shutdown(struct snd_pcm_substream *substream)
{
	/* set = unmute headphone */
	locomo_gpio_write(&poodle_locomo_device.dev,
		POODLE_LOCOMO_GPIO_MUTE_L, 1);
	locomo_gpio_write(&poodle_locomo_device.dev,
		POODLE_LOCOMO_GPIO_MUTE_R, 1);
}

static int poodle_hw_params(struct snd_pcm_substream *substream,
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
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK, clk,
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

static struct snd_soc_ops poodle_ops = {
	.startup = poodle_startup,
	.hw_params = poodle_hw_params,
	.shutdown = poodle_shutdown,
};

static int poodle_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = poodle_jack_func;
	return 0;
}

static int poodle_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card =  snd_kcontrol_chip(kcontrol);

	if (poodle_jack_func == ucontrol->value.integer.value[0])
		return 0;

	poodle_jack_func = ucontrol->value.integer.value[0];
	poodle_ext_control(soc_card);
	return 1;
}

static int poodle_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = poodle_spk_func;
	return 0;
}

static int poodle_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card =  snd_kcontrol_chip(kcontrol);

	if (poodle_spk_func == ucontrol->value.integer.value[0])
		return 0;

	poodle_spk_func = ucontrol->value.integer.value[0];
	poodle_ext_control(soc_card);
	return 1;
}

static int poodle_amp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *control, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		locomo_gpio_write(&poodle_locomo_device.dev,
			POODLE_LOCOMO_GPIO_AMP_ON, 0);
	else
		locomo_gpio_write(&poodle_locomo_device.dev,
			POODLE_LOCOMO_GPIO_AMP_ON, 1);

	return 0;
}

/* poodle soc_card dapm widgets */
static const struct snd_soc_dapm_widget wm8731_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_SPK("Ext Spk", poodle_amp_event),
};

/* Corgi soc_card audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},
};

static const char *jack_function[] = {"Off", "Headphone"};
static const char *spk_function[] = {"Off", "On"};
static const struct soc_enum poodle_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new wm8731_poodle_controls[] = {
	SOC_ENUM_EXT("Jack Function", poodle_enum[0], poodle_get_jack,
		poodle_set_jack),
	SOC_ENUM_EXT("Speaker Function", poodle_enum[1], poodle_get_spk,
		poodle_set_spk),
};

/*
 * WM8731 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
#define WM8731_I2C_ADDR	0x1b
static unsigned short normal_i2c[] = { WM8731_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8731_i2c_driver;
static struct i2c_client client_template;

static int poodle_wm8731_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

/*
 * Logic for a wm8731 as connected on a Sharp SL-C7x0 Device
 */
static int poodle_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm8731_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	/* set up poodle codec pins */
	snd_soc_dapm_disable_pin(soc_card, "LLINEIN");
	snd_soc_dapm_disable_pin(soc_card, "RLINEIN");
	snd_soc_dapm_enable_pin(soc_card, "MICIN");

	/* add poodle specific controls */
	ret = snd_soc_add_new_controls(soc_card, wm8731_poodle_controls,
		soc_card, ARRAY_SIZE(wm8731_poodle_controls));
	if (ret < 0)
		return ret;

	/* Add poodle specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
			wm8731_dapm_widgets, ARRAY_SIZE(wm8731_dapm_widgets));
	if (ret < 0)
		return ret;

	/* Set up poodle specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;


	snd_soc_card_config_codec(codec, NULL, poodle_wm8731_write,
		soc_card->private_data);
	snd_soc_dapm_sync(soc_card);

	return 0;
}

static struct snd_soc_pcm_config hifi_pcm_config = {
	.name		= "HiFi",
	.codec		= wm8731_codec_id,
	.codec_dai	= wm8731_codec_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa2xx_i2s_dai_id,
	.ops		= &poodle_ops,
	.playback	= 1,
	.capture	= 1,
};

static int wm8731_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *i2c;
	struct snd_soc_card *soc_card;
	int ret;

	if (addr != WM8731_I2C_ADDR)
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

	soc_card = snd_soc_card_create("poodle", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM8731";
	soc_card->init = poodle_init;
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

static int wm8731_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_card *soc_card = i2c_get_clientdata(client);

	snd_soc_card_free(soc_card);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int wm8731_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8731_i2c_probe);
}

static struct i2c_driver wm8731_i2c_driver = {
	.driver = {
		.name = "WM8731 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8731,
	.attach_adapter = wm8731_i2c_attach,
	.detach_client =  wm8731_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8731",
	.driver = &wm8731_i2c_driver,
};

static int __init poodle_wm8731_probe(struct platform_device *pdev)
{
	int ret;

	if (!machine_is_poodle())
		return -ENODEV;

	locomo_gpio_set_dir(&poodle_locomo_device.dev,
		POODLE_LOCOMO_GPIO_AMP_ON, 0);
	/* should we mute HP at startup - burning power ?*/
	locomo_gpio_set_dir(&poodle_locomo_device.dev,
		POODLE_LOCOMO_GPIO_MUTE_L, 0);
	locomo_gpio_set_dir(&poodle_locomo_device.dev,
		POODLE_LOCOMO_GPIO_MUTE_R, 0);

	/* register I2C driver for WM8731 codec control */
	ret = i2c_add_driver(&wm8731_i2c_driver);
	if (ret < 0)
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
	return ret;
}

static int __exit poodle_wm8731_remove(struct platform_device *pdev)
{
	i2c_del_driver(&wm8731_i2c_driver);
	return 0;
}

#ifdef CONFIG_PM
static int poodle_wm8731_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;
	return snd_soc_suspend(soc_card, state);
}

static int poodle_wm8731_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;
	return snd_soc_resume(soc_card);
}

#else
#define poodle_wm8731_suspend NULL
#define poodle_wm8731_resume  NULL
#endif

static struct platform_driver poodle_wm8731_driver = {
	.probe		= poodle_wm8731_probe,
	.remove		= __devexit_p(poodle_wm8731_remove),
	.suspend	= poodle_wm8731_suspend,
	.resume		= poodle_wm8731_resume,
	.driver		= {
		.name		= "poodle-wm8731",
		.owner		= THIS_MODULE,
	},
};

static int __init poodle_asoc_init(void)
{
	return platform_driver_register(&poodle_wm8731_driver);
}

static void __exit poodle_asoc_exit(void)
{
	platform_driver_unregister(&poodle_wm8731_driver);
}

module_init(poodle_asoc_init);
module_exit(poodle_asoc_exit);

/* Module information */
MODULE_AUTHOR("Richard Purdie");
MODULE_DESCRIPTION("ALSA SoC Poodle");
MODULE_LICENSE("GPL");
