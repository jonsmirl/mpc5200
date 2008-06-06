/*
 * SoC audio for HTC Magician
 *
 * Copyright (c) 2006 Philipp Zabel <philipp.zabel@gmail.com>
 *
 * based on spitz.c,
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
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <sound/driver.h>
#include <sound/initval.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/hardware/scoop.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/magician.h>
#include <asm/arch/magician_cpld.h>
#include <asm/mach-types.h>

#include "../codecs/uda1380.h"
#include "pxa2xx-pcm.h"

#define MAGICIAN_HP_ON     0
#define MAGICIAN_HP_OFF    1

#define MAGICIAN_SPK_ON    0
#define MAGICIAN_SPK_OFF   1

#define MAGICIAN_MIC       0
#define MAGICIAN_MIC_EXT   1

/*
 * SSP GPIO's
 */
#define GPIO23_SSPSCLK_MD	(23 | GPIO_ALT_FN_2_OUT)
#define GPIO24_SSPSFRM_MD	(24 | GPIO_ALT_FN_2_OUT)
#define GPIO25_SSPTXD_MD	(25 | GPIO_ALT_FN_2_OUT)

static int magician_hp_func = MAGICIAN_HP_OFF;
static int magician_spk_func = MAGICIAN_SPK_ON;
static int magician_in_sel = MAGICIAN_MIC;

extern struct platform_device magician_cpld;

static void magician_ext_control(struct snd_soc_card *soc_card)
{
	if (magician_spk_func == MAGICIAN_SPK_ON)
		snd_soc_dapm_enable_pin(soc_card, "Speaker");
	else
		snd_soc_dapm_disable_pin(soc_card, "Speaker");

	if (magician_hp_func == MAGICIAN_HP_ON)
		snd_soc_dapm_enable_pin(soc_card, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");

	switch (magician_in_sel) {
	case MAGICIAN_MIC:
		snd_soc_dapm_disable_pin(soc_card, "Headset Mic");
		snd_soc_dapm_enable_pin(soc_card, "Call Mic");
		break;
	case MAGICIAN_MIC_EXT:
		snd_soc_dapm_disable_pin(soc_card, "Call Mic");
		snd_soc_dapm_enable_pin(soc_card, "Headset Mic");
		break;
	}
	snd_soc_dapm_sync(soc_card);
}

static int magician_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	/* check the jack status at stream startup */
	magician_ext_control(soc_card);

	return 0;
}

/*
 * Magician uses SSP port for playback.
 */
static int magician_playback_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	unsigned int acps, acds, div4;
	int ret = 0;

	/*
	 * Rate = SSPSCLK / (word size(16))
	 * SSPSCLK = (ACPS / ACDS) / SSPSCLKDIV(div4 or div1)
	 */
	switch (params_rate(params)) {
	case 8000:
		acps = 32842000;
		acds = PXA2XX_SSP_CLK_AUDIO_DIV_32;	/* wrong - 32 bits/sample */
		div4 = PXA2XX_SSP_CLK_SCDB_4;
		break;
	case 11025:
		acps = 5622000;
		acds = PXA2XX_SSP_CLK_AUDIO_DIV_8;	/* 16 bits/sample, 1 slot */
		div4 = PXA2XX_SSP_CLK_SCDB_4;
		break;
	case 22050:
		acps = 5622000;
		acds = PXA2XX_SSP_CLK_AUDIO_DIV_4;
		div4 = PXA2XX_SSP_CLK_SCDB_4;
		break;
	case 44100:
		acps = 11345000;
		acds = PXA2XX_SSP_CLK_AUDIO_DIV_4;
		div4 = PXA2XX_SSP_CLK_SCDB_4;
		break;
	case 48000:
		acps = 12235000;
		acds = PXA2XX_SSP_CLK_AUDIO_DIV_4;
		div4 = PXA2XX_SSP_CLK_SCDB_4;
		break;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_MSB |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_MSB |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set audio clock as clock source */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_SSP_CLK_AUDIO, 0,
			SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set the SSP audio system clock ACDS divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai,
			PXA2XX_SSP_AUDIO_DIV_ACDS, acds);
	if (ret < 0)
		return ret;

	/* set the SSP audio system clock SCDB divider4 */
	ret = snd_soc_dai_set_clkdiv(cpu_dai,
			PXA2XX_SSP_AUDIO_DIV_SCDB, div4);
	if (ret < 0)
		return ret;

	/* set SSP audio pll clock */
	ret = snd_soc_dai_set_pll(cpu_dai, 0, 0, acps);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * We have to enable the SSP port early so the UDA1380 can flush
 * it's register cache. The UDA1380 can only write it's interpolator and
 * decimator registers when the link is running.
 */
static int magician_playback_prepare(struct snd_pcm_substream *substream)
{
	/* enable SSP clock - is this needed ? */
	SSCR0_P(1) |= SSCR0_SSE;

	/* FIXME: ENABLE I2S */
	SACR0 |= SACR0_BCKD;
	SACR0 |= SACR0_ENB;
	pxa_set_cken(CKEN8_I2S, 1);

	return 0;
}

static int magician_playback_hw_free(struct snd_pcm_substream *substream)
{
	/* FIXME: DISABLE I2S */
	SACR0 &= ~SACR0_ENB;
	SACR0 &= ~SACR0_BCKD;
	pxa_set_cken(CKEN8_I2S, 0);
	return 0;
}

/*
 * Magician uses I2S for capture.
 */
static int magician_capture_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	int ret = 0;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
			SND_SOC_DAIFMT_MSB | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_MSB | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as output */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
			SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * We have to enable the I2S port early so the UDA1380 can flush
 * it's register cache. The UDA1380 can only write it's interpolator and
 * decimator registers when the link is running.
 */
static int magician_capture_prepare(struct snd_pcm_substream *substream)
{
	SACR0 |= SACR0_ENB;
	return 0;
}

static struct snd_soc_ops magician_capture_ops = {
	.startup = magician_startup,
	.hw_params = magician_capture_hw_params,
	.prepare = magician_capture_prepare,
};

static struct snd_soc_ops magician_playback_ops = {
	.startup = magician_startup,
	.hw_params = magician_playback_hw_params,
	.prepare = magician_playback_prepare,
	.hw_free = magician_playback_hw_free,
};

static int magician_get_jack(struct snd_kcontrol * kcontrol,
			     struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = magician_hp_func;
	return 0;
}

static int magician_set_hp(struct snd_kcontrol * kcontrol,
			     struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);

	if (magician_hp_func == ucontrol->value.integer.value[0])
		return 0;

	magician_hp_func = ucontrol->value.integer.value[0];
	magician_ext_control(soc_card);
	return 1;
}

static int magician_get_spk(struct snd_kcontrol * kcontrol,
			    struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = magician_spk_func;
	return 0;
}

static int magician_set_spk(struct snd_kcontrol * kcontrol,
			    struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);

	if (magician_spk_func == ucontrol->value.integer.value[0])
		return 0;

	magician_spk_func = ucontrol->value.integer.value[0];
	magician_ext_control(soc_card);
	return 1;
}

static int magician_get_input(struct snd_kcontrol * kcontrol,
			      struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = magician_in_sel;
	return 0;
}

static int magician_set_input(struct snd_kcontrol * kcontrol,
			      struct snd_ctl_elem_value * ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);

	if (magician_in_sel == ucontrol->value.integer.value[0])
		return 0;

	magician_in_sel = ucontrol->value.integer.value[0];

	switch (magician_in_sel) {
	case MAGICIAN_MIC:
		magician_egpio_disable(&magician_cpld,
				       EGPIO_NR_MAGICIAN_IN_SEL0);
		magician_egpio_enable(&magician_cpld,
				      EGPIO_NR_MAGICIAN_IN_SEL1);
		break;
	case MAGICIAN_MIC_EXT:
		magician_egpio_disable(&magician_cpld,
				       EGPIO_NR_MAGICIAN_IN_SEL0);
		magician_egpio_disable(&magician_cpld,
				       EGPIO_NR_MAGICIAN_IN_SEL1);
	}

	return 1;
}

static int magician_spk_power(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		magician_egpio_enable(&magician_cpld,
				      EGPIO_NR_MAGICIAN_SPK_POWER);
	else
		magician_egpio_disable(&magician_cpld,
				       EGPIO_NR_MAGICIAN_SPK_POWER);
	return 0;
}

static int magician_hp_power(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *k, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		magician_egpio_enable(&magician_cpld,
				      EGPIO_NR_MAGICIAN_EP_POWER);
	else
		magician_egpio_disable(&magician_cpld,
				       EGPIO_NR_MAGICIAN_EP_POWER);
	return 0;
}

static int magician_mic_bias(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *k, int event)
{
//	if (SND_SOC_DAPM_EVENT_ON(event))
//		magician_egpio_enable(&magician_cpld,
//			EGPIO_NR_MAGICIAN_MIC_POWER);
//	else
//		magician_egpio_disable(&magician_cpld,
//			EGPIO_NR_MAGICIAN_MIC_POWER);
	return 0;
}

/* magician soc_card dapm widgets */
static const struct snd_soc_dapm_widget uda1380_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", magician_hp_power),
	SND_SOC_DAPM_SPK("Speaker", magician_spk_power),
	SND_SOC_DAPM_MIC("Call Mic", magician_mic_bias),
	SND_SOC_DAPM_MIC("Headset Mic", magician_mic_bias),
};

/* magician soc_card audio_map */
static const struct snd_soc_dapm_route audio_map[] = {

	/* Headphone connected to VOUTL, VOUTR */
	{"Headphone Jack", NULL, "VOUTL"},
	{"Headphone Jack", NULL, "VOUTR"},

	/* Speaker connected to VOUTL, VOUTR */
	{"Speaker", NULL, "VOUTL"},
	{"Speaker", NULL, "VOUTR"},

	/* Mics are connected to VINM */
	{"VINM", NULL, "Headset Mic"},
	{"VINM", NULL, "Call Mic"},
};

static const char *hp_function[] = { "On", "Off" };
static const char *spk_function[] = { "On", "Off" };
static const char *input_select[] = { "Call Mic", "Headset Mic" };
static const struct soc_enum magician_enum[] = {
	SOC_ENUM_SINGLE_EXT(4, hp_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, input_select),
};

static const struct snd_kcontrol_new uda1380_magician_controls[] = {
	SOC_ENUM_EXT("Headphone Switch", magician_enum[0], magician_get_jack,
			magician_set_hp),
	SOC_ENUM_EXT("Speaker Switch", magician_enum[1], magician_get_spk,
			magician_set_spk),
	SOC_ENUM_EXT("Input Select", magician_enum[2], magician_get_input,
			magician_set_input),
};

#define UDA1380_I2C_ADDR	0x10
static unsigned short normal_i2c[] = { UDA1380_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver uda1380_i2c_driver;
static struct i2c_client client_template;

static int magician_uda1380_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

/*
 * Logic for a uda1380 as connected on a HTC Magician
 */
static int magician_uda1380_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, uda1380_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	/* NC codec pins */
	snd_soc_dapm_disable_pin(soc_card, "VOUTLHP");
	snd_soc_dapm_disable_pin(soc_card, "VOUTRHP");

	/* FIXME: is anything connected here? */
	snd_soc_dapm_disable_pin(soc_card, "VINL");
	snd_soc_dapm_disable_pin(soc_card, "VINR");

	/* Add magician specific controls */
	ret = snd_soc_add_new_controls(soc_card, uda1380_magician_controls,
		soc_card, ARRAY_SIZE(uda1380_magician_controls));
	if (ret < 0)
		return ret;

	/* Add magician specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
			uda1380_dapm_widgets, ARRAY_SIZE(uda1380_dapm_widgets));
	if (ret < 0)
		return ret;

	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	snd_soc_card_config_codec(codec, NULL, magician_uda1380_write,
		soc_card->private_data);

	snd_soc_dapm_sync(soc_card);

	snd_soc_card_init_codec(codec, soc_card);
	return 0;
}

static struct snd_soc_pcm_config pcm_config[] = {
{
	.name		= "HiFi Playback",
	.codec		= uda1380_codec_id,
	.codec_dai	= uda1380_codec_dai_id,
	.platform	= pxa_platform_id,
//	.cpu_dai	= pxa2xx_ssp1_dai_id,
	.ops		= &magician_playback_ops,
	.playback	= 1,
},
{
	.name		= "HiFi Capture",
	.codec		= uda1380_codec_id,
	.codec_dai	= uda1380_codec_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa2xx_i2s_dai_id,
	.ops		= &magician_capture_ops,
	.capture	= 1,
},};

static int magician_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_card *soc_card;
	struct i2c_client *i2c;
	int ret;

	if (addr != UDA1380_I2C_ADDR)
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

	soc_card = snd_soc_card_create("magician", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "magician";
	soc_card->init = magician_uda1380_init;
	soc_card->private_data = i2c;
	i2c_set_clientdata(i2c, soc_card);

	ret = snd_soc_card_create_pcms(soc_card, pcm_config,
					ARRAY_SIZE(pcm_config));
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

static int magician_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_card *soc_card = i2c_get_clientdata(client);

	snd_soc_card_free(soc_card);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int magician_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, magician_i2c_probe);
}

static struct i2c_driver uda1380_i2c_driver = {
	.driver = {
		.name = "magician Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_UDA1380,
	.attach_adapter = magician_i2c_attach,
	.detach_client =  magician_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "magician",
	.driver = &uda1380_i2c_driver,
};

static int __init magician_init(void)
{
	int ret;

	if (!machine_is_magician())
		return -ENODEV;

	magician_egpio_enable(&magician_cpld, EGPIO_NR_MAGICIAN_CODEC_POWER);

	/* we may need to have the clock running here - pH5 */
	magician_egpio_enable(&magician_cpld, EGPIO_NR_MAGICIAN_CODEC_RESET);
	udelay(5);
	magician_egpio_disable(&magician_cpld, EGPIO_NR_MAGICIAN_CODEC_RESET);

	/* correct place? we'll need it to talk to the uda1380 */
	request_module("i2c-pxa");

	ret = i2c_add_driver(&uda1380_i2c_driver);
	if (ret < 0)
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);

	pxa_gpio_mode(GPIO23_SSPSCLK_MD);
	pxa_gpio_mode(GPIO24_SSPSFRM_MD);
	pxa_gpio_mode(GPIO25_SSPTXD_MD);

	return ret;
}

static void __exit magician_exit(void)
{
	i2c_del_driver(&uda1380_i2c_driver);

	magician_egpio_disable(&magician_cpld, EGPIO_NR_MAGICIAN_SPK_POWER);
	magician_egpio_disable(&magician_cpld, EGPIO_NR_MAGICIAN_EP_POWER);
	magician_egpio_disable(&magician_cpld, EGPIO_NR_MAGICIAN_MIC_POWER);
	magician_egpio_disable(&magician_cpld, EGPIO_NR_MAGICIAN_CODEC_POWER);
}

module_init(magician_init);
module_exit(magician_exit);

MODULE_AUTHOR("Philipp Zabel");
MODULE_DESCRIPTION("ALSA SoC Magician");
MODULE_LICENSE("GPL");
