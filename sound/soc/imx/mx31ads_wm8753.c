/*
 * mx31ads_wm8753.c  --  SoC audio for mx31ads
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  mx31ads audio amplifier code taken from arch/arm/mach-pxa/mx31ads.c
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    30th Oct 2005   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/hardware.h>

#include "../codecs/wm8753.h"
#include "imx31-pcm.h"
#include "imx-ssi.h"

static struct snd_soc_machine *imx31ads_mach;

static int mx31ads_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int pll_out = 0, bclk = 0, fmt = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		fmt = SND_SOC_DAIFMT_CBS_CFS;
		pll_out = 12288000;
		break;
	case 48000:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->ops->set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | fmt);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | fmt);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, WM8753_MCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM8753_BCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* codec PLL input is 13 MHz */
	ret = codec_dai->ops->set_pll(codec_dai, WM8753_PLL1, 13000000, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int mx31ads_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;

	/* disable the PLL */
	return codec_dai->ops->set_pll(codec_dai, WM8753_PLL1, 0, 0);
}

/*
 * mx31ads WM8753 HiFi DAI opserations.
 */
static struct snd_soc_ops mx31ads_hifi_ops = {
	.hw_params = mx31ads_hifi_hw_params,
	.hw_free = mx31ads_hifi_hw_free,
};

static int hifi_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	pcm_link->audio_ops = &mx31ads_hifi_ops;
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

struct snd_soc_pcm_link_ops hifi_pcm = {
	.new	= hifi_pcm_new,
};

static int mx31ads_voice_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void mx31ads_voice_shutdown(struct snd_pcm_substream *substream)
{
}

static int mx31ads_voice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int pll_out = 0, bclk = 0, pcmdiv = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
		pll_out = 12288000;
		pcmdiv = WM8753_PCM_DIV_6; /* 2.048 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 256kHz */
		break;
	case 16000:
		pll_out = 12288000;
		pcmdiv = WM8753_PCM_DIV_3; /* 4.096 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 512kHz */
		break;
	case 48000:
		pll_out = 12288000;
		pcmdiv = WM8753_PCM_DIV_1; /* 12.288 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 1.536 MHz */
		break;
	case 11025:
		pll_out = 11289600;
		pcmdiv = WM8753_PCM_DIV_4; /* 11.2896 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 352.8 kHz */
		break;
	case 22050:
		pll_out = 11289600;
		pcmdiv = WM8753_PCM_DIV_2; /* 11.2896 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 705.6 kHz */
		break;
	case 44100:
		pll_out = 11289600;
		pcmdiv = WM8753_PCM_DIV_1; /* 11.2896 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 1.4112 MHz */
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->ops->set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, WM8753_PCMCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM8753_VXCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM8753_PCMDIV, pcmdiv);
	if (ret < 0)
		return ret;

	/* codec PLL input is 13 MHz */
	ret = codec_dai->ops->set_pll(codec_dai, WM8753_PLL2, 13000000, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int mx31ads_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;

	/* disable the PLL */
	return codec_dai->ops->set_pll(codec_dai, WM8753_PLL2, 0, 0);
}

static struct snd_soc_ops mx31ads_voice_ops = {
	.startup = mx31ads_voice_startup,
	.shutdown = mx31ads_voice_shutdown,
	.hw_params = mx31ads_voice_hw_params,
	.hw_free = mx31ads_voice_hw_free,
};

static int voice_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	pcm_link->audio_ops = &mx31ads_voice_ops;
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

struct snd_soc_pcm_link_ops voice_pcm = {
	.new	= voice_pcm_new,
};

/* example machine audio_mapnections */
static const char* audio_map[][3] = {

	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC1N", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic1 Jack"},
	{"Mic Bias", NULL, "Mic1 Jack"},

	{"ACIN", NULL, "ACOP"},
	{NULL, NULL, NULL},
};

/* headphone detect support on my board */
static const char * hp_pol[] = {"Headphone", "Speaker"};
static const struct soc_enum wm8753_enum =
	SOC_ENUM_SINGLE(WM8753_OUTCTL, 1, 2, hp_pol);

static const struct snd_kcontrol_new wm8753_imx31ads_controls[] = {
	SOC_SINGLE("Headphone Detect Switch", WM8753_OUTCTL, 6, 1, 0),
	SOC_ENUM("Headphone Detect Polarity", wm8753_enum),
};

/*
 * WM8753 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
#define WM8753_I2C_ADDR	0x1a
static unsigned short normal_i2c[] = { WM8753_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8753_i2c_driver;
static struct i2c_client client_template;

static int imx31ads_wm8753_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

static int imx31ads_mach_probe(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	int i, ret;

	pcm_link = list_first_entry(&machine->active_list,
		struct snd_soc_pcm_link, active_list);
	codec = pcm_link->codec;

	/* set up imx31ads codec pins */
	snd_soc_dapm_set_endpoint(machine, "RXP", 0);
	snd_soc_dapm_set_endpoint(machine, "RXN", 0);
	snd_soc_dapm_set_endpoint(machine, "MIC2", 0);

	/* add imx31ads specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8753_imx31ads_controls); i++) {
		if ((ret = snd_ctl_add(machine->card,
				snd_soc_cnew(&wm8753_imx31ads_controls[i],
					machine, NULL))) < 0)
			return ret;
	}

	/* set up imx31ads specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync_endpoints(machine);

	codec->control_data = imx31ads_mach->private_data;
	codec->mach_write = imx31ads_wm8753_write;
	codec->ops->io_probe(codec, imx31ads_mach);

	/* register card with ALSA upper layers */
	ret = snd_soc_register_card(imx31ads_mach);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register sound card\n",
			__FUNCTION__);
		return ret;
	}

	return 0;
}

struct snd_soc_machine_ops imx31ads_mach_ops = {
	.mach_probe = imx31ads_mach_probe,
};

static int wm8753_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *i2c;
	struct snd_soc_pcm_link * hifi, *voice;
	int ret;

	if (addr != WM8753_I2C_ADDR)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL)
		return -ENOMEM;
	i2c_set_clientdata(i2c, imx31ads_mach);
	imx31ads_mach->private_data = i2c;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		printk("failed to attach codec at addr %x\n", addr);
		goto attach_err;
	}

	/* imx31ads wm8753 hifi interface */
	hifi = snd_soc_pcm_link_new(imx31ads_mach, "imx31ads-hifi",
		&hifi_pcm, imx31_pcm, wm8753_codec, wm8753_hifi_dai,
		imx_ssi_1);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto attach_err;
	}
	ret =  snd_soc_pcm_link_attach(hifi);
	if (ret < 0)
		goto link_err;

	voice = snd_soc_pcm_link_new(imx31ads_mach, "imx31ads-voice",
		&voice_pcm, imx31_pcm, wm8753_codec, wm8753_voice_dai,
		imx_ssi_3);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(voice);
	if (ret < 0)
		goto link_err;

	return ret;

link_err:
	snd_soc_machine_free(imx31ads_mach);
attach_err:
	i2c_detach_client(i2c);
	kfree(i2c);
	return ret;
}

static int wm8753_i2c_detach(struct i2c_client *client)
{
	snd_soc_machine_free(imx31ads_mach);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int wm8753_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8753_i2c_probe);
}

static struct i2c_driver wm8753_i2c_driver = {
	.driver = {
		.name = "WM8753 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8753,
	.attach_adapter = wm8753_i2c_attach,
	.detach_client =  wm8753_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8753",
	.driver = &wm8753_i2c_driver,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 * It also registers devices for platform DMA, I2S, SSP and registers an
 * I2C driver to probe the codec.
 */
static int __init imx31ads_wm8753_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	int ret;

	machine = kzalloc(sizeof(struct snd_soc_machine), GFP_KERNEL);
	if (machine == NULL)
		return -ENOMEM;

	machine->owner = THIS_MODULE;
	machine->pdev = pdev;
	machine->name = "IMX31ADS";
	machine->longname = "WM8753";
	machine->ops = &imx31ads_mach_ops;
	pdev->dev.driver_data = machine;

	/* register card */
	imx31ads_mach = machine;
	ret = snd_soc_new_card(machine, 2, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", __func__);
		kfree(machine);
		return ret;
	}

	/* register I2C driver for WM8753 codec control */
	ret = i2c_add_driver(&wm8753_i2c_driver);
	if (ret < 0) {
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
		goto err;
	}

	return ret;

err:
	kfree(machine);
	return ret;
}

static int __exit imx31ads_wm8753_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;

	i2c_del_driver(&wm8753_i2c_driver);
	imx31ads_mach = NULL;
	kfree(machine);
	return 0;
}

#ifdef CONFIG_PM
static int imx31ads_wm8753_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	return snd_soc_suspend(machine, state);
}

static int imx31ads_wm8753_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	return snd_soc_resume(machine);
}

#else
#define imx31ads_machine_suspend NULL
#define imx31ads_machine_resume  NULL
#endif

static struct platform_driver imx31ads_wm8753_driver = {
	.probe		= imx31ads_wm8753_probe,
	.remove		= __devexit_p(imx31ads_wm8753_remove),
	.suspend	= imx31ads_wm8753_suspend,
	.resume		= imx31ads_wm8753_resume,
	.driver		= {
		.name		= "IMX31ADS-WM8753",
		.owner		= THIS_MODULE,
	},
};

static int __init imx31ads_asoc_init(void)
{
	return platform_driver_register(&imx31ads_wm8753_driver);
}

static void __exit imx31ads_asoc_exit(void)
{
	platform_driver_unregister(&imx31ads_wm8753_driver);
}

module_init(imx31ads_asoc_init);
module_exit(imx31ads_asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM8753 mx31ads");
MODULE_LICENSE("GPL");
