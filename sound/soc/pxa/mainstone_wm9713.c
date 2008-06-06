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

#include "../codecs/wm9713.h"
#include "pxa2xx-pcm.h"

#define GPIO11_SSP2RX_MD	(11 | GPIO_ALT_FN_2_IN)
#define GPIO13_SSP2TX_MD	(13 | GPIO_ALT_FN_1_OUT)
#define GPIO22_SSP2CLKS_MD	(22 | GPIO_ALT_FN_3_IN)
#define GPIO88_SSP2FRMS_MD	(88 | GPIO_ALT_FN_3_IN)
#define GPIO22_SSP2CLKM_MD	(22 | GPIO_ALT_FN_3_OUT)
#define GPIO88_SSP2FRMM_MD	(88 | GPIO_ALT_FN_3_OUT)
#define GPIO22_SSP2SYSCLK_MD	(22 | GPIO_ALT_FN_2_OUT)

static int mainstone_voice_startup(struct snd_pcm_substream *substream)
{
	/* enable USB on the go MUX so we can use SSPFRM2 */
	MST_MSCWR2 |= MST_MSCWR2_USB_OTG_SEL;
	MST_MSCWR2 &= ~MST_MSCWR2_USB_OTG_RST;
	return 0;
}

static void mainstone_voice_shutdown(struct snd_pcm_substream *substream)
{
	/* disable USB on the go MUX so we can use ttyS0 */
	MST_MSCWR2 &= ~MST_MSCWR2_USB_OTG_SEL;
	MST_MSCWR2 |= MST_MSCWR2_USB_OTG_RST;
}

static int mainstone_voice_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	unsigned int bclk = 0, pcmdiv = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
		pcmdiv = WM9713_PCMDIV(12); /* 2.048 MHz */
		bclk = WM9713_PCMBCLK_DIV_16; /* 128kHz */
		break;
	case 16000:
		pcmdiv = WM9713_PCMDIV(6); /* 4.096 MHz */
		bclk = WM9713_PCMBCLK_DIV_16; /* 256kHz */
		break;
	case 48000:
		pcmdiv = WM9713_PCMDIV(2); /* 12.288 MHz */
		bclk = WM9713_PCMBCLK_DIV_16; /* 512kHz */
		break;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the SSP system clock as input (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_SSP_CLK_PLL, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM9713_PCMBCLK_DIV, bclk);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM9713_PCMCLK_DIV, pcmdiv);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops mainstone_voice_ops = {
	.startup = mainstone_voice_startup,
	.shutdown = mainstone_voice_shutdown,
	.hw_params = mainstone_voice_hw_params,
};

/* mainstone soc_card dapm widgets */
static const struct snd_soc_dapm_widget mainstone_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic 1", NULL),
	SND_SOC_DAPM_MIC("Mic 2", NULL),
	SND_SOC_DAPM_MIC("Mic 3", NULL),
};

/* example soc_card audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 1"},
	/* mic is connected to mic2A - with bias */
	{"MIC2A", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 2"},
	/* mic is connected to mic2B - with bias */
	{"MIC2B", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 3"},
};

static int mainstone_wm9713_write(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	ac97->bus->ops->write(ac97, reg, val);
	return 0;
}

static int mainstone_wm9713_read(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	val = ac97->bus->ops->read(ac97, reg);
	return 0;
}

static int mainstone_wm9713_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	struct snd_ac97_bus_ops *ac97_ops;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm9713_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	snd_soc_card_config_codec(codec, mainstone_wm9713_read,
		mainstone_wm9713_write, codec->ac97);

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

	/* set up mainstone codec pins */
	snd_soc_dapm_disable_pin(soc_card, "RXP");
	snd_soc_dapm_disable_pin(soc_card, "RXN");

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

static struct snd_soc_pcm_config pcm_configs[] = {
	{
		.name		= "HiFi",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_hifi_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa_ac97_hifi_dai_id,
		.playback	= 1,
		.capture	= 1,
	},
#if 0
	{
		.name		= "Voice",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_voice_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa2xx_ssp2_dai_id,
		.ops		= &mainstone_voice_ops,
		.playback	= 1,
		.capture	= 1,
	},
#endif
	{
		.name		= "Aux",
		.codec		= wm9713_codec_id,
		.codec_dai	= wm9713_codec_aux_dai_id,
		.platform	= pxa_platform_id,
		.cpu_dai	= pxa_ac97_aux_dai_id,
		.playback	= 1,
	},
};

/*
 * This is an example soc_card initialisation for a wm9713 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int mainstone_wm9713_probe(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card;
	int ret;

	/* mainstone wm8753 voice interface */
	pxa_gpio_mode(GPIO11_SSP2RX_MD);
	pxa_gpio_mode(GPIO13_SSP2TX_MD);
	pxa_gpio_mode(GPIO22_SSP2CLKS_MD);
	pxa_gpio_mode(GPIO88_SSP2FRMS_MD);

	soc_card = snd_soc_card_create("mainstone_wm9713", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM9713";
	soc_card->init = mainstone_wm9713_init,
	soc_card->private_data = pdev;
	platform_set_drvdata(pdev, soc_card);

	ret = snd_soc_card_create_pcms(soc_card, pcm_configs,
				  ARRAY_SIZE(pcm_configs));
	if (ret < 0)
		goto err;

	ret = snd_soc_card_register(soc_card);
	return ret;

err:
	snd_soc_card_free(soc_card);
	return ret;
}

static int __exit mainstone_wm9713_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	snd_soc_card_free(soc_card);

	/* disable speaker */
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

#ifdef CONFIG_PM
static long mst_audio_suspend_mask;

static int mainstone_wm9713_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	mst_audio_suspend_mask = MST_MSCWR2;
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_suspend_pcms(soc_card, state);
}

static int mainstone_wm9713_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	MST_MSCWR2 &= mst_audio_suspend_mask | ~MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_resume_pcms(soc_card);
}

#else
#define mainstone_wm9713_suspend NULL
#define mainstone_wm9713_resume  NULL
#endif

static struct platform_driver mainstone_wm9713_driver = {
	.probe		= mainstone_wm9713_probe,
	.remove		= __devexit_p(mainstone_wm9713_remove),
	.suspend	= mainstone_wm9713_suspend,
	.resume		= mainstone_wm9713_resume,
	.driver		= {
		.name		= "Mainstone-WM9713",
		.owner		= THIS_MODULE,
	},
};

static struct platform_device codec = {
	.name		= "wm9713-codec",
	.id		= -1,
};

static struct platform_device platform = {
	.name		= "Mainstone-WM9713",
	.id		= -1,
};

static struct platform_device *devices[] = {
	&codec,
	&platform,
};

static int __init mainstone_asoc_init(void)
{
	platform_add_devices(&devices[0], ARRAY_SIZE(devices));
	return platform_driver_register(&mainstone_wm9713_driver);
}

static void __exit mainstone_asoc_exit(void)
{
	platform_driver_unregister(&mainstone_wm9713_driver);
}

module_init(mainstone_asoc_init);
module_exit(mainstone_asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM9713 Mainstone");
MODULE_LICENSE("GPL");
