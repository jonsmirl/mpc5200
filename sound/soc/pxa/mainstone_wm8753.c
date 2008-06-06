/*
 * mainstone.c  --  SoC audio for Mainstone
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
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
 *    30th Oct 2005   Initial version.
 *
 *
 * This is an example soc_card driver for a wm8753 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mainstone.h>
#include <asm/arch/audio.h>

#include "../codecs/wm8753.h"
#include "pxa2xx-pcm.h"

/*
 * SSP GPIO's
 */
#define GPIO11_SSP2RX_MD	(11 | GPIO_ALT_FN_2_IN)
#define GPIO13_SSP2TX_MD	(13 | GPIO_ALT_FN_1_OUT)
#define GPIO22_SSP2CLKS_MD	(22 | GPIO_ALT_FN_3_IN)
#define GPIO88_SSP2FRMS_MD	(88 | GPIO_ALT_FN_3_IN)
#define GPIO22_SSP2CLKM_MD	(22 | GPIO_ALT_FN_3_OUT)
#define GPIO88_SSP2FRMM_MD	(88 | GPIO_ALT_FN_3_OUT)
#define GPIO22_SSP2SYSCLK_MD	(22 | GPIO_ALT_FN_2_OUT)

static int mainstone_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_rdai = pcm_runtime->codec_dai;
	unsigned int pll_out = 0, bclk = 0, fmt = 0;
	int ret = 0;

	/*
	 * The WM8753 is far better at generating accurate audio clocks than the
	 * pxa2xx I2S controller, so we will use it as master when we can.
	 * i.e all rates except 8k and 16k as BCLK must be 64 * rate when the
	 * pxa27x or pxa25x is slave. Note this restriction does not apply to SSP
	 * I2S emulation mode.
	 */
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
	ret = snd_soc_dai_set_fmt(codec_rdai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | fmt);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_rdai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | fmt);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_rdai, WM8753_MCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_rdai, PXA2XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_rdai, WM8753_BCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* codec PLL input is 13 MHz */
	ret = snd_soc_dai_set_pll(codec_rdai, WM8753_PLL1, 13000000, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int mainstone_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *codec_rdai = pcm_runtime->codec_dai;

	/* disable the PLL */
	return snd_soc_dai_set_pll(codec_rdai, WM8753_PLL1, 0, 0);
}

/*
 * Mainstone WM8753 HiFi DAI opserations.
 */
static struct snd_soc_ops mainstone_hifi_ops = {
	.hw_params = mainstone_hifi_hw_params,
	.hw_free = mainstone_hifi_hw_free,
};

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
	struct snd_soc_dai *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_rdai = pcm_runtime->codec_dai;
	unsigned int pll_out = 0, bclk = 0, pcmdiv = 0;
	int ret = 0;

	/*
	 * The WM8753 is far better at generating accurate audio clocks than the
	 * pxa2xx SSP controller, so we will use it as master when we can.
	 */
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
	ret = snd_soc_dai_set_fmt(codec_rdai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_rdai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_rdai, WM8753_PCMCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the SSP system clock as input (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_rdai, PXA2XX_SSP_CLK_PLL, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_rdai, WM8753_VXCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_rdai, WM8753_PCMDIV, pcmdiv);
	if (ret < 0)
		return ret;

	/* codec PLL input is 13 MHz */
	ret = snd_soc_dai_set_pll(codec_rdai, WM8753_PLL2, 13000000, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int mainstone_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *codec_rdai = pcm_runtime->codec_dai;

	/* disable the PLL */
	return snd_soc_dai_set_pll(codec_rdai, WM8753_PLL2, 0, 0);
}

static struct snd_soc_ops mainstone_voice_ops = {
	.startup = mainstone_voice_startup,
	.shutdown = mainstone_voice_shutdown,
	.hw_params = mainstone_voice_hw_params,
	.hw_free = mainstone_voice_hw_free,
};

/* example soc_card audio_mapnections */
static const struct snd_soc_dapm_route audio_map[] = {

	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC1N", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic1 Jack"},
	{"Mic Bias", NULL, "Mic1 Jack"},

	{"ACIN", NULL, "ACOP"},
};

/* headphone detect support on my board */
static const char * hp_pol[] = {"Headphone", "Speaker"};
static const struct soc_enum wm8753_enum =
	SOC_ENUM_SINGLE(WM8753_OUTCTL, 1, 2, hp_pol);

static const struct snd_kcontrol_new wm8753_mainstone_controls[] = {
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

static int mainstone_wm8753_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

static int mainstone_wm8753_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm8753_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	/* set up mainstone codec pins */
	snd_soc_dapm_disable_pin(soc_card, "RXP");
	snd_soc_dapm_disable_pin(soc_card, "RXN");
	snd_soc_dapm_disable_pin(soc_card, "MIC2");

	/* add mainstone specific controls */
	ret = snd_soc_add_new_controls(soc_card, wm8753_mainstone_controls,
		soc_card, ARRAY_SIZE(wm8753_mainstone_controls));
	if (ret < 0)
		return ret;

	/* set up mainstone specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	snd_soc_dapm_sync(soc_card);

	snd_soc_card_config_codec(codec, NULL, mainstone_wm8753_write,
		soc_card->private_data);

	snd_soc_card_init_codec(codec, soc_card);

	/* enable speaker */
	MST_MSCWR2 &= ~MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

static struct snd_soc_pcm_config pcm_config[] = {
{
	.name		= "HiFi",
	.codec		= wm8753_codec_id,
	.codec_dai	= wm8753_codec_hifi_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa2xx_i2s_dai_id,
	.ops		= &mainstone_hifi_ops,
	.playback	= 1,
	.capture	= 1,
},
{
	.name		= "Voice",
	.codec		= wm8753_codec_id,
	.codec_dai	= wm8753_codec_voice_dai_id,
	.platform	= pxa_platform_id,
//	.cpu_dai	= imx_ssi_id1_0,
	.ops		= &mainstone_voice_ops,
	.playback	= 1,
	.capture	= 1,
},};

static int wm8753_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_card *soc_card;
	struct i2c_client *i2c;
	int ret;

	if (addr != WM8753_I2C_ADDR)
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

	/* mainstone wm8753 voice interface */
	pxa_gpio_mode(GPIO11_SSP2RX_MD);
	pxa_gpio_mode(GPIO13_SSP2TX_MD);
	pxa_gpio_mode(GPIO22_SSP2CLKS_MD);
	pxa_gpio_mode(GPIO88_SSP2FRMS_MD);

	soc_card = snd_soc_card_create("mainstone_wm8753", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM8753";
	soc_card->init = mainstone_wm8753_init;
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

static int wm8753_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_card *soc_card = i2c_get_clientdata(client);

	snd_soc_card_free(soc_card);
	i2c_detach_client(client);
	kfree(client);

	/* disable speaker */
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
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
 * This function will register the snd_soc_pcm_runtime drivers.
 * It also registers devices for platform DMA, I2S, SSP and registers an
 * I2C driver to probe the codec.
 */
static int __init mainstone_wm8753_probe(struct platform_device *pdev)
{
	int ret;

	/* register I2C driver for WM8753 codec control */
	ret = i2c_add_driver(&wm8753_i2c_driver);
	if (ret < 0) {
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
	}
	return ret;
}

static int __exit mainstone_wm8753_remove(struct platform_device *pdev)
{
	i2c_del_driver(&wm8753_i2c_driver);
	return 0;
}

#ifdef CONFIG_PM
static long mst_audio_suspend_mask;

static int mainstone_wm8753_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;

	mst_audio_suspend_mask = MST_MSCWR2;
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_suspend(soc_card, state);
}

static int mainstone_wm8753_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;

	MST_MSCWR2 &= mst_audio_suspend_mask | ~MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_resume(soc_card);
}

#else
#define mainstone_wm8753_suspend NULL
#define mainstone_wm8753_resume  NULL
#endif

static struct platform_driver mainstone_wm8753_driver = {
	.probe		= mainstone_wm8753_probe,
	.remove		= __devexit_p(mainstone_wm8753_remove),
	.suspend	= mainstone_wm8753_suspend,
	.resume		= mainstone_wm8753_resume,
	.driver		= {
		.name		= "Mainstone-WM8753",
		.owner		= THIS_MODULE,
	},
};

static int __init mainstone_asoc_init(void)
{
	return platform_driver_register(&mainstone_wm8753_driver);
}

static void __exit mainstone_asoc_exit(void)
{
	platform_driver_unregister(&mainstone_wm8753_driver);
}

module_init(mainstone_asoc_init);
module_exit(mainstone_asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM8753 Mainstone");
MODULE_LICENSE("GPL");
