/*
 * eti_b1_wm8731  --  SoC audio for AT91RM9200-based Endrelia ETI_B1 board.
 *
 * Author:	Frank Mandarino <fmandarino@endrelia.com>
 *		Endrelia Technologies Inc.
 * Created:	Mar 29, 2006
 *
 * Based on eti_b1.c by:
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
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/arch/hardware.h>
#include <asm/arch/at91_pio.h>
#include <asm/arch/gpio.h>

#include "../codecs/wm8731.h"
#include "at91-pcm.h"
#include "at91-ssc.h"

#if 0
#define	DBG(x...)	printk(KERN_INFO "eti_b1_wm8731: " x)
#else
#define	DBG(x...)
#endif

#define AT91_PIO_TF1	(1 << (AT91_PIN_PB6 - PIN_BASE) % 32)
#define AT91_PIO_TK1	(1 << (AT91_PIN_PB7 - PIN_BASE) % 32)
#define AT91_PIO_TD1	(1 << (AT91_PIN_PB8 - PIN_BASE) % 32)
#define AT91_PIO_RD1	(1 << (AT91_PIN_PB9 - PIN_BASE) % 32)
#define AT91_PIO_RK1	(1 << (AT91_PIN_PB10 - PIN_BASE) % 32)
#define AT91_PIO_RF1	(1 << (AT91_PIN_PB11 - PIN_BASE) % 32)

static struct clk *pck1_clk;
static struct clk *pllb_clk;

static int eti_b1_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* cpu clock is the AT91 master clock sent to the SSC */
	ret = snd_soc_dai_set_sysclk(cpu_dai, AT91_SYSCLK_MCK,
		60000000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* codec system clock is supplied by PCK1, set to 12MHz */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK,
		12000000, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* Start PCK1 clock. */
	clk_enable(pck1_clk);
	DBG("pck1 started\n");

	return 0;
}

static void eti_b1_shutdown(struct snd_pcm_substream *substream)
{
	/* Stop PCK1 clock. */
	clk_disable(pck1_clk);
	DBG("pck1 stopped\n");
}

static int eti_b1_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

#ifdef CONFIG_SND_AT91_SOC_ETI_SLAVE
	unsigned int rate;
	int cmr_div, period;

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

	/*
	 * The SSC clock dividers depend on the sample rate.  The CMR.DIV
	 * field divides the system master clock MCK to drive the SSC TK
	 * signal which provides the codec BCLK.  The TCMR.PERIOD and
	 * RCMR.PERIOD fields further divide the BCLK signal to drive
	 * the SSC TF and RF signals which provide the codec DACLRC and
	 * ADCLRC clocks.
	 *
	 * The dividers were determined through trial and error, where a
	 * CMR.DIV value is chosen such that the resulting BCLK value is
	 * divisible, or almost divisible, by (2 * sample rate), and then
	 * the TCMR.PERIOD or RCMR.PERIOD is BCLK / (2 * sample rate) - 1.
	 */
	rate = params_rate(params);

	switch (rate) {
	case 8000:
		cmr_div = 25;	/* BCLK = 60MHz/(2*25) = 1.2MHz */
		period = 74;	/* LRC = BCLK/(2*(74+1)) = 8000Hz */
		break;
	case 32000:
		cmr_div = 7;	/* BCLK = 60MHz/(2*7) ~= 4.28571428MHz */
		period = 66;	/* LRC = BCLK/(2*(66+1)) = 31982.942Hz */
		break;
	case 48000:
		cmr_div = 13;	/* BCLK = 60MHz/(2*13) ~= 2.3076923MHz */
		period = 23;	/* LRC = BCLK/(2*(23+1)) = 48076.923Hz */
		break;
	default:
		printk(KERN_WARNING "unsupported rate %d on ETI-B1 board\n",
			rate);
		return -EINVAL;
	}

	/* set the MCK divider for BCLK */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, AT91SSC_CMR_DIV, cmr_div);
	if (ret < 0)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set the BCLK divider for DACLRC */
		ret = snd_soc_dai_set_clkdiv(cpu_dai,
						AT91SSC_TCMR_PERIOD, period);
	} else {
		/* set the BCLK divider for ADCLRC */
		ret = snd_soc_dai_set_clkdiv(cpu_dai,
						AT91SSC_RCMR_PERIOD, period);
	}
	if (ret < 0)
		return ret;

#else /* CONFIG_SND_AT91_SOC_ETI_SLAVE */
	/*
	 * Codec in Master Mode.
	 */

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

#endif /* CONFIG_SND_AT91_SOC_ETI_SLAVE */

	return 0;
}

static struct snd_soc_ops eti_b1_ops = {
	.startup = eti_b1_startup,
	.hw_params = eti_b1_hw_params,
	.shutdown = eti_b1_shutdown,
};

static const struct snd_soc_dapm_widget eti_b1_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {

	/* speaker connected to LHPOUT */
	{"Ext Spk", NULL, "LHPOUT"},

	/* mic is connected to Mic Jack, with WM8731 Mic Bias */
	{"MICIN", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
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

static int eti_b1_wm8731_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

/*
 * Logic for a wm8731 as connected on a Sharp SL-C7x0 Device
 */
static int eti_b1_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dai *dai;
	struct at91_ssc_periph *ssc;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, wm8731_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	dai = snd_soc_card_get_dai(soc_card, at91_ssc_id1);
	if (dai == NULL)
		return -ENODEV;
	ssc = dai->private_data;
	ssc->pid = AT91RM9200_ID_SSC1;

	/* set up eti_b1 codec pins */
	snd_soc_dapm_disable_pin(soc_card, "RLINEIN");
	snd_soc_dapm_disable_pin(soc_card, "LLINEIN");

	/* always connected */
	snd_soc_dapm_enable_pin(soc_card, "Int Mic");
	snd_soc_dapm_enable_pin(soc_card, "Ext Spk");

	/* Add eti_b1 specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
			eti_b1_dapm_widgets, ARRAY_SIZE(eti_b1_dapm_widgets));
	if (ret < 0)
		return ret;

	/* Set up eti_b1 specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	snd_soc_dapm_sync(soc_card);

	snd_soc_card_config_codec(codec, NULL, eti_b1_wm8731_write,
		soc_card->private_data);

	if (!request_mem_region(AT91RM9200_BASE_SSC1, SZ_16K, at91_ssc_id1)) {
		DBG("SSC1 memory region is busy\n");
		return -EBUSY;
	}

	ssc->base = ioremap(AT91RM9200_BASE_SSC1, SZ_16K);
	if (!ssc->base) {
		DBG("SSC1 memory ioremap failed\n");
		release_mem_region(AT91RM9200_BASE_SSC1, SZ_16K);
		return -ENOMEM;
	}

	snd_soc_card_init_codec(codec, soc_card);
	return 0;
}

static void eti_b1_exit(struct snd_soc_card *soc_card)
{
	struct at91_ssc_periph *ssc;
	struct snd_soc_dai *dai;

	dai = snd_soc_card_get_dai(soc_card, at91_ssc_id1);
	if (dai == NULL)
		return;
	ssc = dai->private_data;

	iounmap(ssc->base);
	release_mem_region(AT91RM9200_BASE_SSC1, SZ_16K);
}

static struct snd_soc_pcm_config hifi_pcm_config = {
	.name		= "HiFi",
	.codec		= wm8731_codec_id,
	.codec_dai	= wm8731_codec_dai_id,
	.platform	= at91_platform_id,
	.cpu_dai	= at91_ssc_id1,
	.ops		= &eti_b1_ops,
	.playback	= 1,
	.capture	= 1,
};

static int wm8731_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_card *soc_card;
	struct i2c_client *i2c;
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

	soc_card = snd_soc_card_create("eti_b1", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "WM8731";
	soc_card->init = eti_b1_init;
	soc_card->exit = eti_b1_exit;
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

static int __init eti_b1_wm8731_probe(struct platform_device *pdev)
{
	int ret;
	u32 ssc_pio_lines;

	ssc_pio_lines = AT91_PIO_TF1 | AT91_PIO_TK1 | AT91_PIO_TD1
			| AT91_PIO_RD1 /* | AT91_PIO_RK1 */ | AT91_PIO_RF1;

	/* Reset all PIO registers and assign lines to peripheral A */
	at91_sys_write(AT91_PIOB + PIO_PDR,  ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_ODR,  ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_IFDR, ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_CODR, ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_IDR,  ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_MDDR, ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_PUDR, ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_ASR,  ssc_pio_lines);
	at91_sys_write(AT91_PIOB + PIO_OWDR, ssc_pio_lines);

	/*
	 * Set PCK1 parent to PLLB and its rate to 12 Mhz.
	 */
	pllb_clk = clk_get(NULL, "pllb");
	pck1_clk = clk_get(NULL, "pck1");

	clk_set_parent(pck1_clk, pllb_clk);
	clk_set_rate(pck1_clk, 12000000);

	DBG("MCLK rate %luHz\n", clk_get_rate(pck1_clk));

	/* assign the GPIO pin to PCK1 */
	at91_set_B_periph(AT91_PIN_PA24, 0);

#ifdef CONFIG_SND_AT91_SOC_ETI_SLAVE
	printk(KERN_INFO "eti_b1_wm8731: Codec in Slave Mode\n");
#else
	printk(KERN_INFO "eti_b1_wm8731: Codec in Master Mode\n");
#endif

	/* register I2C driver for WM8731 codec control */
	ret = i2c_add_driver(&wm8731_i2c_driver);
	if (ret < 0)
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
	return ret;
}

static int __exit eti_b1_wm8731_remove(struct platform_device *pdev)
{
	i2c_del_driver(&wm8731_i2c_driver);
	clk_put(pck1_clk);
	clk_put(pllb_clk);
	return 0;
}

#ifdef CONFIG_PM
static int eti_b1_wm8731_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;
	return snd_soc_suspend(soc_card, state);
}

static int eti_b1_wm8731_resume(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = pdev->dev.driver_data;
	return snd_soc_resume(soc_card);
}

#else
#define eti_b1_wm8731_suspend NULL
#define eti_b1_wm8731_resume  NULL
#endif

static struct platform_driver eti_b1_wm8731_driver = {
	.probe		= eti_b1_wm8731_probe,
	.remove		= __devexit_p(eti_b1_wm8731_remove),
	.suspend	= eti_b1_wm8731_suspend,
	.resume		= eti_b1_wm8731_resume,
	.driver		= {
		.name		= "eti_b1-wm8731",
		.owner		= THIS_MODULE,
	},
};

static int __init eti_b1_asoc_init(void)
{
	return platform_driver_register(&eti_b1_wm8731_driver);
}

static void __exit eti_b1_asoc_exit(void)
{
	platform_driver_unregister(&eti_b1_wm8731_driver);
}

module_init(eti_b1_asoc_init);
module_exit(eti_b1_asoc_exit);

/* Module information */
MODULE_AUTHOR("Frank Mandarino <fmandarino@endrelia.com>");
MODULE_DESCRIPTION("ALSA SoC ETI-B1-WM8731");
MODULE_LICENSE("GPL");
