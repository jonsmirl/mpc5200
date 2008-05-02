/*
 * linux/sound/pxa2xx-ac97.c -- AC97 support for the Intel PXA2xx chip.
 *
 * Author:	Nicolas Pitre
 * Created:	Dec 02, 2004
 * Copyright:	MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <linux/mutex.h>
#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>

#include "pxa2xx-pcm.h"

static DEFINE_MUTEX(car_mutex);
static DECLARE_WAIT_QUEUE_HEAD(gsr_wq);
static volatile long gsr_bits;
static struct clk *ac97_clk;
#ifdef CONFIG_PXA27x
static struct clk *ac97conf_clk;
#endif


struct pxa_ac97_data {
	struct snd_soc_dai *dai[3];
};

/*
 * Beware PXA27x bugs:
 *
 *   o Slot 12 read from modem space will hang controller.
 *   o CDONE, SDONE interrupt fails after any slot 12 IO.
 *
 * We therefore have an hybrid approach for waiting on SDONE (interrupt or
 * 1 jiffy timeout if interrupt never comes).
 */

static unsigned short pxa2xx_ac97_read(struct snd_ac97 *ac97,
	unsigned short reg)
{
	unsigned short val = -1;
	volatile u32 *reg_addr;

	mutex_lock(&car_mutex);

	/* set up primary or secondary codec/modem space */
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	reg_addr = ac97->num ? &SAC_REG_BASE : &PAC_REG_BASE;
#else
	if (reg == AC97_GPIO_STATUS)
		reg_addr = ac97->num ? &SMC_REG_BASE : &PMC_REG_BASE;
	else
		reg_addr = ac97->num ? &SAC_REG_BASE : &PAC_REG_BASE;
#endif
	reg_addr += (reg >> 1);

#ifndef CONFIG_PXA27x
	if (reg == AC97_GPIO_STATUS) {
		/* read from controller cache */
		val = *reg_addr;
		goto out;
	}
#endif

	/* start read access across the ac97 link */
	GSR = GSR_CDONE | GSR_SDONE;
	gsr_bits = 0;
	val = *reg_addr;

	wait_event_timeout(gsr_wq, (GSR | gsr_bits) & GSR_SDONE, 1);
	if (!((GSR | gsr_bits) & GSR_SDONE)) {
		printk(KERN_ERR "%s: read error (ac97_reg=%x GSR=%#lx)\n",
				__func__, reg, GSR | gsr_bits);
		val = -1;
		goto out;
	}

	/* valid data now */
	GSR = GSR_CDONE | GSR_SDONE;
	gsr_bits = 0;
	val = *reg_addr;
	/* but we've just started another cycle... */
	wait_event_timeout(gsr_wq, (GSR | gsr_bits) & GSR_SDONE, 1);

out:	mutex_unlock(&car_mutex);
	return val;
}

static void pxa2xx_ac97_write(struct snd_ac97 *ac97, unsigned short reg,
	unsigned short val)
{
	volatile u32 *reg_addr;

	mutex_lock(&car_mutex);

	/* set up primary or secondary codec/modem space */
#if defined(CONFIG_PXA27x) || defined(CONFIG_PXA3xx)
	reg_addr = ac97->num ? &SAC_REG_BASE : &PAC_REG_BASE;
#else
	if (reg == AC97_GPIO_STATUS)
		reg_addr = ac97->num ? &SMC_REG_BASE : &PMC_REG_BASE;
	else
		reg_addr = ac97->num ? &SAC_REG_BASE : &PAC_REG_BASE;
#endif
	reg_addr += (reg >> 1);

	GSR = GSR_CDONE | GSR_SDONE;
	gsr_bits = 0;
	*reg_addr = val;
	wait_event_timeout(gsr_wq, (GSR | gsr_bits) & GSR_CDONE, 1);
	if (!((GSR | gsr_bits) & GSR_CDONE))
		printk(KERN_ERR "%s: write error (ac97_reg=%x GSR=%#lx)\n",
				__func__, reg, GSR | gsr_bits);

	mutex_unlock(&car_mutex);
}

static void pxa2xx_ac97_warm_reset(struct snd_ac97 *ac97)
{
#ifdef CONFIG_PXA3xx
	int timeout = 100;
#endif
	gsr_bits = 0;

#ifdef CONFIG_PXA27x
	/* warm reset broken on Bulverde,
	   so manually keep AC97 reset high */
	pxa_gpio_mode(113 | GPIO_OUT | GPIO_DFLT_HIGH);
	udelay(10);
	GCR |= GCR_WARM_RST;
	pxa_gpio_mode(113 | GPIO_ALT_FN_2_OUT);
	udelay(500);
#elif defined(CONFIG_PXA3xx)
	/* Can't use interrupts */
	GCR |= GCR_WARM_RST;
	while (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR)) && timeout--)
		mdelay(1);
#else
	GCR |= GCR_WARM_RST | GCR_PRIRDY_IEN | GCR_SECRDY_IEN;
	wait_event_timeout(gsr_wq, gsr_bits & (GSR_PCR | GSR_SCR), 1);
#endif

	if (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR)))
		printk(KERN_INFO "%s: warm reset timeout (GSR=%#lx)\n",
				 __func__, gsr_bits);

	GCR &= ~(GCR_PRIRDY_IEN|GCR_SECRDY_IEN);
	GCR |= GCR_SDONE_IE|GCR_CDONE_IE;
}

static void pxa2xx_ac97_cold_reset(struct snd_ac97 *ac97)
{
#ifdef CONFIG_PXA3xx
	int timeout = 1000;

	/* Hold CLKBPB for 100us */
	GCR = 0;
	GCR = GCR_CLKBPB;
	udelay(100);
	GCR = 0;
#endif

	GCR &=  GCR_COLD_RST;  /* clear everything but nCRST */
	GCR &= ~GCR_COLD_RST;  /* then assert nCRST */

	gsr_bits = 0;
#ifdef CONFIG_PXA27x
	/* PXA27x Developers Manual section 13.5.2.2.1 */
	clk_enable(ac97conf_clk);
	udelay(5);
	clk_disable(ac97conf_clk);
	GCR = GCR_COLD_RST;
	udelay(50);

#elif defined(CONFIG_PXA3xx)
	/* Can't use interrupts on PXA3xx */
	GCR &= ~(GCR_PRIRDY_IEN|GCR_SECRDY_IEN);

	GCR = GCR_WARM_RST | GCR_COLD_RST;
	while (!(GSR & (GSR_PCR | GSR_SCR)) && timeout--)
		mdelay(10);

#else
	GCR = GCR_COLD_RST;
	GCR |= GCR_CDONE_IE|GCR_SDONE_IE;
	wait_event_timeout(gsr_wq, gsr_bits & (GSR_PCR | GSR_SCR), 1);
#endif

	if (!((GSR | gsr_bits) & (GSR_PCR | GSR_SCR)))
		printk(KERN_INFO "%s: cold reset timeout (GSR=%#lx)\n",
				 __func__, gsr_bits);

	GCR &= ~(GCR_PRIRDY_IEN|GCR_SECRDY_IEN);
	GCR |= GCR_SDONE_IE|GCR_CDONE_IE;
}

static irqreturn_t pxa2xx_ac97_irq(int irq, void *dev_id)
{
	long status;

	status = GSR;
	if (status) {
		GSR = status;
		gsr_bits |= status;
		wake_up(&gsr_wq);

#ifdef CONFIG_PXA27x
		/* Although we don't use those we still need to clear them
		   since they tend to spuriously trigger when MMC
		   is used (hardware bug? go figure)... */
		MISR = MISR_EOC;
		PISR = PISR_EOC;
		MCSR = MCSR_EOC;
#endif

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static struct snd_ac97_bus_ops pxa2xx_ac97_ops = {
	.read		= pxa2xx_ac97_read,
	.write		= pxa2xx_ac97_write,
	.warm_reset	= pxa2xx_ac97_warm_reset,
	.reset		= pxa2xx_ac97_cold_reset,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ac97_pcm_stereo_out = {
	.name			= "AC97 PCM Stereo out",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRTXPCDR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ac97_pcm_stereo_in = {
	.name			= "AC97 PCM Stereo in",
	.dev_addr		= __PREG(PCDR),
	.drcmr			= &DRCMRRXPCDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST32 | DCMD_WIDTH4,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ac97_pcm_aux_mono_out = {
	.name			= "AC97 Aux PCM (Slot 5) Mono out",
	.dev_addr		= __PREG(MODR),
	.drcmr			= &DRCMRTXMODR,
	.dcmd			= DCMD_INCSRCADDR | DCMD_FLOWTRG |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ac97_pcm_aux_mono_in = {
	.name			= "AC97 Aux PCM (Slot 5) Mono in",
	.dev_addr		= __PREG(MODR),
	.drcmr			= &DRCMRRXMODR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

static struct pxa2xx_pcm_dma_params pxa2xx_ac97_pcm_mic_mono_in = {
	.name			= "AC97 Mic PCM (Slot 6) Mono in",
	.dev_addr		= __PREG(MCDR),
	.drcmr			= &DRCMRRXMCDR,
	.dcmd			= DCMD_INCTRGADDR | DCMD_FLOWSRC |
				  DCMD_BURST16 | DCMD_WIDTH2,
};

#ifdef CONFIG_PM
static int pxa2xx_ac97_suspend(struct platform_device *dev, pm_message_t state)
{
	GCR |= GCR_ACLINK_OFF;
	clk_disable(ac97_clk);
	return 0;
}

static int pxa2xx_ac97_resume(struct platform_device *dev)
{
#ifndef CONFIG_PXA3xx
	pxa_gpio_mode(GPIO31_SYNC_AC97_MD);
	pxa_gpio_mode(GPIO30_SDATA_OUT_AC97_MD);
	pxa_gpio_mode(GPIO28_BITCLK_AC97_MD);
	pxa_gpio_mode(GPIO29_SDATA_IN_AC97_MD);
#endif
#ifdef CONFIG_PXA27x
	/* Use GPIO 113 as AC97 Reset on Bulverde */
	pxa_gpio_mode(113 | GPIO_ALT_FN_2_OUT);
#endif
	clk_enable(ac97_clk);
	return 0;
}

#else
#define pxa2xx_ac97_suspend	NULL
#define pxa2xx_ac97_resume	NULL
#endif

static int pxa2xx_ac97_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *cpu_dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->dma_data = &pxa2xx_ac97_pcm_stereo_out;
	else
		cpu_dai->dma_data = &pxa2xx_ac97_pcm_stereo_in;

	return 0;
}

static int pxa2xx_ac97_hw_aux_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *cpu_dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->dma_data = &pxa2xx_ac97_pcm_aux_mono_out;
	else
		cpu_dai->dma_data = &pxa2xx_ac97_pcm_aux_mono_in;

	return 0;
}

static int pxa2xx_ac97_hw_mic_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *cpu_dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;
	else
		cpu_dai->dma_data = &pxa2xx_ac97_pcm_mic_mono_in;

	return 0;
}

#define PXA2XX_AC97_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

/* for modprobe */
const char pxa_ac97_hifi_dai_id[] = "pxa2xx-ac97-hifi";
EXPORT_SYMBOL_GPL(pxa_ac97_hifi_dai_id);

const char pxa_ac97_aux_dai_id[] = "pxa2xx-ac97-aux";
EXPORT_SYMBOL_GPL(pxa_ac97_aux_dai_id);

const char pxa_ac97_mic_dai_id[] = "pxa2xx-ac97-mic";
EXPORT_SYMBOL_GPL(pxa_ac97_mic_dai_id);

static struct snd_soc_dai_caps pxa_ac97_hifi_playback = {
	.stream_name	= "HiFi Playback",
	.channels_min	= 2,
	.channels_max	= 2,
	.rates		= PXA2XX_AC97_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
};

static struct snd_soc_dai_caps pxa_ac97_hifi_capture = {
	.stream_name	= "HiFi Capture",
	.channels_min	= 2,
	.channels_max	= 2,
	.rates		= PXA2XX_AC97_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
};

static struct snd_soc_dai_caps pxa_ac97_aux_playback = {
	.stream_name	= "Aux Playback",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= PXA2XX_AC97_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
};

static struct snd_soc_dai_caps pxa_ac97_aux_capture = {
	.stream_name	= "Aux Capture",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= PXA2XX_AC97_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
};

static struct snd_soc_dai_caps pxa_ac97_mic_capture = {
	.stream_name	= "Mic Capture",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= PXA2XX_AC97_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16_LE,
};

static struct snd_soc_dai_ops pxa_ac97_hifi_ops = {
	/* alsa ops */
	.hw_params	= pxa2xx_ac97_hw_params,
	/* ac97_ops */
	.ac97_ops	= &pxa2xx_ac97_ops,
};

static struct snd_soc_dai_ops pxa_ac97_aux_ops = {
	/* alsa ops */
	.hw_params	= pxa2xx_ac97_hw_aux_params,
	/* ac97_ops */
	.ac97_ops	= &pxa2xx_ac97_ops,
};

static struct snd_soc_dai_ops pxa_ac97_mic_ops = {
	/* alsa ops */
	.hw_params	= pxa2xx_ac97_hw_mic_params,
	/* ac97_ops */
	.ac97_ops	= &pxa2xx_ac97_ops,
};

struct snd_soc_dai_new dais[] = {
	{
		.name         = pxa_ac97_hifi_dai_id,
		.ac97_control = 1,
		.playback     = &pxa_ac97_hifi_playback,
		.capture      = &pxa_ac97_hifi_capture,
		.ops          = &pxa_ac97_hifi_ops,
	},
	{
		.name         = pxa_ac97_aux_dai_id,
		.ac97_control = 1,
		.playback     = &pxa_ac97_aux_playback,
		.capture      = &pxa_ac97_aux_capture,
		.ops          = &pxa_ac97_aux_ops,
	},
	{
		.name         = pxa_ac97_mic_dai_id,
		.ac97_control = 1,
		.capture      = &pxa_ac97_mic_capture,
		.ops          = &pxa_ac97_mic_ops,
	},
};

/* IRQ and GPIO's could be platform data */
static int pxa2xx_ac97_probe(struct platform_device *pdev)
{
	struct pxa_ac97_data *ac97;
	int ret, i;

	ac97 = kzalloc(sizeof(struct pxa_ac97_data), GFP_KERNEL);
	if (ac97 == NULL)
		return -ENOMEM;

#ifdef CONFIG_PXA27x
	/* Use GPIO 113 as AC97 Reset on Bulverde */
	pxa_gpio_mode(113 | GPIO_ALT_FN_2_OUT);

	ac97conf_clk = clk_get(&pdev->dev, "AC97CONFCLK");
	if (IS_ERR(ac97conf_clk)) {
		ret = -ENODEV;
		goto unwind_data;
	}
#endif

	ac97_clk = clk_get(&pdev->dev, "AC97CLK");
	if (IS_ERR(ac97_clk)) {
		ret = -ENODEV;
		goto unwind_data;
	}
	clk_enable(ac97_clk);

	for (i = 0; i < ARRAY_SIZE(dais); i++) {
		ac97->dai[i] = snd_soc_register_platform_dai(&dais[i],
							     &pdev->dev);
		if (ac97->dai[i] == NULL) {
			ret = -ENOMEM;
			goto unwind_create;
		}
	}

	ret = request_irq(IRQ_AC97, pxa2xx_ac97_irq, IRQF_DISABLED, "AC97",
		ac97->dai[0]);
	if (ret < 0)
		goto unwind_create;

#ifndef CONFIG_PXA3xx
	pxa_gpio_mode(GPIO31_SYNC_AC97_MD);
	pxa_gpio_mode(GPIO30_SDATA_OUT_AC97_MD);
	pxa_gpio_mode(GPIO28_BITCLK_AC97_MD);
	pxa_gpio_mode(GPIO29_SDATA_IN_AC97_MD);
#endif

	platform_set_drvdata(pdev, ac97);
	return ret;

unwind_create:
	i--;
	for (; i >= 0; i--) {
		snd_soc_unregister_platform_dai(ac97->dai[i]);
	}
unwind_data:
	kfree(ac97);
	return ret;
}

static int pxa2xx_ac97_remove(struct platform_device *pdev)
{
	struct pxa_ac97_data *ac97 = platform_get_drvdata(pdev);
	int i;

	GCR |= GCR_ACLINK_OFF;
	free_irq(IRQ_AC97, ac97->dai[0]);
	clk_disable(ac97_clk);

	for (i = 0; i < 3; i++) {
		kfree(ac97->dai[i]->private_data);
		snd_soc_unregister_platform_dai(ac97->dai[i]);
	}

	return 0;
}

static struct platform_driver pxa_ac97_driver = {
	.driver = {
		.name		= "pxa2xx-ac97",
		.owner		= THIS_MODULE,
	},
	.probe		= pxa2xx_ac97_probe,
	.remove		= __devexit_p(pxa2xx_ac97_remove),
	.suspend	= pxa2xx_ac97_suspend,
	.resume		= pxa2xx_ac97_resume,
};

static __init int pxa_ac97_init(void)
{
	return platform_driver_register(&pxa_ac97_driver);
}

static __exit void pxa_ac97_exit(void)
{
	platform_driver_unregister(&pxa_ac97_driver);
}

module_init(pxa_ac97_init);
module_exit(pxa_ac97_exit);

MODULE_AUTHOR("Nicolas Pitre");
MODULE_DESCRIPTION("Intel PXA2xx PCM AC97 AC97 driver");
MODULE_LICENSE("GPL");
