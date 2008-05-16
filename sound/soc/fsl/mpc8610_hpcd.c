/**
 * Freescale MPC8610HPCD ALSA SoC Fabric driver
 *
 * Author: Timur Tabi <timur@freescale.com>
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc.  This file is licensed
 * under the terms of the GNU General Public License version 2.  This
 * program is licensed "as is" without any warranty of any kind, whether
 * express or implied.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/immap_86xx.h>

#include "../codecs/cs4270.h"
#include "fsl_dma.h"
#include "fsl_ssi.h"

/**
 * mpc8610_hpcd_data: fabric-specific ASoC device data
 *
 * This structure contains data for a single sound platform device on an
 * MPC8610 HPCD.  Some of the data is taken from the device tree.
 */
struct mpc8610_hpcd_data {
	struct snd_soc_card soc_card;
	unsigned int dai_format;
	unsigned int codec_clk_direction;
	unsigned int cpu_clk_direction;
	unsigned int clk_frequency;
	struct ccsr_guts __iomem *guts;
};

/**
 * mpc8610_hpcd_audio_init: initalize the board
 *
 * This function is called when platform_device_add() is called.  It is used
 * to initialize the board-specific hardware.
 *
 * Here we program the DMACR and PMUXCR registers.
 */
static int mpc8610_hpcd_audio_init(struct snd_soc_card *soc_card)
{
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;
	struct snd_soc_dai *cpu_dai =
		snd_soc_card_get_dai(soc_card, "fsl,mpc8610-ssi");
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;

	/* Program the signal routing between the SSI and the DMA */
	guts_set_dmacr(soc_card_data->guts,
		ssi_info->dma_info[0].controller_id,
		ssi_info->dma_info[0].channel_id, CCSR_GUTS_DMACR_DEV_SSI);
	guts_set_dmacr(soc_card_data->guts,
		ssi_info->dma_info[1].controller_id,
		ssi_info->dma_info[1].channel_id, CCSR_GUTS_DMACR_DEV_SSI);

	guts_set_pmuxcr_dma(soc_card_data->guts,
		ssi_info->dma_info[0].controller_id,
		ssi_info->dma_info[0].channel_id, 0);
	guts_set_pmuxcr_dma(soc_card_data->guts,
		ssi_info->dma_info[1].controller_id,
		ssi_info->dma_info[1].channel_id, 0);

	/* FIXME: Magic numbers? */
	guts_set_pmuxcr_dma(soc_card_data->guts, 1, 0, 0);
	guts_set_pmuxcr_dma(soc_card_data->guts, 1, 3, 0);
	guts_set_pmuxcr_dma(soc_card_data->guts, 0, 3, 0);

	switch (ssi_info->id) {
	case 0:
		clrsetbits_be32(&soc_card_data->guts->pmuxcr,
			CCSR_GUTS_PMUXCR_SSI1_MASK, CCSR_GUTS_PMUXCR_SSI1_SSI);
		break;
	case 1:
		clrsetbits_be32(&soc_card_data->guts->pmuxcr,
			CCSR_GUTS_PMUXCR_SSI2_MASK, CCSR_GUTS_PMUXCR_SSI2_SSI);
		break;
	}

	return 0;
}

/**
 * mpc8610_hpcd_startup: program the board with various hardware parameters
 *
 * This function takes board-specific information, like clock frequencies
 * and serial data formats, and passes that information to the codec and
 * transport drivers.
 */
static int mpc8610_hpcd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct mpc8610_hpcd_data *soc_card_data = rtd->soc_card->private_data;
	int ret = 0;

	/* Tell the CPU driver what the serial protocol is. */
	ret = snd_soc_dai_set_fmt(cpu_dai, soc_card_data->dai_format);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set CPU driver audio format\n");
		return ret;
	}

	/* Tell the codec driver what the serial protocol is. */
	ret = snd_soc_dai_set_fmt(codec_dai, soc_card_data->dai_format);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set codec driver audio format\n");
		return ret;
	}

	/*
	 * Tell the CPU driver what the clock frequency is, and whether it's a
	 * slave or master.
	 */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, soc_card_data->clk_frequency,
		soc_card_data->cpu_clk_direction);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set CPU driver clock parameters\n");
		return ret;
	}

	/*
	 * Tell the codec driver what the MCLK frequency is, and whether it's
	 * a slave or master.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, soc_card_data->clk_frequency,
		soc_card_data->codec_clk_direction);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set codec driver clock params\n");
		return ret;
	}

	return 0;
}

/**
 * mpc8610_hpcd_audio_exit: Remove the sound device
 *
 * This function is called to remove the sound device for one SSI.  We
 * de-program the DMACR and PMUXCR register.
 */
void mpc8610_hpcd_audio_exit(struct snd_soc_card *soc_card)
{
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;
	struct snd_soc_dai *cpu_dai =
		snd_soc_card_get_dai(soc_card, "fsl,mpc8610-ssi");
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;

	/* Restore the signal routing */

	guts_set_dmacr(soc_card_data->guts,
		ssi_info->dma_info[0].controller_id,
		ssi_info->dma_info[0].channel_id, 0);
	guts_set_dmacr(soc_card_data->guts,
		ssi_info->dma_info[1].controller_id,
		ssi_info->dma_info[1].channel_id, 0);

	switch (ssi_info->id) {
	case 0:
		clrsetbits_be32(&soc_card_data->guts->pmuxcr,
			CCSR_GUTS_PMUXCR_SSI1_MASK, CCSR_GUTS_PMUXCR_SSI1_LA);
		break;
	case 1:
		clrsetbits_be32(&soc_card_data->guts->pmuxcr,
			CCSR_GUTS_PMUXCR_SSI2_MASK, CCSR_GUTS_PMUXCR_SSI1_LA);
		break;
	}
}

/**
 * mpc8610_hpcd_ops: ASoC fabric driver operations
 */
static struct snd_soc_ops mpc8610_hpcd_ops = {
	.startup = mpc8610_hpcd_startup,
};

static struct snd_soc_pcm_config mpc8610_pcm_config = {
	.name		= "MPC8610 HPCD",
	.codec		= "cirrus,cs4270",
	.codec_dai	= "cirrus,cs4270",
	.platform	= "fsl_pcm",
	.cpu_dai	= "fsl,mpc8610-ssi",
	.ops		= &mpc8610_hpcd_ops,
	.playback	= 1,
	.capture	= 1,
};

/**
 * mpc8610_hpcd_probe: OF probe function for the fabric driver
 *
 * This function gets called when an SSI node is found in the device tree.
 *
 * Although this is a fabric driver, the SSI node is the "master" node with
 * respect to audio hardware connections.  Therefore, we create a new ASoC
 * device for each new SSI node that has a codec attached.
 */
static int mpc8610_hpcd_probe(struct of_device *ofdev,
	const struct of_device_id *match)
{
	struct snd_soc_card *soc_card = NULL;
	struct device_node *guts_np = NULL;
	struct mpc8610_hpcd_data *soc_card_data;
	int ret = -ENODEV;

	soc_card_data = kzalloc(sizeof(struct mpc8610_hpcd_data), GFP_KERNEL);
	if (!soc_card_data) {
		dev_err(&ofdev->dev, "could not allocate card structure\n");
		return -ENOMEM;
	}

	/* Map the global utilities registers. */
	guts_np = of_find_compatible_node(NULL, NULL, "fsl,mpc8610-guts");
	if (!guts_np) {
		dev_err(&ofdev->dev, "could not obtain address of GUTS\n");
		ret = -EINVAL;
		goto error;
	}
	soc_card_data->guts = of_iomap(guts_np, 0);
	of_node_put(guts_np);
	if (!soc_card_data->guts) {
		dev_err(&ofdev->dev, "could not map GUTS\n");
		ret = -EINVAL;
		goto error;
	}

	soc_card = snd_soc_card_create("MPC8610", &ofdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (!soc_card) {
		dev_err(&ofdev->dev, "could not create card\n");
		goto error;
	}

	soc_card->longname = "MPC8610HPCD";
	soc_card->init = mpc8610_hpcd_audio_init;
	soc_card->exit = mpc8610_hpcd_audio_exit;
	soc_card->private_data = soc_card_data;
	soc_card->dev = &ofdev->dev;

	ret = snd_soc_card_create_pcms(soc_card, &mpc8610_pcm_config, 1);
	if (ret) {
		dev_err(&ofdev->dev, "could not create PCMs\n");
		goto error;
	}

	/* every has been added at this point */
	dev_set_drvdata(&ofdev->dev, soc_card);
	ret = snd_soc_card_register(soc_card);
	if (ret) {
		dev_err(&ofdev->dev, "could not register card\n");
		goto error;
	}

	return 0;

error:
	if (soc_card_data->guts)
		iounmap(soc_card_data->guts);

	kfree(soc_card_data);

	if (soc_card)
		snd_soc_card_free(soc_card);

	return ret;
}

/**
 * mpc8610_hpcd_remove: remove the OF device
 *
 * This function is called when the OF device is removed.
 */
static int mpc8610_hpcd_remove(struct of_device *ofdev)
{
	struct snd_soc_card *soc_card = dev_get_drvdata(&ofdev->dev);
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;

	if (soc_card_data->guts)
		iounmap(soc_card_data->guts);

	kfree(soc_card_data);

	snd_soc_card_free(soc_card);

	dev_set_drvdata(&ofdev->dev, NULL);

	return 0;
}

static struct of_device_id mpc8610_hpcd_match[] = {
	{
		.compatible = "fsl,mpc8610-ssi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, mpc8610_hpcd_match);

static struct of_platform_driver mpc8610_hpcd_of_driver = {
	.owner  	= THIS_MODULE,
	.name   	= "mpc8610_hpcd",
	.match_table    = mpc8610_hpcd_match,
	.probe  	= mpc8610_hpcd_probe,
	.remove 	= mpc8610_hpcd_remove,
};

/**
 * mpc8610_hpcd_init: fabric driver initialization.
 *
 * This function is called when this module is loaded.
 */
static int __init mpc8610_hpcd_init(void)
{
	int ret;

	printk(KERN_INFO "Freescale MPC8610 HPCD ALSA SoC fabric driver\n");

	ret = of_register_platform_driver(&mpc8610_hpcd_of_driver);

	if (ret)
		printk(KERN_ERR
			"mpc8610-hpcd: failed to register platform driver\n");

	return ret;
}

/**
 * mpc8610_hpcd_exit: fabric driver exit
 *
 * This function is called when this driver is unloaded.
 */
static void __exit mpc8610_hpcd_exit(void)
{
	of_unregister_platform_driver(&mpc8610_hpcd_of_driver);
}

module_init(mpc8610_hpcd_init);
module_exit(mpc8610_hpcd_exit);

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Freescale MPC8610 HPCD ALSA SoC fabric driver");
MODULE_LICENSE("GPL");
