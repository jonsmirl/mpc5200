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
	struct snd_soc_codec *codec =
		snd_soc_card_get_codec(soc_card, "cirrus,cs4270");
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;
	int ret;

	/* This is stupid. snd_soc_card_config_codec() initializes the
	 * codec->control_data structure, which is supposed to be a pointer to
	 * the i2c_client structure.  But I already assigned that variable in
	 * the codec driver.  After all, the codec driver is the one that
	 * supposed to get probed by the I2C bus.  So in order to avoid
	 * overwriting codec->control_data, I pass it as the 4th parameter.
	 */
	snd_soc_card_config_codec(codec, NULL, NULL, codec->control_data);
	ret = snd_soc_card_init_codec(codec, soc_card);
	if (ret < 0) {
		dev_err(soc_card->dev, "could not initialize codec\n");
		return ret;
	}

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
 *
 * The clock information is stored in the ssi_info data structure because
 * it's gathered by the SSI driver from the device tree.  This isn't really
 * the best way to handle it.  In fact, this problem is a core dillema with
 * ASoC V2: whether the fabric driver should hand the information to the
 * platform/codec drivers, or whether these drivers should get the
 * information themselves.  With device trees, the necessary data may be
 * scattered in various device nodes, so it might be impossible to obtain
 * that information in a generic manner.
 *
 * What makes this problem even more thorny is that some of these drivers
 * are OF drivers and the rest aren't.  The CS4270 can't be an OF driver,
 * yet the information it needs is in an OF device node.  So perhaps the
 * best approach is to have generic codec OF->platform code in the fabric
 * driver (normally this code is in fsl_soc.c).
 *
 * Even more frustrating is the fact that the CS4270 driver is an I2C
 * driver, so it's probed via the I2C bus.  We need to update the powerpc
 * platform to initialize client->dev->archdata.of_node to point to the
 * device node.  Then the CS4270 driver can get its own clock rate.
 */
static int mpc8610_hpcd_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;
	int ret = 0;

	/* Tell the CPU driver what the serial protocol is. */
	ret = snd_soc_dai_set_fmt(cpu_dai, ssi_info->dai_format);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set CPU driver audio format\n");
		return ret;
	}

	/* Tell the codec driver what the serial protocol is. */
	ret = snd_soc_dai_set_fmt(codec_dai, ssi_info->dai_format);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set codec driver audio format\n");
		return ret;
	}

	/*
	 * Tell the CPU driver what the clock frequency is, and whether it's a
	 * slave or master.
	 */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, ssi_info->clk_frequency,
		ssi_info->cpu_clk_direction);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev,
			"could not set CPU driver clock parameters\n");
		return ret;
	}

	/*
	 * Tell the codec driver what the MCLK frequency is, and whether it's
	 * a slave or master.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, ssi_info->clk_frequency,
		ssi_info->codec_clk_direction);
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
static int mpc8610_hpcd_probe(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = NULL;
	struct mpc8610_hpcd_data *soc_card_data;
	struct device_node *guts_np = NULL;
	int ret = -ENODEV;

	soc_card_data = kzalloc(sizeof(struct mpc8610_hpcd_data), GFP_KERNEL);
	if (!soc_card_data) {
		dev_err(&pdev->dev, "could not allocate card structure\n");
		return -ENOMEM;
	}

	/* Map the global utilities registers. */
	guts_np = of_find_compatible_node(NULL, NULL, "fsl,mpc8610-guts");
	if (!guts_np) {
		dev_err(&pdev->dev, "could not obtain address of GUTS\n");
		ret = -EINVAL;
		goto error;
	}
	soc_card_data->guts = of_iomap(guts_np, 0);
	of_node_put(guts_np);
	if (!soc_card_data->guts) {
		dev_err(&pdev->dev, "could not map GUTS\n");
		ret = -EINVAL;
		goto error;
	}

	soc_card = snd_soc_card_create("MPC8610", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (!soc_card) {
		dev_err(&pdev->dev, "could not create card\n");
		goto error;
	}

	soc_card->longname = "MPC8610HPCD";
	soc_card->init = mpc8610_hpcd_audio_init;
	soc_card->exit = mpc8610_hpcd_audio_exit;
	soc_card->private_data = soc_card_data;
	soc_card->dev = &pdev->dev;

	ret = snd_soc_card_create_pcms(soc_card, &mpc8610_pcm_config, 1);
	if (ret) {
		dev_err(&pdev->dev, "could not create PCMs\n");
		goto error;
	}

	platform_set_drvdata(pdev, soc_card);

	ret = snd_soc_card_register(soc_card);
	if (ret) {
		dev_err(&pdev->dev, "could not register card\n");
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

static int mpc8610_hpcd_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;

	if (soc_card_data->guts)
		iounmap(soc_card_data->guts);

	kfree(soc_card_data);

	snd_soc_card_free(soc_card);

	return 0;
}

static struct platform_driver mpc8610_hpcd_driver = {
	.driver = {
		.name   	= "mpc8610_hpcd",
		.owner		= THIS_MODULE,
	},
	.probe  	= mpc8610_hpcd_probe,
	.remove 	= __devexit_p(mpc8610_hpcd_remove),
};

static struct platform_device *pdev;

/**
 * mpc8610_hpcd_init: fabric driver initialization.
 *
 * This function is called when this module is loaded.
 */
static int __init mpc8610_hpcd_init(void)
{
	int ret;

	printk(KERN_INFO "Freescale MPC8610 HPCD ASoC fabric driver\n");

	ret = platform_driver_register(&mpc8610_hpcd_driver);
	if (ret < 0) {
		pr_err("mpc8610-hpcd: could not register platform\n");
		return ret;
	}

	pdev = platform_device_register_simple("mpc8610_hpcd", 0, NULL, 0);
	if (!pdev) {
		pr_err("mpc8610-hpcd: could not register device\n");
		platform_driver_unregister(&mpc8610_hpcd_driver);
		return ret;
	}

	return 0;
}

/**
 * mpc8610_hpcd_exit: fabric driver exit
 *
 * This function is called when this driver is unloaded.
 */
static void __exit mpc8610_hpcd_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&mpc8610_hpcd_driver);
}

module_init(mpc8610_hpcd_init);
module_exit(mpc8610_hpcd_exit);

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Freescale MPC8610 HPCD ASoC fabric driver");
MODULE_LICENSE("GPL");
