/**
 * Freescale MPC8610 HPCD ASoC fabric driver
 *
 * Author: Timur Tabi <timur@freescale.com>
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc.  This file is licensed
 * under the terms of the GNU General Public License version 2.  This
 * program is licensed "as is" without any warranty of any kind, whether
 * express or implied.
 */

#define DEBUG

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

/* To keep memory allocations simple, we define a limit to the number of
 * SSIs we support.
 */
#define MAX_SSI		2

/**
 * mpc8610_hpcd_data: fabric-specific ASoC device data
 *
 * This structure contains data for a single sound platform device on an
 * MPC8610 HPCD.  Some of the data is taken from the device tree.
 */
struct mpc8610_hpcd_data {
	struct snd_soc_card soc_card;
	struct ccsr_guts __iomem *guts;
	struct snd_soc_pcm_config pcm_config;
	char pcm_name[32];
	char ssi_name[32];
	char codec_name[32];
};

/**
 * guts_dmacr_mask: return the DMACR bitmask for a given DMA channel
 * @co: The DMA controller (0 or 1)
 * @ch: The channel on the DMA controller (0, 1, 2, or 3)
 */
static inline u32 guts_dmacr_mask(unsigned int co, unsigned int ch)
{
	unsigned int shift = 16 + (8 * (1 - co) + 2 * (3 - ch));

	return 3 << shift;
}

/**
 * guts_pmuxcr_dma_mask: return the mask for the DMA bits in the PMXUCR
 * @co: The DMA controller (0 or 1)
 * @ch: The channel on the DMA controller (0, 1, 2, or 3)
 *
 * If ch is not 0 or 3, then a value of 0 is returned.
 */
static inline u32 guts_pmuxcr_dma_mask(unsigned int co, unsigned int ch)
{
	if ((ch == 0) || (ch == 3)) {
		unsigned int shift = 2 * (co + 1) - (ch & 1) - 1;

		return 1 << shift;
	} else
		return 0;
}

/**
 * mpc8610_hpcd_audio_init: initalize the board
 *
 * This function is called when platform_device_add() is called.  It is used
 * to initialize the board-specific hardware.
 *
 * If the platform (SSI) driver is loaded last, then cpu_dai->private_data
 * is not yet initialized, so therefore we cannot get ssi_info.
 */
static int mpc8610_hpcd_audio_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;
	int ret = 0;

	codec = snd_soc_card_get_codec(soc_card, soc_card_data->codec_name,
		soc_card_data->pcm_config.codec_num);

	if (!codec) {
		dev_err(soc_card->dev, "could not find codec\n");
		return -ENODEV;
	}

	/* The codec driver should have called
	 * snd_soc_card_config_codec() by now.
	 */
	ret = snd_soc_card_init_codec(codec, soc_card);
	if (ret < 0) {
		dev_err(soc_card->dev,
			"could not initialize codec %s-%u\n",
			codec->name, codec->num);
		return -ENODEV;
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
	struct snd_soc_card *soc_card = rtd->soc_card;
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;
	int ret = 0;

	/* Save the PMUXCR and DMACR registers so that we can restore them
	 * properly later.
	 */
	ssi_info->dmacr = in_be32(&soc_card_data->guts->dmacr);
	ssi_info->pmuxcr = in_be32(&soc_card_data->guts->pmuxcr);

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

static void mpc8610_hpcd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;
	struct snd_soc_card *soc_card = rtd->soc_card;
	struct mpc8610_hpcd_data *soc_card_data = soc_card->private_data;
	u32 mask;

	soc_card_data = soc_card->private_data;
	ssi_info = cpu_dai->private_data;

	/* Restore the DMACR register */
	mask = guts_dmacr_mask(ssi_info->dma_info[0].controller_id,
		ssi_info->dma_info[0].channel_id) |
		guts_dmacr_mask(ssi_info->dma_info[1].controller_id,
		ssi_info->dma_info[1].channel_id);

	clrsetbits_be32(&soc_card_data->guts->dmacr, mask,
		ssi_info->dmacr & mask);

	/* Restore the PMUXCR register */
	mask = guts_pmuxcr_dma_mask(ssi_info->dma_info[0].controller_id,
		ssi_info->dma_info[0].channel_id) |
		guts_pmuxcr_dma_mask(ssi_info->dma_info[1].controller_id,
		ssi_info->dma_info[1].channel_id) |
		(ssi_info->id ? CCSR_GUTS_PMUXCR_SSI2_MASK :
		CCSR_GUTS_PMUXCR_SSI1_MASK);

	clrsetbits_be32(&soc_card_data->guts->pmuxcr, mask,
			ssi_info->pmuxcr & mask);
}

/**
 * find_codec: return the node for the codec of a given SSI
 * @ssi_np: the device node for the SSI to look up
 */
static struct device_node *find_codec(struct device_node *ssi_np)
{
	const phandle *codec_ph;

	codec_ph = of_get_property(ssi_np, "codec-handle", NULL);
	if (!codec_ph)
		return NULL;

	return of_find_node_by_phandle(*codec_ph);
}

static int of_get_integer(struct device_node *np, const char *string)
{
	const u32 *iprop;
	int len;

	iprop = of_get_property(np, string, &len);
	if (!iprop)
		return -1;

	if (len != sizeof(u32))
		return -1;

	return *iprop;
}

/**
 * mpc8610_hpcd_ops: ASoC fabric driver operations
 */
static struct snd_soc_ops mpc8610_hpcd_ops = {
	.startup = mpc8610_hpcd_startup,
	.shutdown = mpc8610_hpcd_shutdown,
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
	struct device_node *ssi_np =
		(struct device_node *)platform_get_resource(pdev, 0, 0)->start;
	int ret = -ENODEV;

	struct device_node *codec_np;
	const char *compat;
	const char *p;
	unsigned int bus;
	unsigned int address;

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
	soc_card->private_data = soc_card_data;
	soc_card->dev = &pdev->dev;

	soc_card_data->pcm_config.name = soc_card_data->pcm_name;
	soc_card_data->pcm_config.codec = soc_card_data->codec_name;
	soc_card_data->pcm_config.codec_dai = soc_card_data->codec_name;
	soc_card_data->pcm_config.platform = "fsl-elo";
	soc_card_data->pcm_config.cpu_dai = soc_card_data->ssi_name;
	soc_card_data->pcm_config.ops = &mpc8610_hpcd_ops;
	soc_card_data->pcm_config.playback = 1;
	soc_card_data->pcm_config.capture = 1;

	sprintf(soc_card_data->pcm_name, "MPC8610HPCD.%u", pdev->id);

	/* Get the SSI name */
	sprintf(soc_card_data->ssi_name, "ssi%u", pdev->id);

	/* Get the codec name and ID */
	codec_np = find_codec(ssi_np);
	if (!codec_np) {
		dev_err(soc_card->dev, "missing codec node for %s\n",
			ssi_np->full_name);
		goto error;
	}

	/* We assume that the first (or only) string in the compatible
	 * field is the one that counts.
	 */
	compat = of_get_property(codec_np, "compatible", NULL);
	if (!compat) {
		dev_err(soc_card->dev,
			"missing compatible property for %s\n",
			ssi_np->full_name);
		goto error;
	}

	/* We only care about the part after the comma */
	p = strchr(compat, ',');
	strcpy(soc_card_data->codec_name, p ? p + 1 : compat);

	/* Now determine the I2C bus and address of the codec */
	bus = of_get_integer(of_get_parent(codec_np), "cell-index");
	if (bus < 0) {
		dev_err(soc_card->dev,
			"cannot determine I2C bus number for %s\n",
			codec_np->full_name);
		goto error;
	}

	address = of_get_integer(codec_np, "reg");
	if (address < 0) {
		dev_err(soc_card->dev,
			"cannot determine I2C address for %s\n",
			codec_np->full_name);
		goto error;
	}

	soc_card_data->pcm_config.codec_num = bus << 16 | address;

	ret = snd_soc_card_create_pcms(soc_card, &soc_card_data->pcm_config, 1);
	if (ret) {
		dev_err(soc_card->dev, "could not create PCMs\n");
		return ret;
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
		.name   	= "MPC8610HPCD",
		.owner		= THIS_MODULE,
	},
	.probe  	= mpc8610_hpcd_probe,
	.remove 	= __devexit_p(mpc8610_hpcd_remove),
};

/* Platform device pointers for each SSI  */
static struct platform_device *pdev[MAX_SSI];

/**
 * mpc8610_hpcd_init: fabric driver initialization.
 *
 * This function is called when this module is loaded.
 */
static int __init mpc8610_hpcd_init(void)
{
	struct resource res;
	struct device_node *ssi_np;
	unsigned int i = 0;
	int ret;

	pr_info("Freescale MPC8610 HPCD ASoC fabric driver\n");

	ret = platform_driver_register(&mpc8610_hpcd_driver);
	if (ret < 0) {
		pr_err("mpc8610-hpcd: could not register platform\n");
		return ret;
	}

	memset(pdev, 0, sizeof(pdev));

	memset(&res, 0, sizeof(res));
	res.name = "ssi";

	for_each_compatible_node(ssi_np, NULL, "fsl,mpc8610-ssi") {
		int id;

		id = of_get_integer(ssi_np, "cell-index");
		if (id < 0) {
			pr_err("mpc8610-hpcd: no cell-index for %s\n",
				ssi_np->full_name);
			continue;
		}
		if (!find_codec(ssi_np))
			/* No codec node? Skip it */
			continue;

		res.start = (resource_size_t) ssi_np;

		pdev[i] = platform_device_register_simple(
			mpc8610_hpcd_driver.driver.name, id, &res, 1);
		if (!pdev[i]) {
			pr_err("mpc8610-hpcd: could not register %s\n",
				ssi_np->full_name);
			continue;
		}

		i++;
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
	unsigned int i;

	for (i = 0; i < MAX_SSI; i++)
		platform_device_unregister(pdev[i]);

	platform_driver_unregister(&mpc8610_hpcd_driver);
}

module_init(mpc8610_hpcd_init);
module_exit(mpc8610_hpcd_exit);

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Freescale MPC8610 HPCD ASoC fabric driver");
MODULE_LICENSE("GPL");
