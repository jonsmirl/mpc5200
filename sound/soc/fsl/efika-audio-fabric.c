/*
 * Efika driver for the PSC of the Freescale MPC52xx configured as AC97 interface
 *
 * Copyright 2008 Jon Smirl, Digispeaker
 * Author: Jon Smirl <jonsmirl@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-of-simple.h>

#include "mpc5200_dma.h"
#include "mpc5200_psc_ac97.h"
#include "../codecs/stac9766.h"

static int efika_init(struct snd_soc_codec *codec)
{
	/* Skeleton driver showing framework for setting
	 * up board specific fabric drivers.
	 *
	 * set up Efika specific controls here
	 *
	 * Loading of this driver is trigger by
	 * platform_device_register_simple("efika-audio-fabric", 0, NULL, 0);
	 * in arch/powerpc/platforms/52xx/efika.c
	 */
	return 0;
}

static int efika_stac9766_probe(struct platform_device *pdev)
{
	of_snd_soc_register_fabric("Efika", NULL, efika_init);
	return 0;
}

#ifdef CONFIG_PM

static int efika_stac9766_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	return 0;
}

static int efika_stac9766_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define efika_stac9766_suspend NULL
#define efika_stac9766_resume  NULL
#endif

static struct platform_driver efika_fabric = {
	.probe	= efika_stac9766_probe,
	.suspend = efika_stac9766_suspend,
 	.resume = efika_stac9766_resume,
	.driver	= {
		.name	= "efika-audio-fabric",
	},
};

static __init int efika_fabric_init(void)
{
	return platform_driver_register(&efika_fabric);
}

static __exit void efika_fabric_exit(void)
{
}

module_init(efika_fabric_init);
module_exit(efika_fabric_exit);


MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_DESCRIPTION(DRV_NAME ": mpc5200 Efika fabric driver");
MODULE_LICENSE("GPL");

