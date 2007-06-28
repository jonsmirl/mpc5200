/*
 * linux/sound/soc/pxa/pxa2xx-ac97.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PXA2XX_AC97_H
#define _PXA2XX_AC97_H

/* pxa2xx DAI ID's */
#define PXA2XX_DAI_AC97_HIFI	0
#define PXA2XX_DAI_AC97_AUX		1
#define PXA2XX_DAI_AC97_MIC		2

extern const char pxa2xx_ac97_hifi[SND_SOC_DAI_NAME_SIZE];
extern const char pxa2xx_ac97_aux[SND_SOC_DAI_NAME_SIZE];
extern const char pxa2xx_ac97_mic[SND_SOC_DAI_NAME_SIZE];

#endif
