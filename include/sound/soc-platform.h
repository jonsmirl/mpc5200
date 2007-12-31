/*
 * linux/sound/soc-platform.h -- ALSA SoC Layer
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_PLATFORM_H
#define __LINUX_SND_SOC_PLATFORM_H

#include <linux/types.h>
#include <linux/workqueue.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>

struct snd_soc_platform;
struct snd_soc_machine;
struct snd_soc_dai;

/* SoC platform interface */
struct snd_soc_platform {
	struct device dev;
	char *name;
	
	struct mutex mutex;
	struct list_head list;
	struct list_head dai_list;
	struct snd_soc_machine *machine;
	
	/* platform ops */
	const struct snd_pcm_ops *pcm_ops;
	
	/* pcm creation and destruction */
	int (*pcm_new)(struct snd_soc_platform *platform, 
		struct snd_card *card, int playback, int capture, 
		struct snd_pcm *pcm);
	void (*pcm_free)(struct snd_pcm *pcm);
	
	void *private_data;
};
#define to_snd_soc_platform(d) \
	container_of(d, struct snd_soc_platform, dev)

/**
 * snd_soc_platform_create - create new ASoC platform.
 * @machine: parent machine
 * @platform_id: platform ID name
 *
 * Creates a new ASoC audio platform device and attaches to parent machine.
 */
int snd_soc_platform_create(struct snd_soc_machine *machine,
	const char *platform_id);

/**
 * snd_soc_platform_add_dai - add DAI to platform.
 * @platform: platform
 * @dai: pointer to DAI
 * @num: number of DAI to add
 *
 * Adds <num> Digital Audio Interfaces to platform.
 */
int snd_soc_platform_add_dai(struct snd_soc_platform *platform, 
	struct snd_soc_dai *dai, int num);

int snd_soc_register_platform(struct snd_soc_platform *platform);
void snd_soc_unregister_platform(struct snd_soc_platform *platform);

/* set runtime hw params */
int snd_soc_set_runtime_hwparams(struct snd_pcm_substream *substream,
	const struct snd_pcm_hardware *hw);

#endif /*SOCPLATFORM_H_*/
