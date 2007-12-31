/*
 * linux/sound/soc-machine.h -- ALSA SoC Layer
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_MACHINE_H
#define __LINUX_SND_SOC_MACHINE_H

#include <linux/types.h>
#include <linux/workqueue.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>

struct snd_soc_pcm_runtime;
struct snd_soc_machine;

struct snd_soc_ops {
		/* ALSA audio operations - optional */
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*hw_params)(struct snd_pcm_substream *, 
		struct snd_pcm_hw_params *);
	int (*hw_free)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
	int (*trigger)(struct snd_pcm_substream *, int);
};

/*
 * PCM runtime information.
 * 
 * Holds information on every ASoC PCM i.e. valid Digital Audio Interface with
 * two endpoints (e.g codec and platform).
 */
struct snd_soc_pcm_runtime {
	char *name;
	
	/* runtime status */
	unsigned int playback_active:1;
	unsigned int capture_active:1;
	unsigned int pop_wait:1;
	
	/* runtime devices */
	struct snd_soc_codec *codec;
	struct snd_soc_dai_runtime *cpu_dai;
	struct snd_soc_dai_runtime *codec_dai;
	struct snd_soc_platform *platform;
	struct snd_soc_machine *machine;

	/* ALSA audio operations - optional */
	struct snd_soc_ops *ops;
	
	/* DAI pcm */
	struct snd_pcm *pcm;
	
	struct delayed_work delayed_work;
	struct list_head list;
	
	void *private_data;
};


/* SoC machine */
struct snd_soc_machine {
	const char *name;
	const char *longname;

	struct device *dev;
	struct mutex mutex;
	struct snd_card *card;
	int pcms;
	
	/* io based probe / remove */
	int (*init)(struct snd_soc_machine *machine);
	int (*exit)(struct snd_soc_machine *machine);
	
	/* dapm events */
	int (*dapm_event)(struct snd_soc_machine *machine, int event);

	/* list of clients */
	struct list_head codec_list;
	struct list_head platform_list;
	struct list_head pcm_list;
	
	/* dapm */
	struct list_head dapm_widgets;
	struct list_head dapm_paths;
	unsigned int dapm_state;
	
	void *private_data;
};

struct snd_soc_machine *snd_soc_machine_create(const char *name,
	struct device *parent, int idx, const char *xid);

int snd_soc_pcm_create(struct snd_soc_machine *machine,
	struct snd_soc_ops *pcm_ops, int codec_dai_id, 
	int platform_dai_id, int playback, int capture);

int snd_soc_machine_register(struct snd_soc_machine *machine);

void snd_soc_machine_free(struct snd_soc_machine *machine);

static inline struct snd_soc_codec *
	snd_soc_get_codec(struct snd_soc_machine *machine, const char *codec_id)
{
	struct snd_soc_codec *codec;
	
	list_for_each_entry(codec, &machine->codec_list, list) {
		if (!strcmp(codec->name, codec_id))
			return codec;
	}
	return NULL;
}

static inline struct snd_soc_platform *
	snd_soc_get_platform(struct snd_soc_machine *machine, 
	const char *platform_id)
{
	struct snd_soc_platform *platform;
	
	list_for_each_entry(platform, &machine->platform_list, list) {
		if (!strcmp(platform->name, platform_id))
			return platform;
	}
	return NULL;
}

static inline struct snd_ac97_bus_ops *
	snd_soc_get_ac97_ops(struct snd_soc_machine *machine, int dai_id)
{
	struct snd_soc_platform *platform;
	struct snd_soc_dai_runtime *rdai;
	
	list_for_each_entry(platform, &machine->platform_list, list) {
		list_for_each_entry(rdai, &platform->dai_list, list) {
			if (rdai->dai->id == dai_id)
				return rdai->dai->ac97_ops;
		}
	}
	return NULL;
}

/* suspend and resume */
int snd_soc_suspend(struct snd_soc_machine *machine, pm_message_t state);
int snd_soc_resume(struct snd_soc_machine *machine);


#endif /*SOCMACHINE_H_*/
