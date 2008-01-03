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
struct snd_soc_ops;

/**
 * snd_soc_machine_create - create new ASoC machine.
 * @name: machine name
 * @parent: parent device
 * @idx: sound card index
 * @xid: sound card ID
 *
 * Creates a new ASoC audio machine device and sound card.
 */
struct snd_soc_machine *snd_soc_machine_create(const char *name,
	struct device *parent, int idx, const char *xid);

/**
 * snd_soc_platform_create - create new ASoC platform.
 * @machine: parent machine
 * @platform_id: platform ID name
 *
 * Creates a new ASoC audio platform device and attaches it to parent machine.
 */
int snd_soc_platform_create(struct snd_soc_machine *machine,
	const char *platform_id);

/**
 * snd_soc_codec_create - create new ASoC codec.
 * @machine: parent machine
 * @codec_id: platform ID name
 *
 * Creates a new ASoC audio codec device and attaches it to parent machine.
 */
int snd_soc_codec_create(struct snd_soc_machine *machine,
	const char *codec_id);

/**
 * snd_soc_pcm_create - create new ASoC PCM.
 * @machine: Machine
 * @pcm_ops: PCM operations.
 * @codec_dai_id: Codec DAI ID.
 * @platform_dai_id: Platform DAI ID.
 * @playback: Number of playback PCM's.
 * @capture: Number of capture PCM's
 *
 * Joins a codec and platform DAI together and creates a ALSA PCM(s).
 */
int snd_soc_pcm_create(struct snd_soc_machine *machine,
	struct snd_soc_ops *pcm_ops, int codec_dai_id, 
	int platform_dai_id, int playback, int capture);

/**
 * snd_soc_machine_register - registers ASoC machine .
 * @machine: machine
 *
 * Registers a machine and it's PCMs. This should be called after all
 * codecs, platforms and PCM's have been created.
 */
int snd_soc_machine_register(struct snd_soc_machine *machine);

/**
 * snd_soc_machine_free - free machine.
 * @machine: machine
 *
 * Frees all machine resources. Can be called at any time during machine
 * initialisation process.
 */
void snd_soc_machine_free(struct snd_soc_machine *machine);

/**
 * snd_soc_suspend - suspends core.
 * @machine: machine
 * @state: suspend state
 * 
 * Suspends the ASoc core and driver.
 */
int snd_soc_suspend(struct snd_soc_machine *machine, pm_message_t state);

/**
 * snd_soc_resume - resume core.
 * @machine: machine
 *
 * Resumes the ASoC core after suspend().
 */
int snd_soc_resume(struct snd_soc_machine *machine);

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


/* 
 * ASoC machine
 * 
 * The ASoC sound card. This glues the codecs to the platforms and provides
 * machine specific operations per PCM along with any othe machine specifics.
 */
struct snd_soc_machine {
	const char *name;
	const char *longname;

	/* Runtime */
	struct device *dev;
	struct mutex mutex;
	struct snd_card *card;
	int pcms;
	
	/* init() and exit() - init is called by snd_soc_machine_register()
	 * whilst exit by snd_soc_machine_free() */
	int (*init)(struct snd_soc_machine *machine);
	int (*exit)(struct snd_soc_machine *machine);
	
	/* bias power level */
	int (*set_bias_level)(struct snd_soc_machine *machine, 
		enum snd_soc_dapm_bias_level level);
	enum snd_soc_dapm_bias_level bias_level;
	
	/* lists of components */
	struct list_head codec_list;
	struct list_head platform_list;
	struct list_head pcm_list;
	
	/* DAPM */
	struct list_head dapm_widgets;
	struct list_head dapm_paths;
	
	void *private_data;
};

/**
 * snd_soc_get_codec - get codec.
 * @machine: machine
 * @codec_id: codec ID
 *
 * Gets codec from ID.
 */
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

/**
 * snd_soc_get_codec - get platform.
 * @machine: machine
 * @codec_id: platform ID
 *
 * Gets platform from ID.
 */
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

/**
 * snd_soc_get_ac97_ops - free machine.
 * @machine: machine
 * @dai_id: DAI ID.
 *
 * Gets AC97 bus operations for DAI.
 */
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

#endif
