/*
 * linux/sound/soc-card.h -- ALSA SoC Card interface
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

struct snd_soc_pcm_runtime;
struct snd_soc_card;
struct snd_soc_ops;
struct snd_soc_pcm_config;

/* soc card registration */
int snd_soc_card_register(struct snd_soc_card *soc_card);
void snd_soc_card_free(struct snd_soc_card *soc_card);
struct snd_soc_card *snd_soc_card_create(const char *name,
	struct device *parent, int idx, const char *xid);

/* pcm creation */
int snd_soc_card_create_pcms(struct snd_soc_card *soc_card,
			struct snd_soc_pcm_config *config, int num);

/* suspend and resume */
int snd_soc_card_suspend_pcms(struct snd_soc_card *soc_card,
	pm_message_t state);
int snd_soc_card_resume_pcms(struct snd_soc_card *soc_card);

/* get codec, dai, platform from ID */
struct snd_soc_pcm_runtime *snd_soc_card_get_pcm(struct snd_soc_card *soc_card,
	const char *pcm_id);

struct snd_ac97_bus_ops *snd_soc_card_get_ac97_ops(
	struct snd_soc_card *soc_card, const char *dai_id);

struct snd_soc_codec *snd_soc_card_get_codec(struct snd_soc_card *soc_card,
	const char *codec_name, int codec_num);

struct snd_soc_platform * snd_soc_card_get_platform(
	struct snd_soc_card *soc_card, const char *platform_id);

struct snd_soc_dai * snd_soc_card_get_dai(struct snd_soc_card *soc_card,
	const char *dai_id);

/* codec initialisation */
void snd_soc_card_config_codec(struct snd_soc_codec *codec,
	unsigned int (*soc_phys_read)(void *, int),
	int (*soc_phys_write)(void *, long, int), void *control_data);

int snd_soc_card_init_codec(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card);

/*
 * PCM configuration information.
 *
 * Defines every ASoC PCM in soc_card driver i.e. valid Digital Audio Interface
 * with two endpoints (e.g codec and platform).
 */
struct snd_soc_pcm_config {
	const char *name;		/* pcm name */
	const char *codec;		/* codec identification */
	const char *codec_dai;		/* codec dai identification */
	int codec_num;			/* codec number (if > 1 same codec) */
	const char *platform;		/* platform identification */
	const char *cpu_dai;		/* platform dai identification */
	int playback;			/* alsa playback pcms */
	int capture;			/* alsa capture alsa pcms */
	struct snd_soc_ops *ops;	/* soc_card pcm ops */
};

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
	const char *name;

	/* runtime status */
	unsigned int playback_active:1;
	unsigned int capture_active:1;
	unsigned int pop_wait:1;

	/* runtime devices */
	struct snd_soc_codec *codec;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_platform *platform;
	struct snd_soc_card *soc_card;

	/* ALSA audio operations - optional */
	struct snd_soc_ops *ops;

	/* DAI pcm */
	struct snd_pcm *pcm;

	struct delayed_work delayed_work;
	struct list_head list;

	void *private_data;
};


/*
 * ASoC soc_card
 *
 * The ASoC sound card. This glues the codecs to the platforms and provides
 * soc_card specific operations per PCM along with any othe soc_card specifics.
 */
struct snd_soc_card {
	const char *name;
	const char *longname;

	/* Runtime */
	struct device *dev;
	struct mutex mutex;
	struct snd_card *card;
	int pcms;
	int is_registered;
	int is_probed;

	/* init() and exit() - init is called by snd_soc_card_register()
	 * whilst exit by snd_soc_card_free() */
	int (*init)(struct snd_soc_card *soc_card);
	void (*exit)(struct snd_soc_card *soc_card);

	/* bias power level */
	int (*set_bias_level)(struct snd_soc_card *soc_card,
		enum snd_soc_dapm_bias_level level);
	enum snd_soc_dapm_bias_level bias_level;

	/* lists of runtime components */
	struct list_head pcm_list;
	struct list_head config_list;
	struct list_head list;

	/* DAPM */
	struct list_head dapm_widgets;
	struct list_head dapm_paths;
	enum snd_soc_dapm_policy policy;

	void *private_data;
};

#endif
