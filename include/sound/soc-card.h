/*
 * linux/sound/soc-card.h -- ALSA SoC Layer
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

/**
 * snd_soc_card_create - create new ASoC soc_card.
 * @name: soc_card name
 * @parent: parent device
 * @idx: sound card index
 * @xid: sound card ID
 *
 * Creates a new ASoC audio soc_card device and sound card.
 */
struct snd_soc_card *snd_soc_card_create(const char *name,
	struct device *parent, int idx, const char *xid);


/**
 * snd_soc_pcm_create - create new ASoC PCM.
 * @soc_card: Machine
 *
 * Joins a codec and platform DAI together and creates a ALSA PCM(s).
 */
int snd_soc_pcm_create(struct snd_soc_card *soc_card,
	struct snd_soc_pcm_config *config);

/**
 * snd_soc_create_pcms - create several ASoC PCM.
 * @soc_card: Machine
 * @configs: Array of PCM configurations
 * @num:     Size of array
 *
 * Joins a liscodec and platform DAI together and creates a ALSA PCM(s).
 */
int snd_soc_create_pcms(struct snd_soc_card *soc_card,
			struct snd_soc_pcm_config *config, int num);

/**
 * snd_soc_card_register - registers ASoC soc_card .
 * @soc_card: soc_card
 *
 * Registers a soc_card and it's PCMs. This should be called after all
 * codecs, platforms and PCM's have been created.
 */
int snd_soc_card_register(struct snd_soc_card *soc_card);

/**
 * snd_soc_card_free - free soc_card.
 * @soc_card: soc_card
 *
 * Frees all soc_card resources. Can be called at any time during soc_card
 * initialisation process.
 */
void snd_soc_card_free(struct snd_soc_card *soc_card);

/**
 * snd_soc_suspend_pcms - suspends soc_card pcms.
 * @soc_card: soc_card
 * @state: suspend state
 *
 * Suspends all soc_card pcms.
 */
int snd_soc_suspend_pcms(struct snd_soc_card *soc_card, pm_message_t state);

/**
 * snd_soc_resume_pcms - resume soc_card pcms.
 * @soc_card: soc_card
 *
 * Resumes soc_card pcms.
 */
int snd_soc_resume_pcms(struct snd_soc_card *soc_card);

/**
 * snd_soc_get_pcm - get pcm.
 * @soc_card: soc_card
 * @pcm_id: pcm ID
 *
 * Gets pcm from ID.
 */
struct snd_soc_pcm_runtime *snd_soc_get_pcm(struct snd_soc_card *soc_card,
	const char *pcm_id);

/**
 * snd_soc_get_ac97_ops - get AC97 operations.
 * @soc_card: soc_card
 * @dai_id:  ID
 *
 * Gets AC97 operations from Digital Audio Interface.
 */
struct snd_ac97_bus_ops *snd_soc_get_ac97_ops(struct snd_soc_card *soc_card,
					      const char *dai_id);

/**
 * snd_soc_get_codec - get codec.
 * @soc_card: soc_card
 * @codec_id: codec ID
 *
 * Gets codec from ID.
 */
struct snd_soc_codec *snd_soc_get_codec(struct snd_soc_card *soc_card,
	const char *codec_id);

/**
 * snd_soc_get_platform - get platform.
 * @soc_card: soc_card
 * @codec_id: platform ID
 *
 * Gets platform from ID.
 */
struct snd_soc_platform * snd_soc_get_platform(struct snd_soc_card *soc_card,
	const char *platform_id);

/**
 * snd_soc_get_dai - get dai.
 * @soc_card: soc_card
 * @codec_id: dai ID
 *
 * Gets dai from ID.
 */
struct snd_soc_dai * snd_soc_get_dai(struct snd_soc_card *soc_card,
	const char *dai_id);

/**
 * snd_soc_codec_set_io - initialise codec IO.
 * @codec: codec
 * @soc_card_read: read function called by codec.
 * @soc_card_write: write function called by codec.
 * @control_data: IO control data - usually I2C, SPI, etc pointer
 *
 * Initialises the codec IO system with the soc_cards codec IO mechanism. 
 */
void snd_soc_codec_set_io(struct snd_soc_codec *codec,
	int (*soc_card_read)(void *, long, int),
	int (*soc_card_write)(void *, long, int), void *control_data);

/**
 * snd_soc_codec_init - initialises codec
 * @codec: codec
 * @soc_card: soc_card
 *
 * Initialises codec hardware. Can perform IO and must only be called after a
 * successful call to snd_soc_codec_set_io(). 
 */
int snd_soc_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card);

/**
 * snd_soc_codec_exit - shutsdown codec hw
 * @codec: codec
 * @soc_card: soc_card
 *
 * Shutsdown codec hardware.
 */
void snd_soc_codec_exit(struct snd_soc_codec *codec,
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
