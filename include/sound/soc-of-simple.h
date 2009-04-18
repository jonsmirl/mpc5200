/*
 * OF helpers for ALSA SoC
 *
 * Copyright (C) 2008, Secret Lab Technologies Ltd.
 */

#ifndef _INCLUDE_SOC_OF_H_
#define _INCLUDE_SOC_OF_H_

#if defined(CONFIG_SND_SOC_OF_SIMPLE) || defined(CONFIG_SND_SOC_OF_SIMPLE_MODULE)

#include <linux/of.h>
#include <sound/soc.h>

#define SOC_OF_SIMPLE_MAX_DAI 2

int of_snd_soc_register_codec(struct snd_soc_codec_device *codec_dev,
			      void *codec_data, struct snd_soc_dai *dai,
			      int count, struct device_node *node);

int of_snd_soc_register_cpu_dai(struct device_node *node,
				 struct snd_soc_dai *cpu_dai, int count);

int of_snd_soc_register_platform(struct snd_soc_platform *platform);

int of_snd_soc_register_fabric(char *name, struct snd_soc_ops *ops,
								int (*init)(struct snd_soc_codec *codec));

#endif

#endif /* _INCLUDE_SOC_OF_H_ */
