#ifndef SOCPRV_H_
#define SOCPRV_H_

/* dapm events signalling */
int snd_soc_dapm_stream_event(struct snd_soc_machine *machine, char *stream,
	enum snd_soc_dapm_stream_event event);
int snd_soc_dapm_set_bias(struct snd_soc_pcm_runtime *pcm_runtime, 
	enum snd_soc_dapm_bias_level level);

/* dapm sys fs - used by the core */
int snd_soc_dapm_sys_add(struct device *dev);

#endif /*SOCPRV_H_*/
