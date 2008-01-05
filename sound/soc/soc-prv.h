#ifndef SOCPRV_H_
#define SOCPRV_H_

/* dapm events signalling */
int snd_soc_dapm_stream_event(struct snd_soc_machine *machine, char *stream,
	enum snd_soc_dapm_stream_event event);
int snd_soc_dapm_set_bias(struct snd_soc_pcm_runtime *pcm_runtime, 
	enum snd_soc_dapm_bias_level level);

/* dapm sys fs - used by the core */
int snd_soc_dapm_sys_add(struct snd_soc_machine *machine);

/* codec register bit access */
int snd_soc_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);
int snd_soc_test_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);

#endif /*SOCPRV_H_*/
