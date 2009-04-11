/*
 * Freescale MPC5200 Audio DMA driver
 */

#ifndef __SOUND_SOC_FSL_MPC5200_DMA_H__
#define __SOUND_SOC_FSL_MPC5200_DMA_H__

int mpc5200_dma_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai);
int mpc5200_dma_hw_free(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai);
void mpc5200_dma_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai);
int mpc5200_dma_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai);

extern const struct snd_pcm_hardware mpc5200_pcm_hardware;

#endif /* __SOUND_SOC_FSL_MPC5200_DMA_H__ */
