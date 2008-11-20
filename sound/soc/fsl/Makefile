# Simple machine driver that extracts configuration from the OF device tree
obj-$(CONFIG_SND_SOC_OF_SIMPLE) += soc-of-simple.o

# MPC8610 HPCD Machine Support
obj-$(CONFIG_SND_SOC_MPC8610_HPCD) += mpc8610_hpcd.o

# MPC8610 Platform Support
obj-$(CONFIG_SND_SOC_MPC8610) += fsl_ssi.o fsl_dma.o

# MPC5200 Platform Support
snd_soc_mpc52xx_ac97_objs += mpc52xx_psc_ac97.o mpc52xx_dma.o

obj-$(CONFIG_SND_SOC_MPC5200_I2S) += mpc5200_psc_i2s.o
obj-$(CONFIG_SND_MPC52xx_SOC_AC97) += snd_soc_mpc52xx_ac97.o

# MPC5200 Machine Support
snd_soc_efika_stac9766-objs := efika_stac9766.o
snd_soc_dspeak01_fabric-objs := dspeak01_fabric.o

obj-$(CONFIG_SND_MPC52xx_SOC_EFIKA) += snd_soc_efika_stac9766.o
obj-$(CONFIG_SND_MPC52xx_SOC_DSPEAK01) += snd_soc_dspeak01_fabric.o
