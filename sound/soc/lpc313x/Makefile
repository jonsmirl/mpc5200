# LPC313X Platform Support
snd-soc-lpc313x-objs := lpc313x-pcm.o
snd-soc-lpc313x-i2s-objs := lpc313x-i2s.o

obj-$(CONFIG_SND_LPC313X_SOC) += snd-soc-lpc313x.o
obj-$(CONFIG_SND_LPC315X_SOC) += snd-soc-lpc313x.o
obj-$(CONFIG_SND_LPC313X_SOC_I2S) += snd-soc-lpc313x-i2s.o lpc313x-i2s-clocking.o

# LPC3XXX Machine Support
snd-soc-lpc313x-uda1380-objs := lpc313x-uda1380.o

obj-$(CONFIG_SND_LPC313X_SOC_I2S_UDA1380) += snd-soc-lpc313x-uda1380.o

# LPC315X Machine Support
snd-soc-lpc315x-codec-objs := lpc315x-codec.o

obj-$(CONFIG_SND_LPC315X_SOC_I2S_LPC315X_CODEC) += snd-soc-lpc315x-codec.o

