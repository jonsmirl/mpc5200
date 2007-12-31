# PXA Platform Support
snd-soc-pxa2xx-objs := pxa2xx-pcm.o

ifneq ($(CONFIG_SND_PXA2XX_SOC_I2S),)
  snd-soc-pxa2xx-objs += pxa2xx-i2s.o
endif

ifneq ($(CONFIG_SND_PXA2XX_SOC_AC97),)
  snd-soc-pxa2xx-objs += pxa2xx-ac97.o
endif

ifneq ($(CONFIG_SND_PXA2XX_SOC_SSP),)
  snd-soc-pxa2xx-objs += pxa2xx-ssp.o
endif

obj-$(CONFIG_SND_PXA2XX_SOC) += snd-soc-pxa2xx.o

# PXA Machine Support
snd-soc-corgi-objs := corgi.o
snd-soc-poodle-objs := poodle.o
snd-soc-tosa-objs := tosa.o
snd-soc-spitz-objs := spitz.o
snd-soc-amesom-tlv320-objs := amesom_tlv320.o
snd-soc-magician-objs := magician.o
snd-soc-h5000-objs := h5000.o
snd-soc-mainstone-wm8753-objs := mainstone_wm8753.o
snd-soc-mainstone-wm9713-objs := mainstone_wm9713.o
snd-soc-mainstone-wm9712-objs := mainstone_wm9712.o

obj-$(CONFIG_SND_PXA2XX_SOC_CORGI) += snd-soc-corgi.o
obj-$(CONFIG_SND_PXA2XX_SOC_POODLE) += snd-soc-poodle.o
obj-$(CONFIG_SND_PXA2XX_SOC_TOSA) += snd-soc-tosa.o
obj-$(CONFIG_SND_PXA2XX_SOC_SPITZ) += snd-soc-spitz.o
obj-$(CONFIG_SND_PXA2XX_SOC_AMESOM_TLV320) += snd-soc-amesom-tlv320.o
obj-$(CONFIG_SND_PXA2XX_SOC_MAGICIAN) += snd-soc-magician.o
obj-$(CONFIG_SND_PXA2XX_SOC_H5000) += snd-soc-h5000.o
obj-$(CONFIG_SND_PXA2XX_SOC_MAINSTONE_WM8753) += snd-soc-mainstone-wm8753.o
obj-$(CONFIG_SND_PXA2XX_SOC_MAINSTONE_WM9713) += snd-soc-mainstone-wm9713.o
obj-$(CONFIG_SND_PXA2XX_SOC_MAINSTONE_WM9712) += snd-soc-mainstone-wm9712.o
