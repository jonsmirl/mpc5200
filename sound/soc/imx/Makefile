# i.MX Platform Support
snd-soc-imx21-objs := imx21-pcm.o
snd-soc-imx31-objs := imx31-pcm.o

ifneq ($(CONFIG_SND_MXC_SOC_SSI),)
  snd-soc-imx31-objs += imx-ssi.o
endif

ifneq ($(CONFIG_SND_MXC_SOC_AC97),)
  snd-soc-imx31-objs += imx-ac97.o
endif

obj-$(CONFIG_SND_MXC_SOC) += snd-soc-imx31.o

# i.MX Machine Support
snd-soc-imx32ads-wm8350-objs := imx32ads-wm8350.o
obj-$(CONFIG_SND_SOC_IMX32ADS_WM8350) += snd-soc-imx32ads-wm8350.o
