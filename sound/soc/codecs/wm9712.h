/*
 * wm9712.h  --  WM9712 Soc Audio driver
 */

#ifndef _WM9712_H
#define _WM9712_H

#define WM9712_DAI_AC97_HIFI	0
#define WM9712_DAI_AC97_AUX		1

extern const char wm9712_hifi_dai[SND_SOC_DAI_NAME_SIZE];
extern const char wm9712_aux_dai[SND_SOC_DAI_NAME_SIZE];
extern const char wm9712_codec[SND_SOC_CODEC_NAME_SIZE];

#endif
