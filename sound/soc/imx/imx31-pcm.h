/*
 * mxc-pcm.h :- ASoC platform header for Freescale i.MX
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MXC_PCM_H
#define _MXC_PCM_H

#include <asm/arch/dma.h>

/* AUDMUX regs definition - MOVE to asm/arch when stable */
#define AUDMUX_IO_BASE_ADDR	IO_ADDRESS(AUDMUX_BASE_ADDR)

#define DAM_PTCR1	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x00)))
#define DAM_PDCR1	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x04)))
#define DAM_PTCR2	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x08)))
#define DAM_PDCR2	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x0C)))
#define DAM_PTCR3	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x10)))
#define DAM_PDCR3	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x14)))
#define DAM_PTCR4	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x18)))
#define DAM_PDCR4	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x1C)))
#define DAM_PTCR5	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x20)))
#define DAM_PDCR5	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x24)))
#define DAM_PTCR6	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x28)))
#define DAM_PDCR6	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x2C)))
#define DAM_PTCR7	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x30)))
#define DAM_PDCR7	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x34)))
#define DAM_CNMCR	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 0x38)))
#define DAM_PTCR(a)	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + a*8)))
#define DAM_PDCR(a)	(*((volatile u32 *)(AUDMUX_IO_BASE_ADDR + 4 + a*8)))


#define AUDMUX_PTCR_TFSDIR		(1 << 31)
#define AUDMUX_PTCR_TFSSEL(x, y)	((x << 30) | (((y - 1) & 0x7) << 27))
#define AUDMUX_PTCR_TCLKDIR		(1 << 26)
#define AUDMUX_PTCR_TCSEL(x, y)		((x << 25) | (((y - 1) & 0x7) << 22))
#define AUDMUX_PTCR_RFSDIR		(1 << 21)
#define AUDMUX_PTCR_RFSSEL(x, y)	((x << 20) | (((y - 1) & 0x7) << 17))
#define AUDMUX_PTCR_RCLKDIR		(1 << 16)
#define AUDMUX_PTCR_RCSEL(x, y)		((x << 15) | (((y - 1) & 0x7) << 12))
#define AUDMUX_PTCR_SYN			(1 << 11)

#define AUDMUX_FROM_TXFS		0
#define AUDMUX_FROM_RXFS		1

#define AUDMUX_PDCR_RXDSEL(x)		(((x - 1) & 0x7) << 13)
#define AUDMUX_PDCR_TXDXEN		(1 << 12)
#define AUDMUX_PDCR_MODE(x)		(((x) & 0x3) << 8)
#define AUDMUX_PDCR_INNMASK(x)		(((x) & 0xff) << 0)

#define AUDMUX_CNMCR_CEN		(1 << 18)
#define AUDMUX_CNMCR_FSPOL		(1 << 17)
#define AUDMUX_CNMCR_CLKPOL		(1 << 16)
#define AUDMUX_CNMCR_CNTHI(x)		(((x) & 0xff) << 8)
#define AUDMUX_CNMCR_CNTLOW(x)		(((x) & 0xff) << 0)

/* i.MX DAI SSP ID's */
#define IMX_DAI_SSI0			0 /* SSI1 FIFO 0 */
#define IMX_DAI_SSI1			1 /* SSI1 FIFO 1 */
#define IMX_DAI_SSI2			2 /* SSI2 FIFO 0 */
#define IMX_DAI_SSI3			3 /* SSI2 FIFO 1 */

/* SSI clock sources */
#define IMX_SSP_SYS_CLK		0

/* SSI audio dividers */
#define IMX_SSI_TX_DIV_2			0
#define IMX_SSI_TX_DIV_PSR			1
#define IMX_SSI_TX_DIV_PM			2
#define IMX_SSI_RX_DIV_2			3
#define IMX_SSI_RX_DIV_PSR			4
#define IMX_SSI_RX_DIV_PM			5

/* SSI Div 2 */
#define IMX_SSI_DIV_2_OFF		~SSI_STCCR_DIV2
#define IMX_SSI_DIV_2_ON		SSI_STCCR_DIV2

int get_ssi_clk(int ssi, struct device *dev);
void put_ssi_clk(int ssi);

struct mxc_pcm_dma_params {
	char *name;			/* stream identifier */
	dma_channel_params params;
};

extern const char imx31_platform_id[];
extern const char imx_ssi_id1_0[];
extern const char imx_ssi_id1_1[];
extern const char imx_ssi_id2_0[];
extern const char imx_ssi_id2_1[];

#endif
