/*  linux/arch/arm/mach-lpc313x/include/mach/dma.h
 *  
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * DMA register defines & structures for LPC313x and LPC315x SoCs.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

#include <mach/constants.h>

/***********************************************************************
 * DMA register definitions
 **********************************************************************/
#define DMACH_SRC_ADDR(ch)    __REG (DMA_PHYS + ((ch) << 5) + 0x00)
#define DMACH_DST_ADDR(ch)    __REG (DMA_PHYS + ((ch) << 5) + 0x04)
#define DMACH_LEN(ch)         __REG (DMA_PHYS + ((ch) << 5) + 0x08)
#define DMACH_CFG(ch)         __REG (DMA_PHYS + ((ch) << 5) + 0x0C)
#define DMACH_EN(ch)          __REG (DMA_PHYS + ((ch) << 5) + 0x10)
#define DMACH_TCNT(ch)        __REG (DMA_PHYS + ((ch) << 5) + 0x1C)
#define DMACH_ALT_EN          __REG (DMA_PHYS + 0x400)
#define DMACH_IRQ_STATUS      __REG (DMA_PHYS + 0x404)
#define DMACH_IRQ_MASK        __REG (DMA_PHYS + 0x408)
#define DMACH_ALT_PHYS(ch)    (DMA_PHYS + 0x200 + ((ch) << 4))
#define DMACH_SOFT_INT_PHYS   (DMA_PHYS + 0x40C )

/***********************************************************************
* Channel CONFIGURATION register defines
***********************************************************************/
#define DMA_CFG_CIRC_BUF        _BIT(18)
#define DMA_CFG_CMP_CH_EN       _BIT(17)
#define DMA_CFG_CMP_CH_NR(n)    _SBF(13, ((n) & 0x7))
#define DMA_CFG_INV_ENDIAN      _BIT(12)
#define DMA_CFG_CIRC_BUF        _BIT(18)
#define DMA_CFG_TX_WORD         _SBF(10, 0x00)
#define DMA_CFG_TX_HWORD        _SBF(10, 0x01)
#define DMA_CFG_TX_BYTE         _SBF(10, 0x02)
#define DMA_CFG_TX_BURST        _SBF(10, 0x03)
#define DMA_CFG_RD_SLV_NR(n)    _SBF(5, ((n) & 0x1F))
#define DMA_CFG_WR_SLV_NR(n)    _SBF(0, ((n) & 0x1F))

#define DMA_CFG_GET_CMP_CH_NR(n)    (((n) >> 13) & 0x7)
#define DMA_CFG_GET_RD_SLV_NR(n)    (((n) >> 5) & 0x1F)
#define DMA_CFG_GET_WR_SLV_NR(n)    ((n) & 0x1F)

/* bit defines for interrupt status and mask register */
#define DMA_IRQS_SOFT         _BIT(30)
#define DMA_IRQS_ABORT        _BIT(31)

/* DMA hardware constants */
#define DMA_MAX_CHANNELS   12
#define DMA_MAX_TRANSFERS  2047

/*bit defines for configuration register */
#define DMA_COMPANION_ENABLE _BIT()
/* DMA slave number defines */
#define DMA_SLV_PCM_TX     1
#define DMA_SLV_PCM_RX     2
#define DMA_SLV_UART_RX    3
#define DMA_SLV_UART_TX    4
#define DMA_SLV_I2C0       5
#define DMA_SLV_I2C1       6
#define DMA_SLV_I2STX0_L   7
#define DMA_SLV_I2STX0_R   8
#define DMA_SLV_I2STX1_L   9
#define DMA_SLV_I2STX1_R   10
#define DMA_SLV_I2SRX0_L   11
#define DMA_SLV_I2SRX0_R   12
#define DMA_SLV_I2SRX1_L   13
#define DMA_SLV_I2SRX1_R   16
#define DMA_SLV_LCD        17
#define DMA_SLV_SPI_TX     18
#define DMA_SLV_SPI_RX     19
#define DMA_SLV_SDMMC      20
#define DMA_CLV_INVALID    0xFF

/* DMA transfer types */
#define DMA_TRANSFER_WORD       0
#define DMA_TRANSFER_HALF_WORD  1
#define DMA_TRANSFER_BYTE       2
#define DMA_TRANSFER_BURST      3

/*
 * Type of interrupt
 */
typedef enum 
{
	DMA_IRQ_FINISHED = 0,
	DMA_IRQ_HALFWAY,
	DMA_IRQ_SOFTINT,
	DMA_IRQ_DMAABORT
} dma_irq_type_t;
 /* 
 * DMA IRQ channel callback function
 * parameters:
 * 1st parameter - channel number for which an IRQ occured
 * 2nd parameter - what type of interrupt has happened for a channel
 * 3rd parameter - additional data (callback-specific context)
 * 4th parameter - registers structure as passed to interrupt handler
 */
typedef void (*dma_cb_t)(int, dma_irq_type_t, void *);

typedef union __dma_config_t{
	struct {
		unsigned int write_slave_nr:5;
		unsigned int read_slave_nr:5;
		unsigned int transfer_size:2;
		unsigned int invert_endian:1;
		unsigned int companion_channel:3;
		unsigned int rsrvd0:1;
		unsigned int companion_enable:1;
		unsigned int circular_buffer:1;
		unsigned int rsrvd1:12;
	} s;
	u32 value;
} dma_config_t;

/*
 * DMA setup structure
 */
typedef struct dma_setup
{
	/* source address for transfer*/
	u32 src_address;
	/* source address for transfer*/
	u32 dest_address;
	/* toatl transfer length*/
	u32 trans_length;
	/* channel configuration */
	//dma_config_t cfg;
	u32 cfg;
} dma_setup_t;

/*
 * SDMA scatter-gather list structure 
 */
typedef struct dma_sg_ll
{
	dma_setup_t setup;
	u32 next_entry;
} dma_sg_ll_t;


/*
 * API definition
 */


/*
 * Program SDMA channel
 *
 * Function parameters:
 * 1st parameter - channel number, obtained from dma_request_channel()
 * 2nd parameter - ptr to the structure containing setup info for the channel
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_prog_channel (unsigned int, dma_setup_t   *);

/*
 * Request SDMA channel
 *
 * Function parameters:
 * 1st parameter - free-form string identifier of channel. Must be non-NULL
 * 2nd parameter - callback function to be invoked when an interrupt
 *                 occurs for this channel
 * 3rd parameter - additional data (callback-specific context) to be passed
 *                 to the callback function when it's invoked
 *
 * Returns: channel number on success, otherwise (negative) failure 
 */
int dma_request_channel (char *, dma_cb_t cb, void *);

/*
 * Request specific SDMA channel
 *
 * Function parameters:
 * 1th parameter - specific channel number
 * 2nd parameter - free-form string identifier of channel. Must be non-NULL
 * 3rd parameter - callback function to be invoked when an interrupt
 *                 occurs for this channel
 * 4th parameter - additional data (callback-specific context) to be passed
 *                 to the callback function when it's invoked
 *
 *
 * Returns: channel number on success, otherwise (negative) failure 
 */
int dma_request_specific_channel (int, char *, dma_cb_t cb, void *);


/*
 * Mask/unmask interrupts for the specified channel
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 * 2nd parameter - 0 to unmask halfway-done interrupt, 1 to mask it
 * 3rd parameter - 0 to unmask on-finish interrupt, 1 to mask it
 *
 * Returns: 0 on success, otherwise failure
 */
int dma_set_irq_mask(unsigned int, int, int);

/*
 * Start SDMA channel
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_start_channel (unsigned int);

/*
 * Stop SDMA channel
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_stop_channel (unsigned int);

/*
 * Release SDMA channel
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_release_channel (unsigned int);

/*
 * Read channel counter
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 * 2nd parameter - ptr to the counter variable to be filled
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_read_counter (unsigned int, unsigned int *);

/*
 * Write channel counter
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 * 2nd parameter - value to be written
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_write_counter (unsigned int, u32);

/*
 * Read current channel state
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 * 2nd parameter - ptr to the source address variable to be filled
 * 3rd parameter - ptr to the destination address variable to be filled
 * 4th parameter - ptr to the transfer length variable to be filled
 * 5th parameter - ptr to the configuration variable to be filled
 * 6th parameter - ptr to the enable flag variable to be filled
 * 7th parameter - ptr to the address counter variable to be filled
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_current_state    (unsigned int, unsigned int *, unsigned int *, unsigned int *, unsigned int  *, unsigned int  *, unsigned int  *);

/*
 * Request SDMA SG channel
 * Reserves two consequent channels
 *
 * Function parameters:
 * 1st parameter - free-form string identifier of channel.
 * 2nd parameter - callback function to be invoked when an interrupt
 *                 occurs for this channel
 * 3rd parameter - additional data (callback-specific context) to be passed
 *                 to the callback function when it's invoked
 * 4th parameter - flag to enable soft IRQ for this channel. Only 1 channel
 *                 may be enabled with soft-irq.
 *
 * Callback parameters:
 * 1st parameter - channel number for which an IRQ occured
 * 2nd parameter - what type of interrupt has happened for a channel
 * 3rd parameter - additional data (callback-specific context)
 * 4th parameter - registers structure as passed to interrupt handler
 *
 * Returns: bigger channel number on success, otherwise negative error code
 */
int dma_request_sg_channel (char *, dma_cb_t cb, void *, int);

/*
 * Request specific SDMA SG channel (actually pair of channels)
 * Reserves two consequent channels with the bigger number as requested
 *
 * Function parameters:
 * 1st parameter - SDMA channel number
 * 2nd parameter - free-form string identifier of channel.
 * 3rd parameter - callback function to be invoked when an interrupt
 *                 occurs for this channel
 * 4th parameter - additional data (callback-specific context) to be passed
 *                 to the callback function when it's invoked
 * 5th parameter - flag to enable soft IRQ for this channel. Only 1 channel
 *                 may be enabled with soft-irq.
 *
 * Callback parameters:
 * 1st parameter - channel number for which an IRQ occured
 * 2nd parameter - what type of interrupt has happened for a channel
 * 3rd parameter - additional data (callback-specific context)
 * 4th parameter - registers structure as passed to interrupt handler
 *
 * Returns: bigger channel number on success, otherwise negative error code
 */
int dma_request_specific_sg_channel (int, char *, dma_cb_t cb, void *, int);

/*
 * Prepare SG list for programming into the SDMA controller
 * This function is intended to set right companion channel for each
 * entry in the list except the last one and to set the last entry of 
 * the scatter-gather list according to one of the methods to define 
 * the last entry described.
 *
 * Function parameters:
 * 1st parameter - channel number returned by dma_request_sg_channel()
 * 2nd parameter - ptr to the first scatter gather list entry
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_prepare_sg_list(int, dma_sg_ll_t *);

/*
 * Program SDMA SG channel
 * This function is very similar to dma_prog_channel, but in this case
 * the user has to supply only the linked-list address
 *
 * Function parameters:
 * 1st parameter - channel number returned by dma_request_sg_channel() 
 * 2nd parameter - physical ptr to the first entry in the linked list
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_prog_sg_channel(int, u32 );

/*
 * Release SDMA SG channel
 *
 * Function parameters:
 * 1st parameter - channel number returned by dma_request_sg_channel()
 *
 * Returns: 0 on success, otherwise failure 
 */
int dma_release_sg_channel (unsigned int);

/*
 * Indicates if DMA channel is enabled
 *
 * Function parameters:
 * 1st parameter - MA channel number
 *
 * Returns: 0 is disabled, otherwise !0
 */
int dma_channel_enabled(unsigned int);

#endif				/* _ASM_ARCH_DMA_H */
