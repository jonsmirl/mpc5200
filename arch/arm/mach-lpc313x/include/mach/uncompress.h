/*  linux/arch/arm/mach-lpc313x/include/mach/uncompress.h
 *  
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Decompression UART routines for LPC313x and LPC315x SoCs.
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

#include "constants.h"
#include "registers.h"


#undef __REG
#define __REG(x)	(*((volatile u32 *)(x)))
/***********************************************************************
 * UART register definitions
 **********************************************************************/
#define UART_DLL      __REG (UART_PHYS + 0x00)
#define UART_FIFO     __REG (UART_PHYS + 0x00)
#define UART_IE       __REG (UART_PHYS + 0x04)
#define UART_DLM      __REG (UART_PHYS + 0x04)
#define UART_IIR      __REG (UART_PHYS + 0x08)
#define UART_FCR      __REG (UART_PHYS + 0x08)
#define UART_LCR      __REG (UART_PHYS + 0x0c)
#define UART_MCR      __REG (UART_PHYS + 0x10)
#define UART_LSR      __REG (UART_PHYS + 0x14)
#define UART_MSR      __REG (UART_PHYS + 0x18)
#define UART_SCR      __REG (UART_PHYS + 0x1c)
#define UART_ACR      __REG (UART_PHYS + 0x20)
#define UART_ICR      __REG (UART_PHYS + 0x24)
#define UART_FDR      __REG (UART_PHYS + 0x28)


#define UART_LOAD_DLL(div)          ((div) & 0xFF)
#define UART_LOAD_DLM(div)          (((div) >> 8) & 0xFF)
#define UART_LCR_DIVLATCH_EN       _BIT(7)
#define UART_LCR_WLEN_8BITS        _SBF(0, 3)
#define UART_FCR_DMA_MODE          _BIT(3)
#define UART_FCR_TXFIFO_FLUSH      _BIT(2)
#define UART_FCR_RXFIFO_FLUSH      _BIT(1)
#define UART_FCR_FIFO_EN           _BIT(0)
#define UART_LSR_FIFORX_ERR        _BIT(7)
#define UART_LSR_TEMT              _BIT(6)
#define UART_LSR_FR                _BIT(3)
#define UART_LSR_PE                _BIT(2)
#define UART_LSR_OE                _BIT(1)
#define UART_LSR_RDR               _BIT(0)

/*
 * The following code assumes the serial port has already been
 * initialized by the bootloader. If you didn't setup a port in
 * your bootloader then nothing will appear (which might be desired).
 *
 * This does not append a newline
 */
static void putc(int c)
{
	while (!(UART_LSR & UART_LSR_TEMT))
		barrier();

	UART_FIFO = c;
}

static inline void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()
