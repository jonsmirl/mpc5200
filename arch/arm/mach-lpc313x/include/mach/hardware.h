/* linux/arch/arm/mach-lpc313x/include/mach/hardware.h
 *  
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Hardware register defines for LPC313x and LPC315x SoCs.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

#include "constants.h"


/* macros to convert phys to virtual & virtual to phys memory location*/
#define io_p2v(x) (0xf0000000 | (((x) & 0xff000000) >> 4) | ((x) & 0x000fffff))
#define io_v2p(x) (             (((x) & 0x0ff00000) << 4) | ((x) & 0x000fffff))

#define MASK_AND_SET(v,m,s)	(v) = ((v)&~(m))|(s)

#ifdef __ASSEMBLY__

# define __REG(x)	io_p2v(x)
# define __PREG(x)	io_v2p(x)

#else

# if 0
#  define __REG(x)	(*((volatile u32 *)io_p2v(x)))
# else
/*
 * This __REG() version gives the same results as the one above,  except
 * that we are fooling gcc somehow so it generates far better and smaller
 * assembly code for access to contigous registers.  
 */
#include <asm/types.h>
typedef struct { volatile u32 offset[4096]; } __regbase;
# define __REGP(x)	((__regbase *)((x)&~4095))->offset[((x)&4095)>>2]
# define __REG(x)	__REGP(io_p2v(x))
typedef struct { volatile u16 offset[4096]; } __regbase16;
# define __REGP16(x)	((__regbase16 *)((x)&~4095))->offset[((x)&4095)>>1]
# define __REG16(x)	__REGP16(io_p2v(x))
typedef struct { volatile u8 offset[4096]; } __regbase8;
# define __REGP8(x)	((__regbase8 *)((x)&~4095))->offset[(x)&4095]
# define __REG8(x)	__REGP8(io_p2v(x))
#endif /* 0 */

# define __REG2(x,y)	\
	( __builtin_constant_p(y) ? (__REG((x) + (y))) \
				  : (*(volatile u32 *)((u32)&__REG(x) + (y))) )

# define __PREG(x)	(io_v2p((u32)&(x)))

/* include CGU header */
#include "cgu.h"

#endif /*__ASSEMBLY__ */


#include "registers.h"

#endif

