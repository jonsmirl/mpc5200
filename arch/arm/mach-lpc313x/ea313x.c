/*  arch/arm/mach-lpc313x/ea313x.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  ea313x board init routines.
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dm9000.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/sizes.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <mach/gpio.h>
#include <mach/i2c.h>
#include <mach/board.h>

static struct lpc313x_mci_irq_data irq_data = {
	.irq = IRQ_SDMMC_CD,
};

static int mci_get_cd(u32 slot_id)
{
	return gpio_get_value(GPIO_MI2STX_BCK0);
}

static irqreturn_t ea313x_mci_detect_interrupt(int irq, void *data)
{
	struct lpc313x_mci_irq_data	*pdata = data;

	/* select the opposite level senstivity */
	int level = mci_get_cd(0)?IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH;

	set_irq_type(pdata->irq, level);

	/* change the polarity of irq trigger */
	return pdata->irq_hdlr(irq, pdata->data);
}

static int mci_init(u32 slot_id, irq_handler_t irqhdlr, void *data)
{
	int ret;
	int level;

	/* enable power to the slot */
	gpio_set_value(GPIO_MI2STX_DATA0, 0);
	/* set cd pins as GPIO pins */
	gpio_direction_input(GPIO_MI2STX_BCK0);

	/* select the opposite level senstivity */
	level = mci_get_cd(0)?IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH;
	/* set card detect irq info */
	irq_data.data = data;
	irq_data.irq_hdlr = irqhdlr;
	set_irq_type(irq_data.irq, level);
	ret = request_irq(irq_data.irq,
			ea313x_mci_detect_interrupt,
			level,
			"mmc-cd", 
			&irq_data);
	/****temporary for PM testing */
	enable_irq_wake(irq_data.irq);

	return irq_data.irq;
}

static int mci_get_ro(u32 slot_id)
{
	return 0;
}

static int mci_get_ocr(u32 slot_id)
{
	return MMC_VDD_32_33 | MMC_VDD_33_34;
}

static void mci_setpower(u32 slot_id, u32 volt)
{
	/* on current version of EA board the card detect
	 * pull-up in on switched power side. So can't do
	 * power management so use the always enable power 
	 * jumper.
	 */
}
static int mci_get_bus_wd(u32 slot_id)
{
	return 4;
}

static void mci_exit(u32 slot_id)
{
	free_irq(irq_data.irq, &irq_data);
}

static struct resource lpc313x_mci_resources[] = {
	[0] = {
		.start  = IO_SDMMC_PHYS,
		.end	= IO_SDMMC_PHYS + IO_SDMMC_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MCI,
		.end	= IRQ_MCI,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct lpc313x_mci_board ea313x_mci_platform_data = {
	.num_slots		= 1,
	.detect_delay_ms	= 250,
	.init 			= mci_init,
	.get_ro			= mci_get_ro,
	.get_cd 		= mci_get_cd,
	.get_ocr		= mci_get_ocr,
	.get_bus_wd		= mci_get_bus_wd,
	.setpower 		= mci_setpower,
	.exit			= mci_exit,
};

static u64 mci_dmamask = 0xffffffffUL;
static struct platform_device	lpc313x_mci_device = {
	.name		= "lpc313x_mmc",
	.num_resources	= ARRAY_SIZE(lpc313x_mci_resources),
	.dev		= {
		.dma_mask		= &mci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ea313x_mci_platform_data,
	},
	.resource	= lpc313x_mci_resources,
};

/*
 * DM9000 ethernet device
 */
#if defined(CONFIG_DM9000)
static struct resource dm9000_resource[] = {
	[0] = {
		.start	= EXT_SRAM1_PHYS,
		.end	= EXT_SRAM1_PHYS + 0xFF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= EXT_SRAM1_PHYS + 0x10000,
		.end	= EXT_SRAM1_PHYS + 0x100FF,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_DM9000_ETH_INT,
		.end	= IRQ_DM9000_ETH_INT,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};
/* ARM MPMC contoller as part of low power design doesn't de-assert nCS and nOE for consecutive 
reads but just changes address. But DM9000 requires nCS and nOE change between address. So access
other chip select area (nCS0) to force de-assertion of nCS1 and nOE1. Or else wait for long time 
such as 80 usecs. 
LPC313x has external logic outside of MPMC IP to toggle nOE to split consecutive reads.
The latest Apex bootloader pacth makes use of this feture.
For this to work SYS_MPMC_WTD_DEL0 & SYS_MPMC_WTD_DEL1 should be programmed with MPMC_STWTRD0
& MPMC_STWTRD1 values. The logic only deactivates the nOE for one clock cycle which is
11nsec but DM9000 needs 80nsec between nOEs. So lets add some dummy instructions such as
reading a GPIO register to compensate for extra 70nsec.
*/
# define DM_IO_DELAY()	do { gpio_get_value(GPIO_MNAND_RYBN3);} while(0)

static void dm9000_dumpblk(void __iomem *reg, int count)
{
	int i;
	int tmp;

	count = (count + 1) >> 1;
	for (i = 0; i < count; i++) {
		DM_IO_DELAY();
		tmp = readw(reg);
	}
}

static void dm9000_inblk(void __iomem *reg, void *data, int count)
{
	int i;
	u16* pdata = (u16*)data;
	count = (count + 1) >> 1;
	for (i = 0; i < count; i++) {
		DM_IO_DELAY();
		*pdata++ = readw(reg);
	}
}

static struct dm9000_plat_data dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM | DM9000_PLATF_SIMPLE_PHY,
	.dumpblk = dm9000_dumpblk,
	.inblk = dm9000_inblk,
};

static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resource),
	.resource	= dm9000_resource,
	.dev		= {
		.platform_data	= &dm9000_platdata,
	}
};
static void __init ea_add_device_dm9000(void)
{
	/*
	 * Configure Chip-Select 2 on SMC for the DM9000.
	 * Note: These timings were calculated for MASTER_CLOCK = 90000000
	 *  according to the DM9000 timings.
	 */
	MPMC_STCONFIG1 = 0x81;
	MPMC_STWTWEN1 = 1;
	MPMC_STWTOEN1 = 1;
	MPMC_STWTRD1 = 4;
	MPMC_STWTPG1 = 1;
	MPMC_STWTWR1 = 1;
	MPMC_STWTTURN1 = 2;
	/* enable oe toggle between consec reads */
	SYS_MPMC_WTD_DEL1 = _BIT(5) | 4;

	/* Configure Interrupt pin as input, no pull-up */
	gpio_direction_input(GPIO_MNAND_RYBN3);

	platform_device_register(&dm9000_device);
}
#else
static void __init ea_add_device_dm9000(void) {}
#endif /* CONFIG_DM9000 */


#if defined (CONFIG_MTD_NAND_LPC313X)
static struct resource lpc313x_nand_resources[] = {
	[0] = {
		.start  = IO_NAND_PHYS,
		.end	= IO_NAND_PHYS + IO_NAND_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start 	= IO_NAND_BUF_PHYS,
		.end 	= IO_NAND_BUF_PHYS + IO_NAND_BUF_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start 	= IRQ_NAND_FLASH,
		.end 	= IRQ_NAND_FLASH,
		.flags	= IORESOURCE_IRQ,
	}
};

#define BLK_SIZE (2048 * 64)
static struct mtd_partition ea313x_nand0_partitions[] = {
	/* The EA3131 board uses the following block scheme:
	128K: Blocks 0   - 0    - LPC31xx info and bad block table
	384K: Blocks 1   - 3    - Apex bootloader
	256K: Blocks 4   - 5    - Apex environment
	4M:   Blocks 6   - 37   - Kernel image
	16M:  Blocks 38  - 165  - Ramdisk image (if used)
	???:  Blocks 166 - end  - Root filesystem/storage */
	{
		.name	= "lpc313x-rootfs",
		.offset	= (BLK_SIZE * 166),
		.size	= MTDPART_SIZ_FULL
	},
};

static struct lpc313x_nand_timing ea313x_nanddev_timing = {
	.ns_trsd	= 36,
	.ns_tals	= 36,
	.ns_talh	= 12,
	.ns_tcls	= 36,
	.ns_tclh	= 12,
	.ns_tdrd	= 36,
	.ns_tebidel	= 12,
	.ns_tch		= 12,
	.ns_tcs		= 48,
	.ns_treh	= 24,
	.ns_trp		= 48,
	.ns_trw		= 24,
	.ns_twp		= 36
};

static struct lpc313x_nand_dev_info ea313x_ndev[] = {
	{
		.name		= "nand0",
		.nr_partitions	= ARRAY_SIZE(ea313x_nand0_partitions),
		.partitions	= ea313x_nand0_partitions
	}
};

static struct lpc313x_nand_cfg ea313x_plat_nand = {
	.nr_devices	= ARRAY_SIZE(ea313x_ndev),
	.devices	= ea313x_ndev,
	.timing		= &ea313x_nanddev_timing,
	.support_16bit	= 0,
};

static u64 nand_dmamask = 0xffffffffUL;
static struct platform_device	lpc313x_nand_device = {
	.name		= "lpc313x_nand",
	.dev		= {
		.dma_mask		= &nand_dmamask,
		.coherent_dma_mask	= 0xffffffff,
				.platform_data	= &ea313x_plat_nand,
	},
	.num_resources	= ARRAY_SIZE(lpc313x_nand_resources),
	.resource	= lpc313x_nand_resources,
};
#endif

#if defined(CONFIG_SPI_LPC313X)
static struct resource lpc313x_spi_resources[] = {
	[0] = {
		.start	= SPI_PHYS,
		.end	= SPI_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SPI,
		.end	= IRQ_SPI,
		.flags	= IORESOURCE_IRQ,
	},
};

static void spi_set_cs_state(int cs_num, int state)
{
	/* Only CS0 is supported, so no checks are needed */
	(void) cs_num;

	/* Set GPO state for CS0 */
	lpc313x_gpio_set_value(GPIO_SPI_CS_OUT0, state);
}

struct lpc313x_spics_cfg lpc313x_stdspics_cfg[] =
{
	/* SPI CS0 */
	[0] =
	{
		.spi_spo	= 0, /* Low clock between transfers */
		.spi_sph	= 0, /* Data capture on first clock edge (high edge with spi_spo=0) */
		.spi_cs_set	= spi_set_cs_state,
	},
};

struct lpc313x_spi_cfg lpc313x_spidata =
{
	.num_cs			= ARRAY_SIZE(lpc313x_stdspics_cfg),
	.spics_cfg		= lpc313x_stdspics_cfg,
};

static u64 lpc313x_spi_dma_mask = 0xffffffffUL;
static struct platform_device lpc313x_spi_device = {
	.name		= "spi_lpc313x",
	.id		= 0,
	.dev		= {
		.dma_mask = &lpc313x_spi_dma_mask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data	= &lpc313x_spidata,
	},
	.num_resources	= ARRAY_SIZE(lpc313x_spi_resources),
	.resource	= lpc313x_spi_resources,
};

/* If both SPIDEV and MTD data flash are enabled with the same chip select, only 1 will work */
#if defined(CONFIG_SPI_SPIDEV)
/* SPIDEV driver registration */
static int __init lpc313x_spidev_register(void)
{
	struct spi_board_info info =
	{
		.modalias = "spidev",
		.max_speed_hz = 1000000,
		.bus_num = 0,
		.chip_select = 0,
	};

	return spi_register_board_info(&info, 1);
}
arch_initcall(lpc313x_spidev_register);
#endif

#if defined(CONFIG_MTD_DATAFLASH)
/* MTD Data FLASH driver registration */
static int __init lpc313x_spimtd_register(void)
{
	struct spi_board_info info =
	{
		.modalias = "mtd_dataflash",
		.max_speed_hz = 30000000,
		.bus_num = 0,
		.chip_select = 0,
	};

	return spi_register_board_info(&info, 1);
}
arch_initcall(lpc313x_spimtd_register);
#endif
#endif

static struct platform_device *devices[] __initdata = {
	&lpc313x_mci_device,
#if defined (CONFIG_MTD_NAND_LPC313X)
	&lpc313x_nand_device,
#endif
#if defined(CONFIG_SPI_LPC313X)
	&lpc313x_spi_device,
#endif
};

static struct map_desc ea313x_io_desc[] __initdata = {
	{
		.virtual	= io_p2v(EXT_SRAM0_PHYS),
		.pfn		= __phys_to_pfn(EXT_SRAM0_PHYS),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(EXT_SRAM1_PHYS + 0x10000),
		.pfn		= __phys_to_pfn(EXT_SRAM1_PHYS + 0x10000),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_SDMMC_PHYS),
		.pfn		= __phys_to_pfn(IO_SDMMC_PHYS),
		.length		= IO_SDMMC_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_USB_PHYS),
		.pfn		= __phys_to_pfn(IO_USB_PHYS),
		.length		= IO_USB_SIZE,
		.type		= MT_DEVICE
	},
};

static struct i2c_board_info ea313x_i2c_devices[] __initdata = {
	{
		I2C_BOARD_INFO("pca9532", 0x60),
	},
};

#if defined(CONFIG_MACH_EA3152)
static struct i2c_board_info ea3152_i2c1_devices[] __initdata = {
	{
		I2C_BOARD_INFO("lpc3152-psu", 0x0C),
	},
};
#endif


static void __init ea313x_init(void)
{
	lpc313x_init();
	/* register i2cdevices */
	lpc313x_register_i2c_devices();
	

	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* add DM9000 device */
	ea_add_device_dm9000();
	
	i2c_register_board_info(0, ea313x_i2c_devices,
		ARRAY_SIZE(ea313x_i2c_devices));

#if defined(CONFIG_MACH_EA3152)
	i2c_register_board_info(1, ea3152_i2c1_devices,
		ARRAY_SIZE(ea3152_i2c1_devices));
#endif
}

static void __init ea313x_map_io(void)
{
	lpc313x_map_io();
	iotable_init(ea313x_io_desc, ARRAY_SIZE(ea313x_io_desc));
}

#if defined(CONFIG_MACH_EA3152)
MACHINE_START(EA3152, "NXP EA3152")
	/* Maintainer: Durgesh Pattamatta, NXP */
	.phys_io	= IO_APB01_PHYS,
	.io_pg_offst	= (io_p2v(IO_APB01_PHYS) >> 18) & 0xfffc,
	.boot_params	= 0x30000100,
	.map_io		= ea313x_map_io,
	.init_irq	= lpc313x_init_irq,
	.timer		= &lpc313x_timer,
	.init_machine	= ea313x_init,
MACHINE_END
#endif

#if defined(CONFIG_MACH_EA313X)
MACHINE_START(EA313X, "NXP EA313X")
	/* Maintainer: Durgesh Pattamatta, NXP */
	.phys_io	= IO_APB01_PHYS,
	.io_pg_offst	= (io_p2v(IO_APB01_PHYS) >> 18) & 0xfffc,
	.boot_params	= 0x30000100,
	.map_io		= ea313x_map_io,
	.init_irq	= lpc313x_init_irq,
	.timer		= &lpc313x_timer,
	.init_machine	= ea313x_init,
MACHINE_END
#endif

