/*  arch/arm/mach-lpc313x/val3154.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  val3154 board init routines.
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

/* mci platform functions */
static int mci_init(u32 slot_id, irq_handler_t , void *);
static int mci_get_ro(u32 slot_id);
static int mci_get_cd(u32 slot_id);
static int mci_get_ocr(u32 slot_id);
static void mci_setpower(u32 slot_id, u32 volt);
static void mci_exit(u32 slot_id);
static int mci_get_bus_wd(u32 slot_id);

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
static struct lpc313x_mci_board val3154_mci_platform_data = {
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
		.platform_data		= &val3154_mci_platform_data,
	},
	.resource	= lpc313x_mci_resources,
};


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

/* If both SPIDEV and MTD data flash are enabled, only 1 will work */
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
#endif


static struct platform_device *devices[] __initdata = {
	&lpc313x_mci_device,
#if defined(CONFIG_SPI_LPC313X)
	&lpc313x_spi_device,
#endif
};

static struct map_desc val3154_io_desc[] __initdata = {
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
};

static struct i2c_board_info val3154_i2c_devices[] __initdata = {
	{
		I2C_BOARD_INFO("lpc3152-psu", 0x0C),
	},
};

static struct lpc313x_mci_irq_data irq_data = {
	.irq = IRQ_SDMMC_CD,
};

static irqreturn_t val3154_mci_detect_interrupt(int irq, void *data)
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
	/* select the opposite level senstivity */
	int level = mci_get_cd(0)?IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH;

	/* set slot_select, cd and wp pins as GPIO pins */
	gpio_direction_input(GPIO_MI2STX_BCK0);

	/* set card detect irq info */
	irq_data.data = data;
	irq_data.irq_hdlr = irqhdlr;
	set_irq_type(irq_data.irq, level);
	ret = request_irq(irq_data.irq,
			val3154_mci_detect_interrupt,
			level,
			"mmc-cd", 
			&irq_data);
	return ret;

}

static int mci_get_ro(u32 slot_id)
{
	return 0;
}

static int mci_get_cd(u32 slot_id)
{
	return gpio_get_value(GPIO_MI2STX_BCK0);
}

static int mci_get_ocr(u32 slot_id)
{
	return MMC_VDD_32_33 | MMC_VDD_33_34;
}

static void mci_setpower(u32 slot_id, u32 volt)
{
	gpio_set_value(GPIO_MI2STX_DATA0, 0);
}
static int mci_get_bus_wd(u32 slot_id)
{
	return 4;
}

static void mci_exit(u32 slot_id)
{
	free_irq(irq_data.irq, &irq_data);
}

void lpc313x_vbus_power(int enable)
{
	if (enable) 
		gpio_set_value(GPIO_I2SRX_DATA0, 1);
	else
		gpio_set_value(GPIO_I2SRX_DATA0, 0);
}

static void __init val3154_init(void)
{
	lpc313x_init();
	/* register i2cdevices */
	lpc313x_register_i2c_devices();
	
	i2c_register_board_info(1, val3154_i2c_devices,
		ARRAY_SIZE(val3154_i2c_devices));

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

static void __init val3154_map_io(void)
{
	lpc313x_map_io();
	iotable_init(val3154_io_desc, ARRAY_SIZE(val3154_io_desc));
}


MACHINE_START(VAL3154, "NXP VAL3154")
	/* Maintainer: Durgesh Pattamatta, NXP */
	.phys_io	= IO_APB01_PHYS,
	.io_pg_offst	= (io_p2v(IO_APB01_PHYS) >> 18) & 0xfffc,
	.boot_params	= 0x30000100,
	.map_io		= val3154_map_io,
	.init_irq	= lpc313x_init_irq,
	.timer		= &lpc313x_timer,
	.init_machine	= val3154_init,
MACHINE_END
