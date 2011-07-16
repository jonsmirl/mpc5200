/*  linux/arch/arm/mach-lpc313x/i2c.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * I2C initialization for LPC313x.
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

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c-pnx.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/i2c.h>
#include <mach/gpio.h>
#include <mach/irqs.h>

#define LPC313x_I2C0_SLV_ADDR            __REG (I2C0_PHYS + 0x014)
#define LPC313x_I2C1_SLV_ADDR            __REG (I2C1_PHYS + 0x014)

static int set_clock_run(struct platform_device *pdev)
{
	if (pdev->id == 0)
		cgu_clk_en_dis( CGU_SB_I2C0_PCLK_ID, 1);
	else
		cgu_clk_en_dis( CGU_SB_I2C1_PCLK_ID, 1);

	udelay(2);
	return 0;
}

static int set_clock_stop(struct platform_device *pdev)
{
	if (pdev->id == 0)
		cgu_clk_en_dis( CGU_SB_I2C0_PCLK_ID, 0);
	else
		cgu_clk_en_dis( CGU_SB_I2C1_PCLK_ID, 0);

	return 0;
}

static int i2c_lpc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int retval = 0;
#ifdef CONFIG_PM
	retval = set_clock_stop(pdev);
#endif
	return retval;
}

static int i2c_lpc_resume(struct platform_device *pdev)
{
	int retval = 0;
#ifdef CONFIG_PM
	retval = set_clock_run(pdev);
#endif
	return retval;
}

static u32 calculate_input_freq(struct platform_device *pdev)
{
	return (FFAST_CLOCK/1000000);
}


static struct i2c_pnx_algo_data lpc_algo_data0 = {
	.base = I2C0_PHYS,
	.irq = IRQ_I2C0,
};

static struct i2c_pnx_algo_data lpc_algo_data1 = {
	.base = I2C1_PHYS,
	.irq = IRQ_I2C1,
};

static struct i2c_adapter lpc_adapter0 = {
	.name = I2C_CHIP_NAME "0",
	.algo_data = &lpc_algo_data0,
};
static struct i2c_adapter lpc_adapter1 = {
	.name = I2C_CHIP_NAME "1",
	.algo_data = &lpc_algo_data1,
};

static struct i2c_pnx_data i2c0_data = {
	.suspend = i2c_lpc_suspend,
	.resume = i2c_lpc_resume,
	.calculate_input_freq = calculate_input_freq,
	.set_clock_run = set_clock_run,
	.set_clock_stop = set_clock_stop,
	.adapter = &lpc_adapter0,
};

static struct i2c_pnx_data i2c1_data = {
	.suspend = i2c_lpc_suspend,
	.resume = i2c_lpc_resume,
	.calculate_input_freq = calculate_input_freq,
	.set_clock_run = set_clock_run,
	.set_clock_stop = set_clock_stop,
	.adapter = &lpc_adapter1,
};

static struct platform_device i2c0_device = {
	.name = "pnx-i2c",
	.id = 0,
	.dev = {
		.platform_data = &i2c0_data,
	},
};

static struct platform_device i2c1_device = {
	.name = "pnx-i2c",
	.id = 1,
	.dev = {
		.platform_data = &i2c1_data,
	},
};

static struct platform_device *devices[] __initdata = {
	&i2c0_device,
	&i2c1_device,
};

void __init lpc313x_register_i2c_devices(void)
{
	cgu_clk_en_dis( CGU_SB_I2C0_PCLK_ID, 1);
	cgu_clk_en_dis( CGU_SB_I2C1_PCLK_ID, 1);

	/* Enable I2C1 signals */
	GPIO_DRV_IP(IOCONF_I2C1, 0x3);

#if defined (CONFIG_MACH_VAL3153) || defined (CONFIG_MACH_EA313X)
	/* on EA and VAL boards UDA1380 is connected to I2C1
	 * whose slave address is same as LPC313x's default slave
	 * adress causing bus contention errors. So change the
	 * deafult slave address register value of LPC313x here.
	 */
	LPC313x_I2C0_SLV_ADDR = 0x06E;
	LPC313x_I2C1_SLV_ADDR = 0x06E;
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
}

