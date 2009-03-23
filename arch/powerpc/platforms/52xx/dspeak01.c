/*
 * Support for Digispeaker DSPEAK01
 *
 * Written by Jon Smirl <jonsmirl@gmail.com>
 * Copyright (C) 2007 Jon Smirl
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG
#include <asm/time.h>
#include <asm/prom.h>
#include <asm/machdep.h>
#include <asm/mpc52xx.h>

/* mpc5200 device tree match tables */
static struct of_device_id mpc5200_cdm_ids[] __initdata = {
	{ .compatible = "fsl,mpc5200-cdm", },
	{ .compatible = "mpc5200-cdm", },
	{}
};

static struct of_device_id mpc5200_gpio_ids[] __initdata = {
	{ .compatible = "fsl,mpc5200-gpio", },
	{ .compatible = "mpc5200-gpio", },
	{}
};

/*
 * Fix clock configuration.
 *
 * Firmware is supposed to be responsible for this.  If you are creating a
 * new board port, do *NOT* duplicate this code.  Fix your boot firmware
 * to set it correctly in the first place
 */
static void __init
digispeaker_fix_clock_config(void)
{
	struct device_node *np;
	struct mpc52xx_cdm  __iomem *cdm;
	/* Map zones */
	np = of_find_matching_node(NULL, mpc5200_cdm_ids);
	cdm = of_iomap(np, 0);
	of_node_put(np);
	if (!cdm) {
		printk(KERN_ERR "%s() failed; expect abnormal behaviour\n",
		       __func__);
		return;
	}

	/* Use internal 48 Mhz */
	out_8(&cdm->ext_48mhz_en, 0x00);
	out_8(&cdm->fd_enable, 0x01);
	if (in_be32(&cdm->rstcfg) & 0x40)	/* Assumes 33Mhz clock */
		out_be16(&cdm->fd_counters, 0x0001);
	else
		out_be16(&cdm->fd_counters, 0x5555);

	/* Unmap the regs */
	iounmap(cdm);
}

/*
 * Fix setting of port_config register.
 *
 * Firmware is supposed to be responsible for this.  If you are creating a
 * new board port, do *NOT* duplicate this code.  Fix your boot firmware
 * to set it correctly in the first place
 */
static void __init
digispeaker_fix_port_config(void)
{
	struct device_node *np;
	struct mpc52xx_gpio __iomem *gpio;
	u32 port_config;

	np = of_find_matching_node(NULL, mpc5200_gpio_ids);
	gpio = of_iomap(np, 0);
	of_node_put(np);
	if (!gpio) {
		printk(KERN_ERR "%s() failed. expect abnormal behavior\n",
		       __func__);
		return;
	}

	/* Set port config */
	port_config = in_be32(&gpio->port_config);

	port_config &= 0xFFFFFF00;	/* Clear PSC1/2 config */
	port_config |= 0x00000076;	/* PSC1/2 both in codec master mode */

	port_config &= ~0x00007000;	/* USB port : Differential mode	*/
	port_config |=  0x00001000;	/*            USB 1 only	*/

	pr_debug("port_config: old:%x new:%x\n",
	         in_be32(&gpio->port_config), port_config);
	out_be32(&gpio->port_config, port_config);

	/* Unmap zone */
	iounmap(gpio);
}

/*
 * Setup the architecture
 */
static void __init digispeaker_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mpc5200_simple_setup_arch()", 0);

	/* Map important registers from the internal memory map */
	mpc52xx_map_common_devices();

	/* Some mpc5200 & mpc5200b related configuration */
	mpc5200_setup_xlb_arbiter();

	mpc52xx_setup_pci();

	/* Fix things that firmware should have done. */
	digispeaker_fix_port_config();
	digispeaker_fix_clock_config();
}

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init digispeaker_probe(void)
{
	unsigned long node = of_get_flat_dt_root();

	return of_flat_dt_is_compatible(node, "digispeaker,dspeak01");
}

static void __init digispeaker_declare_platform_devices(void)
{
	mpc52xx_declare_of_platform_devices();
}

define_machine(digispeaker_platform) {
	.name		= "dspeak01",
	.probe		= digispeaker_probe,
	.setup_arch	= digispeaker_setup_arch,
	.init		= digispeaker_declare_platform_devices,
	.init_IRQ	= mpc52xx_init_irq,
	.get_irq	= mpc52xx_get_irq,
	.restart	= mpc52xx_restart,
	.calibrate_decr	= generic_calibrate_decr,
};
