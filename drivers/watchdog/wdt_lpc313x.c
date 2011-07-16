/*
 * LPC313x Watchdog timer driver
 *
 * drivers/watchdog/wdt_lpc313x.c
 *
 * Copyright (C) 2009 NXP Semiconductors
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <mach/cgu.h>

#define MAX_HEARTBEAT 120
#define DEFAULT_HEARTBEAT 25
#define WDT_IN_USE        0
#define WDT_OK_TO_CLOSE   1

/* Set when a watchdog reset has occurred (read only). 
 * This bit is cleared only by a power on reset.
 */
#define LPC313x_WD_BARK      (0x13004C04)

/* Offset of WDT registers */
#define LPC313x_WDT_IR       0x00
#define LPC313x_WDT_TCR      0x04
#define LPC313x_WDT_TC       0x08
#define LPC313x_WDT_PR       0x0C
#define LPC313x_WDT_PC       0x10
#define LPC313x_WDT_MCR      0x14
#define LPC313x_WDT_MR0      0x18
#define LPC313x_WDT_MR1      0x1C
#define LPC313x_WDT_EMR      0x3C

#define INTR_M0      0x01
#define INTR_M1      0x02

#define TCR_EN       0x01
#define TCR_RST      0x02

#define INTEN_MR0    (1 << 0)
#define RESET_MR0    (1 << 1)
#define STOP_MR0     (1 << 2)
#define INTEN_MR1    (1 << 3)
#define RESET_MR1    (1 << 4)
#define STOP_MR1     (1 << 5)

#define TOGGLE_EMR1   	(3 << 6)

static int nowayout = WATCHDOG_NOWAYOUT;
static int heartbeat = DEFAULT_HEARTBEAT;

static struct lpc313x_wdt {
	spinlock_t lock;
	void __iomem *base;
	unsigned long status;
	unsigned long boot_status;
	struct device *dev;
} lpc313x_wdt;

static void lpc313x_wdt_stop(struct lpc313x_wdt *wdt)
{
	void __iomem *base = wdt->base;

	/* Disable watchdog clock */
	cgu_clk_en_dis(CGU_SB_WDOG_PCLK_ID, 0);

	/* Disable and reset counter */
	writel(TCR_RST, base + LPC313x_WDT_TCR);

	/* Clear interrupts */
	writel(INTR_M1, base + LPC313x_WDT_TCR);
	writel(0, base + LPC313x_WDT_MCR);
	writel(0, base + LPC313x_WDT_PC);
	writel(0, base + LPC313x_WDT_PR);

	writel(0, base + LPC313x_WDT_EMR);

	/* Bring counter out of reset */
	writel(0, base + LPC313x_WDT_TCR);
}

static void lpc313x_wdt_start(struct lpc313x_wdt *wdt)
{
	uint32_t freq;
	void __iomem *base = wdt->base;

	/* Enable WDOG_PCLK and get its frequency */
	cgu_clk_en_dis(CGU_SB_WDOG_PCLK_ID, 1);
	freq = cgu_get_clk_freq(CGU_SB_WDOG_PCLK_ID);
	writel(freq - 1, base + LPC313x_WDT_PR);
	writel(heartbeat, base + LPC313x_WDT_MR1);
	writel(TOGGLE_EMR1, base + LPC313x_WDT_EMR);

	/* Start WDT */
	writel(TCR_EN, base + LPC313x_WDT_TCR);
}

static void lpc313x_wdt_keepalive(struct lpc313x_wdt *wdt)
{
	void __iomem *base = wdt->base;
	writel(0, base + LPC313x_WDT_PC);
	writel(0, base + LPC313x_WDT_TC);
}

/**
 *	wdt_open:
 *	@inode: inode of device
 *	@file: file handle to device
 *
 *	The watchdog device has been opened. The watchdog device is single
 *	open and on opening we load the counters. Counter zero is a 100Hz
 *	cascade, into counter 1 which downcounts to reboot. When the counter
 *	triggers counter 2 downcounts the length of the reset pulse which
 *	set set to be as long as possible.
 */

static int lpc313x_wdt_open(struct inode *inode, struct file *file)
{
	struct lpc313x_wdt *wdt = &lpc313x_wdt;
	if (test_and_set_bit(WDT_IN_USE, &wdt->status))
		return -EBUSY;
	clear_bit(WDT_OK_TO_CLOSE, &wdt->status);

	/*
	 *      Activate
	 */
	lpc313x_wdt_start(wdt);
	return nonseekable_open(inode, file);
}

static ssize_t lpc313x_wdt_write(struct file *file, const char *data,
				 size_t len, loff_t * ppos)
{
	struct lpc313x_wdt *wdt = &lpc313x_wdt;
	if (len) {
		if (!nowayout) {
			size_t i;

			clear_bit(WDT_OK_TO_CLOSE, &wdt->status);

			for (i = 0; i != len; i++) {
				char c;

				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					set_bit(WDT_OK_TO_CLOSE, &wdt->status);
			}
		}
		lpc313x_wdt_keepalive(wdt);
	}

	return len;
}

static const struct watchdog_info ident = {
	.options = WDIOF_CARDRESET | WDIOF_MAGICCLOSE |
	    WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "LPC313x Watchdog",
};

static long lpc313x_wdt_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	int ret = -ENOTTY;
	int time;
	struct lpc313x_wdt *wdt = &lpc313x_wdt;
	void __iomem *base = wdt->base;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *)arg, &ident,
				   sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, (int *)arg);
		break;

	case WDIOC_GETBOOTSTATUS:
		ret = put_user(wdt->boot_status, (int *)arg);
		break;

	case WDIOC_KEEPALIVE:
		lpc313x_wdt_keepalive(wdt);
		dev_vdbg(wdt->dev, "Hearbeat received.\n");
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, (int *)arg);
		if (ret)
			break;

		if (time <= 0 || time > MAX_HEARTBEAT) {
			dev_err(wdt->dev, "Timeout value should be an "
				"integer between 1 to %d\n", MAX_HEARTBEAT);
			ret = -EINVAL;
			break;
		}

		heartbeat = time;
		writel(heartbeat, base + LPC313x_WDT_MR1);
		lpc313x_wdt_keepalive(wdt);
		dev_vdbg(wdt->dev, "Timeout set to: %d\n", time);
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(heartbeat, (int *)arg);
		break;
	}
	return ret;
}

static int lpc313x_wdt_release(struct inode *inode, struct file *file)
{
	struct lpc313x_wdt *wdt = &lpc313x_wdt;
	if (!test_bit(WDT_OK_TO_CLOSE, &wdt->status))
		dev_warn(wdt->dev, "Watchdog timer closed unexpectedly\n");

	if (!nowayout) {
		lpc313x_wdt_stop(wdt);
	}

	clear_bit(WDT_IN_USE, &wdt->status);
	clear_bit(WDT_OK_TO_CLOSE, &wdt->status);

	return 0;
}

static const struct file_operations lpc313x_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = lpc313x_wdt_ioctl,
	.open = lpc313x_wdt_open,
	.write = lpc313x_wdt_write,
	.release = lpc313x_wdt_release,
};

static struct miscdevice lpc313x_wdt_misc = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &lpc313x_wdt_fops,
};

static int lpc313x_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct lpc313x_wdt *wdt = &lpc313x_wdt;
	uint32_t size;

	spin_lock_init(&wdt->lock);
	wdt->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "Unable to get resources.\n");
		return -ENXIO;
	}

	size = res->end - res->start + 1;
	if (devm_request_mem_region(&pdev->dev,
				    res->start, size, pdev->name) == NULL) {
		dev_err(&pdev->dev, "Requested memory region unavailable\n");
		return -EBUSY;
	}

	wdt->base = devm_ioremap(&pdev->dev, res->start, size);
	if (wdt->base == NULL) {
		dev_err(&pdev->dev, "Unable to remap memory region\n");
		return -ENOMEM;
	}

	ret = misc_register(&lpc313x_wdt_misc);
	if (ret < 0) {
		dev_err(&pdev->dev, " lpc313x_wdt : failed to register\n");
		return ret;
	}
	platform_set_drvdata(pdev, wdt);

	wdt->boot_status =
	    (readl((void __iomem *)io_p2v(LPC313x_WD_BARK)) & 0x1) ?
	    WDIOF_CARDRESET : 0;
	lpc313x_wdt_stop(wdt);
	dev_info(&pdev->dev, "Watchdog device driver initialized.\n");
	return 0;
}

static int lpc313x_wdt_remove(struct platform_device *pdev)
{
	struct lpc313x_wdt *wdt = &lpc313x_wdt;

	/* Stop the hardware */
	lpc313x_wdt_stop(wdt);

	misc_deregister(&lpc313x_wdt_misc);
	/* All other resources are automatically de-allocated */
	return 0;
}

static struct platform_driver lpc313x_wdt_driver = {
	.probe = lpc313x_wdt_probe,
	.remove = __devexit_p(lpc313x_wdt_remove),
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "lpc313x-wdt",
		   },
};

static int __init lpc313x_wdt_init(void)
{
	return platform_driver_register(&lpc313x_wdt_driver);
}

static void __exit lpc313x_wdt_exit(void)
{
	platform_driver_unregister(&lpc313x_wdt_driver);
}

module_init(lpc313x_wdt_init);
module_exit(lpc313x_wdt_exit);

MODULE_AUTHOR("NXP Semiconductors");
MODULE_DESCRIPTION("Driver for the LPC313x watchdog");
MODULE_LICENSE("GPL");
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
		 "Watchdog heartbeat period in seconds from 1 to "
		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
		 __MODULE_STRING(DEFAULT_HEARTBEAT));

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		 "Set to 1 to keep watchdog running after device release");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:lpc313x-wdt");
