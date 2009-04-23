/*
 * Maxim max9485 Programmable Audio Clock Generator driver
 *
 * Written by: Jon Smirl <jonsmirl@gmail.com>
 *
 * Copyright (C) 2008 Digispeaker.com
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * This device is usually under the control of ALSA and should not be changed
 * from userspace. The main purpose of this driver is to locate the i2c address
 * of where the chip is located.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/sysfs.h>
#include <linux/i2c/max9485.h>

int max9485_set(struct i2c_client *max9485, u8 value)
{
	return i2c_smbus_write_byte(max9485, value);
}
EXPORT_SYMBOL_GPL(max9485_set);

/*
 * Display the only register
 */
static ssize_t max9485_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;

	rc = i2c_smbus_read_byte(client);
	if (rc < 0)
		return rc;

	return sprintf(buf, "%s%s%s%s%s%s",
			(rc & MAX9485_MCLK ? "MCLK+" : ""),
			(rc & MAX9485_CLK_OUT_2 ? "CLK2+" : ""),
			(rc & MAX9485_CLK_OUT_2 ? "CLK1+" : ""),
			(rc & MAX9485_DOUBLED ? "DOUBLED+" : ""),
			(rc & MAX9485_SCALE_768 ? "768x+" : (rc & MAX9485_SCALE_384 ? "384x+" : "256x+")),
			((rc & 3) == MAX9485_FREQUENCY_48 ? "48Khz" :
			((rc & 3) == MAX9485_FREQUENCY_441 ? "44.1Khz" :
			((rc & 3) == MAX9485_FREQUENCY_32 ? "32Khz" : "12Khz"))));
}
static DEVICE_ATTR(max9485, S_IRUGO, max9485_show, NULL);

/*
 * Called when a max9485 device is matched with this driver
 */
static int max9485_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c bus does not support the max9485\n");
		return -ENODEV;
	}
	return sysfs_create_file(&client->dev.kobj, &dev_attr_max9485.attr);
}

static int max9485_remove(struct i2c_client *client)
{
	sysfs_remove_file(&client->dev.kobj, &dev_attr_max9485.attr);
	return 0;
}

static const struct i2c_device_id max9485_id[] = {
	{ "max9485", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9485_id);

static struct i2c_driver max9485_driver = {
	.driver = {
		.name = "max9485",
	},
	.probe = max9485_probe,
	.remove = max9485_remove,
	.id_table = max9485_id,
};

static int __init max9485_init(void)
{
	return i2c_add_driver(&max9485_driver);
}

static void __exit max9485_exit(void)
{
	i2c_del_driver(&max9485_driver);
}

MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com");
MODULE_DESCRIPTION("Maxim MAX9485 Programmable Audio Clock Generator driver");
MODULE_LICENSE("GPL");

module_init(max9485_init);
module_exit(max9485_exit);
