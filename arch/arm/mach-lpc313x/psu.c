/*  linux/arch/arm/mach-lpc313x/leds.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Analog die register interface (via sysfs) driver.
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


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include <mach/hardware.h>

/* Analog die I2C Register offsets*/
#define    AD_REG_OTGDCLIC_RW                            0x0000
#define    AD_REG_DCDCLIC_RO                             0x0001
#define    AD_REG_CGU_RW                                 0x0002
#define    AD_REG_AIN_0_RW                               0x0010
#define    AD_REG_AIN_1_RW                               0x0011
#define    AD_REG_AOUT_RW                                0x0012
#define    AD_REG_DEC_RW                                 0x0013
#define    AD_REG_INT_0_RW                               0x0014
#define    AD_REG_INT_1_RW                               0x0015
#define    AD_REG_DAIOMUX_RW                             0x0016
#define    AD_REG_AOUTDECINT_RO                          0x0017
#define    AD_REG_RTC_TIME                               0x0020
#define    AD_REG_RTC_ALARM_TIME                         0x0021
#define    AD_REG_RTC_STATUS                             0x0022
#define    AD_REG_RTC_SET_ENA_STAT                       0x0023
#define    AD_REG_RTC_CLR_ENA_STAT                       0x0024
#define    AD_REG_MOD_ID                                 0x03FF

struct psu_data {
	struct i2c_client *client;
};

static struct psu_data g_pca_data;

/* I2C client structure is required for I2C communications.
 * The probe function is called only once when driver is added.
 * Hence the following function be used by the other drivers,
 * to get I2C client structure
 */
struct i2c_client *lpc315x_ad_get_i2c_client_struct(void)
{
	/* Check if psu_data structure is initialised */
	if(!g_pca_data.client) {
		printk(KERN_ERR "I2C not initialised \r\n");
		return NULL;
	}

	return g_pca_data.client;
}
EXPORT_SYMBOL(lpc315x_ad_get_i2c_client_struct);

/* following are the sysfs callback functions */
static ssize_t psu_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	char reg_adr[2];
	u32 reg_val = 0;

	reg_adr[0] = (psa->index >> 8) & 0xFF;
	reg_adr[1] = psa->index & 0xFF;

	i2c_master_send(client, reg_adr, 2);
	i2c_master_recv(client, (char*)&reg_val, 4);
	return sprintf(buf, "0x%08x\n", reg_val);
}

static ssize_t psu_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val = simple_strtoul(buf, NULL, 0);
	char data[6];

	if (val > 0xff)
		return -EINVAL;
	/* 16 bit register address */
	data[0] = (psa->index >> 8) & 0xFF;
	data[1] = psa->index & 0xFF;
	/* 32 bit register value */
	data[2] = (val >> 24) & 0xFF;
	data[3] = (val >> 16) & 0xFF;
	data[4] = (val >> 8) & 0xFF;
	data[5] = val & 0xFF;

	i2c_master_send(client, data, 6);
	return count;
}

/* Define the device attributes */

#define PSU_ENTRY_RO(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO, psu_show, NULL, cmd_idx)

#define PSU_ENTRY_RW(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO | S_IWUSR, psu_show, \
				  psu_store, cmd_idx)

PSU_ENTRY_RW(otgdclic, AD_REG_OTGDCLIC_RW);
PSU_ENTRY_RO(dcdclic, AD_REG_DCDCLIC_RO);
PSU_ENTRY_RW(cgu, AD_REG_CGU_RW);
PSU_ENTRY_RW(ain_0, AD_REG_AIN_0_RW);
PSU_ENTRY_RW(ain_1, AD_REG_AIN_1_RW);
PSU_ENTRY_RW(aout, AD_REG_AOUT_RW);
PSU_ENTRY_RW(dec, AD_REG_DEC_RW);
PSU_ENTRY_RW(int_0, AD_REG_INT_0_RW);
PSU_ENTRY_RW(int_1, AD_REG_INT_1_RW);
PSU_ENTRY_RW(daiomux, AD_REG_DAIOMUX_RW);
PSU_ENTRY_RO(aoutdecint, AD_REG_AOUTDECINT_RO);
PSU_ENTRY_RW(rtc_time, AD_REG_RTC_TIME);
PSU_ENTRY_RW(rtc_alarm_time, AD_REG_RTC_ALARM_TIME);
PSU_ENTRY_RW(rtc_status, AD_REG_RTC_STATUS);
PSU_ENTRY_RW(rtc_set_ena_stat, AD_REG_RTC_SET_ENA_STAT);
PSU_ENTRY_RW(rtc_clr_ena_stat, AD_REG_RTC_CLR_ENA_STAT);
PSU_ENTRY_RO(mod_id, AD_REG_MOD_ID);

static struct attribute *psu_attributes[] = {
	&sensor_dev_attr_otgdclic.dev_attr.attr,
	&sensor_dev_attr_dcdclic.dev_attr.attr,
	&sensor_dev_attr_cgu.dev_attr.attr,
	&sensor_dev_attr_ain_0.dev_attr.attr,
	&sensor_dev_attr_ain_1.dev_attr.attr,
	&sensor_dev_attr_aout.dev_attr.attr,
	&sensor_dev_attr_dec.dev_attr.attr,
	&sensor_dev_attr_int_0.dev_attr.attr,
	&sensor_dev_attr_int_1.dev_attr.attr,
	&sensor_dev_attr_daiomux.dev_attr.attr,
	&sensor_dev_attr_aoutdecint.dev_attr.attr,
	&sensor_dev_attr_rtc_time.dev_attr.attr,
	&sensor_dev_attr_rtc_alarm_time.dev_attr.attr,
	&sensor_dev_attr_rtc_status.dev_attr.attr,
	&sensor_dev_attr_rtc_set_ena_stat.dev_attr.attr,
	&sensor_dev_attr_rtc_clr_ena_stat.dev_attr.attr,
	&sensor_dev_attr_mod_id.dev_attr.attr,
	NULL
};

static struct attribute_group psu_defattr_group = {
	.attrs = psu_attributes,
};


static int psu_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct psu_data *data = &g_pca_data;

	printk(KERN_INFO "PSU_probe\n");
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	memset(data, 0, sizeof(struct psu_data));

	dev_info(&client->dev, "setting platform data\n");
	i2c_set_clientdata(client, data);
	data->client = client;

	printk(KERN_INFO "PSU_probe 1\n");
       /* Register sysfs hooks */
	return sysfs_create_group(&client->dev.kobj,
				  &psu_defattr_group);
}

static int psu_remove(struct i2c_client *client)
{
	struct psu_data *data = i2c_get_clientdata(client);
	sysfs_remove_group(&client->dev.kobj, &psu_defattr_group);
	kfree(data);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id psu_id[] = {
	{ "lpc3152-psu", 0 },
	{ }
};

static struct i2c_driver psu_driver = {
	.driver = {
		.name   = "lpc3152-psu",
	},
	.id_table = psu_id,
	.probe  = psu_probe,
	.remove = psu_remove,
};

static int __init psu_init(void)
{
	cgu_clk_en_dis(CGU_SB_SYSCLK_O_ID, 1);
	return i2c_add_driver(&psu_driver);
}

static void __exit psu_exit(void)
{
	i2c_del_driver(&psu_driver);
}

module_init(psu_init);
module_exit(psu_exit);

