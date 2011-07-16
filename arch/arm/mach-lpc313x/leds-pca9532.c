/*  linux/arch/arm/mach-lpc313x/leds.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * LED driver for ea313x boards.
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
#include <linux/leds-pca9532.h>
#include <asm/leds.h>
#include <mach/gpio.h>

#define VBUS_PWR_EN	6
#define START_STOP_LED	8  /*led5 */
#define IDLE_LED	9  /*led6 */

#define PCA9532_REG_PSC(i) (0x2+(i)*2)
#define PCA9532_REG_PWM(i) (0x3+(i)*2)
#define PCA9532_REG_LS0  0x6
#define LED_REG(led) ((led>>2)+PCA9532_REG_LS0)
#define LED_NUM(led) (led & 0x3)
/* LED states */
#define PCA9532_LED_OFF     0
#define PCA9532_LED_ON      1
#define PCA9532_LED_PWM0    2
#define PCA9532_LED_PWM1    3



#define ldev_to_led(c)       container_of(c, struct pca9532_led, ldev)

struct pca9532_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct input_dev    *idev;
	u32 init;
};

static u8 g_leds[4] = { 0, 0, 0, 0};
static struct pca9532_data g_pca_data;


/* Set LED routing */
static void pca9532_setgpio(int led_id, int state)
{
	struct pca9532_data *data = &g_pca_data;
	struct i2c_client *client = data->client;
	s32 ret;
	u8 reg;

	if (data->init == 0) {
		/* save the setting */
		g_leds[led_id >> 2] &= ~(0x3 << LED_NUM(led_id)*2);
		g_leds[led_id >> 2] |= ((state & 0x3) << LED_NUM(led_id)*2);

		return;
	}

	mutex_lock(&data->update_lock);
	reg = (u8)(g_leds[led_id >> 2] & 0xFF);
	/* zero led bits */
	reg = reg & ~(0x3 << LED_NUM(led_id)*2);
	/* set the new value */
	reg = reg | ((state & 0x3) << LED_NUM(led_id)*2);

#if 0
	ret = i2c_smbus_read_byte_data(client, LED_REG(led_id));
#else
	ret = g_leds[led_id >> 2];
#endif
	printk ("pca9532: r: 0x%x w: 0x%x reg:0x%x\n", (u8)(ret & 0xFF), reg, LED_REG(led_id));
	  
	if (ret != reg)
		i2c_smbus_write_byte_data(client, LED_REG(led_id), reg);

	g_leds[led_id >> 2] = reg;

	mutex_unlock(&data->update_lock);
}


static int pca9532_configure(struct i2c_client *client,	struct pca9532_data *data)
{
	/* set PWM0 for 50-50 duty cycle with longest period*/
	i2c_smbus_write_byte_data(client, PCA9532_REG_PSC(0), 0xFF);
	i2c_smbus_write_byte_data(client, PCA9532_REG_PWM(0), 0x80);

	i2c_smbus_write_byte_data(client, PCA9532_REG_LS0, g_leds[0]);
	i2c_smbus_write_byte_data(client, PCA9532_REG_LS0 + 1, g_leds[1]);
	i2c_smbus_write_byte_data(client, PCA9532_REG_LS0 + 2, g_leds[2]);
	i2c_smbus_write_byte_data(client, PCA9532_REG_LS0 + 3, g_leds[3]);

	return 0;

}
/*
 * Handle LED events.
 */
static void ea313x_leds_event(led_event_t evt)
{
	unsigned long flags;

	local_irq_save(flags);

	switch(evt) {
#ifdef CONFIG_LEDS_TIMER
	case led_timer:		/* Every 50 timer ticks */
		{
			unsigned long is_off = gpio_get_value(GPIO_GPIO2);
			if (is_off)
				gpio_set_value(GPIO_GPIO2, 0);
			else
				gpio_set_value(GPIO_GPIO2, 1);
		}
		break;
#endif
	default:
		break;
	}

	local_irq_restore(flags);
}

static int pca9532_suspend(struct i2c_client * client, pm_message_t mesg)
{
	/* System stop / suspend */
	pca9532_setgpio(START_STOP_LED, PCA9532_LED_PWM0);
	return 0;
}

static int pca9532_resume(struct i2c_client * client)
{
	/* System resume */
	pca9532_setgpio(START_STOP_LED, PCA9532_LED_OFF);
	return 0;
}

static int pca9532_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct pca9532_data *data = &g_pca_data;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	memset(data, 0, sizeof(struct pca9532_data));

	dev_info(&client->dev, "setting platform data\n");
	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->update_lock);

	pca9532_configure(client, data);
	/* now set the led hander */
	leds_event = ea313x_leds_event;
	leds_event(led_start);
	/* flag init complete */
	data->init = 1;

	return 0;

}

static int pca9532_remove(struct i2c_client *client)
{
	struct pca9532_data *data = i2c_get_clientdata(client);
	kfree(data);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id pca9532_id[] = {
	{ "pca9532", 0 },
	{ }
};

static struct i2c_driver pca9532_driver = {
	.driver = {
		.name   = "pca9532",
	},
	.id_table = pca9532_id,
	.probe  = pca9532_probe,
	.remove = pca9532_remove,
	.suspend = pca9532_suspend,
	.resume = pca9532_resume,
};

static int __init pca9532_init(void)
{
	return i2c_add_driver(&pca9532_driver);
}

static void __exit pca9532_exit(void)
{
	i2c_del_driver(&pca9532_driver);
}

void lpc313x_vbus_power(int enable)
{
	if (enable) {
		printk (KERN_INFO "enabling USB host vbus_power\n");
		pca9532_setgpio(VBUS_PWR_EN, PCA9532_LED_ON);
	} else {
		printk (KERN_INFO "disabling USB host vbus_power\n");
		pca9532_setgpio(VBUS_PWR_EN, PCA9532_LED_OFF);
	}
}
__initcall(pca9532_init);

