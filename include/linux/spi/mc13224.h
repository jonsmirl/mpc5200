/* linux/spi/mc13224.h */

struct mc13224_platform_data {
	int gpio_ready;
	int gpio_cs;
	int gpio_attention;
	int irq_ready;
	int irq_attention;
};

