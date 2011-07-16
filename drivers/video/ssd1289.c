/*
 * SSD1289 Framebuffer
 *
 * Copyright (c) 2009 Jean-Christian de Rivaz
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * The Solomon Systech SSD1289 chip drive TFT screen up to 240x320. This
 * driver expect the SSD1286 to be connected to a 16 bits local bus and
 * to be set in the 16 bits parallel interface mode. To use it you must
 * define in your board file a struct platform_device with a name set to
 * "ssd1289" and a struct resource array with two IORESOURCE_MEM: the first
 * for the control register; the second for the data register.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/fb.h>
#include <asm/io.h>

#define SSD1289_REG_OSCILLATION      0x00
#define SSD1289_REG_DRIVER_OUT_CTRL  0x01
#define SSD1289_REG_LCD_DRIVE_AC     0x02
#define SSD1289_REG_POWER_CTRL_1     0x03
#define SSD1289_REG_DISPLAY_CTRL     0x07
#define SSD1289_REG_FRAME_CYCLE      0x0b
#define SSD1289_REG_POWER_CTRL_2     0x0c
#define SSD1289_REG_POWER_CTRL_3     0x0d
#define SSD1289_REG_POWER_CTRL_4     0x0e
#define SSD1289_REG_GATE_SCAN_START  0x0f
#define SSD1289_REG_SLEEP_MODE       0x10
#define SSD1289_REG_ENTRY_MODE       0x11
#define SSD1289_REG_POWER_CTRL_5     0x1e
#define SSD1289_REG_GDDRAM_DATA      0x22
#define SSD1289_REG_WR_DATA_MASK_1   0x23
#define SSD1289_REG_WR_DATA_MASK_2   0x24
#define SSD1289_REG_FRAME_FREQUENCY  0x25
#define SSD1289_REG_GAMMA_CTRL_1     0x30
#define SSD1289_REG_GAMME_CTRL_2     0x31
#define SSD1289_REG_GAMMA_CTRL_3     0x32
#define SSD1289_REG_GAMMA_CTRL_4     0x33
#define SSD1289_REG_GAMMA_CTRL_5     0x34
#define SSD1289_REG_GAMMA_CTRL_6     0x35
#define SSD1289_REG_GAMMA_CTRL_7     0x36
#define SSD1289_REG_GAMMA_CTRL_8     0x37
#define SSD1289_REG_GAMMA_CTRL_9     0x3a
#define SSD1289_REG_GAMMA_CTRL_10    0x3b
#define SSD1289_REG_V_SCROLL_CTRL_1  0x41
#define SSD1289_REG_V_SCROLL_CTRL_2  0x42
#define SSD1289_REG_H_RAM_ADR_POS    0x44
#define SSD1289_REG_V_RAM_ADR_START  0x45
#define SSD1289_REG_V_RAM_ADR_END    0x46
#define SSD1289_REG_FIRST_WIN_START  0x48
#define SSD1289_REG_FIRST_WIN_END    0x49
#define SSD1289_REG_SECND_WIN_START  0x4a
#define SSD1289_REG_SECND_WIN_END    0x4b
#define SSD1289_REG_GDDRAM_X_ADDR    0x4e
#define SSD1289_REG_GDDRAM_Y_ADDR    0x4f

struct ssd1289_page {
	unsigned short x;
	unsigned short y;
	unsigned short *buffer;
	unsigned short len;
};

struct ssd1289 {
	struct device *dev;
	volatile unsigned short *ctrl_io;
	volatile unsigned short *data_io;
	struct fb_info *info;
	unsigned int pages_count;
	struct ssd1289_page *pages;
};

static inline void ssd1289_reg_set(struct ssd1289 *item, unsigned char reg,
				   unsigned short value)
{
	unsigned short ctrl = reg & 0x00ff;

	writew(ctrl, item->ctrl_io);
	writew(value, item->data_io);
}

static void ssd1289_copy(struct ssd1289 *item, unsigned int index)
{
	unsigned short x;
	unsigned short y;
	unsigned short *buffer;
	unsigned int len;
	unsigned int count;

	x = item->pages[index].x;
	y = item->pages[index].y;
	buffer = item->pages[index].buffer;
	len = item->pages[index].len;
	dev_dbg(item->dev,
		"%s: page[%u]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
		__func__, index, x, y, buffer, len);

	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_X_ADDR, x);
	ssd1289_reg_set(item, SSD1289_REG_GDDRAM_Y_ADDR, y);
	writew(SSD1289_REG_GDDRAM_DATA, item->ctrl_io);
	for (count = 0; count < len; count++) {
		writew(buffer[count], item->data_io);
	}
}

static void ssd1289_update(struct fb_info *info, struct list_head *pagelist)
{
	struct ssd1289 *item = (struct ssd1289 *)info->par;
	struct page *page;

	list_for_each_entry(page, pagelist, lru) {
		ssd1289_copy(item, page->index);
	}
}

static void __init ssd1289_update_all(struct ssd1289 *item)
{
	unsigned short index;

	for (index = 0; index < item->pages_count; index++) {
		ssd1289_copy(item, index);
	}
}

static void __init ssd1289_setup(struct ssd1289 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	// OSCEN=1
	ssd1289_reg_set(item, SSD1289_REG_OSCILLATION, 0x0001);
	// DCT=b1010=fosc/4 BT=b001=VGH:+6,VGL:-4
	// DC=b1010=fosc/4 AP=b010=small to medium
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_1, 0xa2a4);
	// VRC=b100:5.5V
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_2, 0x0004);
	// VRH=b1000:Vref*2.165
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_3, 0x0308);
	// VCOMG=1 VDV=b1000:VLCD63*1.05
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_4, 0x3000);
	// nOTP=1 VCM=0x2a:VLCD63*0.77
	ssd1289_reg_set(item, SSD1289_REG_POWER_CTRL_5, 0x00ea);
	// RL=0 REV=1 CAD=0 BGR=1 SM=0 TB=1 MUX=0x13f=319
	ssd1289_reg_set(item, SSD1289_REG_DRIVER_OUT_CTRL, 0x2b3f);
	// FLD=0 ENWS=0 D/C=1 EOR=1 WSMD=0 NW=0x00=0
	ssd1289_reg_set(item, SSD1289_REG_LCD_DRIVE_AC, 0x0600);
	// SLP=0
	ssd1289_reg_set(item, SSD1289_REG_SLEEP_MODE, 0x0000);
	// VSMode=0 DFM=3:65k TRAMS=0 OEDef=0 WMode=0 Dmode=0
	// TY=0 ID=3 AM=0 LG=0
	ssd1289_reg_set(item, SSD1289_REG_ENTRY_MODE, 0x6030);
	// PT=0 VLE=1 SPT=0 GON=1 DTE=1 CM=0 D=3
	ssd1289_reg_set(item, SSD1289_REG_DISPLAY_CTRL, 0x0233);
	// NO=0 SDT=0 EQ=0 DIV=0 SDIV=1 SRTN=1 RTN=9:25 clock
	ssd1289_reg_set(item, SSD1289_REG_FRAME_CYCLE, 0x0039);
	// SCN=0
	ssd1289_reg_set(item, SSD1289_REG_GATE_SCAN_START, 0x0000);

	// PKP1=7 PKP0=7
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_1, 0x0707);
	// PKP3=2 PKP2=4
	ssd1289_reg_set(item, SSD1289_REG_GAMME_CTRL_2, 0x0204);
	// PKP5=2 PKP4=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_3, 0x0204);
	// PRP1=5 PRP0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_4, 0x0502);
	// PKN1=5 PKN0=7
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_5, 0x0507);
	// PKN3=2 PNN2=4
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_6, 0x0204);
	// PKN5=2 PKN4=4
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_7, 0x0204);
	// PRN1=5 PRN0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_8, 0x0502);
	// VRP1=3 VRP0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_9, 0x0302);
	// VRN1=3 VRN0=2
	ssd1289_reg_set(item, SSD1289_REG_GAMMA_CTRL_10, 0x0302);

	// WMR=0 WMG=0
	ssd1289_reg_set(item, SSD1289_REG_WR_DATA_MASK_1, 0x0000);
	// WMB=0
	ssd1289_reg_set(item, SSD1289_REG_WR_DATA_MASK_2, 0x0000);
	// OSC=b1010:548k
	ssd1289_reg_set(item, SSD1289_REG_FRAME_FREQUENCY, 0xa000);
	// SS1=0
	ssd1289_reg_set(item, SSD1289_REG_FIRST_WIN_START, 0x0000);
	// SE1=319
	ssd1289_reg_set(item, SSD1289_REG_FIRST_WIN_END,
			(item->info->var.yres - 1));
	// SS2=0
	ssd1289_reg_set(item, SSD1289_REG_SECND_WIN_START, 0x0000);
	// SE2=0
	ssd1289_reg_set(item, SSD1289_REG_SECND_WIN_END, 0x0000);
	// VL1=0
	ssd1289_reg_set(item, SSD1289_REG_V_SCROLL_CTRL_1, 0x0000);
	// VL2=0
	ssd1289_reg_set(item, SSD1289_REG_V_SCROLL_CTRL_2, 0x0000);
	// HEA=0xef=239 HSA=0
	ssd1289_reg_set(item, SSD1289_REG_H_RAM_ADR_POS,
			(item->info->var.xres - 1) << 8);
	// VSA=0
	ssd1289_reg_set(item, SSD1289_REG_V_RAM_ADR_START, 0x0000);
	// VEA=0x13f=319
	ssd1289_reg_set(item, SSD1289_REG_V_RAM_ADR_END,
			(item->info->var.yres - 1));
}

static int __init ssd1289_video_alloc(struct ssd1289 *item)
{
	unsigned int frame_size;

	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	frame_size = item->info->fix.line_length * item->info->var.yres;
	dev_dbg(item->dev, "%s: item=0x%p frame_size=%u\n",
		__func__, (void *)item, frame_size);

	item->pages_count = frame_size / PAGE_SIZE;
	if ((item->pages_count * PAGE_SIZE) < frame_size) {
		item->pages_count++;
	}
	dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
		__func__, (void *)item, item->pages_count);

	item->info->fix.smem_len = item->pages_count * PAGE_SIZE;
	item->info->fix.smem_start =
	    (unsigned long)vmalloc(item->info->fix.smem_len);
	if (!item->info->fix.smem_start) {
		dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
		return -ENOMEM;
	}
	memset((void *)item->info->fix.smem_start, 0, item->info->fix.smem_len);

	return 0;
}

static void ssd1289_video_free(struct ssd1289 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	kfree((void *)item->info->fix.smem_start);
}

static int __init ssd1289_pages_alloc(struct ssd1289 *item)
{
	unsigned short pixels_per_page;
	unsigned short yoffset_per_page;
	unsigned short xoffset_per_page;
	unsigned short index;
	unsigned short x = 0;
	unsigned short y = 0;
	unsigned short *buffer;
	unsigned int len;

	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	item->pages = kmalloc(item->pages_count * sizeof(struct ssd1289_page),
			      GFP_KERNEL);
	if (!item->pages) {
		dev_err(item->dev, "%s: unable to kmalloc for ssd1289_page\n",
			__func__);
		return -ENOMEM;
	}

	pixels_per_page = PAGE_SIZE / (item->info->var.bits_per_pixel / 8);
	yoffset_per_page = pixels_per_page / item->info->var.xres;
	xoffset_per_page = pixels_per_page -
	    (yoffset_per_page * item->info->var.xres);
	dev_dbg(item->dev, "%s: item=0x%p pixels_per_page=%hu "
		"yoffset_per_page=%hu xoffset_per_page=%hu\n",
		__func__, (void *)item, pixels_per_page,
		yoffset_per_page, xoffset_per_page);

	buffer = (unsigned short *)item->info->fix.smem_start;
	for (index = 0; index < item->pages_count; index++) {
		len = (item->info->var.xres * item->info->var.yres) -
		    (index * pixels_per_page);
		if (len > pixels_per_page) {
			len = pixels_per_page;
		}
		dev_dbg(item->dev,
			"%s: page[%d]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
			__func__, index, x, y, buffer, len);
		item->pages[index].x = x;
		item->pages[index].y = y;
		item->pages[index].buffer = buffer;
		item->pages[index].len = len;

		x += xoffset_per_page;
		if (x >= item->info->var.xres) {
			y++;
			x -= item->info->var.xres;
		}
		y += yoffset_per_page;
		buffer += pixels_per_page;
	}

	return 0;
}

static void ssd1289_pages_free(struct ssd1289 *item)
{
	dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

	kfree(item->pages);
}

static struct fb_ops ssd1289_fbops = {
	.owner        = THIS_MODULE,
	.fb_fillrect  = sys_fillrect,
	.fb_copyarea  = sys_copyarea,
	.fb_imageblit = sys_imageblit,
};

static struct fb_fix_screeninfo ssd1289_fix __initdata = {
	.id          = "SSD1289",
	.type        = FB_TYPE_PACKED_PIXELS,
	.visual      = FB_VISUAL_DIRECTCOLOR,
	.accel       = FB_ACCEL_NONE,
	.line_length = 240 * 2,
};

static struct fb_var_screeninfo ssd1289_var __initdata = {
	.xres		= 240,
	.yres		= 320,
	.xres_virtual	= 240,
	.yres_virtual	= 320,
	.bits_per_pixel	= 16,
	.red		= {6, 5, 0},
	.green		= {11, 5, 0},
	.blue		= {0, 6, 0},
	.activate	= FB_ACTIVATE_NOW,
	.height		= 320,
	.width		= 240,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_deferred_io ssd1289_defio = {
        .delay          = HZ / 50,
        .deferred_io    = &ssd1289_update,
};

static int __init ssd1289_probe(struct platform_device *dev)
{
	int ret = 0;
	struct ssd1289 *item;
	struct resource *ctrl_res;
	struct resource *data_res;
	unsigned int ctrl_res_size;
	unsigned int data_res_size;
	struct resource *ctrl_req;
	struct resource *data_req;
	unsigned short signature;
	struct fb_info *info;

	dev_dbg(&dev->dev, "%s\n", __func__);

	item = kzalloc(sizeof(struct ssd1289), GFP_KERNEL);
	if (!item) {
		dev_err(&dev->dev,
			"%s: unable to kzalloc for ssd1289\n", __func__);
		ret = -ENOMEM;
		goto out;
	}
	item->dev = &dev->dev;
	dev_set_drvdata(&dev->dev, item);

	ctrl_res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!ctrl_res) {
		dev_err(&dev->dev,
			"%s: unable to platform_get_resource for ctrl_res\n",
			__func__);
		ret = -ENOENT;
		goto out_item;
	}
	ctrl_res_size = ctrl_res->end - ctrl_res->start + 1;
	ctrl_req = request_mem_region(ctrl_res->start, ctrl_res_size,
				      dev->name);
	if (!ctrl_req) {
		dev_err(&dev->dev,
			"%s: unable to request_mem_region for ctrl_req\n",
			__func__);
		ret = -EIO;
		goto out_item;
	}
	item->ctrl_io = ioremap(ctrl_res->start, ctrl_res_size);
	if (!item->ctrl_io) {
		ret = -EINVAL;
		dev_err(&dev->dev,
			"%s: unable to ioremap for ctrl_io\n", __func__);
		goto out_item;
	}

	data_res = platform_get_resource(dev, IORESOURCE_MEM, 1);
	if (!data_res) {
		dev_err(&dev->dev,
			"%s: unable to platform_get_resource for data_res\n",
			__func__);
		ret = -ENOENT;
		goto out_item;
	}
	data_res_size = data_res->end - data_res->start + 1;
	data_req = request_mem_region(data_res->start,
				      data_res_size, dev->name);
	if (!data_req) {
		dev_err(&dev->dev,
			"%s: unable to request_mem_region for data_req\n",
			__func__);
		ret = -EIO;
		goto out_item;
	}
	item->data_io = ioremap(data_res->start, data_res_size);
	if (!item->data_io) {
		ret = -EINVAL;
		dev_err(&dev->dev,
			"%s: unable to ioremap for data_io\n", __func__);
		goto out_item;
	}

	dev_dbg(&dev->dev, "%s: ctrl_io=%p data_io=%p\n",
		__func__, item->ctrl_io, item->data_io);

	signature = readw(item->ctrl_io);
	dev_dbg(&dev->dev, "%s: signature=0x%04x\n", __func__, signature);
	if (signature != 0x8989) {
		ret = -ENODEV;
		dev_err(&dev->dev,
			"%s: unknown signature 0x%04x\n", __func__, signature);
		goto out_item;
	}

	dev_info(&dev->dev, "item=0x%p ctrl=0x%p data=0x%p\n", (void *)item,
		 (void *)ctrl_res->start, (void *)data_res->start);

	info = framebuffer_alloc(sizeof(struct ssd1289), &dev->dev);
	if (!info) {
		ret = -ENOMEM;
		dev_err(&dev->dev,
			"%s: unable to framebuffer_alloc\n", __func__);
		goto out_item;
	}
	item->info = info;
	info->par = item;
	info->fbops = &ssd1289_fbops;
	info->flags = FBINFO_FLAG_DEFAULT;
	info->fix = ssd1289_fix;
	info->var = ssd1289_var;

	ret = ssd1289_video_alloc(item);
	if (ret) {
		dev_err(&dev->dev,
			"%s: unable to ssd1289_video_alloc\n", __func__);
		goto out_info;
	}
	info->screen_base = (char __iomem *)item->info->fix.smem_start;

	ret = ssd1289_pages_alloc(item);
	if (ret < 0) {
		dev_err(&dev->dev,
			"%s: unable to ssd1289_pages_init\n", __func__);
		goto out_video;
	}
	info->fbdefio = &ssd1289_defio;
	fb_deferred_io_init(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&dev->dev,
			"%s: unable to register_frambuffer\n", __func__);
		goto out_pages;
	}
	ssd1289_setup(item);
	ssd1289_update_all(item);

	return ret;

out_pages:
	ssd1289_pages_free(item);
out_video:
	ssd1289_video_free(item);
out_info:
	framebuffer_release(info);
out_item:
	kfree(item);
out:
	return ret;
}

static struct platform_driver ssd1289_driver = {
	.probe = ssd1289_probe,
	.driver = {
		   .name = "ssd1289",
		   },
};

static int __init ssd1289_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);

	ret = platform_driver_register(&ssd1289_driver);
	if (ret) {
		pr_err("%s: unable to platform_driver_register\n", __func__);
	}

	return ret;
}

module_init(ssd1289_init);

MODULE_DESCRIPTION("SSD1289 LCD Driver");
MODULE_AUTHOR("Jean-Christian de Rivaz <jc@eclis.ch>");
MODULE_LICENSE("GPL");
