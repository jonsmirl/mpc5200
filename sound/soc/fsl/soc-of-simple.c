/*
 * OF helpers for ALSA SoC Layer
 *
 * Copyright (C) 2008, Secret Lab Technologies Ltd.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-of-simple.h>
#include <sound/initval.h>

MODULE_AUTHOR("Grant Likely <grant.likely@secretlab.ca>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ALSA SoC OpenFirmware bindings");

static DEFINE_MUTEX(of_snd_soc_mutex);
static LIST_HEAD(of_snd_soc_device_list);
static int of_snd_soc_next_index;

struct of_snd_soc_device {
	int id;
	struct list_head list;
	struct snd_soc_device device;
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link[SOC_OF_SIMPLE_MAX_DAI];
	struct platform_device *pdev;
	struct device_node *cpu_dai_node;
	struct device_node *codec_node;
};

/* template values */
struct snd_soc_platform *template_platform;
char *template_name = NULL;
struct snd_soc_ops *template_ops;
int (*template_init)(struct snd_soc_codec *codec);

static struct of_snd_soc_device *
of_snd_soc_get_device(struct device_node *codec_node)
{
	int i;

	struct of_snd_soc_device *of_soc;

	list_for_each_entry(of_soc, &of_snd_soc_device_list, list) {
		if (of_soc->codec_node == codec_node)
			return of_soc;
	}

	of_soc = kzalloc(sizeof(struct of_snd_soc_device), GFP_KERNEL);
	if (!of_soc)
		return NULL;

	/* Initialize the structure and add it to the global list */
	of_soc->codec_node = codec_node;
	of_soc->id = of_snd_soc_next_index++;
	of_soc->card.dai_link = of_soc->dai_link;
	of_soc->device.card = &of_soc->card;
	of_soc->card.num_links = SOC_OF_SIMPLE_MAX_DAI;
	for (i = 0; i < SOC_OF_SIMPLE_MAX_DAI; i++) {
		of_soc->dai_link[i].ops = template_ops;
		of_soc->dai_link[i].init = template_init;
	}
	of_soc->card.name = template_name;
	of_soc->card.platform = template_platform;

	list_add(&of_soc->list, &of_snd_soc_device_list);

	return of_soc;
}

static void of_snd_soc_register_device(struct of_snd_soc_device *of_soc)
{
	struct platform_device *pdev;
	int rc;

	/* Only register the device if both the codec and platform have
	 * been registered */
	if ((!of_soc->device.codec_data) || (!of_soc->cpu_dai_node) ||
									!of_soc->card.platform || !of_soc->card.name)
		return;

	pr_info("platform<-->codec match achieved; registering fabric\n");

	pdev = platform_device_alloc("soc-audio", of_soc->id);
	if (!pdev) {
		pr_err("of_soc: platform_device_alloc() failed\n");
		return;
	}

	pdev->dev.platform_data = of_soc;
	platform_set_drvdata(pdev, &of_soc->device);
	of_soc->device.dev = &pdev->dev;

	/* The ASoC device is complete; register it */
	rc = platform_device_add(pdev);
	if (rc) {
		pr_err("of_soc: platform_device_add() failed\n");
		return;
	}

}

int of_snd_soc_register_codec(struct snd_soc_codec_device *codec_dev,
			      void *codec_data, struct snd_soc_dai *dai,
			      int count, struct device_node *node)
{
	struct of_snd_soc_device *of_soc;
	int i, rc = 0;

	pr_info("registering ASoC codec driver: %s\n", node->full_name);

	mutex_lock(&of_snd_soc_mutex);
	of_soc = of_snd_soc_get_device(node);
	if (!of_soc) {
		rc = -ENOMEM;
		goto out;
	}

	/* Store the codec data */
	of_soc->device.codec_data = codec_data;
	of_soc->device.codec_dev = codec_dev;
	of_soc->card.num_links = min(count, of_soc->card.num_links);
	for (i = 0; i < of_soc->card.num_links; i++) {
		of_soc->dai_link[i].name = dai[i].name;
		of_soc->dai_link[i].codec_dai = &dai[i];
	}
	/* Now try to register the SoC device */
	of_snd_soc_register_device(of_soc);

 out:
	mutex_unlock(&of_snd_soc_mutex);
	return rc;
}
EXPORT_SYMBOL_GPL(of_snd_soc_register_codec);

int of_snd_soc_register_cpu_dai(struct device_node *node,
				 struct snd_soc_dai *cpu_dai, int count)
{
	struct of_snd_soc_device *of_soc;
	struct device_node *codec_node;
	const phandle *handle;
	int i, len, rc = 0;

	pr_info("registering ASoC CPU DAI driver: %s\n", node->full_name);

	handle = of_get_property(node, "codec-handle", &len);
	if (handle && len >= sizeof(handle))
		codec_node = of_find_node_by_phandle(*handle);
	else {
		/* Check for the codec child nodes */
		for_each_child_of_node(node, codec_node) {
			struct platform_device *pdev;
			struct dev_archdata dev_ad = {};
			char name[MODULE_NAME_LEN];
			const u32 *addr;
			int len;

			if (of_modalias_node(codec_node, name, sizeof(name)) < 0)
				continue;

			addr = of_get_property(codec_node, "reg", &len);
			if (!addr || len < sizeof(int) || *addr > (1 << 10) - 1) {
				pr_err("invalid codec reg in device tree\n");
				continue;
			}
			request_module("%s", name);

			pdev = platform_device_alloc(name, 0);

			dev_archdata_set_node(&dev_ad, codec_node);
			pdev->dev.archdata = dev_ad;

			rc = platform_device_add(pdev);
			if (rc) {
				platform_device_put(pdev);
				return rc;
			}
			break;
		}
	}
	if (!codec_node)
		return -ENODEV;
	pr_info("looking for codec: %s\n", codec_node->full_name);

	mutex_lock(&of_snd_soc_mutex);
	of_soc = of_snd_soc_get_device(codec_node);
	if (!of_soc) {
		rc = -ENOMEM;
		goto out;
	}

	of_soc->cpu_dai_node = node;
	of_soc->card.num_links = min(count, of_soc->card.num_links);
	for (i = 0; i < of_soc->card.num_links; i++) {
		of_soc->dai_link[i].stream_name = cpu_dai[i].name;
		of_soc->dai_link[i].cpu_dai = &cpu_dai[i];
	}

	/* Now try to register the SoC device */
	of_snd_soc_register_device(of_soc);

 out:
	mutex_unlock(&of_snd_soc_mutex);
	return rc;
}
EXPORT_SYMBOL_GPL(of_snd_soc_register_cpu_dai);

int of_snd_soc_register_platform(struct snd_soc_platform *platform)
{
	struct of_snd_soc_device *of_soc;
	int rc = 0;

	pr_info("registering ASoC platform driver: %s\n", platform->name);
	template_platform = platform;

	mutex_lock(&of_snd_soc_mutex);
	list_for_each_entry(of_soc, &of_snd_soc_device_list, list) {
		of_soc->card.platform = platform;
		of_snd_soc_register_device(of_soc);
	}
	mutex_unlock(&of_snd_soc_mutex);
	return rc;
}
EXPORT_SYMBOL_GPL(of_snd_soc_register_platform);

int of_snd_soc_register_fabric(char *name, struct snd_soc_ops *ops,
								int (*init)(struct snd_soc_codec *codec))
{
	int i;
	struct of_snd_soc_device *of_soc;

	pr_info("registering ASoC fabric driver: %s\n", name);
	template_name = name;
	template_ops = ops;
	template_init = init;

	mutex_lock(&of_snd_soc_mutex);
	list_for_each_entry(of_soc, &of_snd_soc_device_list, list) {
		for (i = 0; i < SOC_OF_SIMPLE_MAX_DAI; i++) {
			of_soc->dai_link[i].ops = ops;
			of_soc->dai_link[i].init = init;
		}
		of_soc->card.name = name;
		of_snd_soc_register_device(of_soc);
	}
	mutex_unlock(&of_snd_soc_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(of_snd_soc_register_fabric);

/* If no board specific fabric driver has been registered, register a default one */
int register_default_fabric(void)
{
	if (template_name == NULL)
		return of_snd_soc_register_fabric("Default Fabric", NULL, NULL);
	return 0;
}

late_initcall(register_default_fabric);
