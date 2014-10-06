/*
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "../ion_priv.h"
#include <linux/module.h>
#include <linux/of.h>

struct xgold_ion_get_params_data {
	int handle;
	unsigned int size;
	unsigned int addr;
};

enum {
	XGOLD_ION_GET_PARAM = 0,
};

enum {
	ION_HEAP_TYPE_DISPLAY_CARVEOUT = ION_HEAP_TYPE_CUSTOM,
	ION_HEAP_TYPE_VIDEO_CARVEOUT,
};

struct ion_mapper *xgold_user_mapper;
struct ion_heap **heaps;

extern struct ion_handle *ion_handle_get_by_id(struct ion_client *client,
						int id);

static int xgold_ion_get_param(struct ion_client *client,
					unsigned int cmd,
					unsigned long arg)
{
	struct xgold_ion_get_params_data data;
	struct ion_handle *handle;
	ion_phys_addr_t paddr;
	size_t size;
	struct xgold_ion_get_params_data *user_data =
				(struct xgold_ion_get_params_data *)arg;
	struct ion_buffer *buffer;

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;
	handle = ion_handle_get_by_id(client, data.handle);
	buffer = ion_handle_buffer(handle);
	ion_phys(client, handle, &paddr, &size);
	ion_free(client, handle);
	data.addr = (unsigned int) paddr;
	data.size = (unsigned int) size;

	if (copy_to_user(user_data, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}



static long xgold_ion_ioctl(struct ion_client *client,
				   unsigned int cmd,
				   unsigned long arg)
{
	int ret = -ENOTTY;

	switch (cmd) {
	case XGOLD_ION_GET_PARAM:
		ret = xgold_ion_get_param(client, cmd, arg);
		break;
	default:
		pr_err("Unknown custom ioctl\n");
		return -ENOTTY;
	}
	return ret;
}

struct ion_type_name {
	const char *name;
	unsigned int id;
	unsigned int type;
};

static struct ion_type_name ion_heap_type_name[] = {
	{
		.name = "system-heap",
		.id = ION_HEAP_TYPE_SYSTEM,
		.type = ION_HEAP_TYPE_SYSTEM
	}, {
		.name = "carveout-heap",
		.id = ION_HEAP_TYPE_CARVEOUT,
		.type = ION_HEAP_TYPE_CARVEOUT
	}, {
		.name = "cma-heap",
		.id = ION_HEAP_TYPE_DMA,
		.type = ION_HEAP_TYPE_DMA
	}, {
		.name = "display-heap",
		.id = ION_HEAP_TYPE_DISPLAY_CARVEOUT,
		.type = ION_HEAP_TYPE_CARVEOUT
	}, {
		.name = "video-heap",
		.id = ION_HEAP_TYPE_VIDEO_CARVEOUT,
		.type = ION_HEAP_TYPE_CARVEOUT
	},
};

#include <linux/of.h>
#define ION_PROP_HEAP_MEM	"intel,heapmem"

static struct ion_platform_data *xgold_ion_of(struct device_node *node)
{
	struct ion_platform_data *pdata;
	int ret = 0;
	int itype = 0;
	int iheap = 0;
	struct device_node *child;
	struct ion_platform_heap *myheap;
	u32 array[2];
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);

	pdata->nr = of_get_child_count(node);
again:
	pdata->heaps = kcalloc(pdata->nr, sizeof(*myheap), GFP_KERNEL);
	for_each_child_of_node(node, child) {
		for (itype = 0;
				itype < ARRAY_SIZE(ion_heap_type_name);
				itype++) {
			if (!strcmp(ion_heap_type_name[itype].name,
							child->name)) {
				myheap = &pdata->heaps[iheap];
				pr_debug("%s: heap %p\n", __func__, myheap);
				myheap->name = child->name;
				myheap->type = ion_heap_type_name[itype].type;
				myheap->align = 0x100000;
				myheap->id = ion_heap_type_name[itype].id;
				ret = of_property_read_u32_array(child,
						ION_PROP_HEAP_MEM, array, 2);
				if (ret) {
					pr_err("xg_ion Can't read property:%s, skipping node %s\n",
						ION_PROP_HEAP_MEM, child->name);
					continue;
				}
				myheap->base = array[0];
				myheap->size = array[1] - 1;
				pr_info("xg_ion heap %d %s base:0x%08x length:0x%08x\n",
							iheap, child->name,
							array[0], array[1]);
				iheap++;
			}
		}
	}

	if (pdata->nr != iheap) {
		pdata->nr = iheap;
		iheap = 0;
		kfree(pdata->heaps);
		pr_err("%s: mismatch, repeating\n", __func__);
		goto again;
	}

	return pdata;
}

int xgold_ion_probe(struct platform_device *pdev)
{
	int err;
	int i;
	struct ion_device *idev;
	struct ion_platform_data *pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		pdata = xgold_ion_of(pdev->dev.of_node);
		pdev->dev.platform_data = pdata;
	}

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);

	idev = ion_device_create(xgold_ion_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < pdata->nr; i++) {
		struct ion_platform_heap *heap_data =  &pdata->heaps[i];

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);
	return 0;
err:
	for (i = 0; i < pdata->nr; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	kfree(heaps);
	return err;
}

int xgold_ion_remove(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < pdata->nr; i++)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}
static const struct of_device_id xgold_ion[] = {
	{
		.compatible = "intel,ion",
	},
	{}
};

MODULE_DEVICE_TABLE(of, xgold_ion);

static struct platform_driver ion_driver = {
	.probe = xgold_ion_probe,
	.remove = xgold_ion_remove,
	.driver = {
		.name = "xgold-ion",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(xgold_ion),
	}
};

static int ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);

