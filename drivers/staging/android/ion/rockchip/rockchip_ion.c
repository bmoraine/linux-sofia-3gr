/*
 * drivers/staging/android/ion/rockchip/rockchip_ion.c
 *
 * Copyright (C) 2014 Meiyou.chen <cmy@rock-chips.com>
 * Copyright (C) 2014 ROCKCHIP, Inc.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/dma-buf.h>
#include <linux/dma-contiguous.h>
#include <linux/memblock.h>
#include <linux/of_gpio.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/sizes.h>
#include <linux/rockchip_ion.h>
#include <sofia/vvpu_vbpipe.h>

#include "../ion_priv.h"

static struct ion_device *idev;
static struct ion_heap **heaps;

#define MAX_ION_HEAP	(10)

#define ALIGN(x, a)	(((x)+((a)-1))&(~((a)-1)))

static struct ion_platform_heap ion_plat_heap[MAX_ION_HEAP];

struct ion_platform_data ion_pdata = {
	.nr = 0,
	.heaps = ion_plat_heap,
};

struct ion_heap_desc {
	unsigned int id;
	enum ion_heap_type type;
	const char *name;
};

static struct ion_heap_desc ion_heap_meta[] = {
	{
		.id	= ION_HEAP_TYPE_SYSTEM,
		.type	= ION_HEAP_TYPE_SYSTEM,
		.name	= "system-heap",
	}, {
		.id	= ION_HEAP_TYPE_CARVEOUT,
		.type	= ION_HEAP_TYPE_CARVEOUT,
		.name	= "carveout-heap",
	}, {
		.id	= ION_HEAP_TYPE_DMA,
		.type	= ION_HEAP_TYPE_DMA,
		.name	= "cma-heap",
	}, {
		.id	= ION_HEAP_TYPE_SECURE,
		.type	= ION_HEAP_TYPE_DMA,
		.name	= "secured-rga-heap",
	}, {
		.id	= ION_HEAP_TYPE_SECURE2,
		/*.type	= ION_HEAP_TYPE_SECURE,*/
		.type	= ION_HEAP_TYPE_DMA,
		.name	= "secured-heap",
	},
};

static int rk_ion_get_phys(struct ion_client *client,
			   unsigned long arg)
{
	struct ion_phys_data data;
	struct ion_handle *handle;
	int ret;

	/* TODO: add compat here */
	if (is_compat_task())
		return -EFAULT;

	if (copy_from_user(&data, (void __user *)arg,
			   sizeof(struct ion_phys_data)))
		return -EFAULT;

	handle = ion_handle_get_by_id(client, data.handle);
	if (IS_ERR(handle))
		return PTR_ERR(handle);

	ret = ion_phys(client, handle, &data.phys,
		       (size_t *)&data.size);
	ion_handle_put(handle);
	if (ret < 0)
		return ret;
	if (copy_to_user((void __user *)arg, &data,
			 sizeof(struct ion_phys_data)))
		return -EFAULT;

	return 0;
}

static int rk_ion_secure_alloc(struct ion_client *client,
			       unsigned int cmd,
			       unsigned long arg)
{
	struct ion_phys_data *data = NULL;
	struct device *dev;
	struct vvpu_secvm_cmd vvpu_cmd;
	int vvpu_ret;
	int ret = 0;

	/* TODO: add compat here */
	if (is_compat_task())
		return -EFAULT;

	dev = ion_struct_device_from_client(client);
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (copy_from_user(data, (void __user *)arg,
			   sizeof(*data))) {
		ret = -EFAULT;
		goto free_data;
	}

	/* call into secure VM to allocate a secure video buffer */
	memset(&vvpu_cmd, 0, sizeof(cmd));

	vvpu_cmd.payload[0] = VVPU_VTYPE_MEM;
	vvpu_cmd.payload[1] = VVPU_VOP_MEM_ALLOC;
	vvpu_cmd.payload[2] = 0;
	vvpu_cmd.payload[3] = 0;
	vvpu_cmd.payload[4] = ALIGN(data->size, PAGE_SIZE);
	vvpu_cmd.payload[5] = 0;

	/* execute command */
	vvpu_ret = vvpu_call(dev, &vvpu_cmd);

	if (vvpu_ret == 0) {
		dev_err(dev, "error allocating secure memory\n");
	} else {
		data->size = (unsigned int)vvpu_cmd.payload[4];
		data->phys = (unsigned int)vvpu_cmd.payload[5];

		dev_info(dev, "ion_alloc_secure() 0x%lx / %lu\n",
			 data->phys, data->size);
	}

	if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
		ret = -EFAULT;

		goto free_data;
	}

free_data:
	kfree(data);
	return ret;
}

static int rk_ion_secure_free(struct ion_client *client,
			      unsigned int cmd,
			      unsigned long arg)
{
	struct ion_phys_data *data;
	struct device *dev;
	struct vvpu_secvm_cmd vvpu_cmd;
	int vvpu_ret;
	int ret = 0;

	/* TODO: add compat here */
	if (is_compat_task())
		return -EFAULT;

	dev = ion_struct_device_from_client(client);
	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
		ret = -EFAULT;
		goto free_data;
	}

	/* call into secure VM to allocate a secure video buffer */
	memset(&vvpu_cmd, 0, sizeof(cmd));

	vvpu_cmd.payload[0] = VVPU_VTYPE_MEM;
	vvpu_cmd.payload[1] = VVPU_VOP_MEM_FREE;
	vvpu_cmd.payload[2] = 0;
	vvpu_cmd.payload[3] = 0;
	vvpu_cmd.payload[4] = ALIGN(data->size, PAGE_SIZE);
	vvpu_cmd.payload[5] = data->phys;

	/* execute command */
	vvpu_ret = vvpu_call(dev, &vvpu_cmd);

	if (vvpu_ret == 0)
		dev_err(dev, "error freeing secure memory\n");

	/* leave data.size and data.addr alone */
	dev_info(dev, "ion_free_secure() 0x%lx / %lu\n",
		data->phys, data->size);
	if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
		ret = -EFAULT;
		goto free_data;
	}

free_data:
	kfree(data);
	return ret;
}

static long rk_custom_ioctl(struct ion_client *client,
			    unsigned int cmd,
			    unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case ION_IOC_GET_PHYS:
		ret = rk_ion_get_phys(client,  arg);
		break;
	case ION_IOC_ALLOC_SECURE:
		ret = rk_ion_secure_alloc(client, cmd, arg);
		break;
	case ION_IOC_FREE_SECURE:
		ret = rk_ion_secure_free(client, cmd, arg);
		break;
	default:
		return -ENOTTY;
	}
	return ret;
}

/* Return result of step for heap array. */
static int rk_ion_of_heap(struct ion_platform_heap *myheap,
			  struct device_node *node)
{
	int ret, itype;

	for (itype = 0; itype < ARRAY_SIZE(ion_heap_meta); itype++) {
		if (strcmp(ion_heap_meta[itype].name, node->name))
			continue;

		struct cma *cma_area;
		phys_addr_t base = 0;
		phys_addr_t size = 0;
		myheap->name = node->name;
		myheap->align = SZ_1M;
		myheap->id = ion_heap_meta[itype].id;

		if (!strcmp("system-heap", node->name)) {
			myheap->type = ION_HEAP_TYPE_SYSTEM;
			return 1;
		}

		ret = of_get_reserved_memory_region(
				node,
				&size, &base, &cma_area);
		if (ret) {
			pr_err("rk_ion Can't find memory def! Skip %s\n",
			       node->name);
			continue;
		}

		myheap->base = base;
		myheap->size = size;

		if (cma_area) {
			myheap->priv2 = cma_area;
			myheap->type = ION_HEAP_TYPE_DMA;
		} else if (myheap->base && !cma_area) {
			myheap->type = ION_HEAP_TYPE_CARVEOUT;
		} else if (size) {
			myheap->type = ION_HEAP_TYPE_SYSTEM;
		} else {
			pr_err("rk_ion unknown memory type! Skip %s\n",
			       node->name);
			continue;
		}

		pr_info("rk_ion heap %s base:%pa length:0x%08x type %d\n",
			node->name, &myheap->base,
			myheap->size, myheap->type);
		return 1;
	}
	return 0;
}

static struct ion_platform_data *rk_ion_of(struct device_node *node)
{
	struct ion_platform_data *pdata;
	int iheap = 0;
	struct device_node *child;
	struct ion_platform_heap *myheap;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->nr = of_get_child_count(node);
again:
	pdata->heaps = kcalloc(pdata->nr, sizeof(*myheap), GFP_KERNEL);
	for_each_child_of_node(node, child) {
		iheap += rk_ion_of_heap(&pdata->heaps[iheap], child);
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

static int rk_ion_probe(struct platform_device *pdev)
{
	int err;
	int i;
	struct ion_platform_data *pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		pdata = rk_ion_of(pdev->dev.of_node);
		if (pdata == NULL)
			return -EINVAL;
		pdev->dev.platform_data = pdata;
	}

	heaps = kcalloc(pdata->nr, sizeof(*heaps), GFP_KERNEL);
	if (heaps == NULL)
		return -ENOMEM;

	idev = ion_device_create(rk_custom_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < pdata->nr; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		heap_data->priv = &pdev->dev;
		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);
	rk_ion_handler_init(pdev->dev.of_node, idev, pdata);
	return 0;
err:
	for (i = 0; i < pdata->nr; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}

	kfree(heaps);
	return err;
}

static int rk_ion_remove(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	rk_ion_handler_exit();
	ion_device_destroy(idev);
	for (i = 0; i < pdata->nr; i++)
		ion_heap_destroy(heaps[i]);

	kfree(heaps);
	return 0;
}

struct ion_client *rockchip_ion_client_create(const char *name)
{
	if (idev == NULL) {
		pr_err("rockchip ion idev is NULL\n");
		return NULL;
	}

	return ion_client_create(idev, name);
}
EXPORT_SYMBOL_GPL(rockchip_ion_client_create);

static const struct of_device_id rk_ion_match[] = {
	{ .compatible = "rockchip,ion", },
	{}
};

static struct platform_driver ion_driver = {
	.probe = rk_ion_probe,
	.remove = rk_ion_remove,
	.driver = {
		.name = "ion-rk",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rk_ion_match),
	},
};

static int __init rk_ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void __exit rk_ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

subsys_initcall(rk_ion_init);
module_exit(rk_ion_exit);

MODULE_AUTHOR("Meiyou.chen <cmy@rock-chips.com>");
MODULE_DESCRIPTION("SoFIA-3GR Ion driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, rk_ion_match);
