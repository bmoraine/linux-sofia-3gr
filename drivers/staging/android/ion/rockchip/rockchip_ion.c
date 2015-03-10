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
#include <linux/sizes.h>
#include <linux/rockchip_ion.h>
#include <sofia/vvpu_vbpipe.h>

#include "../ion_priv.h"

static struct ion_device *idev;
static struct ion_heap **heaps;

#define MAX_ION_HEAP	(10)

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
		.id = ION_HEAP_TYPE_SECURE,
		/*.type	= ION_HEAP_TYPE_SECURE,*/
		.type	= ION_HEAP_TYPE_DMA,
		.name	= "secured-heap",
	},
};

static u64 ion_dmamask = DMA_BIT_MASK(32);

struct device rk_ion_cma_dev = {
	.dma_mask = &ion_dmamask,
	.init_name = "rk_ion_cma",
};

static int rk_ion_populate_heap(struct ion_platform_heap *heap)
{
	unsigned int i;
	int ret = -EINVAL;
	unsigned int len = ARRAY_SIZE(ion_heap_meta);

	for (i = 0; i < len; ++i) {
		if (ion_heap_meta[i].id == heap->id) {
			heap->name = ion_heap_meta[i].name;
			heap->type = ion_heap_meta[i].type;
			if (heap->id == ION_HEAP_TYPE_DMA)
				heap->priv = &rk_ion_cma_dev;
			ret = 0;
			break;
		}
	}
	return ret;
}

static int rk_ion_get_phys(struct ion_client *client,
			      unsigned long arg)
{
	struct ion_phys_data data;
	struct ion_handle *handle;
	int ret;

	if (is_compat_task()) {
		/* TODO: add compat here */
		return -EFAULT;
	} else {
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
	}
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

	dev = ion_struct_device_from_client(client);
	dev_info(dev, "sofia_ion_secure_alloc()\n");
	if (is_compat_task()) {
		/* TODO: add compat here */
		return -EFAULT;
	} else {
		data = kmalloc(sizeof(struct ion_phys_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		if (copy_from_user(data, (void __user *)arg,
					sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
	}

	/* call into secure VM to allocate a secure video buffer */
	memset(&vvpu_cmd, 0, sizeof(cmd));

	vvpu_cmd.payload[0] = VVPU_VTYPE_MEM;
	vvpu_cmd.payload[1] = VVPU_VOP_MEM_ALLOC;
	vvpu_cmd.payload[2] = 0;
	vvpu_cmd.payload[3] = 0;
	vvpu_cmd.payload[4] = data->size;
	vvpu_cmd.payload[5] = 0;

	/* execute command */
	vvpu_ret = vvpu_call(dev, &vvpu_cmd);

	if (vvpu_ret == 0)
		dev_err(dev, "error allocating secure memory\n");
	else {
		data->size = (unsigned int) vvpu_cmd.payload[4];
		data->phys = (unsigned int) vvpu_cmd.payload[5];

		dev_info(dev, "ion_alloc_secure() 0x%lx / %lu\n",
			data->phys, data->size);
	}

	if (is_compat_task()) {
		/* TODO: add compat here */
		return -EFAULT;
	} else {
		if (copy_to_user((void __user *) arg, data, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}

		kfree(data);
	}

	return 0;

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

	dev = ion_struct_device_from_client(client);
	dev_info(dev, "sofia_ion_secure_free()\n");
	if (is_compat_task()) {
		/* TODO: add compat here */
		return -EFAULT;
	} else {
		data = kmalloc(sizeof(struct ion_phys_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
	}

	/* call into secure VM to allocate a secure video buffer */
	memset(&vvpu_cmd, 0, sizeof(cmd));

	vvpu_cmd.payload[0] = VVPU_VTYPE_MEM;
	vvpu_cmd.payload[1] = VVPU_VOP_MEM_FREE;
	vvpu_cmd.payload[2] = 0;
	vvpu_cmd.payload[3] = 0;
	vvpu_cmd.payload[4] = data->size;
	vvpu_cmd.payload[5] = data->phys;

	/* execute command */
	vvpu_ret = vvpu_call(dev, &vvpu_cmd);

	if (vvpu_ret == 0)
		dev_err(dev, "error freeing secure memory\n");

	/* leave data.size and data.addr alone */
	if (is_compat_task()) {
		/* TODO: add compat here */
		return -EFAULT;
	} else {
		if (copy_to_user((void __user *) arg, data, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
		kfree(data);
	}

	return 0;
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

static int __init rk_ion_find_heap(unsigned long node,
				      const char *uname,
				      int depth,
				      void *data)
{
	const __be32 *prop;
	unsigned long len;
	struct ion_platform_heap *heap;
	struct ion_platform_data *pdata =
			(struct ion_platform_data *)data;

	if (!pdata) {
		pr_err("ion heap has no platform data\n");
		return -EINVAL;
	}

	if (pdata->nr >= MAX_ION_HEAP) {
		pr_err("ion heap is too much\n");
		return -EINVAL;
	}

	prop = of_get_flat_dt_prop(node, "rockchip,ion_heap", &len);
	if (!prop || (len != sizeof(unsigned long)))
		return 0;

	heap = &pdata->heaps[pdata->nr++];
	heap->base = 0;
	heap->size = 0;
	heap->align = 0;
	heap->id = be32_to_cpu(prop[0]);
	rk_ion_populate_heap(heap);

	prop = of_get_flat_dt_prop(node, "reg", &len);
	if (prop && (len >= 2 * sizeof(unsigned long))) {
		heap->base = be32_to_cpu(prop[0]);
		heap->size = be32_to_cpu(prop[1]);
		heap->align = SZ_1M;
	}
	return 0;
}

static int rk_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata;
	int err;
	int i;

	err = device_register(&rk_ion_cma_dev);
	if (err) {
		pr_err("Could not register %s\n",
		       dev_name(&rk_ion_cma_dev));
		return err;
	}

	if (pdev->dev.of_node) {
		pdata = &ion_pdata;
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	} else {
		pdata = pdev->dev.platform_data;
	}

	heaps = kcalloc(pdata->nr, sizeof(*heaps), GFP_KERNEL);

	idev = ion_device_create(rk_custom_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < pdata->nr; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

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
	if (of_scan_flat_dt(rk_ion_find_heap, (void *)&ion_pdata))
		pr_err("%s: Couldn't find ion heap\n", __func__);

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
