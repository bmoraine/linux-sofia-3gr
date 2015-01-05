/*
 * drivers/staging/android/ion/rockchip/rockchip_ion.c
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
#include <linux/dma-buf.h>
#include <linux/dma-contiguous.h>
#include <linux/memblock.h>
#include <linux/of_gpio.h>
#include <linux/of_fdt.h>
#include <linux/rockchip_ion.h>

#include "../ion_priv.h"

static struct ion_device *idev;
static int num_heaps;
static struct ion_heap **heaps;

struct ion_heap_desc {
	unsigned int id;
	enum ion_heap_type type;
	const char *name;
};

#define MAX_ION_HEAP	(10)

static struct ion_platform_heap ion_plat_heap[MAX_ION_HEAP];

struct ion_platform_data ion_pdata = {
	.nr = 0,
	.heaps = ion_plat_heap,
};

static struct ion_heap_desc ion_heap_meta[] = {
	{
		.id	= ION_VMALLOC_HEAP_ID,
		.type	= ION_HEAP_TYPE_SYSTEM,
		.name	= ION_VMALLOC_HEAP_NAME,
	},
	{
		.id	= ION_CMA_HEAP_ID,
		.type	= ION_HEAP_TYPE_DMA,
		.name	= ION_CMA_HEAP_NAME,
	},
	{
		.id	= ION_CARVEOUT_HEAP_ID,
		.type	= ION_HEAP_TYPE_CARVEOUT,
		.name	= ION_CARVEOUT_HEAP_NAME,
	},
};

struct device rk_ion_cma_dev = {
	.coherent_dma_mask = DMA_BIT_MASK(32),
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
			ret = 0;
			break;
		}
	}

	if (ret)
		pr_err("fail to populate heap %d", ret);

	return ret;
}

static long rk_custom_ioctl(struct ion_client *client,
			       unsigned int cmd,
			       unsigned long arg)
{
	struct ion_handle *handle;
	struct ion_phys_data pdata;
	struct ion_share_id_data sdata;
	struct dma_buf *dmabuf;
	int fd = 0;
	int ret = 0;

	switch (cmd) {
	case ION_IOC_GET_PHYS:
		if (copy_from_user(&pdata, (void __user *)arg,
				   sizeof(struct ion_phys_data)))
			return -EFAULT;
		handle = ion_handle_get_by_id(client, pdata.handle);
		if (IS_ERR(handle))
			return PTR_ERR(handle);
		ret = ion_phys(client, handle, &pdata.phys,
			       (size_t *)&pdata.size);
		ion_handle_put(handle);
		if (ret < 0)
			return ret;
		if (copy_to_user((void __user *)arg, &pdata,
				 sizeof(struct ion_phys_data)))
			return -EFAULT;
		break;
	case ION_IOC_GET_SHARE_ID:
		if (copy_from_user(&sdata, (void __user *)arg,
				   sizeof(struct ion_share_id_data)))
			return -EFAULT;
		dmabuf = dma_buf_get(sdata.fd);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		sdata.id = (unsigned int)dmabuf;
		if (copy_to_user((void __user *)arg, &sdata,
				 sizeof(struct ion_share_id_data)))
			return -EFAULT;
		break;
	case ION_IOC_SHARE_BY_ID:
		if (copy_from_user(&sdata, (void __user *)arg,
				   sizeof(struct ion_share_id_data)))
			return -EFAULT;
		fd = dma_buf_fd((struct dma_buf *)sdata.id, O_CLOEXEC);
		if (fd < 0)
			return fd;
		sdata.fd = fd;
		if (copy_to_user((void __user *)arg, &sdata,
				 sizeof(struct ion_share_id_data)))
			return -EFAULT;
		break;
	case ION_IOC_CLEAN_CACHES:
	case ION_IOC_INV_CACHES:
	case ION_IOC_CLEAN_INV_CACHES:
		break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static int __init rk_ion_find_heap(unsigned long node, const char *uname,
				      int depth, void *data)
{
	const __be32 *prop;
	unsigned long len;
	struct ion_platform_heap *heap;
	struct ion_platform_data *pdata = (struct ion_platform_data *)data;

	if (!pdata) {
		pr_err("ion heap has no platform data\n");
		return -EINVAL;
	}

	if (pdata->nr >= MAX_ION_HEAP) {
		pr_err("ion heap is too much\n");
		return -EINVAL;
	}

	if (!of_flat_dt_is_compatible(node, "rockchip,ion-heap"))
		return 0;

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
		if (len == 3 * sizeof(unsigned long))
			heap->align = be32_to_cpu(prop[2]);
	}

	pr_info("ion heap(%s): base(%lx) size(%x) align(%lx)\n",
		heap->name, heap->base, heap->size, heap->align);
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

	num_heaps = pdata->nr;

	heaps = kcalloc(num_heaps, sizeof(*heaps), GFP_KERNEL);

	idev = ion_device_create(rk_custom_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

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
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	kfree(heaps);
	return err;
}

static int rk_ion_remove(struct platform_device *pdev)
{
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(idev);
	for (i = 0; i < num_heaps; i++)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}

static const struct of_device_id rk_ion_match[] = {
	{ .compatible = "rockchip,ion", },
	{}
};

MODULE_DEVICE_TABLE(of, rk_ion_match);

static struct platform_driver ion_driver = {
	.probe = rk_ion_probe,
	.remove = rk_ion_remove,
	.driver = {
		.name = "ion-rk",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rk_ion_match),
	},
};

static int __init ion_init(void)
{
	if (of_scan_flat_dt(rk_ion_find_heap, (void *)&ion_pdata))
		pr_err("%s: Couldn't find ion heap\n", __func__);

	return platform_driver_register(&ion_driver);
}

static void __exit ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

subsys_initcall(ion_init);
module_exit(ion_exit);

struct ion_client *rockchip_ion_client_create(const char *name)
{
	if (idev == NULL) {
		pr_err("rockchip_ion_client_create idev is NULL\n");
		return NULL;
	}

	return ion_client_create(idev, name);
}
EXPORT_SYMBOL_GPL(rockchip_ion_client_create);
