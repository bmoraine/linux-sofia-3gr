/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_gem.h>
#include <drm/drm_vma_manager.h>

#include <linux/dma-attrs.h>
#include <linux/dma-buf.h>

#include <linux/rockchip_ion.h>
#include <linux/rockchip_iovmm.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"

static int rockchip_gem_alloc_buf(struct rockchip_gem_object *rk_obj,
				  bool alloc_kmap)
{
	struct drm_gem_object *obj = &rk_obj->base;
	struct drm_device *drm = obj->dev;
	struct rockchip_drm_private *priv = drm->dev_private;

	rk_obj->handle = ion_alloc(priv->ion_client, (size_t)obj->size,
				   0, ION_HEAP_SYSTEM_MASK, 0);

	if (!rk_obj->handle) {
		DRM_ERROR("failed to allocate %#x byte dma buffer", obj->size);
		return -ENOMEM;
	}

	if (alloc_kmap)
		rk_obj->kvaddr = ion_map_kernel(priv->ion_client,
						rk_obj->handle);
	else
		rk_obj->kvaddr = 0;

	return 0;
}

static void rockchip_gem_free_buf(struct rockchip_gem_object *rk_obj)
{
	struct drm_gem_object *obj = &rk_obj->base;
	struct drm_device *drm = obj->dev;
	struct rockchip_drm_private *priv = drm->dev_private;

	ion_free(priv->ion_client, rk_obj->handle);
}

static int rockchip_drm_gem_object_mmap(struct drm_gem_object *obj,
					struct vm_area_struct *vma)

{
	int ret;
	struct rockchip_gem_object *rk_obj = to_rockchip_obj(obj);
	struct drm_device *drm = obj->dev;
	struct rockchip_drm_private *priv = drm->dev_private;
	struct dma_buf *dma_buf = NULL;

	if (IS_ERR_OR_NULL(rk_obj->handle)) {
		DRM_ERROR("failed to get ion handle:%ld\n",
			  PTR_ERR(rk_obj->handle));
		return -ENOMEM;
	}

	dma_buf = ion_share_dma_buf(priv->ion_client, rk_obj->handle);
	if (IS_ERR_OR_NULL(dma_buf)) {
		DRM_ERROR("get ion share dma buf failed\n");
		return -ENOMEM;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = dma_buf_mmap(dma_buf, vma, 0);

	dma_buf_put(dma_buf);

	return ret;
}

int rockchip_gem_mmap_buf(struct drm_gem_object *obj,
			  struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret)
		return ret;

	return rockchip_drm_gem_object_mmap(obj, vma);
}

/* drm driver mmap file operations */
int rockchip_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_gem_object *obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;

	obj = vma->vm_private_data;

	return rockchip_drm_gem_object_mmap(obj, vma);
}

struct rockchip_gem_object *
	rockchip_gem_create_object(struct drm_device *drm, unsigned int size,
				   bool alloc_kmap)
{
	struct rockchip_gem_object *rk_obj;
	struct drm_gem_object *obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	rk_obj = kzalloc(sizeof(*rk_obj), GFP_KERNEL);
	if (!rk_obj)
		return ERR_PTR(-ENOMEM);

	obj = &rk_obj->base;

	drm_gem_private_object_init(drm, obj, size);

	ret = rockchip_gem_alloc_buf(rk_obj, alloc_kmap);
	if (ret)
		goto err_free_rk_obj;

	return rk_obj;

err_free_rk_obj:
	kfree(rk_obj);
	return ERR_PTR(ret);
}

/*
 * rockchip_gem_free_object - (struct drm_driver)->gem_free_object callback
 * function
 */
void rockchip_gem_free_object(struct drm_gem_object *obj)
{
	struct rockchip_gem_object *rk_obj;

	drm_gem_free_mmap_offset(obj);

	rk_obj = to_rockchip_obj(obj);

	rockchip_gem_free_buf(rk_obj);

	kfree(rk_obj);
}

/*
 * rockchip_gem_create_with_handle - allocate an object with the given
 * size and create a gem handle on it
 *
 * returns a struct rockchip_gem_object* on success or ERR_PTR values
 * on failure.
 */
static struct rockchip_gem_object *
rockchip_gem_create_with_handle(struct drm_file *file_priv,
				struct drm_device *drm, unsigned int size,
				unsigned int *handle)
{
	struct rockchip_gem_object *rk_obj;
	struct drm_gem_object *obj;
	int ret;

	rk_obj = rockchip_gem_create_object(drm, size, false);
	if (IS_ERR(rk_obj))
		return ERR_CAST(rk_obj);

	obj = &rk_obj->base;

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, obj, handle);
	if (ret)
		goto err_handle_create;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(obj);

	return rk_obj;

err_handle_create:
	rockchip_gem_free_object(obj);

	return ERR_PTR(ret);
}

int rockchip_gem_dumb_map_offset(struct drm_file *file_priv,
				 struct drm_device *dev, uint32_t handle,
				 uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		return -EINVAL;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto out;

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
	DRM_DEBUG_KMS("offset = 0x%llx\n", *offset);

out:
	drm_gem_object_unreference_unlocked(obj);

	return 0;
}

/*
 * rockchip_gem_dumb_create - (struct drm_driver)->dumb_create callback
 * function
 *
 * This aligns the pitch and size arguments to the minimum required. wrap
 * this into your own function if you need bigger alignment.
 */
int rockchip_gem_dumb_create(struct drm_file *file_priv,
			     struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	struct rockchip_gem_object *rk_obj;
	int min_pitch = DIV_ROUND_UP(args->width * args->bpp, 8);

	/*
	 * align to 64 bytes since Mali requires it.
	 */
	min_pitch = ALIGN(min_pitch, 64);

	if (args->pitch < min_pitch)
		args->pitch = min_pitch;

	if (args->size < args->pitch * args->height)
		args->size = args->pitch * args->height;

	rk_obj = rockchip_gem_create_with_handle(file_priv, dev, args->size,
						 &args->handle);

	return PTR_ERR_OR_ZERO(rk_obj);
}

/*
 * Allocate a sg_table for this GEM object.
 * Note: Both the table's contents, and the sg_table itself must be freed by
 *       the caller.
 * Returns a pointer to the newly allocated sg_table, or an ERR_PTR() error.
 */
struct sg_table *rockchip_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct drm_device *drm = obj->dev;
	struct rockchip_gem_object *rk_obj = to_rockchip_obj(obj);
	struct rockchip_drm_private *priv = drm->dev_private;
	struct scatterlist *sg, *ion_sg;
	struct sg_table *sgt;
	struct sg_table *ion_sgt;
	int i;

	ion_sgt = ion_sg_table(priv->ion_client, rk_obj->handle);
	if (!ion_sgt)
		return ERR_PTR(-ENOMEM);
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);
	if (sg_alloc_table(sgt, ion_sgt->nents, GFP_KERNEL))
		return ERR_PTR(-ENOMEM);

	for (i = 0, sg = sgt->sgl, ion_sg = ion_sgt->sgl; i < ion_sgt->nents;
		i++, sg = sg_next(sg), ion_sg = sg_next(ion_sg))
		memcpy(sg, ion_sg, sizeof(*sg));

	return sgt;
}

void *rockchip_gem_prime_vmap(struct drm_gem_object *obj)
{
	struct rockchip_gem_object *rk_obj = to_rockchip_obj(obj);

	return rk_obj->kvaddr;
}

void rockchip_gem_prime_vunmap(struct drm_gem_object *obj, void *vaddr)
{
	/* Nothing to do */
}
