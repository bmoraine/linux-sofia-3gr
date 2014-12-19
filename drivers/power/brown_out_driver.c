/**
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#define DRIVER_NAME "ag6x0_brown_out"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/power/battery_id.h>

/* Masks and offsets for BAT_UV_DET register bits */
#define BAT_UV_DET_OFFSET	0xC20
/* EnablePMUShutdown */
#define BAT_UV_DET_EPSD_O	(25)

/* EnableSIMShutdown */
#define BAT_UV_DET_ESSD_O	(24)

/* BUV Detection Level */
#define BAT_UV_DET_BULVL_O	(1)
#define BAT_UV_DET_BULVL_M	(0xf)
#define BUV_DET_LEVEL_3050MV	(0xb)

/* Buv Enable */
#define BAT_UV_DET_EBUV_O	(0)

/* Masks and offsets for BAT_SUPERVISION register bits */
#define BAT_SUPERV_OFFSET	0xC04
/* SYSOFF_Debounce */
#define BAT_SUPERV_SOFFDB_O	(6)
#define BAT_SUPERV_SOFFDB_M	(0x7)
#define SYSOFF_DEBOUNCE_1MS	(0x4)

/* SystemOff_Level */
#define BAT_SUPERV_SOFFLV_O	(2)
#define BAT_SUPERV_SOFFLV_M	(0xf)
#define SYSTEMOFF_LEVEL_2900MV	(0xc)

/* SYSOFF_Enable */
#define BAT_SUPERV_SOFFEN_O	(0)
#define BAT_SUPERV_SOFFEN_M	(0x3)
#define SYSOFF_ENABLE_ALL	(0x3)

/* Masks and offsets for BAT_SUPERVISION register bits */
#define BAT_SUPERV_WR_OFFSET	0xC10
/* Write Strobe to accept written values */
#define BAT_SUPERV_WR_WS_O	(0)

struct brown_out_drv_data {
	struct idi_peripheral_device *p_idi_device;

	struct device_state_pm_state *pm_state_en;
	struct device_state_pm_state *pm_state_dis;

	struct resource *pmu_res;
};

static struct brown_out_drv_data bnt_drv_data;

static unsigned brown_out_pmu_ioread(struct brown_out_drv_data *hal_data,
					unsigned offset)
{
	int ret = 0;
	unsigned reg;
	unsigned addr;

	if (unlikely(hal_data == NULL))
		BUG();

	addr = hal_data->pmu_res->start + offset;

	ret = idi_client_ioread(hal_data->p_idi_device, addr, &reg);
	if (ret)
		BUG();

	return reg;
}

static void brown_out_pmu_iowrite(struct brown_out_drv_data *hal_data,
					unsigned offset,
					unsigned data)
{
	int ret = 0;
	unsigned addr;

	if (unlikely(hal_data == NULL))
		BUG();

	addr = hal_data->pmu_res->start + offset;

	ret = idi_client_iowrite(hal_data->p_idi_device, addr, data);
	if (ret)
		BUG();
}



/**
 * brown_out_drv_probe - Initialises the driver, when the device has been found.
 */
static int __init brown_out_drv_probe(struct idi_peripheral_device *ididev,
						const struct idi_device_id *id)
{
	struct resource *pmu_res;
	u32 bat_uv_det, bat_supervision, bat_supervision_wr;
	int ret;

	/* Store platform device in static instance. */
	bnt_drv_data.p_idi_device = ididev;

	ret =  idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
			__func__);
		return -EINVAL;
	}

	bnt_drv_data.pm_state_en =
		idi_peripheral_device_pm_get_state_handler(ididev, "enable");
	if (bnt_drv_data.pm_state_en == NULL) {
		pr_err("%s: Unable to get handler for PM state 'enable'!\n",
			__func__);
		return -EINVAL;
	}

	bnt_drv_data.pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(ididev, "disable");
	if (bnt_drv_data.pm_state_dis == NULL) {
		pr_err("%s: Unable to get handler for PM state 'disable'!\n",
			__func__);
		return -EINVAL;
	}

	pr_info("%s: Getting PM state handlers: OK\n", __func__);

	pmu_res =
		idi_get_resource_byname(&ididev->resources, IORESOURCE_MEM,
					"pmu");
	if (pmu_res == NULL) {
		pr_err("getting PMU resource failed!\n");
		return -EINVAL;
	}

	bnt_drv_data.pmu_res = pmu_res;

	ret = idi_set_power_state(bnt_drv_data.p_idi_device,
					bnt_drv_data.pm_state_en, true);

	if (ret) {
		pr_err("%s: setting PM state '%s' failed!\n", __FILE__,
			bnt_drv_data.pm_state_en->name);
		ret = -EIO;
		goto set_pm_state_fail;
	}

	/* Configuring Battery Undervoltage Detection to:
	EPSD (EnablePMUShutdown) =enable
	ESSD (EnableSIMShutdown) =enable
	BULVL (BUV Detection Level) =3.05V
	EBUV (BUVEnable) =enable */
	bat_uv_det = brown_out_pmu_ioread(&bnt_drv_data, BAT_UV_DET_OFFSET);

	bat_uv_det |= (1 << BAT_UV_DET_EPSD_O) |
		(1 << BAT_UV_DET_ESSD_O) | (1 << BAT_UV_DET_EBUV_O);

	bat_uv_det &= ~(BAT_UV_DET_BULVL_M << BAT_UV_DET_BULVL_O);
	bat_uv_det |= (BUV_DET_LEVEL_3050MV << BAT_UV_DET_BULVL_O);

	brown_out_pmu_iowrite(&bnt_drv_data, BAT_UV_DET_OFFSET, bat_uv_det);

	/* Configuring Battery Supervision to:
	SYSOFFDB (System Off Debounce time) =1ms
	SYSOFFLV (System Off Level) =2.95V
	SYSOFFEN (System Off Enable) =enable all */
	bat_supervision = brown_out_pmu_ioread(&bnt_drv_data,
						BAT_SUPERV_OFFSET);

	bat_supervision &= ~(BAT_SUPERV_SOFFDB_M << BAT_SUPERV_SOFFDB_O |
				BAT_SUPERV_SOFFLV_M << BAT_SUPERV_SOFFLV_O |
				BAT_SUPERV_SOFFEN_M << BAT_SUPERV_SOFFEN_O);

	bat_supervision |= (SYSOFF_DEBOUNCE_1MS << BAT_SUPERV_SOFFDB_O);
	bat_supervision |= (SYSTEMOFF_LEVEL_2900MV << BAT_SUPERV_SOFFLV_O);
	bat_supervision |= (SYSOFF_ENABLE_ALL << BAT_SUPERV_SOFFEN_O);

	brown_out_pmu_iowrite(&bnt_drv_data,
				BAT_SUPERV_OFFSET, bat_supervision);


	/* Issuing Battery Supervision Write Strobe */
	bat_supervision_wr = brown_out_pmu_ioread(&bnt_drv_data,
						BAT_SUPERV_WR_OFFSET);

	bat_supervision_wr |= (1 << BAT_SUPERV_WR_WS_O);
	brown_out_pmu_iowrite(&bnt_drv_data,
				BAT_SUPERV_WR_OFFSET, bat_supervision_wr);

	ret = idi_set_power_state(bnt_drv_data.p_idi_device,
					bnt_drv_data.pm_state_dis, false);

	if (ret) {
		pr_err("%s: setting PM state '%s' failed!\n", __FILE__,
			bnt_drv_data.pm_state_dis->name);
	}

	return 0;

set_pm_state_fail:

	return ret;
}

/**
 * brown_out_drv_remove - Release allocated resources.
 */
static int __exit brown_out_drv_remove(struct idi_peripheral_device *ididev)
{

	return 0;
}

static struct idi_peripheral_driver brown_out_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = NULL,
	},

	.p_type = IDI_BNT,
	.probe = brown_out_drv_probe,
	.remove = brown_out_drv_remove,
};

static int __init brown_out_drv_init(void)
{
	int ret;

	ret = idi_register_peripheral_driver(&brown_out_driver);
	return ret;
}

static void __exit brown_out_drv_exit(void)
{
	idi_unregister_peripheral_driver(&brown_out_driver);
}

late_initcall(brown_out_drv_init);
module_exit(brown_out_drv_exit);

MODULE_DESCRIPTION("AGOLD6x0 Brown Out Driver");
MODULE_LICENSE("GPL v2");
