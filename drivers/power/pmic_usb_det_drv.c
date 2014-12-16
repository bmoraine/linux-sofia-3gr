/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
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
 * with this program; If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define DRIVER_NAME "pmic_usb_det_drv"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/usb/otg.h>
#include <linux/power_supply.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <sofia/vmm_pmic.h>
#include <sofia/vmm_pmic-ext.h>

#define BYTE_MASK 0xFF

/* PMIC DEV slave addresses */
#define DEV1 0x4E
#define DEV3 0x5E

/* PMIC DEV1 registers */
#define CHGRIRQ0 0x09
#define MCHGRIRQ0 0x17
#define CTYPE_O 4

#define CHGRIRQ1 0x0A
#define MCHGRIRQ1 0x18
#define VBUSDET_O 0

#define IRQLVL1 0x02
#define MIRQLVL1 0x0E
#define CHGR_O 5

#define SPWRSRC 0x20
#define SVBUSDET_O 0

/* PMIC DEV3 registers */
#define USBSRCDETSTATUS0 0x029
#define USBSRCDETRSLT_O 2
#define USBSRCDETRSLT_W 4
#define SUSBHWDET_O 0
#define SUSBHWDET_W 2

#define VBUSDETCTRL 0x1D
#define VBUSDETEN_O 2

#define USBPHYCTRL 0x08
#define USBPHYRSTB_O 0
#define CTYP_DIS_O 3

#define set_field(_reg, _offset, _size, _val)\
do {\
	u8 _tmp;\
	_tmp = (_val) & ((1 << (_size)) - 1);\
	_reg &= ~(((1 << (_size)) - 1) << (_offset));\
	_reg |= _tmp << (_offset);\
} while (0)

#define get_field(_reg, _offset, _size)\
({\
	u8 _val;\
	_val = ((_reg) >> (_offset)) & ((1 << (_size)) - 1);\
	_val;\
})

enum {
	VBUS_OFF = 0,
	VBUS_ON,

	CLEAR = 0,
	SET = 1,

	USB_UNKNOWN = 0,
	USB_SDP = 1,
	USB_DCP = 2,
	USB_CDP = 3,
	USB_ACA = 4,
	USB_SE1 = 5,
	USB_MHL = 6,
	USB_FLOAT = 7,
	USB_OTHER = 8,
};

/**
 * PMIC charger detection driver internal structure
 *
 * @otg_handle		USB OTG handle used for sending notifications
 * @cable_props		Power supply class cable property (notification payload)
 * @vbus_state		VBUS presence status
 * @cable_type		USB cable type detected
 * @irq_vbusdet		IRQ number for VBUS detection
 * @irq_ctyp		IRQ number for USB cable type detection
 */
struct pmic_usb_det_data {
	struct usb_phy *otg_handle;
	struct power_supply_cable_props cable_props;
	int vbus_state;
	int cable_type;
	int irq_vbusdet;
	int irq_ctyp;
	struct device_pm_platdata *pm_platdata;
};

/**
 * PMIC register read access wrapper
 *
 * @dev		PMIC device slave address
 * @reg		PMIC register offset
 * @p_val	Pointer to read output buffer
 * return	0 on success, otherwise error code
 */
static int pmic_reg_read(u32 dev, u32 reg, u8 *p_val)
{
	u32 vmm_addr, data;
	int ret;

	if (!p_val)
		return -EINVAL;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	ret = vmm_pmic_reg_read(vmm_addr, &data);
	data &= BYTE_MASK;
	*p_val = (u8) data;

	return ret;
}

/**
 * PMIC register write access wrapper
 *
 * @dev		PMIC device slave address
 * @reg		PMIC register offset
 * @val		Data to be written
 * return	0 on success, otherwise error code
 */
static int pmic_reg_write(u32 dev, u32 reg, u8 val)
{
	u32 vmm_addr, data;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);
	data = (u32) val;

	return vmm_pmic_reg_write(vmm_addr, data);
}

/**
 * PMIC register field atomic set wrapper (read-modify-write)
 *
 * @dev		PMIC device slave address
 * @reg		PMIC register offset
 * @mask	Register field mask
 * @val		Date to be written
 * return	0 on success, otherwise error code
 */
static int pmic_reg_set(u32 dev, u32 reg, u8 mask, u8 val)
{
	u32 vmm_addr;

	vmm_addr = ((dev & BYTE_MASK) << 24) | (reg & BYTE_MASK);

	return pmic_reg_set_field(vmm_addr, mask, val);
}

/**
 * Interface to enable/disable PMIC USB type detection
 *
 * @enable	Whether to enable/disable USB type detection
 * return	0 on success, otherwise error code
 */
static int pmic_usb_enable_usb_det(bool enable)
{
	int ret;
	u8 reg;

	ret = pmic_reg_read(DEV3, USBPHYCTRL, &reg);
	if (ret) {
		pr_err("%s - fail to read DEV3 USBPHYCTRL\n", __func__);
		return ret;
	}

	if (enable) {
		set_field(reg, USBPHYRSTB_O, 1, CLEAR);
		set_field(reg, CTYP_DIS_O, 1, CLEAR);
	} else {
		set_field(reg, USBPHYRSTB_O, 1, SET);
		set_field(reg, CTYP_DIS_O, 1, SET);
	}
	ret = pmic_reg_write(DEV3, USBPHYCTRL, reg);
	if (ret) {
		pr_err("%s - fail to write DEV3 USBPHYCTRL\n", __func__);
		return ret;
	}

	return 0;
}

/**
 * Detects VBUS presence status and notifies USB driver
 *
 * @pdata	Pointer to the charger type detection driver data structure
 * return	0 on success, otherwise error code
 */
static int pmic_usb_vbus_det(struct pmic_usb_det_data *pdata)
{
	u8 reg;
	int ret;

	if (!pdata)
		return -EINVAL;

	ret = pmic_reg_read(DEV1, SPWRSRC, &reg);
	if (ret) {
		pr_err("%s - fail to read DEV1 SPWRSRC\n", __func__);
		return ret;
	}

	if (get_field(reg, SVBUSDET_O, 1)) {
		pr_info("VBUS connected\n");

		/* Enable the detection circuit and start CTYP detection */
		ret = pmic_usb_enable_usb_det(true);
		if (ret) {
			pr_err("%s - fail to enable USB detection\n", __func__);
			return ret;
		}

		pdata->vbus_state = VBUS_ON;
	} else {
		pr_info("VBUS disconnected\n");
		pdata->cable_props.chrg_evt =
			POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		pdata->cable_props.chrg_type = pdata->cable_type;
		pdata->cable_props.ma = 0;

		/* Disable the internal detection circuit */
		ret = pmic_usb_enable_usb_det(false);
		if (ret) {
			pr_err("%s - fail to disable USB detection\n",
								__func__);
			return ret;
		}
		pdata->vbus_state = VBUS_OFF;

		/* Notify PSY about charger removal */
		atomic_notifier_call_chain(&pdata->otg_handle->notifier,
					USB_EVENT_NONE, &pdata->cable_props);
	}

	return 0;
}

/**
 * Detects USB cable type and notifies USB driver
 *
 * @pdata	Pointer to the charger type detection driver data structure
 * return	0 on success, otherwise error code
 */
static int pmic_usb_cable_det(struct pmic_usb_det_data *pdata)
{
	u8 reg, val;
	int ret;
	int *p_cable_type;

	if (!pdata)
		return -EINVAL;

	p_cable_type = &pdata->cable_type;

	ret = pmic_reg_read(DEV3, USBSRCDETSTATUS0, &reg);
	if (ret) {
		pr_err("%s - fail to read DEV3 USBSRCDETSTATUS0\n", __func__);
		return ret;
	}

	/* Check for detection status */
	val = get_field(reg, SUSBHWDET_O, SUSBHWDET_W);
	if (val != 0x2) {
		pr_err("Detection not completed, status=%d\n", val);
		return -EIO;
	}

	/* Check for detection result */
	val = get_field(reg, USBSRCDETRSLT_O, USBSRCDETRSLT_W);
	switch (val) {
	case USB_UNKNOWN:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pr_info("USB cable type - Unknown\n");
		break;
	case USB_SDP:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
		pr_info("USB cable type - SDP\n");
		break;
	case USB_DCP:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		pr_info("USB cable type - DCP\n");
		break;
	case USB_CDP:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		pr_info("USB cable type - CDP\n");
		break;
	case USB_ACA:
		/* ACA, report it as Unknown at this moment */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pr_info("USB cable type - ACA, report it as Unknown\n");
		break;
	case USB_SE1:
		/* SE1, report it as DCP at this moment */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		pr_info("USB cable type - SE1, report it as DCP\n");
		break;
	case USB_MHL:
		/* MHL, report it as NONE at this moment */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pr_info("USB cable type - MHL, report it as Unknown\n");
		break;
	case USB_FLOAT:
		/* FLOATING, report it as NONE */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pr_info("USB cable type - Floating, report it as Unknown\n");
		break;

	default:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pr_info("USB cable type - Unknown\n");
		break;
	}

	/* Send notification to USB driver */
	pdata->cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
	pdata->cable_props.chrg_type = *p_cable_type;
	pdata->cable_props.ma = 0;

	/* Disable the internal detection circuit */
	ret = pmic_usb_enable_usb_det(false);
	if (ret) {
		pr_err("%s - fail to disable USB detection\n", __func__);
		return ret;
	}

	/* Notify USB driver about the type detection result */
	atomic_notifier_call_chain(&pdata->otg_handle->notifier,
					USB_EVENT_VBUS,
					&pdata->cable_props);

	return 0;
}

/**
 * Common interrupt handler for VBUS and USB type detection
 *
 * @irq		IRQ number
 * @pd		Pointer to the client specified interrupt data
 * return	IRQ_HANDLED if the interrupt is handled, otherwise IRQ_NONE
 */
static irqreturn_t pmic_usb_det_cb(int irq, void *pd)
{
	irqreturn_t irqret = IRQ_NONE;
	struct pmic_usb_det_data *pdata = (struct pmic_usb_det_data *)pd;

	if (!pdata) {
		pr_err("%s: fail to get platform data\n", __func__);
		return irqret;
	}

	if (irq == pdata->irq_vbusdet) {
		pmic_usb_vbus_det(pdata);
		irqret = IRQ_HANDLED;
	} else if (irq == pdata->irq_ctyp) {
		pmic_usb_cable_det(pdata);
		irqret = IRQ_HANDLED;
	}

	return irqret;
}


/**
 * Configures PMIC interrupts for VBUS and USB type detection
 *
 * @pdata	Pointer to the charger type detection driver data structure
 * return	0 on success, otherwise error code
 */
static int pmic_usb_setup_det_irq(struct pmic_usb_det_data *pdata)
{
	int ret;
	u8 mask = 0;

	if (!pdata)
		return -EINVAL;

	if (!IS_ERR_VALUE(pdata->irq_vbusdet)) {
		ret = request_threaded_irq(pdata->irq_vbusdet, NULL,
					pmic_usb_det_cb,
					IRQF_SHARED | IRQF_ONESHOT,
					DRIVER_NAME, pdata);

		if (ret) {

			pr_err("%s: setup irq %d failed: %d\n", __func__,
					pdata->irq_vbusdet, ret);
			pdata->irq_vbusdet = -ENXIO;
			pdata->irq_ctyp = -ENXIO;
			return -EINVAL;
		}
	}

	if (!IS_ERR_VALUE(pdata->irq_ctyp)) {
		ret = request_threaded_irq(pdata->irq_ctyp, NULL,
					pmic_usb_det_cb,
					IRQF_SHARED | IRQF_ONESHOT,
					DRIVER_NAME, pdata);

		if (ret) {

			pr_err("%s: setup irq %d failed: %d\n", __func__,
					pdata->irq_ctyp, ret);
			ret = -EINVAL;
			goto setup_ctyp_irq_fail;
		}
	}

	/* Clear Pending CTYP interrupt */
	mask = 0;
	set_field(mask, CTYPE_O, 1, SET);
	ret = pmic_reg_write(DEV1, CHGRIRQ0, mask);
	if (ret) {
		pr_err("%s: fail to clear CTYP IRQ\n", __func__);
		goto unmask_or_clear_fail;
	}

	/* Clear Pending VBUSDET interrupt */
	mask = 0;
	set_field(mask, VBUSDET_O, 1, SET);
	ret = pmic_reg_write(DEV1, CHGRIRQ1, mask);
	if (ret) {
		pr_err("%s: fail to clear VBUSDET IRQ\n", __func__);
		goto unmask_or_clear_fail;
	}
	/* Unmask CTYP interrupts */
	mask = 0;
	set_field(mask, CTYPE_O, 1, SET);
	ret = pmic_reg_set(DEV1, MCHGRIRQ0, mask, 0);
	if (ret) {
		pr_err("%s: fail to umask CTYP IRQ\n", __func__);
		goto unmask_or_clear_fail;
	}

	/* Unmask VBUSDET interrupt */
	mask = 0;
	set_field(mask, VBUSDET_O, 1, SET);
	ret = pmic_reg_set(DEV1, MCHGRIRQ1, mask, 0);
	if (ret) {
		pr_err("%s: fail to umask VBUSDET IRQ\n", __func__);
		goto unmask_or_clear_fail;
	}

	/* Unmask LVL1 CHGR interrupt */
	mask = 0;
	set_field(mask, CHGR_O, 1, SET);
	ret = pmic_reg_set(DEV1, MIRQLVL1, mask, 0);
	if (ret) {
		pr_err("%s: fail to umask LVL1 CHGR IRQ\n", __func__);
		goto unmask_or_clear_fail;
	}

	/* Enable VBUSDETCTRL_REG:VBUSDETEN bit */
	mask = 0;
	ret = pmic_reg_read(DEV3, VBUSDETCTRL, &mask);
	if (ret) {
		pr_err("%s: fail to read DEV3 VBUSDETCTRL\n", __func__);
		goto unmask_or_clear_fail;
	}
	set_field(mask, VBUSDETEN_O , 1, SET);
	ret = pmic_reg_write(DEV3, VBUSDETCTRL, mask);
	if (ret) {
		pr_err("%s: fail to write DEV3 VBUSDETCTRL\n", __func__);
		goto unmask_or_clear_fail;
	}

	return 0;

unmask_or_clear_fail:
	free_irq(pdata->irq_ctyp, pdata);
setup_ctyp_irq_fail:
	pdata->irq_ctyp = -ENXIO;
	pdata->irq_vbusdet = -ENXIO;
	free_irq(pdata->irq_vbusdet, pdata);

	return ret;
}

static int pmic_usb_det_drv_probe(struct platform_device *pdev)
{
	struct pmic_usb_det_data *pdata;
	struct usb_phy *otg_handle;
	int ret;

	if (!pdev)
		return -EINVAL;

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct pmic_usb_det_data),
				GFP_KERNEL);

	if (!pdata) {
		pr_err("%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	pdata->vbus_state = -1;
	pdata->cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
	pdata->irq_ctyp = -ENXIO;
	pdata->irq_vbusdet = -ENXIO;

	platform_set_drvdata(pdev, pdata);

	/* Get USB phy */
	otg_handle = usb_get_phy(USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(otg_handle)) {
		pr_err("%s: fail to get OTG transceiver\n", __func__);
		return -EBUSY;
	}

	pdata->otg_handle = otg_handle;

	/* Get interrupt from device tree */
	pdata->irq_vbusdet = platform_get_irq_byname(pdev, "vbusdet");
	pdata->irq_ctyp = platform_get_irq_byname(pdev, "ctype");

	if (!pdata->irq_vbusdet || !pdata->irq_ctyp) {
		ret = -EBUSY;
		pr_err("%s: can't get irq, irq_vbusdet=%d, irq_ctyp=%d\n",
			__func__, pdata->irq_vbusdet, pdata->irq_ctyp);
		goto fail;
	}

	ret = pmic_usb_setup_det_irq(pdata);
	if (ret)
		goto fail;

	/* Disable the internal detection circuit */
	ret = pmic_usb_enable_usb_det(false);
	if (ret)
		goto fail;

	/* Read the VBUS presence status for initial update, and trigger
	USB type detection if applicable */
	ret = pmic_usb_vbus_det(pdata);
	if (ret)
		goto fail;

	return 0;

fail:
	usb_put_phy(otg_handle);

	return ret;
}

static int __exit pmic_usb_det_drv_remove(struct platform_device *pdev)
{
	struct pmic_usb_det_data *pdata;

	if (!pdev)
		return -EINVAL;

	pdata = platform_get_drvdata(pdev);

	if (!pdata) {
		pr_err("%s: fail to get platform data\n", __func__);
		return -EINVAL;
	}
	if (pdata->irq_vbusdet != -ENXIO)
		free_irq(pdata->irq_vbusdet, pdata);
	if (pdata->irq_ctyp != -ENXIO)
		free_irq(pdata->irq_ctyp, pdata);

	if (pdata->otg_handle)
		usb_put_phy(pdata->otg_handle);

	devm_kfree(&pdev->dev, pdata);

	return 0;
}

/**
 * Called when the system is attempting to suspend
 * @pdev	Pointer to the device.
 * returns	0
 */
static int pmic_usb_det_suspend(struct device *pdev)
{
	struct pmic_usb_det_data *pdata;

	if (!pdev)
		return -EINVAL;

	pdata = dev_get_drvdata(pdev);

	if (!pdata) {
		pr_err("%s: fail to get platform data\n", __func__);
		return -EINVAL;
	}

	if (device_may_wakeup(pdev)) {
		enable_irq_wake(pdata->irq_vbusdet);
		enable_irq_wake(pdata->irq_ctyp);
	}

	return 0;
}

/**
 * Called when the system is resuming from suspend
 * @pdev	Pointer to the device.
 * returns	0 on success, otherwise error code
 */
static int pmic_usb_det_resume(struct device *pdev)
{
	struct pmic_usb_det_data *pdata;

	if (!pdev)
		return -EINVAL;

	pdata = dev_get_drvdata(pdev);

	if (!pdata) {
		pr_err("%s: fail to get platform data\n", __func__);
		return -EINVAL;
	}

	if (device_may_wakeup(pdev)) {
		disable_irq_wake(pdata->irq_vbusdet);
		disable_irq_wake(pdata->irq_ctyp);
	}

	return 0;
}

const struct dev_pm_ops pmic_usb_det_pm = {
	.suspend = pmic_usb_det_suspend,
	.resume = pmic_usb_det_resume,
};

static const struct of_device_id pmic_usb_det_id[] = {
	{
		.compatible = "intel,pmic_usb_det",
	},
	{}

};

MODULE_DEVICE_TABLE(of, pmic_usb_det_id);

static struct platform_driver usb_det_drv = {
	.probe = pmic_usb_det_drv_probe,
	.remove = pmic_usb_det_drv_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(pmic_usb_det_id),
		.pm = &pmic_usb_det_pm,
	},
};

static int __init usb_det_drv_init(void)
{
	return platform_driver_register(&usb_det_drv);
}

static void __exit usb_det_drv_exit(void)
{
	platform_driver_unregister(&usb_det_drv);
}

late_initcall(usb_det_drv_init);
module_exit(usb_det_drv_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC USB type detection driver");
