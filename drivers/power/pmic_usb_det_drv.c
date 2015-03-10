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
#define USBIDGNDDET_O 4
#define USBIDFLTDET_O 3

#define IRQLVL1 0x02
#define MIRQLVL1 0x0E
#define CHGR_O 5

#define SPWRSRC 0x20
#define SUSBIDDET_O 3
#define SUSBIDDET_W 2
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

#define USBIDCTRL 0x05
#define ACA_DETEN_O 1
#define USB_IDEN_O 0

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

	ID_ACA = 0,
	ID_GND = 1,
	ID_FLT = 2,

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
 * PMIC USB detection driver internal structure
 *
 * @pdev		Device pointer for USB detection driver
 * @otg_handle		USB OTG handle used for sending notifications
 * @cable_props		Power supply class cable property (notification payload)
 * @vbus_state		VBUS presence status
 * @cable_type		USB cable type detected
 * @usbid_state		USBID ground status
 * @irq_vbusdet		IRQ number for VBUS detection
 * @irq_ctyp		IRQ number for USB cable type detection
 * @irq_idflt		IRQ number for USBID floating detection
 * @irq_idgnd		IRQ number for USBID grounded detection
 * @irq_vbusdet		Whether wake up is allowed by VBUS detection IRQ
 * @irq_ctyp		Whether wake up is allowed by USB cable type IRQ
 * @irq_idflt		Whether wake up is allowed by USBID floating IRQ
 * @irq_idgnd		Whether wake up is allowed by USBID grounded IRQ
 */
struct pmic_usb_det_data {
	struct device *pdev;
	struct usb_phy *otg_handle;
	struct power_supply_cable_props cable_props;
	int vbus_state;
	int cable_type;
	int usbid_state;
	int irq_vbusdet;
	int irq_ctyp;
	int irq_idflt;
	int irq_idgnd;
	bool irqwake_vbusdet;
	bool irqwake_ctyp;
	bool irqwake_idflt;
	bool irqwake_idgnd;
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
 * @pdata	Pointer to the charger type detection driver data structure
 * @enable	Whether to enable/disable USB type detection
 * return	0 on success, otherwise error code
 */
static int pmic_usb_enable_usb_det(struct pmic_usb_det_data *pdata, bool enable)
{
	int ret;
	u8 reg;

	if (!pdata || !pdata->pdev)
		return -EINVAL;

	ret = pmic_reg_read(DEV3, USBPHYCTRL, &reg);
	if (ret) {
		dev_err(pdata->pdev, "%s - fail to read DEV3 USBPHYCTRL\n",
			__func__);
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
		dev_err(pdata->pdev, "%s - fail to write DEV3 USBPHYCTRL\n",
			__func__);
		return ret;
	}

	return 0;
}

/**
 * Interface to enable/disable USBID and ACA detection
 *
 * @pdata	Pointer to the charger type detection driver data structure
 * @id_enable	Whether to enable/disable USBID detection
 * @aca_enable	Whether to enable/disable ACA detection
 * return	0 on success, otherwise error code
 */
static int pmic_usb_enable_id_aca_det(struct pmic_usb_det_data *pdata,
					bool id_enable, bool aca_enable)
{
	int ret;
	u8 reg;

	if (!pdata || !pdata->pdev)
		return -EINVAL;

	ret = pmic_reg_read(DEV3, USBIDCTRL, &reg);
	if (ret) {
		dev_err(pdata->pdev, "%s - fail to read DEV3 USBIDCTRL\n",
			__func__);
		return ret;
	}

	if (id_enable) {
		set_field(reg, USB_IDEN_O, 1, SET);
		if (aca_enable)
			set_field(reg, ACA_DETEN_O, 1, SET);
	} else {
		set_field(reg, USB_IDEN_O, 1, CLEAR);
		set_field(reg, ACA_DETEN_O, 1, CLEAR);
	}
	ret = pmic_reg_write(DEV3, USBIDCTRL, reg);
	if (ret) {
		dev_err(pdata->pdev, "%s - fail to write DEV3 USBIDCTRL\n",
			__func__);
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
	bool during_boot = false;

	if (!pdata || !pdata->pdev)
		return -EINVAL;

	if (pdata->vbus_state == -1)
		during_boot = true;

	/* Ignore the VBUS report during boost */
	if (pdata->usbid_state == ID_GND || pdata->usbid_state == ID_ACA)
		return 0;

	ret = pmic_reg_read(DEV1, SPWRSRC, &reg);
	if (ret) {
		dev_err(pdata->pdev, "%s - fail to read DEV1 SPWRSRC\n",
			__func__);
		return ret;
	}

	if (get_field(reg, SVBUSDET_O, 1)) {
		dev_info(pdata->pdev, "VBUS connected\n");

		/* Enable the detection circuit and start CTYP detection */
		ret = pmic_usb_enable_usb_det(pdata, true);
		if (ret) {
			dev_err(pdata->pdev,
				"%s - fail to enable USB detection\n",
				__func__);
			return ret;
		}

		pdata->vbus_state = VBUS_ON;
	} else {
		/* Ignore the unwanted VBUS removal after stopping boost */
		if (pdata->vbus_state == VBUS_OFF)
			return 0;

		dev_info(pdata->pdev, "VBUS disconnected\n");
		/* Disable the internal detection circuit */
		ret = pmic_usb_enable_usb_det(pdata, false);
		if (ret) {
			dev_err(pdata->pdev,
				"%s - fail to disable USB detection\n",
								__func__);
			return ret;
		}
		pdata->vbus_state = VBUS_OFF;

		/* Notify PSY/USB about NONE event if it's not during boot */
		if (!during_boot) {
			pdata->cable_props.chrg_evt =
				POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
			pdata->cable_props.chrg_type = pdata->cable_type;
			pdata->cable_props.ma = 0;
			pdata->cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;

			atomic_notifier_call_chain(&pdata->otg_handle->notifier,
					USB_EVENT_NONE, &pdata->cable_props);
		}
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
	enum usb_phy_events event = USB_EVENT_VBUS;

	if (!pdata || !pdata->pdev)
		return -EINVAL;

	p_cable_type = &pdata->cable_type;

	ret = pmic_reg_read(DEV3, USBSRCDETSTATUS0, &reg);
	if (ret) {
		dev_err(pdata->pdev,
			"%s - fail to read DEV3 USBSRCDETSTATUS0\n",
			__func__);
		return ret;
	}

	/* Check for detection status */
	val = get_field(reg, SUSBHWDET_O, SUSBHWDET_W);
	if (val != 0x2) {
		dev_err(pdata->pdev,
			"Detection not completed, status=%d\n", val);
		return -EIO;
	}

	/* Check for detection result */
	val = get_field(reg, USBSRCDETRSLT_O, USBSRCDETRSLT_W);
	switch (val) {
	case USB_UNKNOWN:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		dev_info(pdata->pdev, "USB cable type - Unknown\n");
		break;
	case USB_SDP:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
		dev_info(pdata->pdev, "USB cable type - SDP\n");
		break;
	case USB_DCP:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		dev_info(pdata->pdev, "USB cable type - DCP\n");
		break;
	case USB_CDP:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		dev_info(pdata->pdev, "USB cable type - CDP\n");
		break;
	case USB_ACA:
		if (*p_cable_type == POWER_SUPPLY_CHARGER_TYPE_USB_ACA) {
			dev_info(pdata->pdev,
				"ACA charger already reported by USBID\n");
			return 0;
		}
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
		event = USB_EVENT_ID;
		dev_info(pdata->pdev, "USB cable type - ACA\n");
		break;
	case USB_SE1:
		/* SE1, report it as DCP at this moment */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		dev_info(pdata->pdev,
			"USB cable type - SE1, report it as DCP\n");
		break;
	case USB_MHL:
		/* MHL, report it as NONE at this moment */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		dev_info(pdata->pdev,
			"USB cable type - MHL, report it as Unknown\n");
		break;
	case USB_FLOAT:
		/* FLOATING, report it as NONE */
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		dev_info(pdata->pdev,
			"USB cable type - Floating, report it as Unknown\n");
		break;

	default:
		*p_cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		dev_info(pdata->pdev, "USB cable type - Unknown\n");
		break;
	}

	/* Send notification to USB driver */
	pdata->cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
	pdata->cable_props.chrg_type = *p_cable_type;
	pdata->cable_props.ma = 0;

	/* Disable the internal detection circuit */
	ret = pmic_usb_enable_usb_det(pdata, false);
	if (ret) {
		dev_err(pdata->pdev,
			"%s - fail to disable USB detection\n", __func__);
		return ret;
	}

	/* Notify USB driver about the type detection result */
	atomic_notifier_call_chain(&pdata->otg_handle->notifier, event,
				&pdata->cable_props);

	return 0;
}

/**
 * Detects USBID presence status and notifies USB driver
 *
 * @pdata	Pointer to the charger type detection driver data structure
 * @gnd		Indicating whether USBID is grounded (true) or floating (false)
 * @validation	Perform validation between interurpt and ID result
 * return	0 on success, otherwise error code
 */
static int pmic_usb_id_det(struct pmic_usb_det_data *pdata, bool gnd,
				bool validation)
{
	u8 reg, id;
	int ret;
	struct power_supply_cable_props *pcp;
	enum usb_phy_events event;
	bool during_boot = false;

	if (!pdata || !pdata->pdev)
		return -EINVAL;

	if (pdata->usbid_state == -1)
		during_boot = true;

	pcp = &pdata->cable_props;

	ret = pmic_reg_read(DEV1, SPWRSRC, &reg);
	if (ret) {
		dev_err(pdata->pdev,
			"%s - fail to read DEV1 SPWRSRC\n", __func__);
		return ret;
	}

	id = get_field(reg, SUSBIDDET_O, SUSBIDDET_W);

	if (validation && ((gnd && (id > ID_GND)) || (!gnd && (id < ID_FLT)))) {
		dev_err(pdata->pdev,
			"%s - USBID detection error (gnd=%d, id=%d)\n",
			__func__, gnd, id);
		return -EIO;
	}

	switch (id) {
	case ID_ACA:
		if (pdata->cable_type == POWER_SUPPLY_CHARGER_TYPE_USB_ACA) {
			dev_info(pdata->pdev,
				"ACA charger already reported by CTYP\n");
			return 0;
		}

		dev_info(pdata->pdev, "ACA charger detected\n");
		event = USB_EVENT_ID;
		pdata->cable_type = POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
		pcp->chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
		pcp->chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		break;

	case ID_GND:
		dev_info(pdata->pdev, "USBID grounded\n");
		event = USB_EVENT_ID;
		pdata->cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pcp->chrg_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		pcp->chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		break;

	case ID_FLT:
		dev_info(pdata->pdev, "USBID floating\n");
		event = USB_EVENT_NONE;
		pcp->chrg_type = pdata->cable_type;
		pcp->chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		pdata->cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
		break;

	default:
		dev_err(pdata->pdev,
			"%s - USBID detection error (gnd=%d, id=%d)\n",
			__func__, gnd, id);
		return -EIO;
		break;
	}

	pdata->usbid_state = id;

	pcp->ma = 0;

	/* Notify PSY/USB if not during boot or VBUS is not detected */
	if (!during_boot || pdata->vbus_state == VBUS_OFF)
		atomic_notifier_call_chain(&pdata->otg_handle->notifier, event,
					pcp);

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

	if (!pdata)
		return irqret;

	if (irq == pdata->irq_vbusdet) {
		pmic_usb_vbus_det(pdata);
		irqret = IRQ_HANDLED;
	} else if (irq == pdata->irq_ctyp) {
		pmic_usb_cable_det(pdata);
		irqret = IRQ_HANDLED;
	} else if (irq == pdata->irq_idflt) {
		pmic_usb_id_det(pdata, false, true);
		irqret = IRQ_HANDLED;
	} else if (irq == pdata->irq_idgnd) {
		pmic_usb_id_det(pdata, true, true);
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

	if (!pdata || !pdata->pdev)
		return -EINVAL;

	ret = devm_request_threaded_irq(pdata->pdev, pdata->irq_vbusdet, NULL,
				pmic_usb_det_cb,
				IRQF_SHARED | IRQF_NO_SUSPEND | IRQF_ONESHOT,
				DRIVER_NAME, pdata);
	if (ret) {
		dev_err(pdata->pdev, "%s: setup irq %d failed: %d\n", __func__,
				pdata->irq_vbusdet, ret);
		return ret;
	}

	ret = devm_request_threaded_irq(pdata->pdev, pdata->irq_ctyp, NULL,
				pmic_usb_det_cb,
				IRQF_SHARED | IRQF_NO_SUSPEND | IRQF_ONESHOT,
				DRIVER_NAME, pdata);
	if (ret) {
		dev_err(pdata->pdev, "%s: setup irq %d failed: %d\n", __func__,
				pdata->irq_ctyp, ret);
		return ret;
	}

	ret = devm_request_threaded_irq(pdata->pdev, pdata->irq_idflt, NULL,
				pmic_usb_det_cb,
				IRQF_SHARED | IRQF_NO_SUSPEND | IRQF_ONESHOT,
				DRIVER_NAME, pdata);
	if (ret) {
		dev_err(pdata->pdev, "%s: setup irq %d failed: %d\n", __func__,
				pdata->irq_idflt, ret);
		return ret;
	}

	ret = devm_request_threaded_irq(pdata->pdev, pdata->irq_idgnd, NULL,
				pmic_usb_det_cb,
				IRQF_SHARED | IRQF_NO_SUSPEND | IRQF_ONESHOT,
				DRIVER_NAME, pdata);
	if (ret) {
		dev_err(pdata->pdev, "%s: setup irq %d failed: %d\n", __func__,
				pdata->irq_idgnd, ret);
		return ret;
	}

	/* Clear Pending CTYP interrupt */
	mask = 0;
	set_field(mask, CTYPE_O, 1, SET);
	ret = pmic_reg_write(DEV1, CHGRIRQ0, mask);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to clear CTYP IRQ\n", __func__);
		return ret;
	}

	/* Clear Pending VBUSDET interrupt */
	mask = 0;
	set_field(mask, VBUSDET_O, 1, SET);
	ret = pmic_reg_write(DEV1, CHGRIRQ1, mask);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to clear VBUSDET IRQ\n",
			__func__);
		return ret;
	}
	/* Unmask CTYP interrupts */
	mask = 0;
	set_field(mask, CTYPE_O, 1, SET);
	ret = pmic_reg_set(DEV1, MCHGRIRQ0, mask, 0);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to umask CTYP IRQ\n", __func__);
		return ret;
	}

	/* Unmask VBUSDET interrupt */
	mask = 0;
	set_field(mask, VBUSDET_O, 1, SET);
	ret = pmic_reg_set(DEV1, MCHGRIRQ1, mask, 0);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to umask VBUSDET IRQ\n",
			__func__);
		return ret;
	}

	/* Unmask USBIDFLTDET interrupt */
	mask = 0;
	set_field(mask, USBIDFLTDET_O, 1, SET);
	ret = pmic_reg_set(DEV1, MCHGRIRQ1, mask, 0);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to umask USBIDFLTDET IRQ\n",
			__func__);
		return ret;
	}

	/* Unmask USBIDGNDDET interrupt */
	mask = 0;
	set_field(mask, USBIDGNDDET_O, 1, SET);
	ret = pmic_reg_set(DEV1, MCHGRIRQ1, mask, 0);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to umask USBIDGNDDET IRQ\n",
			__func__);
		return ret;
	}

	/* Unmask LVL1 CHGR interrupt */
	mask = 0;
	set_field(mask, CHGR_O, 1, SET);
	ret = pmic_reg_set(DEV1, MIRQLVL1, mask, 0);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to umask LVL1 CHGR IRQ\n",
			__func__);
		return ret;
	}

	/* Enable VBUSDETCTRL_REG:VBUSDETEN bit */
	mask = 0;
	ret = pmic_reg_read(DEV3, VBUSDETCTRL, &mask);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to read DEV3 VBUSDETCTRL\n",
			__func__);
		return ret;
	}
	set_field(mask, VBUSDETEN_O , 1, SET);
	ret = pmic_reg_write(DEV3, VBUSDETCTRL, mask);
	if (ret) {
		dev_err(pdata->pdev, "%s: fail to write DEV3 VBUSDETCTRL\n",
			__func__);
	}

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
		dev_err(&pdev->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	pdata->pdev = &pdev->dev;
	pdata->vbus_state = -1;
	pdata->cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE;
	pdata->usbid_state = -1;
	pdata->irq_ctyp = -ENXIO;
	pdata->irq_vbusdet = -ENXIO;
	pdata->irqwake_vbusdet = false;
	pdata->irqwake_ctyp = false;
	pdata->irqwake_idflt = false;
	pdata->irqwake_idgnd = false;

	platform_set_drvdata(pdev, pdata);

	/* Get USB phy */
	otg_handle = devm_usb_get_phy(&pdev->dev, USB_PHY_TYPE_USB2);
	if (IS_ERR_OR_NULL(otg_handle)) {
		dev_err(&pdev->dev, "%s: fail to get OTG transceiver\n",
			__func__);
		return -EBUSY;
	}

	pdata->otg_handle = otg_handle;

	/* Get interrupt from device tree */
	pdata->irq_vbusdet = platform_get_irq_byname(pdev, "vbusdet");
	pdata->irq_ctyp = platform_get_irq_byname(pdev, "ctype");
	pdata->irq_idflt = platform_get_irq_byname(pdev, "usbidflt");
	pdata->irq_idgnd = platform_get_irq_byname(pdev, "usbidgnd");

	if (IS_ERR_VALUE(pdata->irq_vbusdet) ||
		IS_ERR_VALUE(pdata->irq_ctyp) ||
		IS_ERR_VALUE(pdata->irq_idflt) ||
		IS_ERR_VALUE(pdata->irq_idgnd)) {
		ret = -EBUSY;
		dev_err(&pdev->dev,
			"%s: can't get irq, vbus=%d ctyp=%d idflt=%d idgnd=%d\n",
			__func__, pdata->irq_vbusdet, pdata->irq_ctyp,
			pdata->irq_idflt, pdata->irq_idgnd);
		return ret;
	}

	ret = pmic_usb_setup_det_irq(pdata);
	if (ret)
		return ret;

	/* Disable the internal detection circuit */
	ret = pmic_usb_enable_usb_det(pdata, false);
	if (ret)
		return ret;

	/* Read the VBUS presence status for initial update, and trigger
	USB type detection if applicable */
	ret = pmic_usb_vbus_det(pdata);
	if (ret)
		return ret;

	/* Enable USBID and ACA detection */
	ret = pmic_usb_enable_id_aca_det(pdata, true, true);
	if (ret)
		return ret;

	/* Read the USBID and ACA detection result */
	ret = pmic_usb_id_det(pdata, false, false);
	if (ret)
		return ret;

	ret = device_init_wakeup(&pdev->dev, true);
	if (ret)
		dev_err(&pdev->dev, "%s: can't set wakeup src\n", __func__);

	return ret;
}

static int __exit pmic_usb_det_drv_remove(struct platform_device *pdev)
{
	int ret;

	ret = device_init_wakeup(&pdev->dev, false);
	if (ret)
		dev_err(&pdev->dev, "%s: can't clear wakeup src\n", __func__);

	/* Managed device resources will be freed automatically */

	return ret;
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
		dev_err(pdev, "%s: fail to get platform data\n", __func__);
		return -EINVAL;
	}

	if (device_may_wakeup(pdev)) {
		if (!enable_irq_wake(pdata->irq_vbusdet))
			pdata->irqwake_vbusdet = true;
		if (!enable_irq_wake(pdata->irq_ctyp))
			pdata->irqwake_ctyp = true;
		if (!enable_irq_wake(pdata->irq_idflt))
			pdata->irqwake_idflt = true;
		if (!enable_irq_wake(pdata->irq_idgnd))
			pdata->irqwake_idgnd = true;
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
		dev_err(pdev, "%s: fail to get platform data\n", __func__);
		return -EINVAL;
	}

	if (device_may_wakeup(pdev)) {
		if (pdata->irqwake_vbusdet) {
			disable_irq_wake(pdata->irq_vbusdet);
			pdata->irqwake_vbusdet = false;
		}

		if (pdata->irqwake_ctyp) {
			disable_irq_wake(pdata->irq_ctyp);
			pdata->irqwake_ctyp = false;
		}

		if (pdata->irqwake_idflt) {
			disable_irq_wake(pdata->irq_idflt);
			pdata->irqwake_idflt = false;
		}

		if (pdata->irqwake_idgnd) {
			disable_irq_wake(pdata->irq_idgnd);
			pdata->irqwake_idgnd = false;
		}
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
