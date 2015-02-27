/*
 * Component: INTEL AFE ALSA SOC driver
 *
 * Copyright (C) 2013, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */

/*On XMM6321, accessory detection/identification is supported by AGOLD AFE.
 *There are common register bits in AFE which are used for Accessory detection
 invoked by accessory module as well as MIC and headset related
 functionalities. The file implements mechanism to combine settings requested
 by accessory identification and for normal MIC and headset use cases
 */

#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/soc.h>
#include "intel_pmic_afe.h"
#include "afe_acc_det.h"

/* Masks for AFE Register AUDIO IN CTRL fileds */

/* Bit 5-6 of AFE_AUDIOINCTRL3_REG */
#define AFE_MASK_AUDIOINCTRL_VUMIC_MODE   0x9F

/* Bit 3 of AFE_AUDIOINCTRL4_REG*/
#define AFE_MASK_AUDIOINCTRL_HZ_VUMIC     0xFD

/* Bit 7 of AFE_AUDIOINCTRL3_REG */
#define AFE_MASK_AUDIOINCTRL_VMIC_SELL     0x7F

/* Bit 0 of AFE_AUDIOINCTRL4_REG */
#define AFE_MASK_AUDIOINCTRL_VMIC_SELH     0xFE

/* Bit 4-5 of AFE_AUDIOINCTRL3_REG */
#define AFE_MASK_AUDIOINCTRL_MIC_LDO      0xE7

/* Bit 4 of AFE_AUDIOINCTRL3_REG */
#define AFE_MASK_AUDIOINCTRL_VMIC_MODE    0xEF

/* Bit 7 of AFE_BCON4_REG */
#define AFE_MASK_BCON_XB_ON               0x7F

/* Bit Shifts for PMIC AFE Register AUDIOINCTRL3 fields */
#define AFE_SHIFT_AUDIOINCTRL_VMIC_SELL    7
#define AFE_SHIFT_AUDIOINCTRL_VUMIC_MODE   5
#define AFE_SHIFT_AUDIOINCTRL_VMIC_MODE    4

/* Bit Shifts for PMIC AFE Register AUDIOINCTRL4 fields */
#define AFE_SHIFT_AUDIOINCTRL_MIC_LDO      3
#define AFE_SHIFT_AUDIOINCTRL_HZ_VUMIC     2
#define AFE_SHIFT_AUDIOINCTRL_VMIC_SELH    0

/* Bit Shifts for PMIC AFE Register BCON4 fields */
#define AFE_SHIFT_BCON_XB_ON               7


/* Structure to hold accessory detection settings */
struct afe_accessory_detect {
	/* Stores the current accessory identification settings */
	struct afe_acc_det acc_det_param;
	bool mic_ldo_on;  /* Indicates that MIC is ON */
	bool vmic_mode; /*vmic mode setting from ASOC */
};

/*Initialize the accessory detection settings to keep the settings disabled
 *for low power consumption.
 *The settings will get updated by accessory driver
 */
struct afe_accessory_detect pmic_acc_det_data = {
	.acc_det_param = {
		.vumic_conf = {
			.vmode = AFE_VUMIC_MODE_ULP,
			.hzmic = AFE_HZVUMIC_NORMAL_POWER_DOWN,
			.vmicsel = AFE_VMICSEL_2_1_V,
		},
		.micldo_mode = AFE_MICLDO_MODE_LOW_POWER,
		.xb_mode = AFE_XB_ON,
	},
	.mic_ldo_on = 0,
	.vmic_mode = 0,
};

int pmic_afe_set_acc_det_with_lock(struct afe_acc_det acc_det_par)
{
	int result = -EINVAL;
	u8 reg;
	struct afe_data *afe_private_data = afe_get_private_data();

	if ((NULL != afe_private_data) && (NULL != afe_private_data->codec)) {
		/* The interface would get called from Accessory driver
		 * context. Register update may already be in progress for
		 * other use cases. Get a lock */
		mutex_lock(&afe_private_data->codec->mutex);
		/* Store the requested Accessory detect settings */
		pmic_acc_det_data.acc_det_param = acc_det_par;
		mutex_unlock(&afe_private_data->codec->mutex);
		afe_debug("%s: Accessory detection settings requested\n",
				__func__);
		afe_debug("micldo_mode: %d\n",
				pmic_acc_det_data.acc_det_param.micldo_mode);
		afe_debug("xb_mode: %d\n",
				pmic_acc_det_data.acc_det_param.xb_mode);
		afe_debug("vumic_mode: %d\n",
			pmic_acc_det_data.acc_det_param.vumic_conf.vmode);
		afe_debug("hzvumic: %d\n",
			pmic_acc_det_data.acc_det_param.vumic_conf.hzmic);
		afe_debug("vmic_sel: %d\n",
			pmic_acc_det_data.acc_det_param.vumic_conf.vmicsel);

		/* TODO: There were spurios interrupts observed on xg223 if
		 * accessory detect parameters are changed while MIC1 is
		 * active. SW workaround similar to xg223 will be required if
		 * the same issue is observed here, while doing HW testing */

		/* AGOLD_AFE_AUDIOINCTRL and AGOLD_AFE_BCON have to be set for
		 * accessory detection functionality */
			mutex_lock(&afe_private_data->codec->mutex);

		reg = snd_soc_read(afe_private_data->codec,
					AFE_AUDIOINCTRL3_REG);
		result = snd_soc_write(afe_private_data->codec,
					AFE_AUDIOINCTRL3_REG, reg);
		if (0 == result) {
			reg = snd_soc_read(afe_private_data->codec,
				AFE_AUDIOINCTRL4_REG);
			result = snd_soc_write(afe_private_data->codec,
					AFE_AUDIOINCTRL4_REG, reg);
		}
		if (0 == result) {
			reg = snd_soc_read(afe_private_data->codec,
				AFE_BCON4_REG);
			result = snd_soc_write(afe_private_data->codec,
						AFE_BCON4_REG, reg);
			}
			mutex_unlock(&afe_private_data->codec->mutex);
		}
	return result;
}
EXPORT_SYMBOL_GPL(pmic_afe_set_acc_det_with_lock);

/* Function calculates the register values for AFE_AUDIOINCTRLi_REG and
 * AFE_BCON. Certain register bits are requested both by MIC use cases
 * (like speech call, recording) and accessory identification use cases.
 * This function arrives at a consolidated values by combining 2 sets of values.
 * Prioriy will be given to MIC/headset use case settings.
 * When the MIC is not being used, accessory identification values requested
 * will be used. afe_private_data->codec->mutex has to be acquired before
 * calling this function.
 */
int pmic_afe_calculate_acc_settings(unsigned int reg,
		unsigned int requested_value,
		unsigned int *final_value)
{
	if (!final_value)
		return -EINVAL;

	*final_value = requested_value;
	/* The bits for accessory identification are spread over
	AFE_AUDIOINCTRL2_REG, AFE_AUDIOINCTRL3_REG and AFE_BCON4_REG
	registers */
	if (AFE_AUDIOINCTRL3_REG == reg) {
		/* Clear the bits that are unused by ASOC/DAPM,
		 * so that they are freshly calculated */
		*final_value &= AFE_MASK_AUDIOINCTRL_VUMIC_MODE;

		if (AFE_VUMIC_MODE_POWER_DOWN == pmic_acc_det_data.mic_ldo_on) {
			*final_value |=
				(pmic_acc_det_data.acc_det_param.
				 vumic_conf.vmode & 0x3) <<
				AFE_SHIFT_AUDIOINCTRL_VUMIC_MODE;

			/* Changing MICLDO mode to low power while vmic
			 * mode is active results in false interrupt.
			 * Change mic ldo and vmic mode together.
			 * As request is to turn off ldo from ASOC,
			 * vmic mode can be put off */
			*final_value &= AFE_MASK_AUDIOINCTRL_VMIC_MODE;
			*final_value |= AFE_VUMIC_MODE_POWER_DOWN <<
				AFE_SHIFT_AUDIOINCTRL_VMIC_MODE;

			*final_value &= AFE_MASK_AUDIOINCTRL_VMIC_SELL;
			*final_value |=
				(pmic_acc_det_data.acc_det_param.
				 vumic_conf.vmicsel & 0x1) <<
				AFE_SHIFT_AUDIOINCTRL_VMIC_SELL;
		} else {
			/* DAPM sequence has mic ldo enabled.
			Program vmic, vumic and mic ldo together*/
			pr_debug("%s MIC LDO is On. Using ASOC values\n",
				       __func__);
			/* if any of mic is active, vumicmode
			should not be off */
			*final_value &= AFE_MASK_AUDIOINCTRL_VUMIC_MODE;
			*final_value |= AFE_VUMIC_MODE_NORMAL <<
				AFE_SHIFT_AUDIOINCTRL_VUMIC_MODE;

			/* ASOC request for vmic mode bias widget
			is stored and programmed here while enabling
			mic ldo */
			*final_value &= AFE_MASK_AUDIOINCTRL_VMIC_MODE;
			*final_value |=
				(pmic_acc_det_data.vmic_mode & 0x1) <<
				AFE_SHIFT_AUDIOINCTRL_VMIC_MODE;
		}
	} else if (AFE_AUDIOINCTRL4_REG == reg) {
		/* Clear the bits that are unused by ASOC/DAPM,
		so that they are freshly calculated */
		*final_value &= AFE_MASK_AUDIOINCTRL_MIC_LDO;

		if (AFE_VUMIC_MODE_POWER_DOWN == pmic_acc_det_data.mic_ldo_on) {
			/* DAPM sequence has mic ldo disabled
			use acc settings */
			pr_debug("%s MIC LDO is off. Using acc value\n",
					__func__);
			*final_value |=
				(pmic_acc_det_data.acc_det_param.
				 micldo_mode & 0x3) <<
				AFE_SHIFT_AUDIOINCTRL_MIC_LDO;

			*final_value &= AFE_MASK_AUDIOINCTRL_VMIC_SELH;
			*final_value |=
				((pmic_acc_det_data.acc_det_param.
				vumic_conf.vmicsel & 0x2) >> 1) <<
				AFE_SHIFT_AUDIOINCTRL_VMIC_SELH;
		} else {
			/* DAPM sequence has mic ldo enabled.
			Program vmic, vumic and mic ldo together*/
			pr_debug("%s MIC LDO is On. Using ASOC values\n"
				, __func__);
			/* Update mic ldo and vmic mode together */
			*final_value |= AFE_MICLDO_MODE_NORMAL <<
				AFE_SHIFT_AUDIOINCTRL_MIC_LDO;
		}
		*final_value &= AFE_MASK_AUDIOINCTRL_HZ_VUMIC;
		*final_value |=
			(pmic_acc_det_data.acc_det_param.
			 vumic_conf.hzmic & 0x1) <<
			AFE_SHIFT_AUDIOINCTRL_HZ_VUMIC;
	} else if (AFE_BCON4_REG == reg) {
		if (0 == (*final_value & ~(AFE_MASK_BCON_XB_ON))) {
			pr_debug("%s Using ACC requested val for XB\n",
						__func__);
			/* Request for XB off. So use val requested by
			ACC driver */
			*final_value |=
				(pmic_acc_det_data.acc_det_param.
				 xb_mode & 0x1) << AFE_SHIFT_BCON_XB_ON;
		}
	}

	return 0;
}

/* Activity on MIC. Update the mic_ldo status */
int pmic_afe_acc_update_mic_status(int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		pmic_acc_det_data.mic_ldo_on = 1;
		break;

	case SND_SOC_DAPM_PRE_PMD:
		pmic_acc_det_data.mic_ldo_on = 0;
		break;
	}
	return 0;
}

/* Activity on VMIC mode from ASOC */
int pmic_afe_acc_update_mic1_mode_status(int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		pmic_acc_det_data.vmic_mode = 1;
		break;

	case SND_SOC_DAPM_PRE_PMD:
		pmic_acc_det_data.vmic_mode = 0;
		break;
	}
	return 0;
}

