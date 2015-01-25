/*
 * Component: AGOLD AFE ALSA SOC driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
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

/**
 * On XMM6321, accessory detection/identification is supported by AGOLD AFE.
 *
 * There are common register bits in AFE which are used for Accessory detection
 * invoked by accessory module as well as MIC and headset related
 * functionalities.
 *
 * This file implements mechanism to combine settings requested
 * by accessory identification and for normal MIC and headset use cases.
 */

#include <linux/kernel.h>
#include <sound/core.h>
#include <sound/soc.h>
#include "agold_afe.h"
#include "afe_acc_det.h"

/* Masks for AGOLD AFE Register AUDIO IN CTRL fileds */

/* Bit 21-22 */
#define MASK_AUDIOINCTRL_VUMIC_MODE   0xFF9FFFFF

/* Bit 26 */
#define MASK_AUDIOINCTRL_HZ_VUMIC     0xFBFFFFFF

/* Bit 23-24 */
#define MASK_AUDIOINCTRL_VMIC_SEL     0xFE7FFFFF

/* Bit 27-28 */
#define MASK_AUDIOINCTRL_MIC_LDO      0xE7FFFFFF

/* Bit 20 */
#define MASK_AUDIOINCTRL_VMIC_MODE    0xFFEFFFFF

/* Bit 31 */
#define MASK_BCON_XB_ON               0x7FFFFFFF

/* Bit Shifts for AGOLD AFE Register AUDIO IN CTRL fields */
#define SHIFT_AUDIOINCTRL_VUMIC_MODE   21
#define SHIFT_AUDIOINCTRL_HZ_VUMIC     26
#define SHIFT_AUDIOINCTRL_VMIC_SEL     23
#define SHIFT_AUDIOINCTRL_MIC_LDO      27
#define SHIFT_AUDIOINCTRL_VMIC_MODE    20
#define SHIFT_BCON_XB_ON               31

/* Structure to hold accessory detection settings */
struct agold_afe_accessory_detect {
	/* Stores the current accessory identification settings */
	struct afe_acc_det acc_det_param;
	bool mic_ldo_on;  /* Indicates that MIC is ON */
	bool vmic_mode; /*vmic mode setting from ASOC */
};

/*Initialize the accessory detection settings to keep the settings disabled
 *for low power consumption.
 *The settings will get updated by accessory driver
 */
struct agold_afe_accessory_detect acc_det_data = {
	.acc_det_param = {
		.vumic_conf = {
			.vmode = AFE_VUMIC_MODE_ULP,
			.hzmic = AFE_HZVUMIC_NORMAL_POWER_DOWN,
			.vmicsel = AFE_VMICSEL_2_2_V,
		},
		.micldo_mode = AFE_MICLDO_MODE_LOW_POWER,
		.xb_mode = AFE_XB_OFF,
	},
	.mic_ldo_on = 0,
	.vmic_mode = 0,
};

int agold_afe_set_acc_det_with_lock(struct afe_acc_det acc_det_par)
{
	int result = -EINVAL;
	unsigned int current_reg;
	struct agold_afe_data *afe_private_data = agold_afe_get_private_data();
	struct afe_acc_det *acd_par = &acc_det_data.acc_det_param;

	if ((NULL != afe_private_data) && (NULL != afe_private_data->codec)) {
		/* The interface would get called from Accessory driver
		 * context. Register update may already be in progress for
		 * other use cases. Get a lock */
		mutex_lock(&afe_private_data->codec->mutex);
		/* Store the requested Accessory detect settings */
		acc_det_data.acc_det_param = acc_det_par;
		mutex_unlock(&afe_private_data->codec->mutex);
		afe_debug("%s: Accessory detection settings requested\n",
				__func__);
		afe_debug("micldo_mode: %d\n", acd_par->micldo_mode);
		afe_debug("xb_mode: %d\n", acd_par->xb_mode);
		afe_debug("vumic_mode: %d\n", acd_par->vumic_conf.vmode);
		afe_debug("hzvumic: %d\n", acd_par->vumic_conf.hzmic);
		afe_debug("vmic_sel: %d\n", acd_par->vumic_conf.vmicsel);

		/* TODO: There were spurios interrupts observed on xg223 if
		 * accessory detect parameters are changed while MIC1 is
		 * active. SW workaround similar to xg223 will be required if
		 * the same issue is observed here, while doing HW testing */

		/* AGOLD_AFE_AUDIOINCTRL and AGOLD_AFE_BCON have to be set for
		 * accessory detection functionality */
		mutex_lock(&afe_private_data->codec->mutex);
		current_reg = snd_soc_read(afe_private_data->codec,
				AGOLD_AFE_AUDIOINCTRL);
		result = snd_soc_write(afe_private_data->codec,
			AGOLD_AFE_AUDIOINCTRL, current_reg);
		if (0 == result) {
			current_reg = snd_soc_read(afe_private_data->codec,
					AGOLD_AFE_BCON);
			result = snd_soc_write(afe_private_data->codec,
				AGOLD_AFE_BCON, current_reg);
		}
		mutex_unlock(&afe_private_data->codec->mutex);
	}

	return result;
}
EXPORT_SYMBOL_GPL(agold_afe_set_acc_det_with_lock);

/**
 * agold_afe_calculate_acc_settings() - calculate AFE regs for ACC detection
 * @reg:				register to calculate settings for
 * @requested_value:	value requested by ASOC/DAPM
 * @*final_value:		pointer to where the result should be stored
 *
 * Function calculates the register values for AGOLD_AFE_AUDIOINCTRL and
 * AGOLD_AFE_BCON. Certain register bits are requested both by MIC use cases
 * (like speech call, recording) and accessory identification use cases.
 *
 * This function arrives at a consolidated value by combining 2 sets of values.
 * Prioriy will be given to MIC/headset use case settings.
 *
 * When the MIC is not being used, accessory identification values requested
 * will be used. afe_private_data->codec->mutex has to be acquired before
 * calling this function.
 *
 * Return: Returns 0 if input pointer is valid, -EINVAL otherwise.
 */
int agold_afe_calculate_acc_settings(unsigned int reg,
		unsigned int requested_value,
		unsigned int *final_value)
{
	if (!final_value)
		return -EINVAL;

	*final_value = requested_value;
	/* The bits for accessory identification are spread over
	 * AGOLD_AFE_AUDIOINCTRL and AGOLD_AFE_BCON registers */
	if (AGOLD_AFE_AUDIOINCTRL == reg) {
		/* Clear the bits that are unused by ASOC/DAPM, so that they are
		freshly calculated */
		*final_value &= ~(1 << SHIFT_AUDIOINCTRL_MIC_LDO);
		*final_value &= ~(1 << SHIFT_AUDIOINCTRL_VUMIC_MODE);

		if (!acc_det_data.mic_ldo_on) {
			struct afe_acc_det *p_acd_par =
				&acc_det_data.acc_det_param;
			/* DAPM sequence has mic ldo disabled.
				Use acc settings */
			afe_debug("%s MIC LDO is off. Using acc value\n",
				__func__);
			*final_value = (*final_value &
					MASK_AUDIOINCTRL_MIC_LDO);
			*final_value |= (p_acd_par->micldo_mode <<
					SHIFT_AUDIOINCTRL_MIC_LDO);

			*final_value = (*final_value &
					MASK_AUDIOINCTRL_VUMIC_MODE);
			*final_value |= (p_acd_par->vumic_conf.vmode <<
					SHIFT_AUDIOINCTRL_VUMIC_MODE);

			*final_value = (*final_value &
					MASK_AUDIOINCTRL_VMIC_SEL);
			*final_value |= (p_acd_par->vumic_conf.vmicsel <<
					SHIFT_AUDIOINCTRL_VMIC_SEL);

			/* Changing MICLDO mode to low power while vmic mode
			 * is active results in false interrupt. Change mic ldo
			 * and vmic mode together. As request is to turn off
			 * ldo from ASOC, vmic mode can be put off */
			*final_value = (*final_value &
					MASK_AUDIOINCTRL_VMIC_MODE);
			*final_value |= (0 << SHIFT_AUDIOINCTRL_VMIC_MODE);
		} else {
			/* DAPM sequence has mic ldo enabled.
			 * Program vmic, vumic and mic ldo together*/
			afe_debug("%s MIC LDO is On. Using ASOC values\n",
				__func__);
			/* if any of mic is active, vumicmode should
			 * not be off */
			*final_value = (*final_value &
					MASK_AUDIOINCTRL_VUMIC_MODE);
			*final_value |= (AFE_VUMIC_MODE_NORMAL <<
					SHIFT_AUDIOINCTRL_VUMIC_MODE);

			/* Update mic ldo and vmic mode together */
			*final_value = *final_value & MASK_AUDIOINCTRL_MIC_LDO;
			*final_value |= (AFE_MICLDO_MODE_NORMAL <<
					SHIFT_AUDIOINCTRL_MIC_LDO);

			/* ASOC request for vmic mode bias widget is stored and
			 * programmed here while enabling mic ldo */
			*final_value = (*final_value &
					MASK_AUDIOINCTRL_VMIC_MODE);
			*final_value |= (acc_det_data.vmic_mode <<
					SHIFT_AUDIOINCTRL_VMIC_MODE);
		}
		*final_value = *final_value & MASK_AUDIOINCTRL_HZ_VUMIC;
		*final_value |= (acc_det_data.acc_det_param.vumic_conf.hzmic <<
				SHIFT_AUDIOINCTRL_HZ_VUMIC);
	} else if (AGOLD_AFE_BCON == reg) {
		if (!(*final_value & ~(MASK_BCON_XB_ON))) {
			afe_debug("%s Using ACC req val for XB\n", __func__);
			/* Request for XB off so use val requested by
			 * ACC driver */
			*final_value |= (acc_det_data.acc_det_param.xb_mode <<
					SHIFT_BCON_XB_ON);
		}
	}
	return 0;
}

/* Activity on MIC. Update the mic_ldo status */
int agold_afe_acc_update_mic_status(int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		acc_det_data.mic_ldo_on = 1;
		break;

	case SND_SOC_DAPM_PRE_PMD:
		acc_det_data.mic_ldo_on = 0;
		break;
	}
	return 0;
}

/* Activity on VMIC mode from ASOC */
int agold_afe_acc_update_mic1_mode_status(int event)
{
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		acc_det_data.vmic_mode = 1;
		break;

	case SND_SOC_DAPM_PRE_PMD:
		acc_det_data.vmic_mode = 0;
		break;
	}
	return 0;
}
