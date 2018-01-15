/*
 * Component: AGOLD AFE ACCESSORY DETECTION INTERFACE
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

#ifndef __AGOLD_ACC_DETE_H

#define __AGOLD_ACC_DETE_H

/**
 * Enumeration to set the mode for vumic voltage source
 */
enum agold_afe_vumic_mode {
	AGOLD_AFE_VUMIC_MODE_POWER_DOWN,
	AGOLD_AFE_VUMIC_MODE_ULP,
	AGOLD_AFE_VUMIC_MODE_NORMAL,
	AGOLD_AFE_VUMIC_MODE_END
};

/**
 * Enumeration to set the high impedence for vumic
 */
enum agold_afe_hzvumic {
	AGOLD_AFE_HZVUMIC_NORMAL_POWER_DOWN,
	AGOLD_AFE_HZVUMIC_HIGH_IMPEDANCE,
	AGOLD_AFE_HZVUMIC_END
};

/**
 * Enumeration to set the vumic voltage level.
 */
enum agold_afe_vmicsel {
	AGOLD_AFE_VMICSEL_1_8_V = 0,
	AGOLD_AFE_VMICSEL_2_0_V = 1,
	AGOLD_AFE_VMICSEL_2_1_V = 2,
	AGOLD_AFE_VMICSEL_2_2_V = 3,
	AGOLD_AFE_VMICSEL_1_9_V = 0,
	AGOLD_AFE_VMICSEL_END
};

/**
 * Enum Value which corresponds to the micldo modes
 */
enum agold_afe_micldo_mode {
	AGOLD_AFE_MICLDO_MODE_OFF = 0,	 /* /< Micldo off*/
	AGOLD_AFE_MICLDO_MODE_NORMAL = 2,	/* /< Micldo in normal mode*/
	AGOLD_AFE_MICLDO_MODE_LOW_POWER = 1,	/* /< Micldo in low powermode*/
	AGOLD_AFE_MICLDO_MODE_END	/* /< Indicates end of enum */
};

/**
 * Enum Value which corresponds to the central biasing
 */
enum agold_afe_xb {
	AGOLD_AFE_XB_OFF,	/* /< Central Biasing off */
	AGOLD_AFE_XB_ON,	/* /< Central Biasing on */
	AGOLD_AFE_XB_END	/* /< Indicates end of enum*/
};

/**
 * Structure for configuring the VUMIC source
 */
struct agold_afe_vumic_conf {
	enum agold_afe_vumic_mode vmode;
	enum agold_afe_hzvumic    hzmic;
	enum agold_afe_vmicsel    vmicsel;
};

/**
 * Configuration structure for accessory detection parameters
 */
struct agold_afe_acc_det {
	/* Vumic acts as a pull up voltage for ACD */
	struct agold_afe_vumic_conf vumic_conf;
	/* Micldo mode for ACD */
	enum agold_afe_micldo_mode micldo_mode;
	/* Central biasing mode for ACD */
	enum agold_afe_xb xb_mode;
};

/**
 * Interface function to configure the Audio front end register for
 * accessory detection
 */
int agold_afe_set_acc_det_with_lock(struct agold_afe_acc_det acc_det_par);

int agold_afe_calculate_acc_settings(unsigned int reg,
					unsigned int requested_value,
					unsigned int *final_value);

int agold_afe_acc_update_mic_status(int event);

int agold_afe_acc_update_mic1_mode_status(int event);

#endif
