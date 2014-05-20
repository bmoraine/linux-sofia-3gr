/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

/* FIXME */
#include <../../sound/soc/codecs/agold_acc_det.h>

#define AHJ_TYPE_MIN_MV 1000
#define AHJ_TYPE_MAX_MV 1700

#define HEADPHONE_MIN_MV 0
#define HEADPHONE_MAX_MV 50

static struct hook_key_cfg xgold_hs_keymap[] = {
	{0, 50, KEY_FORWARDMAIL , 0},
	{100, 150, KEY_VOLUMEUP, 0},
	{275, 325, KEY_VOLUMEDOWN, 0},
};

#define XGOLD_DETECT_INSERTION        0x80FB
#define XGOLD_DETECT_REMOVAL		  0xC0F3
#define XGOLD_DETECT_REMOVAL_HOOK     0xCAF3
#define XGOLD_DETECT_HOOK_RELEASE     0xC2F3
#define XGOLD_OFF_EDG1                3
#define XGOLD_OFF_EDG2                11

#define ACC_VBIAS_ENABLE \
{\
	struct agold_afe_acc_det acc_det_par;\
	acc_det_par.vumic_conf.vmode = AGOLD_AFE_VUMIC_MODE_ULP;\
	acc_det_par.vumic_conf.hzmic = AGOLD_AFE_HZVUMIC_NORMAL_POWER_DOWN;\
	acc_det_par.vumic_conf.vmicsel = AGOLD_AFE_VMICSEL_2_1_V;\
	acc_det_par.micldo_mode = AGOLD_AFE_MICLDO_MODE_NORMAL;\
	acc_det_par.xb_mode = AGOLD_AFE_XB_ON;\
	if (agold_afe_set_acc_det_with_lock(acc_det_par)) { \
		dev_err(headset_device_data->sdev.dev, \
				"ACC_VBIAS_ENABLE ERROR\n"); \
	} \
}

#define ACC_VBIAS_ULP_ON \
{\
	struct agold_afe_acc_det acc_det_par;\
	acc_det_par.vumic_conf.vmode = AGOLD_AFE_VUMIC_MODE_ULP;\
	acc_det_par.vumic_conf.hzmic = AGOLD_AFE_HZVUMIC_NORMAL_POWER_DOWN;\
	acc_det_par.vumic_conf.vmicsel = AGOLD_AFE_VMICSEL_1_9_V;\
	acc_det_par.micldo_mode = AGOLD_AFE_MICLDO_MODE_LOW_POWER;\
	acc_det_par.xb_mode = AGOLD_AFE_XB_OFF;\
	if (agold_afe_set_acc_det_with_lock(acc_det_par)) { \
		dev_err(headset_device_data->sdev.dev, \
				"ACC_VBIAS_ULP_ON ERROR\n"); \
	} \
}
