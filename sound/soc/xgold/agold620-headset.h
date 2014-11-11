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
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/input.h>

/* FIXME */
#include <../../codecs/agold_acc_det.h>

#define AHJ_TYPE_MIN_MV 475
#define AHJ_TYPE_MAX_MV 1700

#define HEADPHONE_MIN_MV 0
#define HEADPHONE_MAX_MV 50

struct hs_key_cfg {
	int min_mv;
	int max_mv;
	enum snd_jack_types type;
	int key_code;
	int pressed;
};

static struct hs_key_cfg xgold_hs_keymap[] = {
	{0, 50, SND_JACK_BTN_0 , KEY_MEDIA, 0},
	{100, 150, SND_JACK_BTN_1, KEY_VOLUMEUP, 0},
	{275, 325, SND_JACK_BTN_2, KEY_VOLUMEDOWN, 0},
};

#define XGOLD_DETECT_INSERTION        0x80FB
#define XGOLD_DETECT_REMOVAL		  0xC0F3
#define XGOLD_DETECT_REMOVAL_HOOK     0xCAF3
#define XGOLD_DETECT_HOOK_RELEASE     0xC2F3
#define XGOLD_OFF_EDG1                3
#define XGOLD_OFF_EDG2                11

#define VBIAS_SETTLING_TIME_MS	20

/**
 * Different VBIAS settings
**/
enum xgold_vbias {
	XGOLD_VBIAS_ENABLE,
	XGOLD_VBIAS_ULP_ON,
};

enum xgold_headset_type {
	XGOLD_HEADSET_REMOVED,
	XGOLD_HEAD_SET,
	XGOLD_HEAD_PHONE,
	XGOLD_INVALID,
	XGOLD_ERROR
};






