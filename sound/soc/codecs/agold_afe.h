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

#ifndef __AGOLD_AFE_H

#define __AGOLD_AFE_H

#include <linux/clk.h>
#include <linux/idi/idi_interface.h>

/*supported playback rate and format*/
#define AGOLD_AFE_OUT_RATES SNDRV_PCM_RATE_8000_48000
#define AGOLD_AFE_OUT_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define AGOLD_AFE_IN_RATES  SNDRV_PCM_RATE_8000_48000
#define AGOLD_AFE_IN_FORMAT SNDRV_PCM_FMTBIT_S16_LE


#define AFE_REG_RES_NAME	"register"
#define AFE_IN_RES_NAME		"afe_in"
#define AFE_OUT_RES_NAME	"afe_out"
#define DSP_IN_RES_NAME		"dsp_in"
#define DSP_OUT_RES_NAME	"dsp_out"


/* AGOLD AFE Registers */
enum agold_afe_register {
	AGOLD_AFE_PWR = 0,
	AGOLD_AFE_BCON,
	AGOLD_AFE_AUDOUTCTRL1,
	AGOLD_AFE_AUDOUTCTRL2,
	AGOLD_AFE_AUDOUTCTRL3,
	AGOLD_AFE_AUDIOINCTRL,
	AGOLD_AFE_GAIN_OUT,
	AGOLD_AFE_GAIN_IN,
	AGOLD_AFE_AUD2IDICTRL,
	AGOLD_AFE_MUTE,
#ifdef CONFIG_SND_SOC_AGOLD_620
	AGOLD_AFE_DIGMIC_CONTROL1,
#endif
	AGOLD_AFE_CACHEREGNUM,
};

#define AFE_PWR_HSR 0x4
#define AFE_PWR_HSL 0x2
#define AFE_GAIN_IN_HSOCCALL_MASK 0x00FC0000
#define AFE_GAIN_IN_HSOCCALR_MASK 0xFC000000
#define AFE_GAIN_IN_HSOCCALL_OFFSET 18
#define AFE_GAIN_IN_HSOCCALR_OFFSET 26
#define AFE_AUDOUTCTRL1_HSOCSOC_OFFSET 21
#define AFE_GAIN_OUT_HS_OFFSET 24

/* AFE BCON Register */
#define AFE_BCON_AUD_OUTRATE_POS 0x02
#define AFE_BCON_AUD_INRATE_POS	0x06
#define AFE_BCON_XBON		0x80000000
#define AFE_BCON_FMR_DIRECT	0x00080000
#define AFE_BCON_MUTE_DIS	0x00010000
#define AFE_BCON_AFE_PWR	0x00008000
#define AFE_BCON_AUDINCLK	0x00000100
#define AFE_BCON_AUDINSTRT	0x00000020
#define AFE_BCON_AUDOUTSTRT	0x00000002
#define AFE_BCON_MODE		0x00000001
#define AFE_POWER_LSB_HSOCCOMP		(1 << 26)
#define AFE_POWER_HSOCCOMP_MASK		0x4000000
#define AFE_HS_GAIN_MASK 0x1F000000


/* AFE AUD2IDI CTRL */
#define AFE_AUD2IDICTRL_IDI_EN   0x80000000
/* CNT_OUT_MAX to maximum, bits 26 to 30 */
#define AFE_AUD2IDICTRL_CNT_MAX  (31 << 26)
/* CNT_OUT_MIN to minimum, bits 21 to 25 */
#define AFE_AUD2IDICTRL_CNT_MIN  (0 << 21)

#define AG6X0_AFE_IF_ID_OFFSET		0xC
#define AG6X0_AFE_IF_SWCID_OFFSET	0x1C
#define AG6X0_AFE_POWER			0x100
#define AG6X0_AFE_BCON			0x104
#define AG6X0_AFE_AUDOUTCTRL1		0x108
#define AG6X0_AFE_AUDOUTCTRL2		0x10C
#define AG6X0_AFE_AUDOUTCTRL3		0x110
#define AG6X0_AFE_AUDIOINCTRL		0x114
#define AG6X0_AFE_GAIN_OUT		0x118
#define AG6X0_AFE_GAIN_IN		0x11C
#define AG6X0_AFE_AUD2IDICTRL		0x120
#define AG6X0_AFE_MUTE			0x124
#define AG6X0_AFE_DIGMICCTRL		0x128

/* Default charge pump frequency in khz*/
#define AGOLD_AFE_CP_DEFAULT_FREQ_KHZ 13000

#define AGOLD_AFE_FIFO_MAX_SIZE_WORDS  32
enum agold_afe_requester {
	AGOLD_AFE_REQUESTER_ASOC, /* /< Request is from ASOC module */
	AGOLD_AFE_REQUESTER_ACCESSORY_IDENTIFICATION /* /< Request is from
					ACCESSORY identification module*/
};

enum agold_afe_hs_calib_event {
	AGOLD_AFE_PREPARE_CALIBRATION,
	AGOLD_AFE_STOP_CALIBRATION,
	AGOLD_AFE_CALIBRATE_LEFT_CH,
	AGOLD_AFE_CALIBRATE_RIGHT_CH
};

/* Enumeration for analog output gain applied in e.g. the AFE module  */
enum aud_analog_gain_value {
	AUD_ANALOG_GAIN_MAX_MINUS_18DB, /* Maximum analog gain minus 18 dB */
	AUD_ANALOG_GAIN_MAX_MINUS_15DB, /* Maximum analog gain minus 15 dB */
	AUD_ANALOG_GAIN_MAX_MINUS_12DB, /* Maximum analog gain minus 12 dB */
	AUD_ANALOG_GAIN_MAX_MINUS_9DB,  /* Maximum analog gain minus 9 dB */
	AUD_ANALOG_GAIN_MAX_MINUS_6DB,  /* Maximum analog gain minus 6 dB */
	AUD_ANALOG_GAIN_MAX_MINUS_3DB,  /* Maximum analog gain minus 3 dB */
	AUD_ANALOG_GAIN_MAX,            /* Maximum analog gain */
	AUD_ANALOG_GAIN_END,            /* Invalid, only used internally */
};

enum eAFE_GAIN_OUT_HSGAIN {
	AFE_GAIN_OUT_HSGAIN_HS_OFF = 0,
	AFE_GAIN_OUT_HSGAIN_HS_M9_1DB = 1,
	AFE_GAIN_OUT_HSGAIN_HS_M6_0DB = 4,
	AFE_GAIN_OUT_HSGAIN_HS_M3_1DB = 8,
	AFE_GAIN_OUT_HSGAIN_HS_P0_0DB = 14,
	AFE_GAIN_OUT_HSGAIN_HS_P2_3DB = 20,
	AFE_GAIN_OUT_HSGAIN_HS_P6_0DB = 28,
	AFE_GAIN_OUT_HSGAIN_HS_M8_0DB = 2,
	AFE_GAIN_OUT_HSGAIN_HS_M6_9DB = 3,
	AFE_GAIN_OUT_HSGAIN_HS_M4_4DB = 6,
	AFE_GAIN_OUT_HSGAIN_HS_M3_7DB = 7,
	AFE_GAIN_OUT_HSGAIN_HS_M5_2DB = 5,
	AFE_GAIN_OUT_HSGAIN_HS_M1_9DB = 10,
	AFE_GAIN_OUT_HSGAIN_HS_M0_9DB = 12,
	AFE_GAIN_OUT_HSGAIN_HS_M2_5DB = 9,
	AFE_GAIN_OUT_HSGAIN_HS_M1_4DB = 11,
	AFE_GAIN_OUT_HSGAIN_HS_M0_4DB = 13,
	AFE_GAIN_OUT_HSGAIN_HS_P0_8DB = 16,
	AFE_GAIN_OUT_HSGAIN_HS_P1_6DB = 18,
	AFE_GAIN_OUT_HSGAIN_HS_P2_9DB = 22,
	AFE_GAIN_OUT_HSGAIN_HS_P4_1DB = 24,
	AFE_GAIN_OUT_HSGAIN_HS_P0_4DB = 15,
	AFE_GAIN_OUT_HSGAIN_HS_P1_2DB = 17,
	AFE_GAIN_OUT_HSGAIN_HS_P1_9DB = 19,
	AFE_GAIN_OUT_HSGAIN_HS_P2_6DB = 21,
	AFE_GAIN_OUT_HSGAIN_HS_P3_5DB = 23,
	AFE_GAIN_OUT_HSGAIN_HS_P4_6DB = 25,
	AFE_GAIN_OUT_HSGAIN_HS_P5_6DB = 27,
	AFE_GAIN_OUT_HSGAIN_HS_P5_1DB = 26
};

#define AFE_NOF_EPHSLS_HW_GAIN_VALUES  (AUD_ANALOG_GAIN_END+1)

struct channels {
	u8 left; /* value to be programmed for left channel offset */
	u8 right; /* value to be programmed right channel offset */
};

struct afe_power_state {
	int hs_on;
	int direct_dac_on;
	int cp_freq;
	enum snd_soc_bias_level current_bias;
};

struct xgold_afe_reg {
	unsigned offset;
	unsigned shift;
	unsigned width;
	unsigned mask;
};

struct agold_afe_data {
	void __iomem *membase;
	void __iomem *fifobase;
	resource_size_t fifosize;
	struct snd_soc_codec *codec;
	struct idi_peripheral_device *dev;
	struct idi_channel_config rx_config;
	struct idi_channel_config tx_config;
	struct afe_power_state afe_pow;
	int codec_force_shutdown;
	int dac_dsp_transit;
	struct work_struct afe_trigger_work;
	int cmd;
	int stream;
	/* Added for NKERNEL */
	struct clk *clk;
	struct xgold_afe_reg rst;
	struct reset_control *aferst;
	/* Shadow registers */
	unsigned power_cfg;
	unsigned bcon_cfg;
	unsigned audoutctrl1_cfg;
	unsigned audoutctrl2_cfg;
	unsigned audoutctrl3_cfg;
	unsigned audinctrl_cfg;
	unsigned gainin_cfg;
	unsigned gainout_cfg;
	unsigned aud2idictrl_cfg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	int afe_in_samplerate;
};

struct agold_afe_data *agold_afe_get_private_data(void);
void agold_afe_update_bcon(void);

#define	afe_err(fmt, arg...) \
		pr_err("snd: afe: "fmt, ##arg)

#define	afe_debug(fmt, arg...) \
		pr_debug("snd: afe: "fmt, ##arg)


#endif

