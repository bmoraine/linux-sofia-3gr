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
enum agold_afe_requester {
	AGOLD_AFE_REQUESTER_ASOC, /* /< Request is from ASOC module */
	AGOLD_AFE_REQUESTER_ACCESSORY_IDENTIFICATION /* /< Request is from
					ACCESSORY identification module*/
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
	struct snd_soc_codec *codec;
	struct idi_peripheral_device *dev;
	struct idi_channel_config rx_config;
	struct idi_channel_config tx_config;
	struct afe_power_state afe_pow;
	int codec_force_shutdown;
	int dac_dsp_transit;
	/* Added for NKERNEL */
	struct clk *clk;
	struct xgold_afe_reg rst;
	struct reset_control *aferst;
	void __iomem *cgu;
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
};

struct agold_afe_data *agold_afe_get_private_data(void);
void agold_afe_update_bcon(void);

#define	afe_err(fmt, arg...) \
		pr_err("snd: afe: "fmt, ##arg)

#define	afe_debug(fmt, arg...) \
		pr_debug("snd: afe: "fmt, ##arg)


#endif

