/*
 * Component: INTEL PMIC AFE ALSA SOC driver
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

#ifndef __INTEL_AFE_H

#define __INTEL_AFE_H

#include <linux/regmap.h>
/*supported playback rate and format*/
#define AFE_OUT_RATES SNDRV_PCM_RATE_8000_48000
#define AFE_OUT_FORMAT SNDRV_PCM_FMTBIT_S16_LE

#define AFE_IN_RATES  SNDRV_PCM_RATE_8000_48000
#define AFE_IN_FORMAT SNDRV_PCM_FMTBIT_S16_LE


#define AFE_REG_RES_NAME	"register"
#define AFE_IN_RES_NAME		"afe_in"
#define AFE_OUT_RES_NAME	"afe_out"
#define DSP_IN_RES_NAME		"dsp_in"
#define DSP_OUT_RES_NAME	"dsp_out"


/* AFE Registers offsets */
#define PLLA_CTRL_1_REG_OFFSET		0x0
#define PLLA_CTRL_2_REG_OFFSET		0x01
#define PLLA_CTRL_3_REG_OFFSET		0x02
#define PLLA_STATUS_REG				0x08
#define PLLA_PWRCRTL_REG			0x09
#define AFE_POWER1_REG_OFFSET		0x40
#define AFE_POWER2_REG_OFFSET		0x41
#define AFE_POWER3_REG_OFFSET		0x42
#define AFE_POWER_STAT_REG_OFFSET   0x43
#define AFE_BCON1_REG_OFFSET        0x44
#define AFE_BCON2_REG_OFFSET        0x45
#define AFE_BCON3_REG_OFFSET        0x46
#define AFE_BCON4_REG_OFFSET        0x47
#define AFE_AUDOUTCTRL1A_REG_OFFSET 0x48
#define AFE_AUDOUTCTRL1B_REG_OFFSET 0x49
#define AFE_AUDOUTCTRL1C_REG_OFFSET 0x4A
#define AFE_AUDOUTCTRL1D_REG_OFFSET 0x4B
#define AFE_AUDOUTCTRL2A_REG_OFFSET 0x4C
#define AFE_AUDOUTCTRL2B_REG_OFFSET 0x4D
#define AFE_AUDOUTCTRL2C_REG_OFFSET 0x4E
#define AFE_AUDOUTCTRL2D_REG_OFFSET 0x4F
#define AFE_AUDOUTCTRL3A_REG_OFFSET 0x50
#define AFE_AUDOUTCTRL3B_REG_OFFSET 0x51
#define AFE_AUDOUTCTRL3C_REG_OFFSET 0x52
#define AFE_AUDOUTCTRL3D_REG_OFFSET 0x53
#define AFE_AUDIOINCTRL1_REG_OFFSET 0x54
#define AFE_AUDIOINCTRL2_REG_OFFSET 0x55
#define AFE_AUDIOINCTRL3_REG_OFFSET 0x56
#define AFE_AUDIOINCTRL4_REG_OFFSET 0x57
#define AFE_GAIN_OUT1_REG_OFFSET    0x60
#define AFE_GAIN_OUT2_REG_OFFSET    0x61
#define AFE_GAIN_OUT3_REG_OFFSET    0x62
#define AFE_GAIN_OUT4_REG_OFFSET    0x63
#define AFE_GAIN_IN1_REG_OFFSET     0x70
#define AFE_GAIN_IN2_REG_OFFSET     0x71
#define AFE_GAIN_IN3_REG_OFFSET     0x72
#define AFE_GAIN_IN4_REG_OFFSET     0x73
#define AFE_DIGMICCTRL_REG_OFFSET   0x80
#define I2S_CTRL_LOW_REG_OFFSET    0x90
#define I2S_CTRL_HIGH_REG_OFFSET   0x91
#define I2S_CSEL_LOW_REG_OFFSET    0x92
#define I2S_CSEL_HIGH_REG_OFFSET   0x93
#define I2S_WRADDR_REG_OFFSET      0x94
#define I2S_RDADDR_REG_OFFSET      0x95
#define I2S_NUM0_LOW_REG_OFFSET    0x96
#define I2S_NUM0_HIGH_REG_OFFSET   0x97
#define I2S_DEN0_LOW_REG_OFFSET    0x98
#define I2S_DEN0_HIGH_REG_OFFSET   0x99
#define I2S_DEN1_LOW_REG_OFFSET    0x9A
#define I2S_DEN1_HIGH_REG_OFFSET   0x9B
#define I2S_TXCLL_LOW_REG_OFFSET   0x9C
#define I2S_TXCLL_HIGH_REG_OFFSET  0x9D
#define I2S_TXCHL_LOW_REG_OFFSET   0x9E
#define I2S_TXCHL_HIGH_REG_OFFSET  0x9F
#define I2S_RXCLL_LOW_REG_OFFSET   0xA0
#define I2S_RXCLL_HIGH_REG_OFFSET  0xA1
#define I2S_RXCHL_LOW_REG_OFFSET   0xA2
#define I2S_RXCHL_HIGH_REG_OFFSET  0xA3
#define I2S_RXCONF_LOW_REG_OFFSET  0xB0
#define I2S_RXCONF_HIGH_REG_OFFSET 0xB1
#define I2S_TXCONF_LOW_REG_OFFSET  0xC0
#define I2S_TXCONF_HIGH_REG_OFFSET 0xC1

#define AFE_CACHEREGNUM (AFE_GAIN_IN4_REG_OFFSET - AFE_POWER1_REG_OFFSET + 1)

#define AUDINRATE_BIT 6
#define AUDOUTRATE_BIT 2
#define RXPCM_BIT 6
#define TXPCM_BIT 5
#define RXPCMCON_BIT 4
#define TXPCMON_BIT 3
#define RXSTART_BIT 2
#define TXSTART_BIT  1
#define I2SON_BIT 0
#define SAMPLE_WIDTH_BIT 5
#define FRAME_PERIOD_BIT 3
#define STEREO_BIT 1
#define I2S_TXCLKSEL_BIT 0
#define I2S_CLK0SEL_BIT 2
#define I2S_RXCLKSEL_BIT 4
#define I2S_CLK1SEL_BIT 6
#define BCON_MODE_BIT 0
#define BCON_AUDOUTSTART_BIT 1
#define BCON_AUDINSTART_BIT 5
#define BCON_AFE_PWRON_BIT 7
#define BCON_XBON_BIT 7
#define NSLON_BIT 0
#define NSRON_BIT 1
#define EPLDOON_BIT 1
#define HSLDOON_BIT 2

/* Default charge pump frequency in khz*/
#define AFE_CP_DEFAULT_FREQ_KHZ 13000

#define AFE_FIFO_MAX_SIZE_WORDS  32

#define I2S_HAL_CONFIG_STEREO_MODE_DUAL_MONO_LEFT (3 << STEREO_BIT)
#define I2S_HAL_CONFIG_STEREO_MODE_DUAL_MONO_RIGHT (2 << STEREO_BIT)
#define I2S_HAL_CONFIG_STEREO_MODE_STEREO (0 << STEREO_BIT)
#define I2S_HAL_FRAME_PERIOD_32 (2 << FRAME_PERIOD_BIT)
#define I2S_HAL_FRAME_PERIOD_48 (1 << FRAME_PERIOD_BIT)
#define I2S_HAL_FRAME_PERIOD_64 (0 << FRAME_PERIOD_BIT)
#define I2S_HAL_PERIOD_32_bits (32)
#define I2S_HAL_PERIOD_48_bits (48)
#define I2S_HAL_PERIOD_64_bits (64)
#define I2S_HAL_SAMPLE_WIDTH_16 (0 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_PCM_32 (1 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_PCM_48 (2 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_PCM_64 (3 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_18 (4 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_20 (5 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_24 (6 << SAMPLE_WIDTH_BIT)
#define I2S_HAL_SAMPLE_WIDTH_32 (7 << SAMPLE_WIDTH_BIT)

/* CLK0_SEL defines */
#define I2S_HAL_CSEL_MASTER_F0 (0)
#define I2S_HAL_CSEL_MASTER_F1 (1)
#define I2S_HAL_CSEL_SLAVE_EXT (2)

/* CLK_tx and CLK_rx defines */
#define I2S_HAL_CSEL_TX_F0 (0)
#define I2S_HAL_CSEL_TX_CLK1 (1)
#define I2S_HAL_CSEL_TX_CLK0 (2)
#define I2S_HAL_CSEL_TX_F1 (3)

#define I2S_HAL_CSEL_RX_F1 (0)
#define I2S_HAL_CSEL_RX_CLK0 (1)
#define I2S_HAL_CSEL_RX_CLK1 (2)
#define I2S_HAL_CSEL_RX_F0 (3)
/*
*  0,  :bit 0 'data_alignment'- 0: left aligned, 1: right aligned
*  0,  :bit 1-4 'reserved'
*  0,  :bit 5 'clk1_out' - 0: low, 1: high
*  0,  :bit 6 'clk1_cont' - 0: stopped and set to clk1_out,
*							1: continues running
*  1   :bit 7 'wa1_len' - 0: 1 cycle, 1: 2 cycles
*/
#define I2S_DEFAULT_SETTING_RXCONF_HIGH_REG 0x0

/*
*  0,  :bit 0 'edge'- 0: falling(normal)/rising(pcm),
*						1: rising/falling
*  1,  :bit 1 'wa_delay' - 0: at edge, 1: one clock after edge
*  1,  :bit 2 'wa_polarity' - 0: wa=1 means right, 1: wa=1 means left
*  2,  :bit 3-4 'frame_period' - 0: 64 clocks, 1: 48, 2: 32
*  0   :bit 5-7 'sample_width' - configured at interface level
*
*  Polarity bit set to compensate for left/right swap I2S IP block on
*  PMIC RX side.
*/
#define I2S_DEFAULT_SETTING_RXCONF_LOW_REG 0x16

/*
*  0,  :bit 0 'data_alignment'- 0: left aligned, 1: right aligned
*  3,  :bit 1-2 'mono' - configured at interface level
*  0,  :bit 3 'mute_l' - 0: active, 1: muted
*  0,  :bit 4 'mute_r' - 0: active, 1: muted,
*  0,  :bit 5 'clk0_out' - 0: low, 1: high
*  0,  :bit 6 'clk0_cont' - 0: stopped and set to clk0_out,
*                            1: continues running
*  1   :bit 7 'wa0_len' - 0: 1 cycle, 1: 2 cycles
*/
#define I2S_DEFAULT_SETTING_TXCONF_HIGH_REG 0x00

/*
*  1,  :bit 0 'edge'- 0: falling(normal)/rising(pcm),
*						1: rising/falling
*  1,  :bit 1 'wa_delay' - 0: at edge, 1: one clock after edge
*  0,  :bit 2 'wa_polarity' - 0: wa=1 means right, 1: wa=1 means left
*  2,  :bit 3-4 'frame_period' - 0: 64 clocks, 1: 48, 2: 32
*  0   :bit 5-7 'sample_width' - configured at interface level
*/
#define I2S_DEFAULT_SETTING_TXCONF_LOW_REG 0x13

enum afe_requester {
	AFE_REQUESTER_ASOC, /* /< Request is from ASOC module */
	AFE_REQUESTER_ACCESSORY_IDENTIFICATION /* /< Request is from
					ACCESSORY identification module*/
};

enum afe_reg_list {
	AFE_PLLA_CTRL_1_REG,
	AFE_PLLA_CTRL_2_REG,
	AFE_PLLA_CTRL_3_REG,
	AFE_PLLA_STATUS_REG,
	AFE_PLLA_PWRCRTL_REG,
	AFE_POWER1_REG,
	AFE_POWER2_REG,
	AFE_POWER3_REG,
	AFE_POWER_STAT_REG,
	AFE_BCON1_REG,
	AFE_BCON2_REG,
	AFE_BCON3_REG,
	AFE_BCON4_REG,
	AFE_AUDOUTCTRL1A_REG,
	AFE_AUDOUTCTRL1B_REG,
	AFE_AUDOUTCTRL1C_REG,
	AFE_AUDOUTCTRL1D_REG,
	AFE_AUDOUTCTRL2A_REG,
	AFE_AUDOUTCTRL2B_REG,
	AFE_AUDOUTCTRL2C_REG,
	AFE_AUDOUTCTRL2D_REG,
	AFE_AUDOUTCTRL3A_REG,
	AFE_AUDOUTCTRL3B_REG,
	AFE_AUDOUTCTRL3C_REG,
	AFE_AUDOUTCTRL3D_REG,
	AFE_AUDIOINCTRL1_REG,
	AFE_AUDIOINCTRL2_REG,
	AFE_AUDIOINCTRL3_REG,
	AFE_AUDIOINCTRL4_REG,
	AFE_GAIN_OUT1_REG,
	AFE_GAIN_OUT2_REG,
	AFE_GAIN_OUT3_REG,
	AFE_GAIN_OUT4_REG,
	AFE_GAIN_IN1_REG,
	AFE_GAIN_IN2_REG,
	AFE_GAIN_IN3_REG,
	AFE_GAIN_IN4_REG,
	AFE_DIGMICCTRL_REG,
	I2S_CTRL_LOW_REG,
	I2S_CTRL_HIGH_REG,
	I2S_CSEL_LOW_REG,
	I2S_CSEL_HIGH_REG,
	I2S_WRADDR_REG,
	I2S_RDADDR_REG,
	I2S_NUM0_LOW_REG,
	I2S_NUM0_HIGH_REG,
	I2S_DEN0_LOW_REG,
	I2S_DEN0_HIGH_REG,
	I2S_DEN1_LOW_REG,
	I2S_DEN1_HIGH_REG,
	I2S_TXCLL_LOW_REG,
	I2S_TXCLL_HIGH_REG,
	I2S_TXCHL_LOW_REG,
	I2S_TXCHL_HIGH_REG,
	I2S_RXCLL_LOW_REG,
	I2S_RXCLL_HIGH_REG,
	I2S_RXCHL_LOW_REG,
	I2S_RXCHL_HIGH_REG,
	I2S_RXCONF_LOW_REG,
	I2S_RXCONF_HIGH_REG,
	I2S_TXCONF_LOW_REG,
	I2S_TXCONF_HIGH_REG,
	AFE_REG_END
};

enum I2sTransModeEnum {
	I2S_TRANS_MODE_PCM,
	I2S_TRANS_MODE_NORMAL,
	I2S_TRANS_MODE_PCM_BURST
};

struct afe_power_state {
	int hs_on;
	int direct_dac_on;
	int cp_freq;
	enum snd_soc_bias_level pm_state_lvl;
};

enum pmic_rev {
	PMIC_A0,
	PMIC_B0,
};

struct afe_data {
	struct platform_device *plat_dev;
	struct regmap *regmap;
	struct snd_soc_codec *codec;
	struct afe_power_state afe_pow;
	int codec_force_shutdown;
	enum I2sTransModeEnum transmode;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
	struct work_struct afe_trigger_work;
	int cmd;
	int stream;
	enum pmic_rev pmic;
};

struct afe_data *afe_get_private_data(void);
void afe_update_bcon(void);

#define afe_debug pr_debug
#define afe_err pr_err

#endif
