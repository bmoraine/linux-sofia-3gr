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
 * Suryaprakash
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#ifdef CONFIG_PLATFORM_DEVICE_PM
#include <linux/platform_device_pm.h>
#endif
#include <sofia/vmm_pmic.h>

#include "intel_pmic_afe.h"

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
#include "afe_acc_det.h"
#endif

#ifdef CONFIG_SND_SOC_AGOLD_PT_DEBUG
#define AFE_NOF_BYTES_PER_REG 1
#endif

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE

#endif

/* TODO: Replace this with enum exposed by VMM once the
	VMM header files are pushed to linux sf mainline */
#define I2C_SLAVE_ADDRESS 0x6d

struct afe_reg {
	u16 reg_addr;
};

static const u8 afe_reg_cache[] = {
	0x76, /* PLLA_CTRL_1 */
	0x62, /* PLLA_CTRL_2 */
	0x6,  /* PLLA_CTRL_3 */
	0x0,  /* PLLA_STATUS */
	0x0,  /* PLLA_PWRCRTL_REG */
	0x0,  /* AFE_POWER1 */
	0x0,  /* AFE_POWER2 */
	0x0,  /* AFE_POWER3 */
	0x0,  /* AFE_POWER_STAT */
	0x0,  /* AFE_BCON1 */
	0x1,  /* AFE_BCON2 */
	0x0,  /* AFE_BCON3 */
	0x0,  /* AFE_BCON4 */
	0x0,  /* AFE_AUDOUTCTRL1A_REG */
	0x10, /* AFE_AUDOUTCTRL1B_REG */
	0x80, /* AFE_AUDOUTCTRL1C_REG */
	0x0,  /* AFE_AUDOUTCTRL1D_REG */
	0x0,  /* AFE_AUDOUTCTRL2A_REG */
	0x10, /* AFE_AUDOUTCTRL2B_REG */
	0x08, /* AFE_AUDOUTCTRL2C_REG */
	0x28, /* AFE_AUDOUTCTRL2D_REG */
	0xFF, /* AFE_AUDOUTCTRL3A_REG */
	0xFF, /* AFE_AUDOUTCTRL3B_REG */
	0xFF, /* AFE_AUDOUTCTRL3C_REG */
	0xFF, /* AFE_AUDOUTCTRL3D_REG */
	0x40, /* AFE_AUDIOINCTRL1_REG */
	0x00, /* AFE_AUDIOINCTRL2_REG */
	0x18, /* AFE_AUDIOINCTRL3_REG */
	0x02, /* AFE_AUDIOINCTRL4_REG */
	0x13, /* AFE_GAIN_OUT1_REG */
	0x13, /* AFE_GAIN_OUT2_REG */
	0x06, /* AFE_GAIN_OUT3_REG */
	0x0E, /* AFE_GAIN_OUT4_REG */
	0x5A, /* AFE_GAIN_IN1_REG digmic: 0dB, analog gain: 39 dB */
	0x02, /* AFE_GAIN_IN2_REG digmic: 0dB */
	0x0,  /* AFE_GAIN_IN3_REG */
	0x0,  /* AFE_GAIN_IN4_REG */
	0xC0, /* AFE_DIGMICCTRL_REG */
	0x0,  /* I2S_CTRL_LOW_REG */
	0x0,  /* I2S_CTRL_HIGH_REG */
	0x0,  /* I2S_CSEL_LOW_REG */
	0x0,  /* I2S_CSEL_HIGH_REG */
	0x0,  /* I2S_WRADDR_REG */
	0x0,  /* I2S_RDADDR_REG */
	0x0,  /* I2S_NUM0_LOW_REG */
	0x0,  /* I2S_NUM0_HIGH_REG */
	0x0,  /* I2S_DEN0_LOW_REG  */
	0x0,  /* I2S_DEN0_HIGH_REG */
	0x0,  /* I2S_DEN1_LOW_REG */
	0x0,  /* I2S_DEN1_HIGH_REG */
	0x0,  /* I2S_TXCLL_LOW_REG */
	0x0,  /* I2S_TXCLL_HIGH_REG */
	0x0,  /* I2S_TXCHL_LOW_REG */
	0x0,  /* I2S_TXCHL_HIGH_REG */
	0x0,  /* I2S_RXCLL_LOW_REG */
	0x0,  /* I2S_RXCLL_HIGH_REG */
	0x0,  /* I2S_RXCHL_LOW_REG */
	0x0,  /* I2S_RXCHL_HIGH_REG */
	0x0,  /* I2S_RXCONF_LOW_REG */
	0x0,  /* I2S_RXCONF_HIGH_REG */
	0x0,  /* I2S_TXCONF_LOW_REG */
	0x0,  /* I2S_TXCONF_HIGH_REG */
};

/* FIXME: remove ugly dupplication ASA A0 PMIC becomes deprecated */
static const u8 afe_reg_cache_a0[] = {
	0x76, /* PLLA_CTRL_3 */
	0x62, /* PLLA_CTRL_2 */
	0x6,  /* PLLA_CTRL_1 */
	0x0,  /* PLLA_STATUS */
	0x0,  /* PLLA_PWRCRTL_REG */
	0x0,  /* AFE_POWER1 */
	0x0,  /* AFE_POWER2 */
	0x0,  /* AFE_POWER3 */
	0x0,  /* AFE_POWER_STAT */
	0x0,  /* AFE_BCON1 */
	0x1,  /* AFE_BCON2 */
	0x0,  /* AFE_BCON3 */
	0x0,  /* AFE_BCON4 */
	0x0,  /* AFE_AUDOUTCTRL1A_REG */
	0x10, /* AFE_AUDOUTCTRL1B_REG */
	0x80, /* AFE_AUDOUTCTRL1C_REG */
	0x0,  /* AFE_AUDOUTCTRL1D_REG */
	0x0,  /* AFE_AUDOUTCTRL2A_REG */
	0x10, /* AFE_AUDOUTCTRL2B_REG */
	0x10, /* AFE_AUDOUTCTRL2C_REG */
	0x45, /* AFE_AUDOUTCTRL2D_REG */
	0xFF, /* AFE_AUDOUTCTRL3A_REG */
	0xFF, /* AFE_AUDOUTCTRL3B_REG */
	0xFF, /* AFE_AUDOUTCTRL3C_REG */
	0xFF, /* AFE_AUDOUTCTRL3D_REG */
	0x40, /* AFE_AUDIOINCTRL1_REG */
	0x00, /* AFE_AUDIOINCTRL2_REG */
	0x18, /* AFE_AUDIOINCTRL3_REG */
	0x03, /* AFE_AUDIOINCTRL4_REG */
	0x13, /* AFE_GAIN_OUT1_REG */
	0x13, /* AFE_GAIN_OUT2_REG */
	0x06, /* AFE_GAIN_OUT3_REG */
	0x0E, /* AFE_GAIN_OUT4_REG */
	0x5A, /* AFE_GAIN_IN1_REG digmic: 0dB, analog gain: 39 dB */
	0x02, /* AFE_GAIN_IN2_REG digmic: 0dB */
	0x0,  /* AFE_GAIN_IN3_REG */
	0x0,  /* AFE_GAIN_IN4_REG */
	0xC0, /* AFE_DIGMICCTRL_REG */
	0x0,  /* I2S_CTRL_LOW_REG */
	0x0,  /* I2S_CTRL_HIGH_REG */
	0x0,  /* I2S_CSEL_LOW_REG */
	0x0,  /* I2S_CSEL_HIGH_REG */
	0x0,  /* I2S_WRADDR_REG */
	0x0,  /* I2S_RDADDR_REG */
	0x0,  /* I2S_NUM0_LOW_REG */
	0x0,  /* I2S_NUM0_HIGH_REG */
	0x0,  /* I2S_DEN0_LOW_REG  */
	0x0,  /* I2S_DEN0_HIGH_REG */
	0x0,  /* I2S_DEN1_LOW_REG */
	0x0,  /* I2S_DEN1_HIGH_REG */
	0x0,  /* I2S_TXCLL_LOW_REG */
	0x0,  /* I2S_TXCLL_HIGH_REG */
	0x0,  /* I2S_TXCHL_LOW_REG */
	0x0,  /* I2S_TXCHL_HIGH_REG */
	0x0,  /* I2S_RXCLL_LOW_REG */
	0x0,  /* I2S_RXCLL_HIGH_REG */
	0x0,  /* I2S_RXCHL_LOW_REG */
	0x0,  /* I2S_RXCHL_HIGH_REG */
	0x0,  /* I2S_RXCONF_LOW_REG */
	0x0,  /* I2S_RXCONF_HIGH_REG */
	0x0,  /* I2S_TXCONF_LOW_REG */
	0x0,  /* I2S_TXCONF_HIGH_REG */
};

static const unsigned int afe_default_regs_addr[AFE_REG_END] = {
	PLLA_CTRL_1_REG_OFFSET,
	PLLA_CTRL_2_REG_OFFSET,
	PLLA_CTRL_3_REG_OFFSET,
	PLLA_STATUS_REG,
	PLLA_PWRCRTL_REG,
	AFE_POWER1_REG_OFFSET,
	AFE_POWER2_REG_OFFSET,
	AFE_POWER3_REG_OFFSET,
	AFE_POWER_STAT_REG_OFFSET,
	AFE_BCON1_REG_OFFSET,
	AFE_BCON2_REG_OFFSET,
	AFE_BCON3_REG_OFFSET,
	AFE_BCON4_REG_OFFSET,
	AFE_AUDOUTCTRL1A_REG_OFFSET,
	AFE_AUDOUTCTRL1B_REG_OFFSET,
	AFE_AUDOUTCTRL1C_REG_OFFSET,
	AFE_AUDOUTCTRL1D_REG_OFFSET,
	AFE_AUDOUTCTRL2A_REG_OFFSET,
	AFE_AUDOUTCTRL2B_REG_OFFSET,
	AFE_AUDOUTCTRL2C_REG_OFFSET,
	AFE_AUDOUTCTRL2D_REG_OFFSET,
	AFE_AUDOUTCTRL3A_REG_OFFSET,
	AFE_AUDOUTCTRL3B_REG_OFFSET,
	AFE_AUDOUTCTRL3C_REG_OFFSET,
	AFE_AUDOUTCTRL3D_REG_OFFSET,
	AFE_AUDIOINCTRL1_REG_OFFSET,
	AFE_AUDIOINCTRL2_REG_OFFSET,
	AFE_AUDIOINCTRL3_REG_OFFSET,
	AFE_AUDIOINCTRL4_REG_OFFSET,
	AFE_GAIN_OUT1_REG_OFFSET,
	AFE_GAIN_OUT2_REG_OFFSET,
	AFE_GAIN_OUT3_REG_OFFSET,
	AFE_GAIN_OUT4_REG_OFFSET,
	AFE_GAIN_IN1_REG_OFFSET,
	AFE_GAIN_IN2_REG_OFFSET,
	AFE_GAIN_IN3_REG_OFFSET,
	AFE_GAIN_IN4_REG_OFFSET,
	AFE_DIGMICCTRL_REG_OFFSET,
	I2S_CTRL_LOW_REG_OFFSET,
	I2S_CTRL_HIGH_REG_OFFSET,
	I2S_CSEL_LOW_REG_OFFSET,
	I2S_CSEL_HIGH_REG_OFFSET,
	I2S_WRADDR_REG_OFFSET,
	I2S_RDADDR_REG_OFFSET,
	I2S_NUM0_LOW_REG_OFFSET,
	I2S_NUM0_HIGH_REG_OFFSET,
	I2S_DEN0_LOW_REG_OFFSET,
	I2S_DEN0_HIGH_REG_OFFSET,
	I2S_DEN1_LOW_REG_OFFSET,
	I2S_DEN1_HIGH_REG_OFFSET,
	I2S_TXCLL_LOW_REG_OFFSET,
	I2S_TXCLL_HIGH_REG_OFFSET,
	I2S_TXCHL_LOW_REG_OFFSET,
	I2S_TXCHL_HIGH_REG_OFFSET,
	I2S_RXCLL_LOW_REG_OFFSET,
	I2S_RXCLL_HIGH_REG_OFFSET,
	I2S_RXCHL_LOW_REG_OFFSET,
	I2S_RXCHL_HIGH_REG_OFFSET,
	I2S_RXCONF_LOW_REG_OFFSET,
	I2S_RXCONF_HIGH_REG_OFFSET,
	I2S_TXCONF_LOW_REG_OFFSET,
	I2S_TXCONF_HIGH_REG_OFFSET,
};

static const char *afe_reg_name[AFE_REG_END] = {
	"PLLA_CTRL_1_REG",
	"PLLA_CTRL_2_REG",
	"PLLA_CTRL_3_REG",
	"PLLA_STATUS_REG",
	"PLLA_PWRCRTL_REG",
	"AFE_POWER1_REG",
	"AFE_POWER2_REG",
	"AFE_POWER3_REG",
	"AFE_POWER_STAT_REG",
	"AFE_BCON1_REG",
	"AFE_BCON2_REG",
	"AFE_BCON3_REG",
	"AFE_BCON4_REG",
	"AFE_AUDOUTCTRL1A_REG",
	"AFE_AUDOUTCTRL1B_REG",
	"AFE_AUDOUTCTRL1C_REG",
	"AFE_AUDOUTCTRL1D_REG",
	"AFE_AUDOUTCTRL2A_REG",
	"AFE_AUDOUTCTRL2B_REG",
	"AFE_AUDOUTCTRL2C_REG",
	"AFE_AUDOUTCTRL2D_REG",
	"AFE_AUDOUTCTRL3A_REG",
	"AFE_AUDOUTCTRL3B_REG",
	"AFE_AUDOUTCTRL3C_REG",
	"AFE_AUDOUTCTRL3D_REG",
	"AFE_AUDIOINCTRL1_REG",
	"AFE_AUDIOINCTRL2_REG",
	"AFE_AUDIOINCTRL3_REG",
	"AFE_AUDIOINCTRL4_REG",
	"AFE_GAIN_OUT1_REG",
	"AFE_GAIN_OUT2_REG",
	"AFE_GAIN_OUT3_REG",
	"AFE_GAIN_OUT4_REG",
	"AFE_GAIN_IN1_REG",
	"AFE_GAIN_IN2_REG",
	"AFE_GAIN_IN3_REG",
	"AFE_GAIN_IN4_REG",
	"AFE_DIGMICCTRL_REG",
	"I2S_CTRL_LOW_REG",
	"I2S_CTRL_HIGH_REG",
	"I2S_CSEL_LOW_REG",
	"I2S_CSEL_HIGH_REG",
	"I2S_WRADDR_REG",
	"I2S_RDADDR_REG",
	"I2S_NUM0_LOW_REG",
	"I2S_NUM0_HIGH_REG",
	"I2S_DEN0_LOW_REG",
	"I2S_DEN0_HIGH_REG",
	"I2S_DEN1_LOW_REG",
	"I2S_DEN1_HIGH_REG",
	"I2S_TXCLL_LOW_REG",
	"I2S_TXCLL_HIGH_REG",
	"I2S_TXCHL_LOW_REG",
	"I2S_TXCHL_HIGH_REG",
	"I2S_RXCLL_LOW_REG",
	"I2S_RXCLL_HIGH_REG",
	"I2S_RXCHL_LOW_REG",
	"I2S_RXCHL_HIGH_REG",
	"I2S_RXCONF_LOW_REG",
	"I2S_RXCONF_HIGH_REG",
	"I2S_TXCONF_LOW_REG",
	"I2S_TXCONF_HIGH_REG",
};

static int afe_is_reg_readable(struct snd_soc_codec *codec, unsigned int
reg)
{
	if (reg == PLLA_STATUS_REG)
		return 1;
	else
		return 0;
}

/* Pointer to afe codec driver local data */
/* The codec local driver data is stored as local pointer.
  The reason for not using the snd_soc_codec_set_drvdata()
  and snd_soc_codec_get_drvdata is the need to access the
  driver local data by accessory identification API where
  codec instance wont be available.
*/
static struct afe_data *afe_priv;

/* Returns pointer to the local codec driver data */
struct afe_data *afe_get_private_data(void)
{
	return afe_priv;
}

/* Stores the supplied pointer as local codec driver data pointer*/
static void afe_set_private_data(struct afe_data *afe_pvt)
{
	afe_priv = afe_pvt;
}

static const char * const afe_mic_mux_text[] = {
	"AMIC1", "AMIC2"
};

static const char * const afe_dacsel_ep_text[] = {"RDAC", "LDAC" };

static const char * const afe_dacsel_hs_text[] = {"LDAC", "RDAC" };

static const char * const afe_out_sel_text[] = {"NC", "LS"};

static const char * const afe_hsl_out_sel_text[] = {"NC", "HS"};

static const char * const afe_gen_sw_enum_text[] = {"OFF", "ON"};

static const char * const epout_mode_text[] = {"Differential", "Single Ended"};

static const char * const ep_robust_mode_text[] = { "Standard", "5V Robust" };

static const char * const mic_voltage_text[] = { "2.1", "2.2" };

static const char * const afe_hsps_text[] = { "Enable", "Disable" };

static const char * const afe_hsout_swg[] = { "1.3", "1.5" };

static const char * const afe_hsps_ramp[] = { "50ms", "100ms",
						"200ms", "400ms" };

static const DECLARE_TLV_DB_SCALE(DGAINCR_TLV, -2400, 1200, 1);
static const DECLARE_TLV_DB_SCALE(DGAINFR_TLV, -450, 0, 0);
static const DECLARE_TLV_DB_SCALE(DGAINCL_TLV, -2400, 1200, 1);
static const DECLARE_TLV_DB_SCALE(DGAINFL_TLV, -450, 0, 0);
static const DECLARE_TLV_DB_SCALE(EPGAIN_TLV, -1200, 1200, 1);
static const DECLARE_TLV_DB_SCALE(HSGAIN_TLV, -1200, 600, 1);
static const DECLARE_TLV_DB_SCALE(LSGAIN_TLV, 0, 2400, 1);
static const DECLARE_TLV_DB_SCALE(MICGAIN_TLV, -600, 3900, 1);

static const struct soc_enum afe_ep_mux_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1A_REG, 3, 2, afe_dacsel_ep_text);

static const struct soc_enum afe_hsl_mux_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1C_REG, 6, 2, afe_dacsel_hs_text);

static const struct soc_enum afe_mic_mux_enum =
SOC_ENUM_SINGLE(AFE_AUDIOINCTRL1_REG, 5, 2, afe_mic_mux_text);

static const struct soc_enum afe_out_sel_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(afe_out_sel_text),
					afe_out_sel_text);

static const struct soc_enum afe_hsl_out_sel_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(afe_out_sel_text),
					afe_hsl_out_sel_text);

static const struct soc_enum afe_hs_sw_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL2B_REG, 4, 2, afe_gen_sw_enum_text);

static const struct soc_enum afe_ep_on_mux_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(afe_gen_sw_enum_text),
					afe_gen_sw_enum_text);

static const struct snd_kcontrol_new afe_ep_sel =
SOC_DAPM_ENUM("Route", afe_ep_mux_enum);

static const struct snd_kcontrol_new afe_hsl_sel =
SOC_DAPM_ENUM("Route", afe_hsl_mux_enum);

static const struct snd_kcontrol_new afe_mic_sel =
SOC_DAPM_ENUM("Route", afe_mic_mux_enum);

static const struct snd_kcontrol_new afe_out_sel =
SOC_DAPM_ENUM_VIRT("Route", afe_out_sel_enum);

static const struct snd_kcontrol_new afe_hsl_out_sel =
SOC_DAPM_ENUM_VIRT("Route", afe_hsl_out_sel_enum);

static const struct snd_kcontrol_new afe_hs_sw =
SOC_DAPM_ENUM("Route", afe_hs_sw_enum);

static const struct snd_kcontrol_new afe_ep_sw =
SOC_DAPM_ENUM("Route", afe_ep_on_mux_enum);

static const struct snd_kcontrol_new afe_amic_sw =
SOC_DAPM_SINGLE("Switch", AFE_POWER1_REG, 4, 1, 0);

static const struct snd_kcontrol_new afe_dmic1_sw =
SOC_DAPM_SINGLE("Switch", AFE_DIGMICCTRL_REG, 2, 1, 0);

static const struct snd_kcontrol_new afe_dmic2_sw =
SOC_DAPM_SINGLE("Switch", AFE_DIGMICCTRL_REG, 3, 1, 0);

static const struct soc_enum afe_ep_out_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1A_REG, 4, 2, epout_mode_text);

static const struct soc_enum afe_ep_robust_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1A_REG, 2, 2, ep_robust_mode_text);

static const struct soc_enum afe_mic1_voltage_enum =
SOC_ENUM_SINGLE(AFE_AUDIOINCTRL4_REG, 0, 2, mic_voltage_text);

static const struct soc_enum afe_mic2_voltage_enum =
SOC_ENUM_SINGLE(AFE_AUDIOINCTRL4_REG, 0, 2, mic_voltage_text);

static const struct soc_enum afe_hsps_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1D_REG, 1, 2, afe_hsps_text);

static const struct soc_enum afe_hsout_swg_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1C_REG, 7, 2, afe_hsout_swg);

/*	static const struct soc_enum afe_hshcm_enum =
	SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1, 24, 2, afe_hshcm); */

static const struct soc_enum afe_hsps_ramp_enum =
SOC_ENUM_SINGLE(AFE_AUDOUTCTRL1D_REG, 2, 4, afe_hsps_ramp);

static const struct snd_kcontrol_new afe_snd_controls[] = {
	/* FIXME: SF_3G is setting gains for the left dac for RDAC widget.
	IS this because of an AFE bug? */
	SOC_SINGLE_TLV("RDAC Coarse Gain", AFE_GAIN_OUT1_REG, 2, 6, 0,
		DGAINCR_TLV),
	SOC_SINGLE_TLV("RDAC Fine Gain", AFE_GAIN_OUT1_REG, 0, 3, 0,
		DGAINFR_TLV),
	SOC_SINGLE_TLV("LDAC Coarse Gain", AFE_GAIN_OUT2_REG, 2, 6, 0,
		DGAINCL_TLV),
	SOC_SINGLE_TLV("LDAC Fine Gain", AFE_GAIN_OUT2_REG, 0, 3, 0,
		DGAINFR_TLV),
	SOC_SINGLE_TLV("Earpiece Gain", AFE_GAIN_OUT3_REG, 0, 8, 0,
		EPGAIN_TLV),
	SOC_SINGLE_TLV("Headset Gain", AFE_GAIN_OUT4_REG, 0, 28, 0,
		HSGAIN_TLV),
	SOC_SINGLE_TLV("LS Gain", AFE_GAIN_OUT3_REG, 4, 7, 0, LSGAIN_TLV),
	SOC_SINGLE_TLV("MIC Gain", AFE_GAIN_IN1_REG, 5, 7, 0, MICGAIN_TLV),
	SOC_ENUM("Earpiece Output Mode", afe_ep_out_enum),
	SOC_ENUM("Earpiece Robust Mode", afe_ep_robust_enum),
	SOC_ENUM("MIC1 Voltage", afe_mic1_voltage_enum),
	SOC_ENUM("MIC2 Voltage", afe_mic2_voltage_enum),
	SOC_ENUM("HS Pop Supression Enable", afe_hsps_enum),
	SOC_ENUM("HS Output SWG", afe_hsout_swg_enum),
	/* SOC_ENUM("HS High Current Mode", afe_hshcm_enum), */
	SOC_ENUM("HS Pop Suppression Ramp", afe_hsps_ramp_enum),
	SOC_SINGLE("LS ClassD Frequency Divider", AFE_AUDOUTCTRL2D_REG, 0,
		   255, 0),
	SOC_SINGLE("DMO Enable Switch", AFE_AUDIOINCTRL2_REG, 0, 1, 0),
	SOC_SINGLE("DMO Feeze Switch", AFE_AUDIOINCTRL2_REG, 1, 1, 0),
	/* Recommended value 100001b */
	SOC_SINGLE("DMO AVR value", AFE_AUDIOINCTRL2_REG, 2, 63, 0),
};

/* FIXME: remove A0 when deprecated */
static const struct snd_kcontrol_new afe_snd_controls_a0[] = {
	/* FIXME: SF_3G is setting gains for the left dac for RDAC widget.
	IS this because of an AFE bug? */
	SOC_SINGLE_TLV("RDAC Coarse Gain", AFE_GAIN_OUT1_REG, 2, 6, 0,
		DGAINCR_TLV),
	SOC_SINGLE_TLV("RDAC Fine Gain", AFE_GAIN_OUT1_REG, 0, 3, 0,
		DGAINFR_TLV),
	SOC_SINGLE_TLV("LDAC Coarse Gain", AFE_GAIN_OUT2_REG, 2, 6, 0,
		DGAINCL_TLV),
	SOC_SINGLE_TLV("LDAC Fine Gain", AFE_GAIN_OUT2_REG, 0, 3, 0,
		DGAINFR_TLV),
	SOC_SINGLE_TLV("Earpiece Gain", AFE_GAIN_OUT3_REG, 0, 8, 0,
		EPGAIN_TLV),
	SOC_SINGLE_TLV("Headset Gain", AFE_GAIN_OUT4_REG, 0, 28, 0,
		HSGAIN_TLV),
	SOC_SINGLE_TLV("LS Gain", AFE_GAIN_OUT3_REG, 4, 7, 0, LSGAIN_TLV),
	SOC_SINGLE_TLV("MIC Gain", AFE_GAIN_IN1_REG, 5, 7, 0, MICGAIN_TLV),
	SOC_ENUM("Earpiece Output Mode", afe_ep_out_enum),
	SOC_ENUM("Earpiece Robust Mode", afe_ep_robust_enum),
	SOC_ENUM("MIC1 Voltage", afe_mic1_voltage_enum),
	SOC_ENUM("MIC2 Voltage", afe_mic2_voltage_enum),
	SOC_ENUM("HS Pop Supression Enable", afe_hsps_enum),
	SOC_ENUM("HS Output SWG", afe_hsout_swg_enum),
	/* SOC_ENUM("HS High Current Mode", afe_hshcm_enum), */
	SOC_ENUM("HS Pop Suppression Ramp", afe_hsps_ramp_enum),
	SOC_SINGLE("LS ClassD Frequency Divider", AFE_AUDOUTCTRL2D_REG, 0,
		   255, 0),
};

static int afe_handle_codec_power(struct snd_soc_codec *codec,
		enum snd_soc_bias_level request_level)
{
	struct afe_data *afe =
		(struct afe_data *)snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	afe_debug("%s: request_level: %d\n", __func__, request_level);
#ifdef CONFIG_PLATFORM_DEVICE_PM
	if (SND_SOC_BIAS_OFF == request_level) {
		/* Disable AFE Power */
		ret = device_state_pm_set_state_by_name(
		&afe->plat_dev->dev,
		afe->pm_platdata->pm_state_D3_name);
	} else {
		/* Enable AFE Power */
		ret = device_state_pm_set_state_by_name(
			&afe->plat_dev->dev,
			afe->pm_platdata->pm_state_D0_name);

	}
	if (ret < 0)
		afe_err("\n %s: failed to set PM state error %d",
			__func__, ret);
	else
		afe->afe_pow.pm_state_lvl = request_level;
#endif

	udelay(5);
	return ret;
}
/* Update the AFE register cache */
static inline void afe_write_register_cache(struct snd_soc_codec *codec,
						unsigned int reg, u32 value)
{
	u8 *cache = codec->reg_cache;

	afe_debug("%s: configure AFE register index %d ba %p, val %X\n",
			__func__, reg, cache, value);
	cache[reg] = (u8)value;
}


#ifdef CONFIG_SND_SOC_AGOLD_PT_DEBUG
/* Read directly from the PMIC AFE register */
static inline unsigned int afe_pmic_reg_read(struct snd_soc_codec *codec,
					unsigned int reg)
{
	int ret;
	u32 data;
	u32 reg_address = 0;
	reg_address = I2C_SLAVE_ADDRESS << 24 |
		(afe_default_regs_addr[reg] & 0xffffff);
	ret = vmm_pmic_reg_read(reg_address, &data);
	if (ret < 0)
		return -EINVAL;

	return (unsigned int)data;
}

static int afe_get_reg_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	afe_debug("%s :\n", __func__);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = AFE_NOF_BYTES_PER_REG * AFE_REG_END;
	return 0;
}

/* Read out all supported AFE registers */
static int afe_get_reg_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	u8 reg = 0;
	u32 reg_val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct afe_data *afe = snd_soc_codec_get_drvdata(codec);

	for (reg = 0; reg < AFE_REG_END; reg++) {

		mutex_lock(&codec->mutex);
		reg_val = afe_pmic_reg_read(codec, reg);
		mutex_unlock(&codec->mutex);

		afe_debug("%s :reg %d addr %p val 0x%8.8x\n", __func__, reg,
			  offset, reg_val);

		uinfo->value.bytes.data[reg] =
		  (0xFF & reg_val);
	}
	return 0;
}

/* Set specific AFE register (first byte is the register ID,
   next byte the 8bit reg value ) */
static int afe_set_reg_val(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *uinfo)
{
	u8 reg = uinfo->value.bytes.data[0];
	u32 reg_val = 0;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct afe_data *afe = snd_soc_codec_get_drvdata(codec);

	afe_debug("%s :reg %d offset %x\n", __func__, reg, offset);

	reg_val = (uinfo->value.bytes.data[1]);
	mutex_lock(&codec->mutex);
	afe_write_register_cache(codec, reg, reg_val);
	afe_reg_write(codec, reg, reg_val);
	mutex_unlock(&codec->mutex);

	afe_debug("%s :reg_val 0x%8.8x\n", __func__, reg_val);

	return 0;
}

static const struct snd_kcontrol_new afe_snd_pt_control = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "AFE REG Control",
	.info = afe_get_reg_info,
	.get = afe_get_reg_val,
	.put = afe_set_reg_val,
};

#endif

/*BU_HACK register cache not working
read values from PMIC */

/* Read from AFE register cache */
static inline unsigned int afe_reg_read_cache(struct snd_soc_codec *codec,
					unsigned int reg)
{

	u32 reg_address = 0;
	u32 data = 0;
	int ret = 0;
	if (reg >= AFE_REG_END)
		/* TODO: Optimise by using cache as done on sf_3g */
		return -EIO;

	reg_address = I2C_SLAVE_ADDRESS << 24 |
	(afe_default_regs_addr[reg] & 0xffffff);

	ret = vmm_pmic_reg_read(reg_address, &data);

	if (ret < 0)
		afe_err("pmic read failed %d", ret);

	afe_debug("%s: reg %d return %x\n", __func__, reg, data);

	return (unsigned int)data;

}

/* Sets the requested values to AFE register */
static inline int afe_reg_write(struct snd_soc_codec *codec,
				unsigned int reg, unsigned int value)
{
	unsigned int final_value = (unsigned int)value;
	int ret;
	u32 reg_address = 0;
	enum snd_soc_bias_level pm_state_lvl;
	struct afe_data *afe =
		(struct afe_data *)snd_soc_codec_get_drvdata(codec);

	if (reg >= AFE_REG_END || afe == NULL)
		return -EIO;

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	if ((reg == AFE_AUDIOINCTRL3_REG)
			|| (reg == AFE_AUDIOINCTRL4_REG)
			|| (reg == AFE_BCON4_REG)) {
		/* Combine the values requested for accessory identification
		 * and MIC use case */
		pmic_afe_calculate_acc_settings(reg, value, &final_value);
	}
#endif

	pm_state_lvl = afe->afe_pow.pm_state_lvl;
	if (pm_state_lvl == SND_SOC_BIAS_OFF) {
		afe_debug("Trying to write reg while power domain is off !\n");
		afe_handle_codec_power(codec, SND_SOC_BIAS_STANDBY);
	}

	afe_write_register_cache(codec, reg, (u32)final_value);
	afe_debug("%s : AFE REG OFFSET = %s, value = 0x%0x\n",
		__func__, afe_reg_name[reg],
			final_value);
	reg_address = I2C_SLAVE_ADDRESS << 24 |
		(afe_default_regs_addr[reg] & 0xffffff);
	ret = vmm_pmic_reg_write(reg_address, (u32)final_value);
	if (ret < 0)
		return -EINVAL;

	if (pm_state_lvl == SND_SOC_BIAS_OFF)
		afe_handle_codec_power(codec, SND_SOC_BIAS_OFF);

	return 0;
}
static void afe_trigger_work_handler(struct work_struct *work)
{
	u8 reg = 0;
	struct afe_data *afe =
		container_of(work, struct afe_data, afe_trigger_work);
	struct snd_soc_codec *codec = afe->codec;

	afe_debug(" %s cmd %d:", __func__, afe->cmd);

	switch (afe->cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (afe->stream == SNDRV_PCM_STREAM_CAPTURE) {
			afe_debug("%s : Enabling In start bit\n",
				__func__);
			/* For capture stream, ensure that AUDINSTART is set */
			reg = snd_soc_read(codec,
				AFE_BCON1_REG);
			/* Enable AUDINSTRT */
			reg |= 1 << BCON_AUDINSTART_BIT;
			snd_soc_write(codec,
				AFE_BCON1_REG, reg);
			snd_soc_write(codec, I2S_CTRL_HIGH_REG, 0x0);
			reg = snd_soc_read(codec, I2S_CTRL_LOW_REG);
			reg |= (1 << TXSTART_BIT);
			snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);
		} else {
			afe_debug("Enabling I2S streaming");
			snd_soc_write(codec, I2S_CTRL_HIGH_REG, 0x0);
			reg = snd_soc_read(codec, I2S_CTRL_LOW_REG);
			reg |= (1 << RXSTART_BIT);
			snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (afe->stream == SNDRV_PCM_STREAM_CAPTURE) {
			afe_debug("%s : Disabling In start bit\n",
				__func__);
			reg = snd_soc_read(codec,
				AFE_BCON1_REG);
			/* Disable AUDINSTART */
			reg &= ~(BIT(BCON_AUDINSTART_BIT));
			snd_soc_write(codec,
				AFE_BCON1_REG, reg);
			snd_soc_write(codec, I2S_CTRL_HIGH_REG, 0x0);
			reg = snd_soc_read(codec, I2S_CTRL_LOW_REG);
			reg &= ~(BIT(TXSTART_BIT));
			snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);
		} else if (!codec->active) {
			afe_debug("Disabling I2S streaming");
			snd_soc_write(codec, I2S_CTRL_HIGH_REG, 0x0);
			reg = snd_soc_read(codec, I2S_CTRL_LOW_REG);
			reg &= ~(BIT(RXSTART_BIT));
			snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);
		}
		break;

	default:
		break;
	}
}

static inline int afe_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	int ret = 0;
	struct afe_data *afe = dev_get_drvdata(dev);

	if (!afe) {
		dev_err(dev, "Unable to retrieve afe data\n");
		return -EINVAL;
	}
	if (!IS_ERR_OR_NULL(afe->pinctrl)) {
		if (!IS_ERR_OR_NULL(state)) {
			ret = pinctrl_select_state(afe->pinctrl, state);
			if (ret)
				dev_err(dev, "%d: could not set pins\n",
						__LINE__);
		}
	}
	return ret;
}

static int afe_dacl_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	u8 reg = snd_soc_read(w->dapm->codec, AFE_AUDOUTCTRL1C_REG);
	afe_debug("%s:\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg |= 1 << NSLON_BIT;
		/*NSRON*/
		snd_soc_write(w->dapm->codec, AFE_AUDOUTCTRL1C_REG, reg);
		break;
	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;
	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s:SND_SOC_DAPM_POST_PMD\n", __func__);
		reg &= ~(1 << NSLON_BIT);
		/*NSROFF*/
		snd_soc_write(w->dapm->codec, AFE_AUDOUTCTRL1C_REG, reg);
		break;
	}
	return 0;
}

static int afe_dacr_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	u8 reg = snd_soc_read(w->dapm->codec, AFE_AUDOUTCTRL1C_REG);
	afe_debug("%s:\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg |= (1 << NSRON_BIT);
		/*NSRON*/
		snd_soc_write(w->dapm->codec, AFE_AUDOUTCTRL1C_REG, reg);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s:SND_SOC_DAPM_POST_PMD\n", __func__);
		reg &= ~(1 << NSRON_BIT);	/*NSROFF */
		snd_soc_write(w->dapm->codec, AFE_AUDOUTCTRL1C_REG, reg);
		break;
	}
	return 0;
}

static int afe_hs_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct afe_data *afe =
		snd_soc_codec_get_drvdata(w->dapm->codec);

	afe_debug("%s:\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe->afe_pow.hs_on = 1;
		afe_handle_codec_power(w->dapm->codec,
			w->dapm->codec->dapm.bias_level);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s : SND_SOC_DAPM_POST_PMD\n", __func__);
		afe->afe_pow.hs_on = 0;
		/* For bring up temporarily disable power down */
#if 0
		afe_handle_codec_power(w->dapm->codec,
			w->dapm->codec->dapm.bias_level);
#endif
		break;
	}

	return 0;

}

static int afe_mic_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u8 val = 0;
	int ret = 0;
	afe_debug("%s: Event : %d\n", __func__, event);

#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	ret = pmic_afe_acc_update_mic_status(event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		val = snd_soc_read(w->dapm->codec,
			AFE_AUDIOINCTRL3_REG);
		snd_soc_write(w->dapm->codec,
			AFE_AUDIOINCTRL3_REG, val);
		break;
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_PRE_PMD:
	case SND_SOC_DAPM_POST_PMD:
	default:
		/* Do nothing here */
		break;
	}
	return ret;
#else
	return 0;
#endif
}

static int afe_mic1_mode_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{

	afe_debug("%s: Event : %d\n", __func__, event);
#ifdef CONFIG_SND_SOC_AGOLD_ACC_DET_INTERFACE
	return pmic_afe_acc_update_mic1_mode_status(event);
#else
	return 0;
#endif
}

static int afe_aif_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	afe_debug("%s:\n", __func__);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		afe_debug("%s:SND_SOC_DAPM_PRE_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s:SND_SOC_DAPM_POST_PMD\n", __func__);
		break;
	}
	return 0;
}

static u32 afe_get_hsamp_ramp_time(u32 step)
{
	/*Optimum time in mili seconds. Used as default */
	u32 wait_time = 200000;

	switch (step) {
	case 0:
		wait_time = 50000;
		break;
	case 1:
		wait_time = 100000;
		break;
	case 2:
		wait_time = 200000;
		break;
	case 3:
		wait_time = 400000;
		break;
	default:
		break;
	}
	return wait_time;
}

static int afe_ep_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = 0;
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMU\n",
			__func__);
		reg = snd_soc_read(w->dapm->codec,
			AFE_AUDOUTCTRL1A_REG);
		reg |= (1 << EPLDOON_BIT); /* EPLDO ON */
		snd_soc_write(w->dapm->codec,
			AFE_AUDOUTCTRL1A_REG, reg);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s:SND_SOC_DAPM_POST_PMD\n", __func__);
		reg = snd_soc_read(w->dapm->codec,
			AFE_AUDOUTCTRL1A_REG);
		reg &= ~(1 << EPLDOON_BIT); /* EPLDO OFF */
		snd_soc_write(w->dapm->codec,
			AFE_AUDOUTCTRL1A_REG, reg);
		break;
	}
	return 0;
}

static int afe_hsr_amp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	u32 reg = 0;
	u32 wait_time;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMU\n", __func__);
		reg = snd_soc_read(w->dapm->codec,
			AFE_AUDOUTCTRL1C_REG);
		reg |= (1 << HSLDOON_BIT); /* HSLDO ON */
		snd_soc_write(w->dapm->codec,
			AFE_AUDOUTCTRL1C_REG, reg);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s:SND_SOC_DAPM_POST_PMD\n", __func__);
		/* Wait till ramp down is complete to avoid pops */
		reg = snd_soc_read(w->dapm->codec,
		AFE_AUDOUTCTRL1C_REG);
		wait_time = afe_get_hsamp_ramp_time((reg >> 26) & 0x3);
		afe_debug("%s:Waiting for %d us before turing off HS LDO\n",
			__func__, wait_time);
		usleep_range(wait_time, wait_time + 25000);

		reg &= ~(1 << HSLDOON_BIT); /* HSLDO OFF */
		snd_soc_write(w->dapm->codec,
			AFE_AUDOUTCTRL1C_REG, reg);
		break;
	}
	return 0;
}

static int afe_dmic_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	afe_debug("%s:\n", __func__);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMU:
		afe_debug("%s:SND_SOC_DAPM_POST_PMU\n", __func__);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		afe_debug("%s:SND_SOC_DAPM_PRE_PMD\n", __func__);
		break;

	case SND_SOC_DAPM_POST_PMD:
		afe_debug("%s:SND_SOC_DAPM_POST_PMD\n", __func__);
		break;
	}
	return 0;
}

/* AFE Codec DAPM*/
static const struct snd_soc_dapm_widget afe_dapm_widgets[] = {

	/* DAPM Outputs */
	SND_SOC_DAPM_OUTPUT("HSL"),
	SND_SOC_DAPM_OUTPUT("HSR"),

	SND_SOC_DAPM_OUTPUT("EARPIECE"),
	SND_SOC_DAPM_OUTPUT("LOUDSPEAKER"),

	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_INPUT("AMIC2"),

	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),

	SND_SOC_DAPM_PGA_E("Earpiece Amplifier", AFE_POWER1_REG, 0, 0,
			NULL, 0, afe_ep_amp_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("LS Amp Enable", AFE_POWER1_REG, 3, 0, NULL, 0),

	SND_SOC_DAPM_PGA_E("Headset Right Amp", AFE_POWER1_REG,
			2, 0, NULL, 0, afe_hsr_amp_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA("Headset Left Amp", AFE_POWER1_REG,
			1, 0, NULL, 0),

	SND_SOC_DAPM_VIRT_MUX("LS Output Route", SND_SOC_NOPM, 0, 0,
							&afe_out_sel),


	SND_SOC_DAPM_VIRT_MUX("EP Enable Switch", SND_SOC_NOPM, 0, 0,
			&afe_ep_sw),

	SND_SOC_DAPM_MUX_E("HSR Enable Switch", SND_SOC_NOPM, 0, 0,
			&afe_hs_sw, afe_hs_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_VIRT_MUX("HSL Enable Route", SND_SOC_NOPM, 0, 0,
							&afe_hsl_out_sel),



	SND_SOC_DAPM_SWITCH("DMIC1 Enable", SND_SOC_NOPM,
			0, 0, &afe_dmic1_sw),
	SND_SOC_DAPM_SWITCH("DMIC2 Enable", SND_SOC_NOPM,
			0, 0, &afe_dmic2_sw),

	SND_SOC_DAPM_SWITCH("AMIC On", SND_SOC_NOPM, 0, 0, &afe_amic_sw),

	SND_SOC_DAPM_DAC_E("DACL", "Playback Left", AFE_AUDOUTCTRL1B_REG,
			1, 0, afe_dacl_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("DACR", "Playback Right", AFE_AUDOUTCTRL1B_REG,
			2, 0, afe_dacr_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("EP Output Route", SND_SOC_NOPM, 0, 0,
			&afe_ep_sel),


	SND_SOC_DAPM_MUX("HSL Output Route", SND_SOC_NOPM, 0, 0,
			&afe_hsl_sel),

	SND_SOC_DAPM_AIF_OUT_E("Audio Capture", "Capture", 0,
			SND_SOC_NOPM, 0, 0, afe_aif_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("DMIC On",
			AFE_DIGMICCTRL_REG, 0, 0, NULL, 0,
			afe_dmic_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("MICIN Sel", SND_SOC_NOPM, 0, 0, &afe_mic_sel),

	SND_SOC_DAPM_SUPPLY("MIC1 BIAS", AFE_AUDIOINCTRL3_REG,
			4, 0, afe_mic1_mode_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("MIC2 BIAS", AFE_AUDIOINCTRL3_REG,
			6, 0, NULL,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("MIC Enable", AFE_AUDIOINCTRL4_REG,
			4, 0, NULL, 0, afe_mic_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD)
};

static const struct snd_soc_dapm_route afe_audio_map[] = {

	/* Speaker Output Map */
	{"LS Output Route", "LS", "DACL"},
	{"LS Amp Enable", NULL , "LS Output Route"},
	{"LOUDSPEAKER", NULL, "LS Amp Enable"},

	/* Headset Output Map */
	{"HSL Output Route", "LDAC", "DACL"},
	{"HSL Output Route", "RDAC", "DACR"},
	{"Headset Left Amp", NULL, "HSL Output Route"},
	{"HSL Enable Route", "HS", "Headset Left Amp"},
	{"HSL", NULL, "HSL Enable Route"},

	{"HSR", NULL, "Headset Right Amp"},
	{"Headset Right Amp", NULL , "HSR Enable Switch"},
	{"HSR Enable Switch", "ON", "DACR"},

	/* Earpiece Output Map */
	{"EP Output Route", "RDAC", "DACR"},
	{"EP Output Route", "LDAC", "DACL"},
	{"Earpiece Amplifier", NULL, "EP Output Route"},
	{"EP Enable Switch", "ON", "Earpiece Amplifier"},
	{"EARPIECE", NULL, "EP Enable Switch"},

	/* Audio Input Map */
	{"MICIN Sel", "AMIC1", "AMIC1"},
	{"MICIN Sel", "AMIC2", "AMIC2"},
	{"MIC Enable", NULL, "MICIN Sel"},
	{"AMIC On", "Switch", "MIC Enable"},
	{"Audio Capture", NULL, "AMIC On"},
	{"Audio Capture", NULL, "MIC1 BIAS"},
	{"Audio Capture", NULL, "MIC2 BIAS"},

	{"DMIC On", NULL, "DMIC1"},
	{"DMIC On", NULL, "DMIC2"},
	{"MIC Enable", NULL, "MIC1 BIAS"},
	{"DMIC On", NULL, "MIC Enable"},
	{"DMIC1 Enable", "Switch", "DMIC On"},
	{"DMIC2 Enable", "Switch", "DMIC On"},
	{"Audio Capture", NULL, "DMIC1 Enable"},
	{"Audio Capture", NULL, "DMIC2 Enable"},
};


static void afe_shutdown(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	u8 reg = 0;
	struct afe_data *afe_private_data = afe_get_private_data();
	/*Ensure that AFE stops the input data to DSP. This is required
	incase of delayed powered down where bias may still be kept
	while the receiver(DSP) is not powered and ready */
	afe_debug("%s:\n", __func__);
	mutex_lock(&dai->codec->mutex);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (0 == dai->capture_active) {
			afe_debug("%s: Disabling In start bit\n", __func__);
			reg = snd_soc_read(dai->codec, AFE_BCON1_REG);
			/*disable AUDINSTRT */
			reg &= ~(1 << BCON_AUDINSTART_BIT);
			snd_soc_write(dai->codec, AFE_BCON1_REG, reg);
		}
	}
	if ((0 == dai->playback_active) && (0 == dai->capture_active)) {
		afe_set_pinctrl_state(&afe_private_data->plat_dev->dev,
				afe_private_data->pins_inactive);
		snd_soc_write(dai->codec, AFE_PLLA_PWRCRTL_REG, 0x0);
		snd_soc_write(dai->codec, I2S_CTRL_HIGH_REG, 0x0);
		snd_soc_write(dai->codec, I2S_CTRL_LOW_REG, 0x0);

		reg = snd_soc_read(dai->codec, AFE_BCON1_REG);
		reg &= ~(1 << BCON_AUDOUTSTART_BIT);
		snd_soc_write(dai->codec, AFE_BCON1_REG, reg);
	}
	mutex_unlock(&dai->codec->mutex);
}

/* TODO: use workqueue to access PMIC.
	The existing VMM API is not atomic as expected by the
	trigger interface. Calling the trigger function results
	in "BUG: Scheduling while atomic" */
static int afe_trigger(struct snd_pcm_substream *substream,
			int cmd, struct snd_soc_dai *dai)
{
	struct afe_data *afe_private_data = afe_get_private_data();
	afe_private_data->stream = substream->stream;
	afe_private_data->cmd = cmd;
	/* VMM API's to access PMIC AFE registers are not atomic.
		Afe trigger callback is called from an atomic context.
		Ensure atomicity by scheduling the PMIC register
		access in the afe trigger callback in a worker thread.
	*/
	schedule_work(&afe_private_data->afe_trigger_work);

	return 0;
}

static int afe_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int audio_native = (int)snd_soc_card_get_drvdata(codec->card);
	struct afe_data *afe_private_data = afe_get_private_data();
	unsigned int format;
	u8 reg = 0;
#if 0
	u8 frame_per_supported[3] = {I2S_HAL_PERIOD_32_bits,
						I2S_HAL_PERIOD_48_bits,
						I2S_HAL_PERIOD_64_bits};
#endif
	unsigned int rate, sample_width, num_channels;
#if 0
	u8 i = 0;
	u8 frame_per_needed = 0;
#endif

	rate = params_rate(params);
	num_channels = params_channels(params);
	sample_width = snd_pcm_format_width(params_format(params));
	format = params_format(params);

	mutex_lock(&dai->codec->mutex);
	/* Program PLL to generate 196MHz from 26MHz clk input
		SD[21:0] = 0x66276*/
	if (1 >= dai->active) {
		afe_handle_codec_power(codec, SND_SOC_BIAS_PREPARE);
		/* Enable pins */
		afe_set_pinctrl_state(&afe_private_data->plat_dev->dev,
				afe_private_data->pins_default);
		/* Program PLL to generate 196MHz from 26MHz clk input
		SD[21:0] = 0x66276*/

		if (afe_private_data->pmic == PMIC_A0) {
			/* PLL register are swapped */
			snd_soc_write(codec, AFE_PLLA_CTRL_3_REG, 0x76);
			snd_soc_write(codec, AFE_PLLA_CTRL_2_REG, 0x62);
			snd_soc_write(codec, AFE_PLLA_CTRL_1_REG, 0x6);
		} else {
			snd_soc_write(codec, AFE_PLLA_CTRL_1_REG, 0x76);
			snd_soc_write(codec, AFE_PLLA_CTRL_2_REG, 0x62);
			snd_soc_write(codec, AFE_PLLA_CTRL_3_REG, 0x6);
		}

		snd_soc_write(codec, AFE_PLLA_PWRCRTL_REG, 0x3);
		/* Wait for 100us for the output clock to stabilize */
		udelay(100);
		reg = snd_soc_read(codec, AFE_PLLA_STATUS_REG);
		if (!(reg & 0x1))
			afe_err("\n %s: failed to set lock PLL", __func__);
		/* Power on AFE and set central biasing before writing
			into the AFE registers */
		reg = snd_soc_read(codec, AFE_BCON2_REG);
		reg &= ~(1 << BCON_AFE_PWRON_BIT);
		reg |= (1 << BCON_AFE_PWRON_BIT);
		snd_soc_write(codec, AFE_BCON2_REG, reg);
		snd_soc_write(codec, AFE_BCON4_REG, 0x80);

		afe_private_data->transmode = I2S_TRANS_MODE_NORMAL;
		/* Reset I2SON BIT before configuration of I2S control
		 registers. If one direction is active and the other
		 needs to be configured and activated reset of I2S ON
		 bit is not required.*/
		snd_soc_write(dai->codec, I2S_CTRL_HIGH_REG, 0x0);
		reg = snd_soc_read(codec, I2S_CTRL_LOW_REG);
		reg &= ~(1 << I2SON_BIT);
		snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);
		udelay(10);
		reg |= 1 << I2SON_BIT;
		snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);

		/* Configure i2s normal mode of operation */
		reg |= (0 << RXPCM_BIT) | (0 << TXPCM_BIT);
		/* Enable continuous transmission mode */
		reg |= 1 << RXPCMCON_BIT | 1 << TXPCMON_BIT;
		snd_soc_write(codec, I2S_CTRL_LOW_REG, reg);

		/* Configure PMIC AFE I2S HW in Slave mode */
		reg =
				I2S_HAL_CSEL_SLAVE_EXT << I2S_CLK1SEL_BIT |
				I2S_HAL_CSEL_SLAVE_EXT << I2S_CLK0SEL_BIT |
				I2S_HAL_CSEL_RX_CLK0 << I2S_RXCLKSEL_BIT |
				I2S_HAL_CSEL_TX_CLK0 << I2S_TXCLKSEL_BIT;
		snd_soc_write(codec, I2S_CSEL_HIGH_REG, 0x0);
		/* Temporary hack for bring up */
		snd_soc_write(codec, I2S_CSEL_LOW_REG, 0xaa);
		snd_soc_write(codec,
			I2S_RXCONF_HIGH_REG,
			I2S_DEFAULT_SETTING_RXCONF_HIGH_REG);
		snd_soc_write(codec,
			I2S_RXCONF_LOW_REG,
			I2S_DEFAULT_SETTING_RXCONF_LOW_REG);
		snd_soc_write(codec,
			I2S_TXCONF_HIGH_REG,
			I2S_DEFAULT_SETTING_TXCONF_HIGH_REG);
		snd_soc_write(codec,
			I2S_TXCONF_LOW_REG,
			I2S_DEFAULT_SETTING_TXCONF_LOW_REG);
	}
#if 0
	/* Configure Sample width */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		reg = snd_soc_read(codec, I2S_TXCONF_LOW_REG);
	else
		reg = snd_soc_read(codec, I2S_RXCONF_LOW_REG);
		reg &= ~(7 << SAMPLE_WIDTH_BIT);
		switch (sample_width) {
		case 16:
			reg |= I2S_HAL_SAMPLE_WIDTH_16;
			break;
		case 18:
			reg |= I2S_HAL_SAMPLE_WIDTH_18;
			break;
		case 20:
			reg |= I2S_HAL_SAMPLE_WIDTH_20;
			break;
		case 24:
			reg |= I2S_HAL_SAMPLE_WIDTH_24;
			break;
		case 32:
			reg |= I2S_HAL_SAMPLE_WIDTH_32;
			break;
		default:
		  return -EINVAL;
		}

	/* Configure Frame period */
	reg &= ~(3 << FRAME_PERIOD_BIT);
	frame_per_needed = 2*sample_width;
	for (i = 0; i < 3; i++) {
		if (frame_per_needed <= frame_per_supported[i]) {
			frame_per_needed = frame_per_supported[i];
			break;
		}
	}
	switch (frame_per_needed) {
	case I2S_HAL_PERIOD_64_bits:
		reg |= I2S_HAL_FRAME_PERIOD_64;
		break;
	case I2S_HAL_PERIOD_48_bits:
		reg |= I2S_HAL_FRAME_PERIOD_48;
		break;
	case I2S_HAL_PERIOD_32_bits:
		reg |= I2S_HAL_FRAME_PERIOD_32;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		snd_soc_write(codec,
			I2S_TXCONF_HIGH_REG,
			I2S_DEFAULT_SETTING_TXCONF_HIGH_REG);
		snd_soc_write(codec,
			I2S_TXCONF_LOW_REG, reg);
	} else {
		snd_soc_write(codec,
			I2S_RXCONF_HIGH_REG,
			I2S_DEFAULT_SETTING_RXCONF_HIGH_REG);
		snd_soc_write(codec,
			I2S_RXCONF_LOW_REG, reg);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		reg = snd_soc_read(codec, I2S_TXCONF_HIGH_REG);
	else
		reg = snd_soc_read(codec, I2S_RXCONF_HIGH_REG);

		reg &= ~(3 << STEREO_BIT);

	switch (num_channels) {
	case 2:
		reg |= I2S_HAL_CONFIG_STEREO_MODE_STEREO;
		break;
	case 1:
		reg |= I2S_HAL_CONFIG_STEREO_MODE_DUAL_MONO_LEFT;
		break;
	default:
		return -EINVAL;
	}
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		snd_soc_write(codec,
			I2S_TXCONF_HIGH_REG, reg);
	else
		snd_soc_write(codec,
			I2S_RXCONF_HIGH_REG, reg);
#endif
	reg = snd_soc_read(codec, AFE_BCON1_REG);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {

		/* Program the INRATE bits */
		reg &= ~(0x3 << AUDINRATE_BIT);

		reg |= (0x3 << AUDINRATE_BIT);

		snd_soc_write(codec, AFE_BCON1_REG, reg);

		if (audio_native)
			reg |= BIT(5); /* INSTART */
		snd_soc_write(codec, AFE_BCON1_REG, reg);
	} else {
		/*DSP always output 48KHz */
		reg &= ~(0x7 << AUDOUTRATE_BIT);

		/* BU_HACK 48k sampling has noise
		using 8k for now */
		reg |= (0x4 << AUDOUTRATE_BIT);
		snd_soc_write(codec, AFE_BCON1_REG, reg);

		if (audio_native)
			reg |= BIT(1); /* OUTSTART */
	}
	reg |= BIT(0); /* MODE */
	snd_soc_write(codec, AFE_BCON1_REG, reg);

	mutex_unlock(&dai->codec->mutex);
	return 0;
}

static int afe_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	int i = 0;
	u8 val;
	u8 *cache = codec->reg_cache;
	struct afe_data *afe = NULL;

	afe_debug("---> %s\n", __func__);

	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	codec->dapm.idle_bias_off = 1;

#ifdef CONFIG_SND_SOC_AGOLD_PT_DEBUG
#warning snd_soc_add_controls removed in v3.10. This should be in controls list
	snd_soc_add_controls(codec, &afe_snd_pt_control, 1);
#endif

	afe = (struct afe_data *)snd_soc_codec_get_drvdata(codec);
	if (afe != NULL) {
		afe->codec = codec;
		afe_set_private_data(afe);
	}

	/* Power on AFE before writing into the AFE registers */
	ret = afe_handle_codec_power(codec, SND_SOC_BIAS_STANDBY);
	if (0 > ret)
		return ret;

	for (i = 0; i < AFE_REG_END; i++) {
		if (i != PLLA_STATUS_REG)
			snd_soc_write(codec, i, (unsigned int)cache[i]);
		else
			continue;
	}

	snd_soc_write(codec, AFE_PLLA_PWRCRTL_REG, 0x3);
	/* Wait for 100us for the output clock to stabilize */
	udelay(100);
	val = snd_soc_read(codec, AFE_PLLA_STATUS_REG);
	if (!(val & 0x1))
		afe_err("\n %s: failed to set lock PLL", __func__);

	/* Trigger the AFE_AUDIOINCTRL register bit updates
	required for accessory detection */
	val = snd_soc_read(codec, AFE_AUDIOINCTRL1_REG);
	snd_soc_write(codec, AFE_AUDIOINCTRL1_REG, val);
	val = snd_soc_read(codec, AFE_AUDIOINCTRL3_REG);
	snd_soc_write(codec, AFE_AUDIOINCTRL3_REG, val);
	val = snd_soc_read(codec, AFE_AUDIOINCTRL4_REG);
	snd_soc_write(codec, AFE_AUDIOINCTRL4_REG, val);


	/* Power off AFE again */
	ret = afe_handle_codec_power(codec, SND_SOC_BIAS_OFF);

	afe_debug("<--- %s\n", __func__);
	return ret;
}

static int afe_set_bias(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	u8 reg = snd_soc_read(codec, AFE_BCON2_REG);
	int ret = 0;
	afe_debug("%s : level %d\n", __func__, level);

	switch (level) {

	case SND_SOC_BIAS_PREPARE:
		/* Enable power only if coming from standby */
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			afe_debug("%s : power on AFE\n", __func__);
			/* Enable power for AFE */
			ret = afe_handle_codec_power(codec, level);
			if (ret < 0) {
				afe_err("n %s: Failed to enable AFE power %d\n",
							__func__, ret);
			}

			/*Power on AFE */
			reg |= 1 << BCON_AFE_PWRON_BIT;
			snd_soc_write(codec, AFE_BCON2_REG, reg);

			/* Power on XBON central biasing */
			reg = snd_soc_read(codec, AFE_BCON4_REG);
			reg |= 1 << BCON_XBON_BIT;
			snd_soc_write(codec, AFE_BCON4_REG, reg);

			/* Enable BCON mode */
			reg = snd_soc_read(codec, AFE_BCON1_REG);
			reg |= 1 << BCON_MODE_BIT;
			snd_soc_write(codec, AFE_BCON1_REG, reg);

			/* Enable AUDOUTSTRT */
			reg |= 1 << BCON_AUDOUTSTART_BIT;
			snd_soc_write(codec, AFE_BCON1_REG, reg);
			/* Program Default Out gain */
			reg = snd_soc_read(codec, AFE_GAIN_OUT1_REG);
			snd_soc_write(codec, AFE_GAIN_OUT1_REG, reg);
			reg = snd_soc_read(codec, AFE_GAIN_OUT2_REG);
			snd_soc_write(codec, AFE_GAIN_OUT2_REG, reg);
			reg = snd_soc_read(codec, AFE_GAIN_OUT3_REG);
			snd_soc_write(codec, AFE_GAIN_OUT3_REG, reg);
			reg = snd_soc_read(codec, AFE_GAIN_OUT4_REG);
			snd_soc_write(codec, AFE_GAIN_OUT4_REG, reg);

			/* Program default In gain */
			reg = snd_soc_read(codec, AFE_GAIN_IN1_REG);
			snd_soc_write(codec, AFE_GAIN_IN1_REG, reg);
			reg = snd_soc_read(codec, AFE_GAIN_IN2_REG);
			snd_soc_write(codec, AFE_GAIN_IN2_REG, reg);
			reg = snd_soc_read(codec, AFE_GAIN_IN3_REG);
			snd_soc_write(codec, AFE_GAIN_IN3_REG, reg);
			reg = snd_soc_read(codec, AFE_GAIN_IN4_REG);
			snd_soc_write(codec, AFE_GAIN_IN4_REG, reg);
		}
		/* Report that codec is prepared even if it is
		   already in PREPARE state */
		codec->dapm.bias_level = level;
		break;

	case SND_SOC_BIAS_ON:
		codec->dapm.bias_level = level;
		break;

	case SND_SOC_BIAS_OFF:
		/*Turn off if codec is inactive or system shutting down*/
		if (((!codec->active))
			&& (codec->dapm.bias_level != SND_SOC_BIAS_OFF)) {
			afe_debug("%s : Shutdown AFE\n", __func__);

			/*disable AUDOUT STRT  */
			reg &= ~(1 << BCON_AUDOUTSTART_BIT);
			snd_soc_write(codec, AFE_BCON1_REG, reg);

			/*disable BCON mode */
			reg = snd_soc_read(codec, AFE_BCON1_REG);
			reg &= ~(1 << BCON_MODE_BIT);
			snd_soc_write(codec, AFE_BCON1_REG, reg);

			/* Disable AFE power */
			reg = snd_soc_read(codec, AFE_BCON2_REG);
			reg &= ~(1 << BCON_AFE_PWRON_BIT);
			snd_soc_write(codec, AFE_BCON2_REG, reg);
			reg = snd_soc_read(codec, AFE_BCON4_REG);
			reg &= ~(1 << BCON_XBON_BIT);
			snd_soc_write(codec, AFE_BCON4_REG, reg);

			/* Disable AFE  power */
			/* BU_HACK For bring up temporarily
			disable power down */
#if 0
			ret = afe_handle_codec_power(codec, level);
#endif
			if (ret < 0) {
				afe_err("failed to disable AFE clks %d\n", ret);
				break;
			}

			/* Report level only if AFE is powered down */
			codec->dapm.bias_level = level;
		}
		break;

	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_PREPARE) {
			/* If it is turndown sequence, standby only if all
				streams are closed and codec is not active*/
			if (!codec->active)
				codec->dapm.bias_level = level;

		}
		/* In turn ON sequence, put state to standby*/
		else
			codec->dapm.bias_level = level;
		break;

	default:
		break;
	}
	return ret;
}

static int afe_remove(struct snd_soc_codec *codec)
{
	struct afe_data *afe =
		snd_soc_codec_get_drvdata(codec);

	afe_debug("%s\n", __func__);

	if (afe != NULL) {
		kfree(afe);
		snd_soc_codec_set_drvdata(codec, NULL);
	}
	afe_set_bias(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int afe_suspend(struct snd_soc_codec *codec)
{
	afe_debug("%s\n", __func__);

	/* FIXME */

	return afe_set_bias(codec, SND_SOC_BIAS_OFF);
}

static int afe_resume(struct snd_soc_codec *codec)
{
	afe_debug("%s\n", __func__);

	/* FIXME */

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_afe = {
	.probe = afe_codec_probe,
	.remove = afe_remove,
	.suspend = afe_suspend,
	.resume = afe_resume,
	.read = afe_reg_read_cache,
	.write = afe_reg_write,
	.reg_word_size = sizeof(u8),
	.set_bias_level = afe_set_bias,
	.dapm_widgets = afe_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(afe_dapm_widgets),
	.dapm_routes = afe_audio_map,
	.num_dapm_routes = ARRAY_SIZE(afe_audio_map),
	.reg_cache_size = ARRAY_SIZE(afe_reg_cache),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = afe_reg_cache,
	.readable_register = afe_is_reg_readable,
	.num_controls = ARRAY_SIZE(afe_snd_controls),
	.controls = afe_snd_controls
};

/* FIXME: remove A0 when deprecated */
static struct snd_soc_codec_driver soc_codec_dev_afe_a0 = {
	.probe = afe_codec_probe,
	.remove = afe_remove,
	.suspend = afe_suspend,
	.resume = afe_resume,
	.read = afe_reg_read_cache,
	.write = afe_reg_write,
	.reg_word_size = sizeof(u8),
	.set_bias_level = afe_set_bias,
	.dapm_widgets = afe_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(afe_dapm_widgets),
	.dapm_routes = afe_audio_map,
	.num_dapm_routes = ARRAY_SIZE(afe_audio_map),
	.reg_cache_size = ARRAY_SIZE(afe_reg_cache_a0),
	.reg_word_size = sizeof(u8),
	.reg_cache_default = afe_reg_cache_a0,
	.readable_register = afe_is_reg_readable,
	.num_controls = ARRAY_SIZE(afe_snd_controls_a0),
	.controls = afe_snd_controls_a0
};

static struct snd_soc_dai_ops afe_dai_ops = {
	.hw_params = afe_hw_params,
	.trigger = afe_trigger,
	.shutdown = afe_shutdown,
};

static struct snd_soc_dai_driver afe_dai = {
	.name = "pmic_afe_i2s",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AFE_OUT_RATES,
		.formats = AFE_OUT_FORMAT,
		},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = AFE_IN_RATES,
		.formats = AFE_IN_FORMAT,
		},
	.ops = &afe_dai_ops,
};

static int afe_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	int res = 0;
	struct afe_data *afe;
	struct device_node *np = pdev->dev.of_node;

	afe_debug("%s\n", __func__);

	afe = kzalloc(sizeof(struct afe_data), GFP_KERNEL);
	if (afe == NULL) {
		afe_err("%s :Failed allocating driver data",
			__func__);
		return -ENOMEM;
	}

	afe->plat_dev = pdev;
	platform_set_drvdata(pdev, afe);
	afe->codec_force_shutdown = 0;

	afe->pmic =
		(of_find_property(np, "intel,pmic-B0", NULL)) ? PMIC_B0 :
		PMIC_A0;

	afe_debug("%s: use pmic revision %s\n", __func__,
			(afe->pmic == PMIC_B0) ? "B0" : "A0");

	/* pinctrl */
	afe->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(afe->pinctrl)) {
		afe->pinctrl = NULL;
		goto skip_pinctrl;
	}

	afe->pins_default = pinctrl_lookup_state(afe->pinctrl,
						 PINCTRL_STATE_DEFAULT);
	if (IS_ERR(afe->pins_default))
		afe_err("could not get default pinstate\n");

	afe->pins_sleep = pinctrl_lookup_state(afe->pinctrl,
					       PINCTRL_STATE_SLEEP);
	if (IS_ERR(afe->pins_sleep))
		afe_err("could not get sleep pinstate\n");

	afe->pins_inactive = pinctrl_lookup_state(afe->pinctrl,
					       "inactive");
	if (IS_ERR(afe->pins_inactive))
		afe_err("could not get inactive pinstate\n");

	(void)afe_set_pinctrl_state(&afe->plat_dev->dev,
				afe->pins_inactive);
skip_pinctrl:
	afe->pm_platdata = of_device_state_pm_setup(pdev->dev.of_node);
	if (IS_ERR(afe->pm_platdata)) {
		afe_err("Missing pm platdata properties\n");
		afe->pm_platdata = NULL;
	}

	if (afe->pm_platdata) {
#ifdef CONFIG_PLATFORM_DEVICE_PM
		res = platform_device_pm_set_class(pdev,
				afe->pm_platdata->pm_user_name);

		if (res < 0)
			afe_err("\n %s: failed to set PM class error %d",
				__func__, res);
#endif
	}

	/* Create workqueue to handle the vmm pmic API's which are
	 * non-atomic. Afe trigger expects only atomic operations */
	INIT_WORK(&afe->afe_trigger_work, afe_trigger_work_handler);
	platform_set_drvdata(pdev, afe);

	afe_debug("dev_set_drvdata ret %d\n", ret);
	afe->afe_pow.cp_freq = AFE_CP_DEFAULT_FREQ_KHZ;
	afe->afe_pow.direct_dac_on = 0;
	afe->afe_pow.hs_on = 0;

	if (afe->pmic == PMIC_A0)
		ret = snd_soc_register_codec(&pdev->dev,
				&soc_codec_dev_afe_a0, &afe_dai, 1);
	else
		ret = snd_soc_register_codec(&pdev->dev,
				&soc_codec_dev_afe, &afe_dai, 1);

	if (ret < 0) {
		afe_err("unable to register codec drivers\n");
		kfree(afe);
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	afe_set_private_data(afe);

	return ret;
}

static int afe_device_remove(struct platform_device *pdev)
{
	struct afe_data *afe = platform_get_drvdata(pdev);
	afe_debug("%s :\n", __func__);

	if (afe != NULL) {
		kfree(afe);
		platform_set_drvdata(pdev, NULL);
	}
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static void afe_drv_shutdown(struct platform_device *pdev)
{
	struct afe_data *afe_data = platform_get_drvdata(pdev);
	if (NULL != afe_data) {
		afe_debug("%s: Forcing AFE shutdown\n", __func__);
		afe_data->codec_force_shutdown = 1;
		(void)afe_set_bias(afe_data->codec, SND_SOC_BIAS_OFF);
	}
}

static struct of_device_id intel_pmic_afe_of_match[] = {
	{ .compatible = "intel,pmic_afe", },
	{ },
};

static struct platform_driver pmic_afe_driver = {
	.driver = {
		.name = "intel afe",
		.owner = THIS_MODULE,
		.of_match_table = intel_pmic_afe_of_match,
		},
	.probe = afe_device_probe,
	.remove = afe_device_remove,
	.shutdown = afe_drv_shutdown,
};

static int __init pmic_afe_modinit(void)
{
	int ret = 0;
	afe_debug("%s,\n", __func__);
	ret = platform_driver_register(&pmic_afe_driver);
	if (ret < 0) {
		afe_err("%s : unable to register afe driver\n", __func__);
		return -ENODEV;
	}
	return ret;
}

module_init(pmic_afe_modinit);

static void __exit pmic_afe_modexit(void)
{
	afe_debug("%s,\n", __func__);
	platform_driver_unregister(&pmic_afe_driver);
}

module_exit(pmic_afe_modexit);


MODULE_DESCRIPTION("INTEL AFE V1.0 driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
