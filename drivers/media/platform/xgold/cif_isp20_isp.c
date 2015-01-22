/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - ISP related functionality
 *
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/videobuf-core.h>
#include <media/videobuf-vmalloc.h>	/*for ISP statistics */
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include "cif_isp20_regs.h"
#include "cif_isp20_isp.h"
#include "cif_isp20_pltfrm.h"
#include "cif_isp20.h"

#define _GET_ 0
#define _SET_ 1

#define ISP_DEV_NAME "CIF_ISP"

#define CIFISP_BDM_BYPASS_EN(val)  ((val) << 10)
#define CIFISP_FLT_EN(val)     (val | 1)
#define CIFISP_HST_MANUAL_RST  (1 << 11)
#define CIFISP_HST_PREDIV_SET(val) ((val) << 3)
#define CIFISP_HST_AUTO_RST_EN (1 << 10)

#define CIF_ISP_CTRL_ISP_GAMMA_IN_ENA (1<<6)
#define CIF_ISP_CTRL_ISP_AWB_ENA  (1<<7)
#define CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA (1<<11)

#define CIFISP_AWB_GRID_AUTO_INC_DIS  (1 << 4)

#define CIFISP_AWB_GAIN_R_SET(val)  ((val) << 16)
#define CIFISP_AWB_GAIN_R_READ(val)  ((val) >> 16)
#define CIFISP_AWB_GAIN_B_READ(val)  ((val) & 0xFFFF)

#define CIFISP_AWB_RGB_MEAS_PNT_SET(val)  ((val) << 3)
#define CIFISP_AWB_B_SAT_SET(val)  ((val) << 16)
#define CIFISP_AWB_B_SAT_READ(val)  ((val) >> 16)
#define CIFISP_AWB_R_SAT_READ(val)  ((val) & 0xFFFF)

#define CIFISP_AWB_YMAX_CMP_EN   (1 << 2)
#define CIFISP_AWB_REF_CR_SET(val)  ((val) << 8)
#define CIFISP_AWB_REF_CR_READ(val)  ((val) >> 8)
#define CIFISP_AWB_REF_CB_READ(val)  ((val) & 0xFF)
#define CIFISP_AWB_MAX_CS_SET(val)  ((val) << 8)
#define CIFISP_AWB_MAX_CS_READ(val)  (((val) >> 8) & 0xFF)
#define CIFISP_AWB_MIN_C_READ(val)  ((val) & 0xFF)
#define CIFISP_AWB_MIN_Y_SET(val)  ((val) << 16)
#define CIFISP_AWB_MIN_Y_READ(val)  (((val) >> 16) & 0xFF)
#define CIFISP_AWB_MAX_Y_SET(val)  ((val) << 24)
#define CIFISP_AWB_MAX_Y_READ(val)  ((val) >> 24)
#define CIFISP_AWB_MODE_READ(val)  ((val) & 3)
#define CIFISP_AWB_YMAX_READ(val)  (((val) >> 2) & 1)
#define CIFISP_AWB_RGBPOINT_READ(val)  (((val) >> 3) & 1)

#define CIFISP_AWB_VOFF_SET(val) ((val) << 16)
#define CIFISP_AWB_VOFF_READ(val) ((val) >> 16)
#define CIFISP_AWB_HOFF_READ(val) ((val) & 0xFFFF)
#define CIFISP_AWB_VDIM_SET(val) ((val) << 16)
#define CIFISP_AWB_VDIM_READ(val) ((val) >> 16)
#define CIFISP_AWB_HDIM_READ(val) ((val) & 0xFFFF)

#define CIFISP_AWB_VSIZE_SET(val) ((val) << 16)
#define CIFISP_AWB_VSIZE_READ(val) ((val) >> 16)
#define CIFISP_AWB_HSIZE_READ(val) ((val) & 0xFFFF)

#define CIFISP_AWB_VDIST_SET(val) ((val) << 16)
#define CIFISP_AWB_VDIST_READ(val) ((val) >> 16)
#define CIFISP_AWB_HDIST_READ(val) ((val) & 0xFFFF)

#define CIFISP_AWB_GET_MEAN_CR(val) ((val) & 0xFF)
#define CIFISP_AWB_GET_MEAN_CB(val) (((val) >> 8) & 0xFF)
#define CIFISP_AWB_GET_MEAN_Y(val)  (((val) >> 16) & 0xFF)
#define CIFISP_AWB_GET_MEAN_R(val)  (((val) >> 16) & 0xFFF)
#define CIFISP_AWB_GET_MEAN_B(val)  ((val) & 0xFFF)
#define CIFISP_AWB_GET_MEAN_G(val)  ((val) & 0xFFF)
#define CIFISP_AWB_GET_PIXEL_CNT(val) ((val) & 0xFFFFFFF)

#define CIFISP_LSC_GRADH_SET(val)  ((val) << 11)
#define CIFISP_LSC_SECTH_SET(val)  ((val) << 10)

#define CIFISP_FILTER_COEFF_RESERVED 0xFF808080
#define CIFISP_FILTER_LUM_WEIGHT_RESERVED 0xFF800000
#define CIFISP_FILTER_MODE(val) ((val) << 1)
#define CIFISP_FILTER_SHARP_MODE(val) ((val) << 3)
#define CIFISP_FILTER_CH_V_MODE(val) ((val) << 4)
#define CIFISP_FILTER_CH_H_MODE(val) ((val) << 5)
#define CIFISP_FILTER_MODE_READ(val) (((val) >> 1) & 1)
#define CIFISP_FILTER_SHARP_MODE_READ(val) (((val) >> 3) & 1)
#define CIFISP_FILTER_CH_V_MODE_READ(val) (((val) >> 4) & 1)
#define CIFISP_FILTER_CH_H_MODE_READ(val) (((val) >> 5) & 1)

#define CIFISP_CTK_COEFF_RESERVED 0xFFFFF800
#define CIFISP_XTALK_OFFSET_RESERVED 0xFFFFF000

#define CIFISP_GOC_RESERVED 0xFFFFF800
#define CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA_READ(value) (((value) >> 11) & 1)

#define CIFISP_BPC_HOT_CORR_EN   (1 << 1)
#define CIFISP_BPC_DEAD_CORR_EN   (1 << 2)
#define CIFISP_BPC_REP_APR_BILIN   (1 << 3)
#define CIFISP_BPC_HOT_TURB_ADJ_EN   (1 << 8)
#define CIFISP_BPC_DEAD_TURB_ADJ_EN   (1 << 12)
#define CIFISP_BPC_HOT_SIGN_SENS_EN (1 << 16)
#define CIFISP_BPC_DEAD_SIGN_SENS_EN (1 << 24)
#define CIFISP_BPC_HOT_TURB_SHIFT(value) ((value) << 9)
#define CIFISP_BPC_DEAD_TURB_SHIFT(value) ((value) << 13)
#define CIFISP_BPC_HOT_GRAD_TRIG(value) ((value) << 17)
#define CIFISP_BPC_DEAD_GRAD_TRIG(value) ((value) << 25)

#define CIFISP_BLS_MODE_MEASURED (1 << 1)
#define CIFISP_BLS_WINDOW_1 (1 << 2)
#define CIFISP_BLS_WINDOW_2 (1 << 3)

#define CIFISP_DEGAMMA_X_RESERVED ((1 << 31)|(1 << 27)|(1 << 23)|(1 << 19)|\
	(1 << 15)|(1 << 11)|(1 << 7)|(1 << 3))
#define CIFISP_DEGAMMA_Y_RESERVED 0xFFFFF000

#define CIFISP_EXP_CTRL_AUTOSTOP(val) ((val) << 1)

#define CIFISP_CPROC_CTRL_RESERVED 0xFFFFFFFE
#define CIFISP_CPROC_CONTRAST_RESERVED 0xFFFFFF00
#define CIFISP_CPROC_BRIGHTNESS_RESERVED 0xFFFFFF00
#define CIFISP_CPROC_HUE_RESERVED 0xFFFFFF00
#define CIFISP_CPROC_SATURATION_RESERVED 0xFFFFFF00
#define CIFISP_CPROC_MACC_RESERVED 0xE000E000
#define CIFISP_CPROC_TONE_RESERVED 0xF000
#define CIFISP_CPROC_TONE_Y(value) ((value)<<16)
#define CIFISP_CPROC_TONE_C(value) ((value))
#define CIFISP_CPROC_TONE_Y_READ(value) ((value)>>16)
#define CIFISP_CPROC_TONE_C_READ(value) ((value)&0xFFFF)
#define CIFISP_CPROC_EN 1
#define CIFISP_CPROC_MACC_EN (1<<4)
#define CIFISP_CPROC_TMAP_EN (1<<5)

#define CIFISP_YCFLT_CTRL_RESERVED 0xFFFFFF8E
#define CIFISP_YCFLT_CH_SS_CTRL_RESERVED 0xFFFFFCCC
#define CIFISP_YCFLT_CH_SS_FAC_RESERVED 0xFF00FF00
#define CIFISP_YCFLT_CH_SS_OFFS_RESERVED 0xFF00FF00
#define CIFISP_YCFLT_NR_CTRL_RESERVED 0x80800000
#define CIFISP_YCFLT_EDGE_GAIN_RESERVED 0xFC00FC00
#define CIFISP_YCFLT_CORNER_GAIN_RESERVED 0xFC00FC00
#define CIFISP_YCFLT_FC_CROP_POS_RESERVED 0xFF00FF00
#define CIFISP_YCFLT_FC_CROP_NEG_RESERVED 0xFF00FF00
#define CIFISP_YCFLT_FC_GAIN_POS_RESERVED 0xC000C000
#define CIFISP_YCFLT_FC_GAIN_NEG_RESERVED 0xC000C000

#define CIFISP_LSC_SECT_SIZE_RESERVED 0xFC00FC00
#define CIFISP_LSC_GRAD_RESERVED 0xF000F000
#define CIFISP_LSC_SAMPLE_RESERVED 0xC000C000

#define CIFISP_AFC_THRES_RESERVED 0xFFFF0000
#define CIFISP_AFC_VAR_SHIFT_RESERVED 0xFFF8FFF8
#define CIFISP_AFC_WINDOW_X_RESERVED 0xE000
#define CIFISP_AFC_WINDOW_Y_RESERVED 0xF000
#define CIFISP_AFC_WINDOW_X_MIN 0x5
#define CIFISP_AFC_WINDOW_Y_MIN 0x2
#define CIFISP_AFC_WINDOW_X(value) ((value)<<16)
#define CIFISP_AFC_WINDOW_Y(value) (value)

#define CIFISP_DEBUG (1<<0)
#define CIFISP_ERROR (1<<1)

/* Empirical rough (relative) times it takes to perform
    given function. */
#define CIFISP_MODULE_BPC_PROC_TIME 3
#define CIFISP_MODULE_BLS_PROC_TIME 10
#define CIFISP_MODULE_LSC_PROC_TIME	1747
#define CIFISP_MODULE_FLT_PROC_TIME 15
#define CIFISP_MODULE_BDM_PROC_TIME 1
#define CIFISP_MODULE_SDG_PROC_TIME 53
#define CIFISP_MODULE_GOC_PROC_TIME 1000
#define CIFISP_MODULE_CTK_PROC_TIME 772
#define CIFISP_MODULE_AWB_PROC_TIME 8
#define CIFISP_MODULE_HST_PROC_TIME 5
#define CIFISP_MODULE_AEC_PROC_TIME 5
#define CIFISP_MODULE_AWB_GAIN_PROC_TIME 2
#define CIFISP_MODULE_CPROC_PROC_TIME 5
#define CIFISP_MODULE_MACC_PROC_TIME 32
#define CIFISP_MODULE_TMAP_PROC_TIME 257
/* Leave time for scaler reprogramming */
#define CIFISP_MODULE_YCFLT_PROC_TIME (10 + 500)
#define CIFISP_MODULE_AFC_PROC_TIME 8
#define CIFISP_MODULE_IE_PROC_TIME 5

/* For Debugging only!!! */
#define CIFISP_MODULE_BPC     1
#define CIFISP_MODULE_BLS     3
#define CIFISP_MODULE_LSC     5
#define CIFISP_MODULE_FLT     7
#define CIFISP_MODULE_BDM     9
#define CIFISP_MODULE_SDG     11
#define CIFISP_MODULE_GOC     13
#define CIFISP_MODULE_CTK     15
#define CIFISP_MODULE_AWB     17
#define CIFISP_MODULE_HST     19
#define CIFISP_MODULE_AEC     21
#define CIFISP_MODULE_AWB_GAIN 22
#define CIFISP_MODULE_CPROC     23
#define CIFISP_MODULE_MACC     24
#define CIFISP_MODULE_TMAP     25
#define CIFISP_MODULE_YCFLT    26
#define CIFISP_MODULE_AFC      27
#define CIFISP_MODULE_IE      28

#define CIFISP_MODULE_DEFAULT_VBLANKING_TIME 2000

#define V4L2_DEV_DEBUG_LEVEL 0

#define CIFISP_DPRINT(level, fmt, arg...) \
	do { \
		if (level == CIFISP_ERROR) \
			pr_err(fmt, ##arg); \
		else \
			pr_debug(fmt, ##arg); \
	} while (0)

#define cifisp_iowrite32(d, a) \
	cif_isp20_pltfrm_write_reg(NULL, (d), isp_dev->base_addr + (a))
#define cifisp_ioread32(a) \
	cif_isp20_pltfrm_read_reg(NULL, isp_dev->base_addr + (a))
#define cifisp_iowrite32OR(d, a) \
	cif_isp20_pltfrm_write_reg_OR(NULL, (d), isp_dev->base_addr + (a))
#define cifisp_iowrite32AND(d, a) \
	cif_isp20_pltfrm_write_reg_AND(NULL, (d), isp_dev->base_addr + (a))


/* Set this flag to enable CIF ISP Register debug
#define CIFISP_DEBUG_REG*/
/* Set this flag to trace the isr execution time
#define LOG_ISR_EXE_TIME*/
/* Set this flag to exclude everything except
measurements
#define CIFISP_DEBUG_DISABLE_BLOCKS*/
/* Set this flag to do stast read in a worker-thread. If not set, the read-out
happens in an interrupt context. */
#define STATS_WITH_WQ

#ifdef LOG_CAPTURE_PARAMS
static struct cifisp_last_capture_config g_last_capture_config;
#endif

#ifdef LOG_ISR_EXE_TIME
static unsigned int g_longest_isr_time;
#endif

struct meas_readout_work {
	struct work_struct work;
	struct xgold_isp_dev *isp_dev;
	unsigned int frame_id;
};

#ifdef STATS_WITH_WQ
static struct workqueue_struct *measurement_wq;
#endif

/* Functions for Debugging */
static void cifisp_param_dump(const void *config, unsigned int module);
#ifdef CIFISP_DEBUG_REG
static void cifisp_reg_dump(const struct xgold_isp_dev *isp_dev,
			    unsigned int module, int level);
#endif
static int cifisp_bpc_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->bpc_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->bpc_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->bpc_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_bpc_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_bls_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->bls_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->bls_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->bls_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_bls_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_lsc_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->lsc_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->lsc_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->lsc_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_lsc_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_flt_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->flt_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->flt_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->flt_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_flt_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_bdm_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->bdm_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->bdm_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->bdm_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_flt_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_sdg_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->sdg_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->sdg_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->sdg_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_sdg_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_goc_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->goc_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->goc_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->goc_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_goc_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_ctk_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->ctk_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->ctk_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->ctk_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_ctk_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_awb_meas_enable(struct xgold_isp_dev *isp_dev,
				  bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->awb_meas_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->awb_meas_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->awb_meas_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_awb_meas_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_awb_gain_enable(struct xgold_isp_dev *isp_dev,
				  bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->awb_gain_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->awb_gain_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->awb_gain_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_awb_gain_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_cproc_enable(struct xgold_isp_dev *isp_dev,
			       bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->cproc_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->cproc_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->cproc_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_cproc_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_macc_enable(struct xgold_isp_dev *isp_dev,
			      bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->macc_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->macc_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->macc_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_macc_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_tmap_enable(struct xgold_isp_dev *isp_dev,
			      bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->tmap_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->tmap_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->tmap_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_tmap_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_hst_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->hst_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->tmap_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->hst_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_hst_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_aec_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->aec_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->aec_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->aec_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_aec_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

int cifisp_ycflt_enable(struct xgold_isp_dev *isp_dev,
			       bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->ycflt_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->ycflt_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->ycflt_en = *value;
		if (!in_interrupt())
			spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->ycflt_update = true;
		if (!in_interrupt())
			spin_unlock_irqrestore(&isp_dev->config_lock,
				lock_flags);
	}

	return 0;
}

static int cifisp_afc_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->afc_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->afc_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->afc_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_afc_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}

static int cifisp_ie_enable(struct xgold_isp_dev *isp_dev,
			     bool flag, __s32 *value)
{
	if (flag == _GET_) {
		*value = isp_dev->ie_en;
		return 0;
	}

	CIFISP_DPRINT(CIFISP_DEBUG,
			  "%s %d\n", __func__, *value);

	if (isp_dev->ie_en != *value) {
		unsigned long lock_flags = 0;

		isp_dev->ie_en = *value;
		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
		isp_dev->isp_param_ie_update_needed = true;
		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);
	}

	return 0;
}


/* ISP BP interface function */
static int cifisp_bpc_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_bpc_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "arg is NULL: %s\n", __func__);

		return -EINVAL;
	}

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->bpc_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_BPC);

	if (memcmp(arg, &(isp_dev->bpc_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->corr_config.corr_type != CIFISP_BP_CORR_TYPE_DIRECT ||
	    arg->corr_config.abs_hot_thres > CIFISP_BP_HOT_THRESH_MAX ||
	    arg->corr_config.abs_dead_thres > CIFISP_BP_DEAD_THRESH_MAX ||
	    arg->corr_config.dev_hot_thres > CIFISP_BP_HOT_THRESH_MAX ||
	    arg->corr_config.dev_dead_thres > CIFISP_BP_DEAD_THRESH_MAX ||
	    arg->det_config.bp_hot_turbulence_shift >
	    CIFISP_BPC_MAX_HOT_TB_SHIFT
	    || arg->det_config.bp_dead_turbulence_shift >
	    CIFISP_BPC_MAX_DEAD_TB_SHIFT
	    || arg->det_config.bp_dev_hot_grad_trig_lvl >
	    CIFISP_BPC_MAX_HOT_GRAD_TRIG
	    || arg->det_config.bp_dev_dead_grad_trig_lvl >
	    CIFISP_BPC_MAX_DEAD_GRAD_TRIG) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->bpc_config, arg, sizeof(struct cifisp_bpc_config));
	isp_dev->isp_param_bpc_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP black level substraction interface function */
static int cifisp_bls_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_bls_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->bls_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_BLS);

	if (memcmp(arg, &(isp_dev->bls_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->bls_window1.h_offs > CIFISP_BLS_START_H_MAX ||
	    arg->bls_window1.h_size > CIFISP_BLS_STOP_H_MAX ||
	    arg->bls_window1.v_offs > CIFISP_BLS_START_V_MAX ||
	    arg->bls_window1.v_size > CIFISP_BLS_STOP_V_MAX ||
	    arg->bls_window2.h_offs > CIFISP_BLS_START_H_MAX ||
	    arg->bls_window2.h_size > CIFISP_BLS_STOP_H_MAX ||
	    arg->bls_window2.v_offs > CIFISP_BLS_START_V_MAX ||
	    arg->bls_window2.v_size > CIFISP_BLS_STOP_V_MAX ||
	    arg->bls_samples > CIFISP_BLS_SAMPLES_MAX ||
	    arg->fixed_val.fixed_a > CIFISP_BLS_FIX_SUB_MAX ||
	    arg->fixed_val.fixed_b > CIFISP_BLS_FIX_SUB_MAX ||
	    arg->fixed_val.fixed_c > CIFISP_BLS_FIX_SUB_MAX ||
	    arg->fixed_val.fixed_d > CIFISP_BLS_FIX_SUB_MAX ||
	    arg->fixed_val.fixed_a < (s16) CIFISP_BLS_FIX_SUB_MIN ||
	    arg->fixed_val.fixed_b < (s16) CIFISP_BLS_FIX_SUB_MIN ||
	    arg->fixed_val.fixed_c < (s16) CIFISP_BLS_FIX_SUB_MIN ||
	    arg->fixed_val.fixed_d < (s16) CIFISP_BLS_FIX_SUB_MIN) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->bls_config, arg, sizeof(struct cifisp_bls_config));
	isp_dev->isp_param_bls_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP LS correction interface function */
static int cifisp_lsc_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_lsc_config *arg)
{
	int i;
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->lsc_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_LSC);

	if (memcmp(arg, &(isp_dev->lsc_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	for (i = 0; i < CIFISP_LSC_SIZE_TBL_SIZE; i++) {
		if ((*(arg->x_size_tbl + i) & CIFISP_LSC_SECT_SIZE_RESERVED) ||
		    (*(arg->y_size_tbl + i) & CIFISP_LSC_SECT_SIZE_RESERVED)) {
			CIFISP_DPRINT(CIFISP_ERROR,
				      "incompatible sect size x 0x%x y 0x%x in function: %s\n",
				      *(arg->x_size_tbl + i),
				      *(arg->y_size_tbl + i), __func__);
			return -EINVAL;
		}
	}

	for (i = 0; i < CIFISP_LSC_GRAD_TBL_SIZE; i++) {
		if ((*(arg->x_grad_tbl + i) & CIFISP_LSC_GRAD_RESERVED) ||
		    (*(arg->y_grad_tbl + i) & CIFISP_LSC_GRAD_RESERVED)) {
			CIFISP_DPRINT(CIFISP_ERROR,
				      "incompatible grad x 0x%x y 0x%xin function: %s\n",
				      *(arg->x_grad_tbl + i),
				      *(arg->y_grad_tbl + i), __func__);
			return -EINVAL;
		}
	}

	for (i = 0; i < CIFISP_LSC_DATA_TBL_SIZE; i++) {
		if ((*(arg->r_data_tbl + i) & CIFISP_LSC_SAMPLE_RESERVED) ||
		    (*(arg->g_data_tbl + i) & CIFISP_LSC_SAMPLE_RESERVED) ||
		    (*(arg->b_data_tbl + i) & CIFISP_LSC_SAMPLE_RESERVED)) {
			CIFISP_DPRINT(CIFISP_ERROR,
				      "incompatible sample r 0x%x g 0x%x b 0x%x in function: %s\n",
				      *(arg->r_data_tbl + i),
				      *(arg->g_data_tbl + i),
				      *(arg->b_data_tbl + i), __func__);
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->lsc_config, arg, sizeof(struct cifisp_lsc_config));
	isp_dev->isp_param_lsc_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/*ISP Filtering function*/
static int cifisp_flt_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_flt_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->flt_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_FLT);

	if (memcmp(arg, &(isp_dev->flt_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	/* Parameter check */
	if (arg->flt_mode > CIFISP_FILT_MODE_MAX ||
	    arg->flt_diag_sharp_mode > CIFISP_FILT_DIAG_SHARP_MAX ||
	    arg->flt_chrom_v_mode > CIFISP_FILT_CH_MODE_MAX ||
	    arg->flt_chrom_h_mode > CIFISP_FILT_CH_MODE_MAX ||
	    arg->flt_mask_sharp0 & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_sharp1 & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_blur_max & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_blur & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_lin & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_orth & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_diag & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_v_diag & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_mask_h_diag & CIFISP_FILTER_COEFF_RESERVED ||
	    arg->flt_blur_th0 > CIFISP_FILT_DIAG_TH_MAX ||
	    arg->flt_blur_th1 > CIFISP_FILT_DIAG_TH_MAX ||
	    arg->flt_sharp0_th > CIFISP_FILT_DIAG_TH_MAX ||
	    arg->flt_sharp1_th > CIFISP_FILT_DIAG_TH_MAX ||
	    arg->flt_lum_weight & CIFISP_FILTER_LUM_WEIGHT_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->flt_config, arg, sizeof(struct cifisp_flt_config));
	isp_dev->isp_param_flt_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP demosaic interface function */
static int cifisp_bdm_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_bdm_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->bdm_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_BDM);

	if (memcmp(arg, &(isp_dev->bdm_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	/* Parameter Check */
	if (arg->demosaic_th > CIFISP_BDM_MAX_TH)
		return -EINVAL;

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->bdm_config, arg, sizeof(struct cifisp_bdm_config));
	isp_dev->isp_param_bdm_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP GAMMA correction interface function */
static int cifisp_sdg_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_sdg_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->sdg_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_SDG);

	if (memcmp(arg, &(isp_dev->sdg_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->xa_pnts.gamma_dx0 & CIFISP_DEGAMMA_X_RESERVED ||
	    arg->xa_pnts.gamma_dx1 & CIFISP_DEGAMMA_X_RESERVED ||
	    arg->curve_r.gamma_y0 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y1 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y2 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y3 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y4 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y5 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y6 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y7 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y8 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y9 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y10 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y11 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y12 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y13 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y14 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y15 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_r.gamma_y16 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y0 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y1 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y2 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y3 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y4 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y5 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y6 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y7 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y8 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y9 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y10 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y11 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y12 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y13 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y14 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y15 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_g.gamma_y16 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y0 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y1 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y2 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y3 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y4 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y5 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y6 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y7 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y8 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y9 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y10 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y11 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y12 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y13 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y14 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y15 & CIFISP_DEGAMMA_Y_RESERVED ||
	    arg->curve_b.gamma_y16 & CIFISP_DEGAMMA_Y_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->sdg_config, arg, sizeof(struct cifisp_sdg_config));
	isp_dev->isp_param_sdg_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP GAMMA correction interface function */
static int cifisp_goc_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_goc_config *arg)
{
	unsigned long lock_flags = 0;

	int i;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->goc_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_GOC);

	if (memcmp(arg, &(isp_dev->goc_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++) {
		if (arg->gamma_r[i] & CIFISP_GOC_RESERVED ||
		    arg->gamma_g[i] & CIFISP_GOC_RESERVED ||
		    arg->gamma_b[i] & CIFISP_GOC_RESERVED) {
			CIFISP_DPRINT(CIFISP_ERROR,
				      "incompatible param 0x%x 0x%x 0x%x in  function: %s\n",
				      arg->gamma_r[i],
				      arg->gamma_g[i],
				      arg->gamma_b[i], __func__);
			return -EINVAL;
		}
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->goc_config, arg, sizeof(struct cifisp_goc_config));
	isp_dev->isp_param_goc_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP Cross Talk */
static int cifisp_ctk_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_ctk_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->ctk_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_CTK);

	if (memcmp(arg, &(isp_dev->ctk_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	/* Perform parameter check */
	if (arg->coeff0 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff1 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff2 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff3 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff4 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff5 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff6 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff7 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->coeff8 & CIFISP_CTK_COEFF_RESERVED ||
	    arg->ct_offset_r & CIFISP_XTALK_OFFSET_RESERVED ||
	    arg->ct_offset_g & CIFISP_XTALK_OFFSET_RESERVED ||
	    arg->ct_offset_b & CIFISP_XTALK_OFFSET_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->ctk_config, arg, sizeof(struct cifisp_ctk_config));
	isp_dev->isp_param_ctk_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

/* ISP White Balance Mode */
static int cifisp_awb_meas_param(struct xgold_isp_dev *isp_dev,
				 bool flag, struct cifisp_awb_meas_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->awb_meas_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_AWB);

	if (memcmp(arg, &(isp_dev->awb_meas_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->awb_mode > CIFISP_AWB_MODE_YCBCR ||
	    arg->awb_wnd.h_offs > CIFISP_AWB_GRID_MAX_OFFSET ||
	    arg->awb_wnd.v_offs > CIFISP_AWB_GRID_MAX_OFFSET ||
	    arg->awb_wnd.h_size > CIFISP_AWB_WINDOW_MAX_SIZE ||
	    arg->awb_wnd.v_size > CIFISP_AWB_WINDOW_MAX_SIZE ||
	    arg->max_y > CIFISP_AWB_THRES_MAX_YC ||
	    arg->min_y > CIFISP_AWB_THRES_MAX_YC ||
	    arg->max_csum > CIFISP_AWB_THRES_MAX_YC ||
	    arg->min_c > CIFISP_AWB_THRES_MAX_YC ||
	    arg->frames > CIFISP_AWB_MAX_FRAMES ||
	    arg->awb_ref_cr > CIFISP_AWB_CBCR_MAX_REF ||
	    arg->awb_ref_cb > CIFISP_AWB_CBCR_MAX_REF ||
	    arg->gb_sat > CIFISP_AWB_MAX_SAT ||
	    arg->gr_sat > CIFISP_AWB_MAX_SAT ||
	    arg->b_sat > CIFISP_AWB_MAX_SAT ||
	    arg->r_sat > CIFISP_AWB_MAX_SAT ||
	    arg->grid_v_dim * arg->grid_h_dim > CIFISP_AWB_MAX_GRID ||
	    arg->grid_h_dist > CIFISP_AWB_GRID_MAX_DIST ||
	    arg->grid_v_dist > CIFISP_AWB_GRID_MAX_DIST ||
	    arg->grid_v_dim == 0 ||
	    arg->grid_h_dim == 0) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->awb_meas_config, arg,
	       sizeof(struct cifisp_awb_meas_config));
	isp_dev->isp_param_awb_meas_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_awb_gain_param(struct xgold_isp_dev *isp_dev,
				 bool flag, struct cifisp_awb_gain_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->awb_gain_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_AWB_GAIN);

	if (memcmp(arg, &(isp_dev->awb_gain_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->gain_red > CIFISP_AWB_GAINS_MAX_VAL ||
	    arg->gain_green_r > CIFISP_AWB_GAINS_MAX_VAL ||
	    arg->gain_green_b > CIFISP_AWB_GAINS_MAX_VAL ||
	    arg->gain_blue > CIFISP_AWB_GAINS_MAX_VAL) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->awb_gain_config, arg,
	       sizeof(struct cifisp_awb_gain_config));
	isp_dev->isp_param_awb_gain_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_aec_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_aec_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->aec_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_AEC);

	if (memcmp(arg, &(isp_dev->aec_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->meas_window.h_offs > CIFISP_EXP_MAX_HOFFS ||
	    arg->meas_window.h_size > CIFISP_EXP_MAX_HSIZE ||
	    arg->meas_window.h_size < CIFISP_EXP_MIN_HSIZE ||
	    arg->meas_window.v_offs > CIFISP_EXP_MAX_VOFFS ||
	    arg->meas_window.v_size > CIFISP_EXP_MAX_VSIZE ||
	    arg->meas_window.v_size < CIFISP_EXP_MIN_VSIZE) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->aec_config, arg, sizeof(struct cifisp_aec_config));
	isp_dev->isp_param_aec_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_cproc_param(struct xgold_isp_dev *isp_dev,
			      bool flag, struct cifisp_cproc_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->cproc_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_CPROC);

	if (memcmp(arg, &(isp_dev->cproc_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->c_out_range & CIFISP_CPROC_CTRL_RESERVED ||
	    arg->y_out_range & CIFISP_CPROC_CTRL_RESERVED ||
	    arg->y_in_range & CIFISP_CPROC_CTRL_RESERVED ||
	    arg->contrast & CIFISP_CPROC_CONTRAST_RESERVED ||
	    arg->brightness & CIFISP_CPROC_BRIGHTNESS_RESERVED ||
	    arg->sat & CIFISP_CPROC_SATURATION_RESERVED ||
	    arg->hue & CIFISP_CPROC_HUE_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->cproc_config, arg, sizeof(struct cifisp_cproc_config));
	isp_dev->isp_param_cproc_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_macc_param(struct xgold_isp_dev *isp_dev,
			     bool flag, struct cifisp_macc_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->macc_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_MACC);

	if (memcmp(arg, &(isp_dev->macc_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->seg0.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg0.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg1.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg1.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg2.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg2.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg3.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg3.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg4.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg4.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg5.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg5.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg6.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg6.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg7.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg7.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg8.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg8.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg9.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg9.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg10.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg10.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg11.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg11.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg12.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg12.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg13.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg14.coeff1 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg15.coeff0 & CIFISP_CPROC_MACC_RESERVED ||
	    arg->seg15.coeff1 & CIFISP_CPROC_MACC_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->macc_config, arg, sizeof(struct cifisp_macc_config));
	isp_dev->isp_param_macc_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_tmap_param(struct xgold_isp_dev *isp_dev,
			     bool flag, struct cifisp_tmap_config *arg)
{
	unsigned long lock_flags = 0;
	int i;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->tmap_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_TMAP);

	if (memcmp(arg, &(isp_dev->tmap_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	for (i = 0; i < CIFISP_TONE_MAP_TABLE_SIZE; i++)
		if (arg->tmap_y[i] & CIFISP_CPROC_TONE_RESERVED ||
		    arg->tmap_c[i] & CIFISP_CPROC_TONE_RESERVED) {
			CIFISP_DPRINT(CIFISP_ERROR,
				      "incompatible param in function: %s\n",
				      __func__);
			return -EINVAL;
		}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->tmap_config, arg, sizeof(struct cifisp_tmap_config));
	isp_dev->isp_param_tmap_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_hst_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_hst_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->hst_config), sizeof(*arg));
		return 0;
	}

	if (memcmp(arg, &(isp_dev->hst_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->histogram_predivider > CIFISP_MAX_HIST_PREDIVIDER ||
	    arg->meas_window.v_offs & CIFISP_HIST_WINDOW_RESERVED ||
	    arg->meas_window.h_offs & CIFISP_HIST_WINDOW_RESERVED ||
	    arg->meas_window.v_size & CIFISP_HIST_WINDOW_RESERVED ||
	    arg->meas_window.h_size & CIFISP_HIST_WINDOW_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->hst_config, arg, sizeof(struct cifisp_hst_config));
	isp_dev->isp_param_hst_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_ycflt_param(struct xgold_isp_dev *isp_dev,
			      bool flag, struct cifisp_ycflt_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->ycflt_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_YCFLT);

	if (memcmp(arg, &(isp_dev->ycflt_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->ctrl & CIFISP_YCFLT_CTRL_RESERVED ||
	    arg->chr_ss_ctrl & CIFISP_YCFLT_CH_SS_CTRL_RESERVED ||
	    arg->chr_ss_fac & CIFISP_YCFLT_CH_SS_FAC_RESERVED ||
	    arg->chr_ss_offs & CIFISP_YCFLT_CH_SS_OFFS_RESERVED ||
	    arg->chr_nr_ctrl & CIFISP_YCFLT_NR_CTRL_RESERVED ||
	    arg->lum_eenr_edge_gain & CIFISP_YCFLT_EDGE_GAIN_RESERVED ||
	    arg->lum_eenr_corner_gain & CIFISP_YCFLT_CORNER_GAIN_RESERVED ||
	    arg->lum_eenr_fc_crop_pos & CIFISP_YCFLT_FC_CROP_POS_RESERVED ||
	    arg->lum_eenr_fc_crop_neg & CIFISP_YCFLT_FC_CROP_NEG_RESERVED ||
	    arg->lum_eenr_fc_gain_pos & CIFISP_YCFLT_FC_GAIN_POS_RESERVED ||
	    arg->lum_eenr_fc_gain_neg & CIFISP_YCFLT_FC_GAIN_NEG_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);

	/* This copy will not be changed untill next
	parameter update from HAL */
	memcpy(&isp_dev->ycflt_config, arg, sizeof(struct cifisp_ycflt_config));

	/* This copy might be changed internally according to ISM settings */
	memcpy(&isp_dev->ycflt_config_ism_on, arg,
		sizeof(struct cifisp_ycflt_config));
	/* Need to turn off XNR subsampling if ISM cropping is on */
	isp_dev->ycflt_config_ism_on.chr_ss_ctrl &= ~0x3;

	isp_dev->ycflt_update = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_afc_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_afc_config *arg)
{
	unsigned long lock_flags = 0;
	int i;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->afc_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_AFC);

	if (memcmp(arg, &(isp_dev->afc_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->num_afm_win > CIFISP_AFM_MAX_WINDOWS ||
	    arg->thres & CIFISP_AFC_THRES_RESERVED ||
	    arg->var_shift & CIFISP_AFC_VAR_SHIFT_RESERVED) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < arg->num_afm_win; i++) {
		if (arg->afm_win[i].h_offs & CIFISP_AFC_WINDOW_X_RESERVED ||
		    arg->afm_win[i].h_offs < CIFISP_AFC_WINDOW_X_MIN ||
		    arg->afm_win[i].v_offs & CIFISP_AFC_WINDOW_Y_RESERVED ||
		    arg->afm_win[i].v_offs < CIFISP_AFC_WINDOW_Y_MIN ||
		    arg->afm_win[i].h_size & CIFISP_AFC_WINDOW_X_RESERVED ||
		    arg->afm_win[i].v_size & CIFISP_AFC_WINDOW_Y_RESERVED) {
			CIFISP_DPRINT(CIFISP_ERROR,
				      "incompatible param in function: %s\n",
				      __func__);
			return -EINVAL;

		}

	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->afc_config, arg, sizeof(struct cifisp_afc_config));
	isp_dev->isp_param_afc_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_ie_param(struct xgold_isp_dev *isp_dev,
			    bool flag, struct cifisp_ie_config *arg)
{
	unsigned long lock_flags = 0;

	if (arg == NULL)
		return -EINVAL;

	if (flag == _GET_) {
		memcpy(arg, &(isp_dev->ie_config), sizeof(*arg));
		return 0;
	}

	cifisp_param_dump(arg, CIFISP_MODULE_IE);

	if (memcmp(arg, &(isp_dev->ie_config), sizeof(*arg)) == 0) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "same param in function: %s\n", __func__);
		return 0;
	}

	if (arg->effect != V4L2_COLORFX_NONE &&
			arg->effect != V4L2_COLORFX_BW &&
			arg->effect != V4L2_COLORFX_SEPIA &&
			arg->effect != V4L2_COLORFX_NEGATIVE &&
			arg->effect != V4L2_COLORFX_EMBOSS &&
			arg->effect != V4L2_COLORFX_SKETCH &&
			arg->effect != V4L2_COLORFX_AQUA &&
			arg->effect != V4L2_COLORFX_SET_CBCR) {
		CIFISP_DPRINT(CIFISP_ERROR,
			      "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_dev->config_lock, lock_flags);
	memcpy(&isp_dev->ie_config, arg, sizeof(struct cifisp_ie_config));
	isp_dev->isp_param_ie_update_needed = true;
	spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

	return 0;
}

static int cifisp_last_capture_config(struct cifisp_last_capture_config *arg)
{
#ifdef LOG_CAPTURE_PARAMS
	if (arg == NULL)
		return -EINVAL;

	memcpy(arg, &g_last_capture_config, sizeof(*arg));

	return 0;
#else
	return -EPERM;
#endif
}

/*---------------------------------------------*/
/*                           Helper Functions                               */
/*---------------------------------------------*/
/*!********************************************************************
 *  \FUNCTION    cif_isp_bp_config\n
 *  \PARAMETERS  p_bp_corr_config:   Configuration structure of correction.\n
 *               p_bp_det_config:    Configuration structure of dectection.\n
 *  \DESCRIPTION Initialization of the Bad Pixel Detection and Correction.\n
 *********************************************************************/
static void cifisp_bp_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_bpc_config *pconfig = &(isp_dev->bpc_config);
	const struct cifisp_bp_correction_config *pcor_config =
	    &pconfig->corr_config;
	const struct cifisp_bp_detection_config *pdet_config =
	    &pconfig->det_config;
	u32 new_control = 0;

	if (pcor_config->corr_rep == CIFISP_BP_CORR_REP_LIN)
		new_control |= CIFISP_BPC_REP_APR_BILIN;

	if (pcor_config->corr_type == CIFISP_BP_CORR_TYPE_DIRECT)
		new_control |= CIF_BP_CTRL_COR_TYPE_DIRECT;

	if (pdet_config->bp_hot_turbulence_adj_en)
		new_control |= CIFISP_BPC_HOT_TURB_ADJ_EN;

	new_control |=
	    CIFISP_BPC_HOT_TURB_SHIFT(pdet_config->bp_hot_turbulence_shift);

	if (pdet_config->bp_dead_turbulence_adj_en)
		new_control |= CIFISP_BPC_DEAD_TURB_ADJ_EN;

	new_control |=
	    CIFISP_BPC_DEAD_TURB_SHIFT(pdet_config->bp_dead_turbulence_shift);

	if (pdet_config->bp_dev_hot_sign_sens)
		new_control |= CIFISP_BPC_HOT_SIGN_SENS_EN;

	new_control |=
	    CIFISP_BPC_HOT_GRAD_TRIG(pdet_config->bp_dev_hot_grad_trig_lvl);

	if (pdet_config->bp_dev_dead_sign_sens)
		new_control |= CIFISP_BPC_DEAD_SIGN_SENS_EN;

	new_control |=
	    CIFISP_BPC_DEAD_GRAD_TRIG(pdet_config->bp_dev_dead_grad_trig_lvl);

	cifisp_iowrite32(new_control, CIF_ISP_BP_CTRL);

	cifisp_iowrite32((pcor_config->abs_hot_thres << 16) |
		      (pcor_config->abs_dead_thres), CIF_ISP_BP_CFG1);

	cifisp_iowrite32((pcor_config->dev_hot_thres << 16) |
		      (pcor_config->dev_dead_thres), CIF_ISP_BP_CFG2);
}

/*!********************************************************************
 *  \FUNCTION    cif_isp_bp_en\n
 *  \RETURNVALUE None.\n
 *  \PARAMETERS  p_bp_corr_config:   Configuration structure of correction.\n
 *  \DESCRIPTION Initialization of the Bad Pixel Detection and Correction.\n
 *********************************************************************/
static void cifisp_bp_en(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_bpc_config *pconfig = &(isp_dev->bpc_config);
	u32 new_control = 0;

	switch (pconfig->corr_config.corr_mode) {
	case CIF_ISP_BP_CORR_HOT_EN:
		new_control |= CIFISP_BPC_HOT_CORR_EN;
		break;
	case CIF_ISP_BP_CORR_DEAD_EN:
		new_control |= CIFISP_BPC_DEAD_CORR_EN;
		break;
	case CIF_ISP_BP_CORR_HOT_DEAD_EN:
		new_control |= CIFISP_BPC_HOT_CORR_EN;
		new_control |= CIFISP_BPC_DEAD_CORR_EN;
		break;
	}

	cifisp_iowrite32OR(new_control, CIF_ISP_BP_CTRL);
}

 /*!********************************************************************
  *  \FUNCTION     cif_isp_bp_end\n
  *  \RETURNVALUE  CIF_RESULT_OK:The deinit is finished successfully.\n
  *                CIF_RESULT_ERROR: The parameter is a NULL pointer.\n
  *                CIF_RESULT_ERROR: The feature is not supported.\n
  *  \PARAMETERS   None.\n
  *  \DESCRIPTION  Disable the Bad Pixel Detection and Correction.\n
  *********************************************************************/
static void cifisp_bp_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_BP_CTRL);
}

/* Lens Shade Correction */

/*****************************************************************************/
static void cifisp_lsc_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_LSC_CTRL);
}

/*****************************************************************************/
static bool cifisp_lsc_config(struct xgold_isp_dev *isp_dev)
{
	int i;

	/*To config must be off */
	cifisp_iowrite32(0, CIF_ISP_LSC_CTRL);

	if (isp_dev->lsc_config.config_width != isp_dev->input_width ||
		isp_dev->lsc_config.config_height != isp_dev->input_height) {
		CIFISP_DPRINT(CIFISP_DEBUG,
			"LSC config: lsc_w %d lsc_h %d act_w %d act_h %d\n",
			isp_dev->lsc_config.config_width,
			isp_dev->lsc_config.config_height,
			isp_dev->input_width,
			isp_dev->input_height);
		return false;
	} else
		CIFISP_DPRINT(CIFISP_DEBUG,
			"LSC config: lsc_w %d lsc_h %d\n",
			isp_dev->lsc_config.config_width,
			isp_dev->lsc_config.config_height);

	/* As defined in the header, the order and format of the
	   correction values must match the Table 11, cif 4.2.0.

	   That is the case for the output of the tuning tool. */
	cifisp_iowrite32(0, CIF_ISP_LSC_R_TABLE_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_G_TABLE_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_B_TABLE_ADDR);

	for (i = 0; i < CIFISP_LSC_DATA_TBL_SIZE; i++) {
		cifisp_iowrite32(isp_dev->lsc_config.r_data_tbl[i],
			CIF_ISP_LSC_R_TABLE_DATA);
		cifisp_iowrite32(isp_dev->lsc_config.g_data_tbl[i],
			CIF_ISP_LSC_G_TABLE_DATA);
		cifisp_iowrite32(isp_dev->lsc_config.b_data_tbl[i],
			CIF_ISP_LSC_B_TABLE_DATA);
	}

	if (isp_dev->active_lsc_width !=
		isp_dev->lsc_config.config_width ||
		isp_dev->active_lsc_height !=
		isp_dev->lsc_config.config_height) {
		/* Reset the address counters */
		cifisp_iowrite32(0, CIF_ISP_LSC_XGRAD_RAM_ADDR);
		cifisp_iowrite32(0, CIF_ISP_LSC_YGRAD_RAM_ADDR);
		cifisp_iowrite32(0, CIF_ISP_LSC_XSIZE_RAM_ADDR);
		cifisp_iowrite32(0, CIF_ISP_LSC_YSIZE_RAM_ADDR);

		for (i = 0; i < CIFISP_LSC_SIZE_TBL_SIZE; i++) {
			cifisp_iowrite32(isp_dev->lsc_config.y_size_tbl[i],
			CIF_ISP_LSC_YSIZE_RAM_DATA);
			cifisp_iowrite32(isp_dev->lsc_config.x_size_tbl[i],
			CIF_ISP_LSC_XSIZE_RAM_DATA);
		}

		for (i = 0; i < CIFISP_LSC_GRAD_TBL_SIZE; i++) {
			cifisp_iowrite32(isp_dev->lsc_config.y_grad_tbl[i],
			CIF_ISP_LSC_YGRAD_RAM_DATA);
			cifisp_iowrite32(isp_dev->lsc_config.x_grad_tbl[i],
			CIF_ISP_LSC_XGRAD_RAM_DATA);
		}

		isp_dev->active_lsc_width = isp_dev->lsc_config.config_width;
		isp_dev->active_lsc_height = isp_dev->lsc_config.config_height;
	}

	cifisp_iowrite32(1, CIF_ISP_LSC_CTRL);

	return true;
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_lsc_config_read(const struct xgold_isp_dev *isp_dev,
				   struct cifisp_lsc_config *pconfig)
{
	int i;
	unsigned int on;

	/*To config must be off */
	on = cifisp_ioread32(CIF_ISP_LSC_CTRL);
	cifisp_iowrite32(0, CIF_ISP_LSC_CTRL);

	/* Reset the address counters */
	cifisp_iowrite32(0, CIF_ISP_LSC_XGRAD_RAM_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_YGRAD_RAM_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_XSIZE_RAM_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_YSIZE_RAM_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_R_TABLE_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_G_TABLE_ADDR);
	cifisp_iowrite32(0, CIF_ISP_LSC_B_TABLE_ADDR);

	for (i = 0; i < CIFISP_LSC_ACTIVE_SIZE_TBL_SIZE; i++) {
		*(pconfig->y_size_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_YSIZE_RAM_DATA);
		*(pconfig->x_size_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_XSIZE_RAM_DATA);
	}

	for (i = 0; i < CIFISP_LSC_ACTIVE_GRAD_TBL_SIZE; i++) {
		*(pconfig->y_grad_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_YGRAD_RAM_DATA);
		*(pconfig->x_grad_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_XGRAD_RAM_DATA);
	}

	/* As defined in the header, the order and format
	   of the correction values must
	   match the Table 11, cif 4.2.0.

	   That is the case for the output of the tuning tool. */
	for (i = 0; i < CIFISP_LSC_DATA_TBL_SIZE; i++) {
		*(pconfig->r_data_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_R_TABLE_DATA);
		*(pconfig->g_data_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_G_TABLE_DATA);
		*(pconfig->b_data_tbl + i) =
		    cifisp_ioread32(CIF_ISP_LSC_B_TABLE_DATA);
	}

	cifisp_iowrite32(on, CIF_ISP_LSC_CTRL);
}
#endif

/*****************************************************************************/
static void cifisp_bls_get_meas(const struct xgold_isp_dev *isp_dev,
	struct cifisp_stat_buffer *pbuf)
{
	pbuf->params.ae.bls_val.meas_a =
		cifisp_ioread32(CIF_ISP_BLS_A_MEASURED);
	pbuf->params.ae.bls_val.meas_b =
		cifisp_ioread32(CIF_ISP_BLS_B_MEASURED);
	pbuf->params.ae.bls_val.meas_c =
		cifisp_ioread32(CIF_ISP_BLS_C_MEASURED);
	pbuf->params.ae.bls_val.meas_d =
		cifisp_ioread32(CIF_ISP_BLS_D_MEASURED);
}
/*****************************************************************************/
static void cifisp_bls_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_bls_config *pconfig = &(isp_dev->bls_config);
	u32 new_control = 0;

	/* fixed subtraction values */
	if (pconfig->enable_auto == false) {
		const struct cifisp_bls_fixed_val *pval =
		    &isp_dev->bls_config.fixed_val;

		cifisp_iowrite32(pval->fixed_a, CIF_ISP_BLS_A_FIXED);
		cifisp_iowrite32(pval->fixed_b, CIF_ISP_BLS_B_FIXED);
		cifisp_iowrite32(pval->fixed_c, CIF_ISP_BLS_C_FIXED);
		cifisp_iowrite32(pval->fixed_d, CIF_ISP_BLS_D_FIXED);

		cifisp_iowrite32(new_control, CIF_ISP_BLS_CTRL);
	} else {
		if (pconfig->en_windows & 2) {
			cifisp_iowrite32(pconfig->bls_window2.h_offs,
				      CIF_ISP_BLS_H2_START);
			cifisp_iowrite32(pconfig->bls_window2.h_size,
				      CIF_ISP_BLS_H2_STOP);
			cifisp_iowrite32(pconfig->bls_window2.v_offs,
				      CIF_ISP_BLS_V2_START);
			cifisp_iowrite32(pconfig->bls_window2.v_size,
				      CIF_ISP_BLS_V2_STOP);
			new_control |= CIFISP_BLS_WINDOW_2;
		}

		if (pconfig->en_windows & 1) {
			cifisp_iowrite32(pconfig->bls_window1.h_offs,
				      CIF_ISP_BLS_H1_START);
			cifisp_iowrite32(pconfig->bls_window1.h_size,
				      CIF_ISP_BLS_H1_STOP);
			cifisp_iowrite32(pconfig->bls_window1.v_offs,
				      CIF_ISP_BLS_V1_START);
			cifisp_iowrite32(pconfig->bls_window1.v_size,
				      CIF_ISP_BLS_V1_STOP);
			new_control |= CIFISP_BLS_WINDOW_1;
		}

		cifisp_iowrite32(pconfig->bls_samples, CIF_ISP_BLS_SAMPLES);

		new_control |= CIFISP_BLS_MODE_MEASURED;

		cifisp_iowrite32(new_control, CIF_ISP_BLS_CTRL);
	}

}

/*****************************************************************************/
static void cifisp_bls_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(1, CIF_ISP_BLS_CTRL);
}

/*****************************************************************************/
static void cifisp_bls_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_BLS_CTRL);
}

/* Gamma correction */
/*****************************************************************************/
static void cifisp_sdg_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_sdg_config *pconfig = &(isp_dev->sdg_config);

	cifisp_iowrite32(pconfig->xa_pnts.gamma_dx0, CIF_ISP_GAMMA_DX_LO);
	cifisp_iowrite32(pconfig->xa_pnts.gamma_dx1, CIF_ISP_GAMMA_DX_HI);
	/* R - Curve */
	cifisp_iowrite32(pconfig->curve_r.gamma_y0, CIF_ISP_GAMMA_R_Y0);
	cifisp_iowrite32(pconfig->curve_r.gamma_y1, CIF_ISP_GAMMA_R_Y1);
	cifisp_iowrite32(pconfig->curve_r.gamma_y2, CIF_ISP_GAMMA_R_Y2);
	cifisp_iowrite32(pconfig->curve_r.gamma_y3, CIF_ISP_GAMMA_R_Y3);
	cifisp_iowrite32(pconfig->curve_r.gamma_y4, CIF_ISP_GAMMA_R_Y4);
	cifisp_iowrite32(pconfig->curve_r.gamma_y5, CIF_ISP_GAMMA_R_Y5);
	cifisp_iowrite32(pconfig->curve_r.gamma_y6, CIF_ISP_GAMMA_R_Y6);
	cifisp_iowrite32(pconfig->curve_r.gamma_y7, CIF_ISP_GAMMA_R_Y7);
	cifisp_iowrite32(pconfig->curve_r.gamma_y8, CIF_ISP_GAMMA_R_Y8);
	cifisp_iowrite32(pconfig->curve_r.gamma_y9, CIF_ISP_GAMMA_R_Y9);
	cifisp_iowrite32(pconfig->curve_r.gamma_y10, CIF_ISP_GAMMA_R_Y10);
	cifisp_iowrite32(pconfig->curve_r.gamma_y11, CIF_ISP_GAMMA_R_Y11);
	cifisp_iowrite32(pconfig->curve_r.gamma_y12, CIF_ISP_GAMMA_R_Y12);
	cifisp_iowrite32(pconfig->curve_r.gamma_y13, CIF_ISP_GAMMA_R_Y13);
	cifisp_iowrite32(pconfig->curve_r.gamma_y14, CIF_ISP_GAMMA_R_Y14);
	cifisp_iowrite32(pconfig->curve_r.gamma_y15, CIF_ISP_GAMMA_R_Y15);
	cifisp_iowrite32(pconfig->curve_r.gamma_y16, CIF_ISP_GAMMA_R_Y16);

	/* G - Curve */
	cifisp_iowrite32(pconfig->curve_g.gamma_y0, CIF_ISP_GAMMA_G_Y0);
	cifisp_iowrite32(pconfig->curve_g.gamma_y1, CIF_ISP_GAMMA_G_Y1);
	cifisp_iowrite32(pconfig->curve_g.gamma_y2, CIF_ISP_GAMMA_G_Y2);
	cifisp_iowrite32(pconfig->curve_g.gamma_y3, CIF_ISP_GAMMA_G_Y3);
	cifisp_iowrite32(pconfig->curve_g.gamma_y4, CIF_ISP_GAMMA_G_Y4);
	cifisp_iowrite32(pconfig->curve_g.gamma_y5, CIF_ISP_GAMMA_G_Y5);
	cifisp_iowrite32(pconfig->curve_g.gamma_y6, CIF_ISP_GAMMA_G_Y6);
	cifisp_iowrite32(pconfig->curve_g.gamma_y7, CIF_ISP_GAMMA_G_Y7);
	cifisp_iowrite32(pconfig->curve_g.gamma_y8, CIF_ISP_GAMMA_G_Y8);
	cifisp_iowrite32(pconfig->curve_g.gamma_y9, CIF_ISP_GAMMA_G_Y9);
	cifisp_iowrite32(pconfig->curve_g.gamma_y10, CIF_ISP_GAMMA_G_Y10);
	cifisp_iowrite32(pconfig->curve_g.gamma_y11, CIF_ISP_GAMMA_G_Y11);
	cifisp_iowrite32(pconfig->curve_g.gamma_y12, CIF_ISP_GAMMA_G_Y12);
	cifisp_iowrite32(pconfig->curve_g.gamma_y13, CIF_ISP_GAMMA_G_Y13);
	cifisp_iowrite32(pconfig->curve_g.gamma_y14, CIF_ISP_GAMMA_G_Y14);
	cifisp_iowrite32(pconfig->curve_g.gamma_y15, CIF_ISP_GAMMA_G_Y15);
	cifisp_iowrite32(pconfig->curve_g.gamma_y16, CIF_ISP_GAMMA_G_Y16);

	/* B - Curve */
	cifisp_iowrite32(pconfig->curve_b.gamma_y0, CIF_ISP_GAMMA_B_Y0);
	cifisp_iowrite32(pconfig->curve_b.gamma_y1, CIF_ISP_GAMMA_B_Y1);
	cifisp_iowrite32(pconfig->curve_b.gamma_y2, CIF_ISP_GAMMA_B_Y2);
	cifisp_iowrite32(pconfig->curve_b.gamma_y3, CIF_ISP_GAMMA_B_Y3);
	cifisp_iowrite32(pconfig->curve_b.gamma_y4, CIF_ISP_GAMMA_B_Y4);
	cifisp_iowrite32(pconfig->curve_b.gamma_y5, CIF_ISP_GAMMA_B_Y5);
	cifisp_iowrite32(pconfig->curve_b.gamma_y6, CIF_ISP_GAMMA_B_Y6);
	cifisp_iowrite32(pconfig->curve_b.gamma_y7, CIF_ISP_GAMMA_B_Y7);
	cifisp_iowrite32(pconfig->curve_b.gamma_y8, CIF_ISP_GAMMA_B_Y8);
	cifisp_iowrite32(pconfig->curve_b.gamma_y9, CIF_ISP_GAMMA_B_Y9);
	cifisp_iowrite32(pconfig->curve_b.gamma_y10, CIF_ISP_GAMMA_B_Y10);
	cifisp_iowrite32(pconfig->curve_b.gamma_y11, CIF_ISP_GAMMA_B_Y11);
	cifisp_iowrite32(pconfig->curve_b.gamma_y12, CIF_ISP_GAMMA_B_Y12);
	cifisp_iowrite32(pconfig->curve_b.gamma_y13, CIF_ISP_GAMMA_B_Y13);
	cifisp_iowrite32(pconfig->curve_b.gamma_y14, CIF_ISP_GAMMA_B_Y14);
	cifisp_iowrite32(pconfig->curve_b.gamma_y15, CIF_ISP_GAMMA_B_Y15);
	cifisp_iowrite32(pconfig->curve_b.gamma_y16, CIF_ISP_GAMMA_B_Y16);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_sdg_config_read(const struct xgold_isp_dev *isp_dev,
				   struct cifisp_sdg_config *pconfig)
{
	pconfig->xa_pnts.gamma_dx0 = cifisp_ioread32(CIF_ISP_GAMMA_DX_LO);
	pconfig->xa_pnts.gamma_dx1 = cifisp_ioread32(CIF_ISP_GAMMA_DX_HI);

	pconfig->curve_r.gamma_y0 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y0);
	pconfig->curve_r.gamma_y1 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y1);
	pconfig->curve_r.gamma_y2 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y2);
	pconfig->curve_r.gamma_y3 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y3);
	pconfig->curve_r.gamma_y4 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y4);
	pconfig->curve_r.gamma_y5 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y5);
	pconfig->curve_r.gamma_y6 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y6);
	pconfig->curve_r.gamma_y7 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y7);
	pconfig->curve_r.gamma_y8 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y8);
	pconfig->curve_r.gamma_y9 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y9);
	pconfig->curve_r.gamma_y10 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y10);
	pconfig->curve_r.gamma_y11 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y11);
	pconfig->curve_r.gamma_y12 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y12);
	pconfig->curve_r.gamma_y13 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y13);
	pconfig->curve_r.gamma_y14 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y13);
	pconfig->curve_r.gamma_y15 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y15);
	pconfig->curve_r.gamma_y16 = cifisp_ioread32(CIF_ISP_GAMMA_R_Y16);

	pconfig->curve_g.gamma_y0 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y0);
	pconfig->curve_g.gamma_y1 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y1);
	pconfig->curve_g.gamma_y2 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y2);
	pconfig->curve_g.gamma_y3 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y3);
	pconfig->curve_g.gamma_y4 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y4);
	pconfig->curve_g.gamma_y5 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y5);
	pconfig->curve_g.gamma_y6 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y6);
	pconfig->curve_g.gamma_y7 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y7);
	pconfig->curve_g.gamma_y8 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y8);
	pconfig->curve_g.gamma_y9 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y9);
	pconfig->curve_g.gamma_y10 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y10);
	pconfig->curve_g.gamma_y11 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y11);
	pconfig->curve_g.gamma_y12 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y12);
	pconfig->curve_g.gamma_y13 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y13);
	pconfig->curve_g.gamma_y14 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y13);
	pconfig->curve_g.gamma_y15 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y15);
	pconfig->curve_g.gamma_y16 = cifisp_ioread32(CIF_ISP_GAMMA_G_Y16);

	pconfig->curve_b.gamma_y0 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y0);
	pconfig->curve_b.gamma_y1 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y1);
	pconfig->curve_b.gamma_y2 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y2);
	pconfig->curve_b.gamma_y3 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y3);
	pconfig->curve_b.gamma_y4 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y4);
	pconfig->curve_b.gamma_y5 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y5);
	pconfig->curve_b.gamma_y6 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y6);
	pconfig->curve_b.gamma_y7 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y7);
	pconfig->curve_b.gamma_y8 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y8);
	pconfig->curve_b.gamma_y9 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y9);
	pconfig->curve_b.gamma_y10 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y10);
	pconfig->curve_b.gamma_y11 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y11);
	pconfig->curve_b.gamma_y12 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y12);
	pconfig->curve_b.gamma_y13 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y13);
	pconfig->curve_b.gamma_y14 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y13);
	pconfig->curve_b.gamma_y15 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y15);
	pconfig->curve_b.gamma_y16 = cifisp_ioread32(CIF_ISP_GAMMA_B_Y16);
}
#endif

/*****************************************************************************/
static void cifisp_sdg_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_GAMMA_IN_ENA, CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_sdg_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_GAMMA_IN_ENA, CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_goc_config(const struct xgold_isp_dev *isp_dev)
{
	int i;
	const struct cifisp_goc_config *pconfig = &(isp_dev->goc_config);

	cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA, CIF_ISP_CTRL);

	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		cifisp_iowrite32(pconfig->gamma_r[i], CIF_ISP_GAMMA_OUT_R_TBL);
	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		cifisp_iowrite32(pconfig->gamma_g[i], CIF_ISP_GAMMA_OUT_G_TBL);
	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		cifisp_iowrite32(pconfig->gamma_b[i], CIF_ISP_GAMMA_OUT_B_TBL);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_goc_config_read(const struct xgold_isp_dev *isp_dev,
				   struct cifisp_goc_config *pconfig)
{
	int i, on;

	on = CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA_READ(cifisp_ioread32(CIF_ISP_CTRL));

	cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA, CIF_ISP_CTRL);

	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		pconfig->gamma_r[i] = cifisp_ioread32(CIF_ISP_GAMMA_OUT_R_TBL);
	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		pconfig->gamma_g[i] = cifisp_ioread32(CIF_ISP_GAMMA_OUT_G_TBL);
	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		pconfig->gamma_b[i] = cifisp_ioread32(CIF_ISP_GAMMA_OUT_B_TBL);

	if (on)
		cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA,
		CIF_ISP_CTRL);
}
#endif

/*****************************************************************************/
static void cifisp_goc_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA, CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_goc_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA, CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_bdm_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_bdm_config *pconfig = &(isp_dev->bdm_config);

	/*set demosaic threshold */
	cifisp_iowrite32(pconfig->demosaic_th, CIF_ISP_DEMOSAIC);
}

/*****************************************************************************/
static void cifisp_bdm_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32AND(~(CIFISP_BDM_BYPASS_EN(1)), CIF_ISP_DEMOSAIC);
}

/*****************************************************************************/
static void cifisp_bdm_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_DEMOSAIC);
}

/*****************************************************************************/
static void cifisp_flt_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_flt_config *pconfig = &(isp_dev->flt_config);

	cifisp_iowrite32(pconfig->flt_mask_sharp0, CIF_ISP_FILT_MASK_SHARP_0);
	cifisp_iowrite32(pconfig->flt_mask_sharp1, CIF_ISP_FILT_MASK_SHARP_1);
	cifisp_iowrite32(pconfig->flt_mask_diag, CIF_ISP_FILT_MASK_SHARP_DIAG);
	cifisp_iowrite32(pconfig->flt_mask_blur_max,
		CIF_ISP_FILT_MASK_BLUR_MAX);
	cifisp_iowrite32(pconfig->flt_mask_blur, CIF_ISP_FILT_MASK_BLUR);
	cifisp_iowrite32(pconfig->flt_mask_lin, CIF_ISP_FILT_MASK_LIN);
	cifisp_iowrite32(pconfig->flt_mask_orth, CIF_ISP_FILT_MASK_LIN_ORTH);
	cifisp_iowrite32(pconfig->flt_mask_v_diag, CIF_ISP_FILT_MASK_DIAG);
	cifisp_iowrite32(pconfig->flt_mask_h_diag, CIF_ISP_FILT_MASK_H_DIAG);
	cifisp_iowrite32(pconfig->flt_blur_th0, CIF_ISP_FILT_BLUR_TH0);
	cifisp_iowrite32(pconfig->flt_blur_th1, CIF_ISP_FILT_BLUR_TH1);
	cifisp_iowrite32(pconfig->flt_sharp0_th, CIF_ISP_FILT_SHARP0_TH);
	cifisp_iowrite32(pconfig->flt_sharp1_th, CIF_ISP_FILT_SHARP1_TH);
	cifisp_iowrite32(pconfig->flt_lum_weight, CIF_ISP_FILT_LUM_WEIGHT);

	cifisp_iowrite32(CIFISP_FILTER_MODE(pconfig->flt_mode) |
		      CIFISP_FILTER_SHARP_MODE(pconfig->flt_diag_sharp_mode) |
		      CIFISP_FILTER_CH_V_MODE(pconfig->flt_chrom_v_mode) |
		      CIFISP_FILTER_CH_H_MODE(pconfig->flt_chrom_h_mode),
		      CIF_ISP_FILT_MODE);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_flt_config_read(const struct xgold_isp_dev *isp_dev,
				   struct cifisp_flt_config *pconfig)
{
	unsigned int reg;

	pconfig->flt_mask_sharp0 = cifisp_ioread32(CIF_ISP_FILT_MASK_SHARP_0);
	pconfig->flt_mask_sharp1 = cifisp_ioread32(CIF_ISP_FILT_MASK_SHARP_1);
	pconfig->flt_mask_diag = cifisp_ioread32(CIF_ISP_FILT_MASK_SHARP_DIAG);
	pconfig->flt_mask_blur_max =
		cifisp_ioread32(CIF_ISP_FILT_MASK_BLUR_MAX);
	pconfig->flt_mask_blur = cifisp_ioread32(CIF_ISP_FILT_MASK_BLUR);
	pconfig->flt_mask_lin = cifisp_ioread32(CIF_ISP_FILT_MASK_LIN);
	pconfig->flt_mask_orth = cifisp_ioread32(CIF_ISP_FILT_MASK_LIN_ORTH);
	pconfig->flt_mask_v_diag = cifisp_ioread32(CIF_ISP_FILT_MASK_DIAG);
	pconfig->flt_mask_h_diag = cifisp_ioread32(CIF_ISP_FILT_MASK_H_DIAG);
	pconfig->flt_blur_th0 = cifisp_ioread32(CIF_ISP_FILT_BLUR_TH0);
	pconfig->flt_blur_th1 = cifisp_ioread32(CIF_ISP_FILT_BLUR_TH1);
	pconfig->flt_sharp0_th = cifisp_ioread32(CIF_ISP_FILT_SHARP0_TH);
	pconfig->flt_sharp1_th = cifisp_ioread32(CIF_ISP_FILT_SHARP1_TH);
	pconfig->flt_lum_weight = cifisp_ioread32(CIF_ISP_FILT_LUM_WEIGHT);
	reg = cifisp_ioread32(CIF_ISP_FILT_MODE);
	pconfig->flt_mode = CIFISP_FILTER_MODE_READ(reg);
	pconfig->flt_diag_sharp_mode = CIFISP_FILTER_SHARP_MODE_READ(reg);
	pconfig->flt_chrom_v_mode = CIFISP_FILTER_CH_V_MODE_READ(reg);
	pconfig->flt_chrom_h_mode = CIFISP_FILTER_CH_H_MODE_READ(reg);
}
#endif

/*****************************************************************************/
static void cifisp_flt_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(1, CIF_ISP_FILT_MODE);
}

/*****************************************************************************/
static void cifisp_flt_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_FILT_MODE);
}

/* Auto White Balance */
/*****************************************************************************/
static void cifisp_awb_gain_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_awb_gain_config *pconfig =
	    &(isp_dev->awb_gain_config);

	cifisp_iowrite32(CIFISP_AWB_GAIN_R_SET(pconfig->gain_green_r) |
		      pconfig->gain_green_b, CIF_ISP_AWB_GAIN_G);

	cifisp_iowrite32(CIFISP_AWB_GAIN_R_SET(pconfig->gain_red) |
		      pconfig->gain_blue, CIF_ISP_AWB_GAIN_RB);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_awb_gain_config_read(const struct xgold_isp_dev *isp_dev,
					struct cifisp_awb_gain_config *pconfig)
{
	unsigned int reg = cifisp_ioread32(CIF_ISP_AWB_GAIN_G);

	pconfig->gain_green_r = CIFISP_AWB_GAIN_R_READ(reg);
	pconfig->gain_green_b = CIFISP_AWB_GAIN_B_READ(reg);
	reg = cifisp_ioread32(CIF_ISP_AWB_GAIN_RB);
	pconfig->gain_red = CIFISP_AWB_GAIN_R_READ(reg);
	pconfig->gain_blue = CIFISP_AWB_GAIN_B_READ(reg);
}
#endif

/*****************************************************************************/
static void cifisp_awb_meas_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_awb_meas_config *pconfig =
		&(isp_dev->awb_meas_config);
	unsigned int awb_prob = 0;

	/*based on the mode,configure the awb module */
	if (pconfig->awb_mode == CIFISP_AWB_MODE_RGB) {
		awb_prob = CIFISP_AWB_RGB_MEAS_PNT_SET(pconfig->rgb_meas_pnt);
		/* RGB Thresholds */
		cifisp_iowrite32(CIFISP_AWB_B_SAT_SET(pconfig->gb_sat) |
			pconfig->gr_sat, CIF_ISP_AWB_THRESH_G);
		cifisp_iowrite32(CIFISP_AWB_B_SAT_SET(pconfig->b_sat) |
			pconfig->r_sat, CIF_ISP_AWB_THRESH_RB);
	} else {
		if (pconfig->enable_ymax_cmp)
			awb_prob = CIFISP_AWB_YMAX_CMP_EN;

		/*Reference Cb and Cr */
		cifisp_iowrite32(CIFISP_AWB_REF_CR_SET(pconfig->awb_ref_cr) |
			pconfig->awb_ref_cb, CIF_ISP_AWB_REF);
		/* Yc Threshold */
		cifisp_iowrite32(CIFISP_AWB_MAX_Y_SET(pconfig->max_y) |
			CIFISP_AWB_MIN_Y_SET(pconfig->min_y) |
			CIFISP_AWB_MAX_CS_SET(pconfig->max_csum) |
			pconfig->min_c, CIF_ISP_AWB_THRESH_YC);
	}

	/* Common Configuration */
	cifisp_iowrite32(awb_prob, CIF_ISP_AWB_PROP);
	/*window offset */
	cifisp_iowrite32(CIFISP_AWB_VOFF_SET(pconfig->awb_wnd.v_offs) |
		pconfig->awb_wnd.h_offs, CIF_ISP_AWB_OFFS);
	/*AWB window size */
	cifisp_iowrite32(CIFISP_AWB_VSIZE_SET(pconfig->awb_wnd.v_size) |
		pconfig->awb_wnd.h_size, CIF_ISP_AWB_WND_SIZE);
	/*Grid Dimension */
	/*The given value+1 vertical measurement windows will be used.*/
	cifisp_iowrite32(CIFISP_AWB_VDIM_SET(pconfig->grid_v_dim - 1) |
		(pconfig->grid_h_dim - 1), CIF_ISP_AWB_GRID_DIM);
	/*Grid window distance */
	cifisp_iowrite32(CIFISP_AWB_VDIST_SET(pconfig->grid_v_dist) |
		pconfig->grid_h_dist, CIF_ISP_AWB_GRID_DIST);
	/*Number of frames */
	cifisp_iowrite32(pconfig->frames, CIF_ISP_AWB_FRAMES);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_awb_meas_config_read(const struct xgold_isp_dev *isp_dev,
					struct cifisp_awb_meas_config *pconfig)
{
	unsigned int reg;

	reg = cifisp_ioread32(CIF_ISP_AWB_PROP);

	pconfig->awb_mode = CIFISP_AWB_MODE_READ(reg);
	pconfig->rgb_meas_pnt = CIFISP_AWB_RGBPOINT_READ(reg);
	pconfig->enable_ymax_cmp = CIFISP_AWB_YMAX_READ(reg);
	reg = cifisp_ioread32(CIF_ISP_AWB_THRESH_G);
	pconfig->gr_sat = CIFISP_AWB_R_SAT_READ(reg);
	pconfig->gb_sat = CIFISP_AWB_B_SAT_READ(reg);
	reg = cifisp_ioread32(CIF_ISP_AWB_THRESH_RB);
	pconfig->b_sat = CIFISP_AWB_B_SAT_READ(reg);
	pconfig->r_sat = CIFISP_AWB_R_SAT_READ(reg);
	reg = cifisp_ioread32(CIF_ISP_AWB_REF);
	pconfig->awb_ref_cr = CIFISP_AWB_REF_CR_READ(reg);
	pconfig->awb_ref_cb = CIFISP_AWB_REF_CB_READ(reg);
	reg = cifisp_ioread32(CIF_ISP_AWB_THRESH_YC);
	pconfig->max_y = CIFISP_AWB_MAX_Y_READ(reg);
	pconfig->min_y = CIFISP_AWB_MIN_Y_READ(reg);
	pconfig->max_csum = CIFISP_AWB_MAX_CS_READ(reg);
	pconfig->min_c = CIFISP_AWB_MIN_C_READ(reg);
	/*window offset */
	reg = cifisp_ioread32(CIF_ISP_AWB_OFFS);
	pconfig->awb_wnd.v_offs = CIFISP_AWB_VOFF_READ(reg);
	pconfig->awb_wnd.h_offs = CIFISP_AWB_HOFF_READ(reg);
	/*AWB window size */
	reg = cifisp_ioread32(CIF_ISP_AWB_WND_SIZE);
	pconfig->awb_wnd.v_size = CIFISP_AWB_VSIZE_READ(reg);
	pconfig->awb_wnd.h_size = CIFISP_AWB_HSIZE_READ(reg);
	/*Grid Dimension */
	reg = cifisp_ioread32(CIF_ISP_AWB_GRID_DIM);
	pconfig->grid_v_dim = (CIFISP_AWB_VDIM_READ(reg) + 1);
	pconfig->grid_h_dim = (CIFISP_AWB_HDIM_READ(reg) + 1);
	/*Grid window distance */
	reg = cifisp_ioread32(CIF_ISP_AWB_GRID_DIST);
	pconfig->grid_v_dist = CIFISP_AWB_VDIST_READ(reg);
	pconfig->grid_h_dist = CIFISP_AWB_HDIST_READ(reg);
	/*Number of frames */
	pconfig->frames = cifisp_ioread32(CIF_ISP_AWB_FRAMES);
}
#endif

/*****************************************************************************/
static void cifisp_awb_meas_en(struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_awb_meas_config *pconfig =
		&(isp_dev->awb_meas_config);
	u32 reg_val = cifisp_ioread32(CIF_ISP_AWB_PROP);

	/* switch off */
	reg_val &= 0xFFFFFFFC;

	if (pconfig->awb_mode == CIFISP_AWB_MODE_RGB)
		reg_val |= 1;
	else
		reg_val |= 2;

	cifisp_iowrite32(reg_val, CIF_ISP_AWB_PROP);

	isp_dev->active_meas |= CIF_ISP_AWB_DONE;

	/* Measurements require AWB block be active. */
	cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_AWB_ENA, CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_awb_meas_end(struct xgold_isp_dev *isp_dev)
{
	u32 reg_val = cifisp_ioread32(CIF_ISP_AWB_PROP);

	/* switch off */
	reg_val &= 0xFFFFFFFC;

	cifisp_iowrite32(reg_val, CIF_ISP_AWB_PROP);

	isp_dev->active_meas &= ~CIF_ISP_AWB_DONE;

	if (!isp_dev->awb_gain_en)
		cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_AWB_ENA,
		CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_awb_gain_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_AWB_ENA, CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_awb_gain_end(const struct xgold_isp_dev *isp_dev)
{
	if (!isp_dev->awb_meas_en)
		cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_AWB_ENA,
		CIF_ISP_CTRL);
}

/*****************************************************************************/
static void cifisp_get_awb_meas(struct xgold_isp_dev *isp_dev,
	struct cifisp_stat_buffer *pbuf)
{
	/* Protect against concurrent access from ISR? */
	int i;
	u32 reg_val;
	const struct cifisp_awb_meas_config *pconfig =
		&(isp_dev->awb_meas_config);

	int num_meas = (pconfig->grid_v_dim) * (pconfig->grid_h_dim);

	pbuf->meas_type = CIFISP_STAT_AWB;

	cifisp_iowrite32(0, CIF_ISP_AWB_GRID_POS);

	if (pconfig->awb_mode == CIFISP_AWB_MODE_RGB) {
		for (i = 0; i < num_meas; i++) {
			reg_val = cifisp_ioread32(CIF_ISP_AWB_WHITE_CNT);

			pbuf->params.awb.awb_mean[i].cnt =
				CIFISP_AWB_GET_PIXEL_CNT(reg_val);

			reg_val = cifisp_ioread32(CIF_ISP_AWB_MEAN_RB);

			pbuf->params.awb.awb_mean[i].mean_r =
				CIFISP_AWB_GET_MEAN_R(reg_val);
			pbuf->params.awb.awb_mean[i].mean_b =
				CIFISP_AWB_GET_MEAN_B(reg_val);

			reg_val = cifisp_ioread32(CIF_ISP_AWB_MEAN_G);

			pbuf->params.awb.awb_mean[i].mean_g =
				CIFISP_AWB_GET_MEAN_G(reg_val);
		}
	} else {
		for (i = 0; i < num_meas; i++) {
			reg_val = cifisp_ioread32(CIF_ISP_AWB_WHITE_CNT);
			pbuf->params.awb.awb_mean[i].cnt =
				CIFISP_AWB_GET_PIXEL_CNT(reg_val);
			reg_val = cifisp_ioread32(CIF_ISP_AWB_MEAN_YC);
			pbuf->params.awb.awb_mean[i].mean_cr =
				(u8) CIFISP_AWB_GET_MEAN_CR(reg_val);
			pbuf->params.awb.awb_mean[i].mean_cb =
				(u8) CIFISP_AWB_GET_MEAN_CB(reg_val);
			pbuf->params.awb.awb_mean[i].mean_y =
				(u8) CIFISP_AWB_GET_MEAN_Y(reg_val);
		}
	}
}

/* Auto Exposure */
/*****************************************************************************/
static void cifisp_aec_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_aec_config *pconfig = &(isp_dev->aec_config);

	cifisp_iowrite32(CIFISP_EXP_CTRL_AUTOSTOP(pconfig->autostop),
		      CIF_ISP_EXP_CTRL);

	cifisp_iowrite32(pconfig->meas_window.h_offs, CIF_ISP_EXP_H_OFFSET);
	cifisp_iowrite32(pconfig->meas_window.v_offs, CIF_ISP_EXP_V_OFFSET);
	cifisp_iowrite32(pconfig->meas_window.h_size, CIF_ISP_EXP_H_SIZE);
	cifisp_iowrite32(pconfig->meas_window.v_size, CIF_ISP_EXP_V_SIZE);
}

/*****************************************************************************/
static void cifisp_aec_en(struct xgold_isp_dev *isp_dev)
{
	isp_dev->active_meas |= CIF_ISP_EXP_END;

	cifisp_iowrite32OR(1, CIF_ISP_EXP_CTRL);
}

/*****************************************************************************/
static void cifisp_aec_end(struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_EXP_CTRL);

	isp_dev->active_meas &= ~CIF_ISP_EXP_END;
}

/*****************************************************************************/
static void cifisp_get_aec_meas(struct xgold_isp_dev *isp_dev,
				struct cifisp_stat_buffer *pbuf)
{
	pbuf->meas_type = CIFISP_STAT_AUTOEXP;	/*Set the measurement type */

	pbuf->params.ae.exp_mean[0] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_00);
	pbuf->params.ae.exp_mean[1] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_10);
	pbuf->params.ae.exp_mean[2] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_20);
	pbuf->params.ae.exp_mean[3] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_30);
	pbuf->params.ae.exp_mean[4] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_40);
	pbuf->params.ae.exp_mean[5] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_01);
	pbuf->params.ae.exp_mean[6] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_11);
	pbuf->params.ae.exp_mean[7] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_21);
	pbuf->params.ae.exp_mean[8] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_31);
	pbuf->params.ae.exp_mean[9] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_41);
	pbuf->params.ae.exp_mean[10] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_02);
	pbuf->params.ae.exp_mean[11] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_12);
	pbuf->params.ae.exp_mean[12] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_22);
	pbuf->params.ae.exp_mean[13] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_32);
	pbuf->params.ae.exp_mean[14] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_42);
	pbuf->params.ae.exp_mean[15] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_03);
	pbuf->params.ae.exp_mean[16] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_13);
	pbuf->params.ae.exp_mean[17] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_23);
	pbuf->params.ae.exp_mean[18] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_33);
	pbuf->params.ae.exp_mean[19] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_43);
	pbuf->params.ae.exp_mean[20] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_04);
	pbuf->params.ae.exp_mean[21] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_14);
	pbuf->params.ae.exp_mean[22] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_24);
	pbuf->params.ae.exp_mean[23] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_34);
	pbuf->params.ae.exp_mean[24] =
		(u8) cifisp_ioread32(CIF_ISP_EXP_MEAN_44);
}

/* X-Talk Matrix */
/*****************************************************************************/
static void cifisp_ctk_config(const struct xgold_isp_dev *isp_dev)
{
	/* Nothing to do */
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_ctk_config_read(const struct xgold_isp_dev *isp_dev,
				   struct cifisp_ctk_config *pconfig)
{
	pconfig->coeff0 = cifisp_ioread32(CIF_ISP_CT_COEFF_0);
	pconfig->coeff1 = cifisp_ioread32(CIF_ISP_CT_COEFF_1);
	pconfig->coeff2 = cifisp_ioread32(CIF_ISP_CT_COEFF_2);
	pconfig->coeff3 = cifisp_ioread32(CIF_ISP_CT_COEFF_3);
	pconfig->coeff4 = cifisp_ioread32(CIF_ISP_CT_COEFF_4);
	pconfig->coeff5 = cifisp_ioread32(CIF_ISP_CT_COEFF_5);
	pconfig->coeff6 = cifisp_ioread32(CIF_ISP_CT_COEFF_6);
	pconfig->coeff7 = cifisp_ioread32(CIF_ISP_CT_COEFF_7);
	pconfig->coeff8 = cifisp_ioread32(CIF_ISP_CT_COEFF_8);
	pconfig->ct_offset_r = cifisp_ioread32(CIF_ISP_CT_OFFSET_R);
	pconfig->ct_offset_g = cifisp_ioread32(CIF_ISP_CT_OFFSET_G);
	pconfig->ct_offset_b = cifisp_ioread32(CIF_ISP_CT_OFFSET_B);
}
#endif

/*****************************************************************************/
static void cifisp_ctk_en(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_ctk_config *pconfig = &(isp_dev->ctk_config);

	cifisp_iowrite32(pconfig->coeff0, CIF_ISP_CT_COEFF_0);
	cifisp_iowrite32(pconfig->coeff1, CIF_ISP_CT_COEFF_1);
	cifisp_iowrite32(pconfig->coeff2, CIF_ISP_CT_COEFF_2);
	cifisp_iowrite32(pconfig->coeff3, CIF_ISP_CT_COEFF_3);
	cifisp_iowrite32(pconfig->coeff4, CIF_ISP_CT_COEFF_4);
	cifisp_iowrite32(pconfig->coeff5, CIF_ISP_CT_COEFF_5);
	cifisp_iowrite32(pconfig->coeff6, CIF_ISP_CT_COEFF_6);
	cifisp_iowrite32(pconfig->coeff7, CIF_ISP_CT_COEFF_7);
	cifisp_iowrite32(pconfig->coeff8, CIF_ISP_CT_COEFF_8);
	cifisp_iowrite32(pconfig->ct_offset_r, CIF_ISP_CT_OFFSET_R);
	cifisp_iowrite32(pconfig->ct_offset_g, CIF_ISP_CT_OFFSET_G);
	cifisp_iowrite32(pconfig->ct_offset_b, CIF_ISP_CT_OFFSET_B);
}

/*****************************************************************************/
static void cifisp_ctk_end(const struct xgold_isp_dev *isp_dev)
{
	/* Write back the default values. */
	cifisp_iowrite32(0x80, CIF_ISP_CT_COEFF_0);
	cifisp_iowrite32(0, CIF_ISP_CT_COEFF_1);
	cifisp_iowrite32(0, CIF_ISP_CT_COEFF_2);
	cifisp_iowrite32(0, CIF_ISP_CT_COEFF_3);
	cifisp_iowrite32(0x80, CIF_ISP_CT_COEFF_4);
	cifisp_iowrite32(0, CIF_ISP_CT_COEFF_5);
	cifisp_iowrite32(0, CIF_ISP_CT_COEFF_6);
	cifisp_iowrite32(0, CIF_ISP_CT_COEFF_7);
	cifisp_iowrite32(0x80, CIF_ISP_CT_COEFF_8);

	cifisp_iowrite32(0, CIF_ISP_CT_OFFSET_R);
	cifisp_iowrite32(0, CIF_ISP_CT_OFFSET_G);
	cifisp_iowrite32(0, CIF_ISP_CT_OFFSET_B);
}

/* CPROC */
/*****************************************************************************/
static void cifisp_cproc_config(const struct xgold_isp_dev *isp_dev,
	bool capture)
{
	const struct cifisp_cproc_config *pconfig = &(isp_dev->cproc_config);

	cifisp_iowrite32(pconfig->contrast, CIF_C_PROC_CONTRAST);
	cifisp_iowrite32(pconfig->hue, CIF_C_PROC_HUE);
	cifisp_iowrite32(pconfig->sat, CIF_C_PROC_SATURATION);
	cifisp_iowrite32(pconfig->brightness, CIF_C_PROC_BRIGHTNESS);

	if (!capture || cifisp_is_ie_active(isp_dev)) {
		cifisp_iowrite32OR(
			pconfig->c_out_range << 3 |
			pconfig->y_in_range << 2 |
			pconfig->y_out_range << 1,
			CIF_C_PROC_CTRL);
	} else {
		cifisp_iowrite32OR(
			1 << 3 |
			1 << 2 |
			1 << 1,
			CIF_C_PROC_CTRL);
	}
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_cproc_config_read(const struct xgold_isp_dev *isp_dev,
				     struct cifisp_cproc_config *pconfig)
{
	unsigned int reg;

	pconfig->contrast = cifisp_ioread32(CIF_C_PROC_CONTRAST);
	pconfig->hue = cifisp_ioread32(CIF_C_PROC_HUE);
	pconfig->sat = cifisp_ioread32(CIF_C_PROC_SATURATION);
	pconfig->brightness = cifisp_ioread32(CIF_C_PROC_BRIGHTNESS);
	reg = cifisp_ioread32(CIF_C_PROC_CTRL);
	pconfig->y_out_range = (reg >> 1) & 1;
	pconfig->y_in_range = (reg >> 2) & 1;
	pconfig->c_out_range = (reg >> 3) & 1;
}
#endif

/*****************************************************************************/
static void cifisp_cproc_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(CIFISP_CPROC_EN, CIF_C_PROC_BASE);
}

/*****************************************************************************/
static void cifisp_cproc_end(const struct xgold_isp_dev *isp_dev)
{
	if (!isp_dev->tmap_en && !isp_dev->macc_en)
		cifisp_iowrite32AND(~CIFISP_CPROC_EN, CIF_C_PROC_CTRL);
	else {
		/* Default values */
		cifisp_iowrite32(0x80, CIF_C_PROC_CONTRAST);
		cifisp_iowrite32(0, CIF_C_PROC_HUE);
		cifisp_iowrite32(0x80, CIF_C_PROC_SATURATION);
		cifisp_iowrite32(0, CIF_C_PROC_BRIGHTNESS);

		cifisp_iowrite32AND(0x31, CIF_C_PROC_CTRL);
	}
}

static void cifisp_macc_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_macc_config *pconfig = &(isp_dev->macc_config);

	cifisp_iowrite32AND(~CIFISP_CPROC_MACC_EN, CIF_C_PROC_CTRL);

	cifisp_iowrite32(pconfig->seg0.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg0.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg1.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg1.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg2.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg2.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg3.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg3.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg4.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg4.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg5.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg5.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg6.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg6.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg7.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg7.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg8.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg8.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg9.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg9.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg10.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg10.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg11.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg11.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg12.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg12.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg13.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg13.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg14.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg14.coeff1, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg15.coeff0, CIF_C_PROC_MACC);
	cifisp_iowrite32(pconfig->seg15.coeff1, CIF_C_PROC_MACC);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_macc_config_read(const struct xgold_isp_dev *isp_dev,
				    struct cifisp_macc_config *pconfig)
{
	int on = cifisp_ioread32(CIF_C_PROC_CTRL) & CIFISP_CPROC_MACC_EN;

	cifisp_iowrite32AND(~CIFISP_CPROC_MACC_EN, CIF_C_PROC_CTRL);

	pconfig->seg0.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg0.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg1.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg1.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg2.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg2.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg3.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg3.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg4.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg4.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg5.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg5.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg6.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg6.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg7.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg7.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg8.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg8.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg9.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg9.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg10.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg10.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg11.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg11.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg12.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg12.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg13.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg13.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg14.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg14.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg15.coeff0 = cifisp_ioread32(CIF_C_PROC_MACC);
	pconfig->seg15.coeff1 = cifisp_ioread32(CIF_C_PROC_MACC);

	if (on)
		cifisp_iowrite32OR(CIFISP_CPROC_MACC_EN, CIF_C_PROC_CTRL);
}
#endif

/*****************************************************************************/
static void cifisp_macc_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(CIFISP_CPROC_MACC_EN | CIFISP_CPROC_EN,
			CIF_C_PROC_CTRL);
}

/*****************************************************************************/
static void cifisp_macc_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32AND(~CIFISP_CPROC_MACC_EN, CIF_C_PROC_CTRL);

	if (!isp_dev->tmap_en && !isp_dev->cproc_en)
		cifisp_iowrite32AND(~CIFISP_CPROC_EN, CIF_C_PROC_CTRL);
}

static void cifisp_tmap_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_tmap_config *pconfig = &(isp_dev->tmap_config);
	int i;

	for (i = 0; i < CIFISP_TONE_MAP_TABLE_SIZE; i++)
		cifisp_iowrite32(CIFISP_CPROC_TONE_Y(pconfig->tmap_y[i]) |
			      CIFISP_CPROC_TONE_C(pconfig->tmap_c[i]),
			      CIF_C_PROC_TMAP);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_tmap_config_read(const struct xgold_isp_dev *isp_dev,
				    struct cifisp_tmap_config *pconfig)
{
	int i, reg;

	for (i = 0; i < CIFISP_TONE_MAP_TABLE_SIZE; i++) {
		reg = cifisp_ioread32(CIF_C_PROC_TMAP);
		pconfig->tmap_y[i] = CIFISP_CPROC_TONE_Y_READ(reg);
		pconfig->tmap_c[i] = CIFISP_CPROC_TONE_C_READ(reg);
	}
}
#endif

/*****************************************************************************/
static void cifisp_tmap_en(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32OR(CIFISP_CPROC_TMAP_EN | CIFISP_CPROC_EN,
			CIF_C_PROC_CTRL);
}

/*****************************************************************************/
static void cifisp_tmap_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32AND(~CIFISP_CPROC_TMAP_EN, CIF_C_PROC_CTRL);

	if (!isp_dev->cproc_en && !isp_dev->macc_en)
		cifisp_iowrite32AND(~CIFISP_CPROC_EN, CIF_C_PROC_CTRL);
}

void cifisp_ycflt_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_ycflt_config *pconfig = &(isp_dev->ycflt_config);

	cifisp_iowrite32(pconfig->chr_ss_fac, CIF_YC_FLT_CHR_SS_FAC);
	cifisp_iowrite32(pconfig->chr_ss_offs, CIF_YC_FLT_CHR_SS_OFFS);
	cifisp_iowrite32(pconfig->chr_nr_ctrl, CIF_YC_FLT_CHR_NR_CTRL);
	cifisp_iowrite32(pconfig->lum_eenr_edge_gain,
		      CIF_YC_FLT_LUM_EENR_EDGE_GAIN);
	cifisp_iowrite32(pconfig->lum_eenr_corner_gain,
		      CIF_YC_FLT_LUM_ENNR_CORNER_GAIN);
	cifisp_iowrite32(pconfig->lum_eenr_fc_crop_neg,
		      CIF_YC_FLT_LUM_ENNR_FC_CROP_NEG);
	cifisp_iowrite32(pconfig->lum_eenr_fc_crop_pos,
		      CIF_YC_FLT_LUM_ENNR_FC_CROP_POS);
	cifisp_iowrite32(pconfig->lum_eenr_fc_gain_neg,
		      CIF_YC_FLT_LUM_ENNR_FC_GAIN_NEG);
	cifisp_iowrite32(pconfig->lum_eenr_fc_gain_pos,
		      CIF_YC_FLT_LUM_ENNR_FC_GAIN_POS);
}

#ifdef LOG_CAPTURE_PARAMS
static void cifisp_ycflt_config_read(const struct xgold_isp_dev *isp_dev,
				     struct cifisp_ycflt_config *pconfig)
{
	pconfig->chr_ss_ctrl = cifisp_ioread32(CIF_YC_FLT_CHR_SS_CTRL);
	pconfig->chr_ss_fac = cifisp_ioread32(CIF_YC_FLT_CHR_SS_FAC);
	pconfig->chr_ss_offs = cifisp_ioread32(CIF_YC_FLT_CHR_SS_OFFS);
	pconfig->chr_nr_ctrl = cifisp_ioread32(CIF_YC_FLT_CHR_NR_CTRL);
	pconfig->lum_eenr_edge_gain =
	    cifisp_ioread32(CIF_YC_FLT_LUM_EENR_EDGE_GAIN);
	pconfig->lum_eenr_corner_gain =
	    cifisp_ioread32(CIF_YC_FLT_LUM_ENNR_CORNER_GAIN);
	pconfig->lum_eenr_fc_crop_neg =
	    cifisp_ioread32(CIF_YC_FLT_LUM_ENNR_FC_CROP_NEG);
	pconfig->lum_eenr_fc_crop_pos =
	    cifisp_ioread32(CIF_YC_FLT_LUM_ENNR_FC_CROP_POS);
	pconfig->lum_eenr_fc_gain_neg =
	    cifisp_ioread32(CIF_YC_FLT_LUM_ENNR_FC_GAIN_NEG);
	pconfig->lum_eenr_fc_gain_pos =
	    cifisp_ioread32(CIF_YC_FLT_LUM_ENNR_FC_GAIN_POS);
}
#endif

/*****************************************************************************/
void cifisp_ycflt_en(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_ycflt_config *pconfig;

	if (isp_dev->cif_ism_cropping == true)
		pconfig = &(isp_dev->ycflt_config_ism_on);
	else
		pconfig = &(isp_dev->ycflt_config);

	cifisp_iowrite32(pconfig->ctrl, CIF_YC_FLT_CTRL);
	cifisp_iowrite32(pconfig->chr_ss_ctrl, CIF_YC_FLT_CHR_SS_CTRL);

	if (pconfig->chr_ss_ctrl & 0x1)
		cif_isp20_pltfrm_pr_dbg(NULL, "XNR Vertical SS is ON\n");
}

/*****************************************************************************/
void cifisp_ycflt_end(const struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(1<<6|1<<5|1<<4, CIF_YC_FLT_CTRL);
	cifisp_iowrite32(3<<8|2<<4, CIF_YC_FLT_CHR_SS_CTRL);
}

static void cifisp_afc_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_afc_config *pconfig = &(isp_dev->afc_config);
	int num_of_win = pconfig->num_afm_win;

	/* Switch off to configure. Enabled during normal flow in frame isr. */
	cifisp_iowrite32(0, CIF_ISP_AFM_CTRL);

	if (num_of_win) {
		cifisp_iowrite32(
			CIFISP_AFC_WINDOW_X(pconfig->afm_win[0].h_offs) |
			CIFISP_AFC_WINDOW_Y(pconfig->afm_win[0].v_offs),
			CIF_ISP_AFM_LT_A);
		cifisp_iowrite32(
			CIFISP_AFC_WINDOW_X(pconfig->afm_win[0].h_size +
			pconfig->afm_win[0].h_offs) |
			CIFISP_AFC_WINDOW_Y(pconfig->afm_win[0].v_size +
			pconfig->afm_win[0].v_offs),
			CIF_ISP_AFM_RB_A);
		num_of_win--;
	}

	if (num_of_win) {
		cifisp_iowrite32(
			CIFISP_AFC_WINDOW_X(pconfig->afm_win[1].h_offs) |
			CIFISP_AFC_WINDOW_Y(pconfig->afm_win[1].v_offs),
			CIF_ISP_AFM_LT_B);
		cifisp_iowrite32(
			CIFISP_AFC_WINDOW_X(pconfig->afm_win[1].h_size +
			pconfig->afm_win[1].h_offs) |
			CIFISP_AFC_WINDOW_Y(pconfig->afm_win[1].v_size +
			pconfig->afm_win[1].v_offs),
			CIF_ISP_AFM_RB_B);
		num_of_win--;
	}

	if (num_of_win) {
		cifisp_iowrite32(
			CIFISP_AFC_WINDOW_X(pconfig->afm_win[2].h_offs) |
			CIFISP_AFC_WINDOW_Y(pconfig->afm_win[2].v_offs),
			CIF_ISP_AFM_LT_C);
		cifisp_iowrite32(
			CIFISP_AFC_WINDOW_X(pconfig->afm_win[2].h_size +
			pconfig->afm_win[2].h_offs) |
			CIFISP_AFC_WINDOW_Y(pconfig->afm_win[2].v_size +
			pconfig->afm_win[2].v_offs),
			CIF_ISP_AFM_RB_C);
	}

	cifisp_iowrite32(pconfig->thres, CIF_ISP_AFM_THRES);
	cifisp_iowrite32(pconfig->var_shift, CIF_ISP_AFM_VAR_SHIFT);
}

/*****************************************************************************/
static void cifisp_afc_en(struct xgold_isp_dev *isp_dev)
{
	isp_dev->active_meas |= CIF_ISP_AFM_FIN;

	cifisp_iowrite32(1, CIF_ISP_AFM_CTRL);
}

/*****************************************************************************/
static void cifisp_afc_end(struct xgold_isp_dev *isp_dev)
{
	cifisp_iowrite32(0, CIF_ISP_AFM_CTRL);
	isp_dev->active_meas &= ~CIF_ISP_AFM_FIN;
}

/*****************************************************************************/
static void cifisp_get_afc_meas(struct xgold_isp_dev *isp_dev,
				struct cifisp_stat_buffer *pbuf)
{
	pbuf->meas_type = CIFISP_STAT_AFM_FIN;

	pbuf->params.af.window[0].sum =
		cifisp_ioread32(CIF_ISP_AFM_SUM_A);
	pbuf->params.af.window[0].lum =
		cifisp_ioread32(CIF_ISP_AFM_LUM_A);
	pbuf->params.af.window[1].sum =
		cifisp_ioread32(CIF_ISP_AFM_SUM_B);
	pbuf->params.af.window[1].lum =
		cifisp_ioread32(CIF_ISP_AFM_LUM_B);
	pbuf->params.af.window[2].sum =
		cifisp_ioread32(CIF_ISP_AFM_SUM_C);
	pbuf->params.af.window[2].lum =
		cifisp_ioread32(CIF_ISP_AFM_LUM_C);
}

/* HISTOGRAM CALCULATION */
/*****************************************************************************/
static void cifisp_hst_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_hst_config *pconfig = &(isp_dev->hst_config);

	cifisp_iowrite32(CIFISP_HST_PREDIV_SET(pconfig->histogram_predivider),
		      CIF_ISP_HIST_PROP);
	cifisp_iowrite32(pconfig->meas_window.h_offs, CIF_ISP_HIST_H_OFFS);
	cifisp_iowrite32(pconfig->meas_window.h_size, CIF_ISP_HIST_H_SIZE);
	cifisp_iowrite32(pconfig->meas_window.v_offs, CIF_ISP_HIST_V_OFFS);
	cifisp_iowrite32(pconfig->meas_window.v_size, CIF_ISP_HIST_V_SIZE);
}

static void cifisp_hst_en(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_hst_config *pconfig = &(isp_dev->hst_config);

	cifisp_iowrite32OR(CIFISP_HST_MANUAL_RST | pconfig->mode,
			CIF_ISP_HIST_PROP);
}

/*****************************************************************************/
static void cifisp_hst_end(const struct xgold_isp_dev *isp_dev)
{
	/*Disable measurement */
	cifisp_iowrite32(0, CIF_ISP_HIST_PROP);
}

/*****************************************************************************/
static void cifisp_get_hst_meas(const struct xgold_isp_dev *isp_dev,
				struct cifisp_stat_buffer *pbuf)
{
	int i;

	for (i = 0; i < CIFISP_HIST_BIN_N_MAX; i++) {
		pbuf->params.awb.hist_bins[i] =
		    cifisp_ioread32(CIF_ISP_HIST_BIN_BASE + (i * 4));
	}

	cifisp_iowrite32OR(CIFISP_HST_MANUAL_RST, CIF_ISP_HIST_PROP);
}

/* IMAGE EFFECT */
/*****************************************************************************/
static void cifisp_ie_config(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_ie_config *pconfig = &(isp_dev->ie_config);

	switch (pconfig->effect) {
	case V4L2_COLORFX_SET_CBCR:
		cifisp_iowrite32(pconfig->eff_tint, CIF_IMG_EFF_TINT);
		break;
	/*Color selection is similiar to water color(AQUA):
		grayscale + selected color w threshold*/
	case V4L2_COLORFX_AQUA:
		cifisp_iowrite32(pconfig->color_sel, CIF_IMG_EFF_COLOR_SEL);
		break;
	case V4L2_COLORFX_EMBOSS:
		cifisp_iowrite32(pconfig->eff_mat_1, CIF_IMG_EFF_MAT_1);
		cifisp_iowrite32(pconfig->eff_mat_2, CIF_IMG_EFF_MAT_2);
		cifisp_iowrite32(pconfig->eff_mat_3, CIF_IMG_EFF_MAT_3);
		break;
	case V4L2_COLORFX_SKETCH:
		cifisp_iowrite32(pconfig->eff_mat_3, CIF_IMG_EFF_MAT_3);
		cifisp_iowrite32(pconfig->eff_mat_4, CIF_IMG_EFF_MAT_4);
		cifisp_iowrite32(pconfig->eff_mat_5, CIF_IMG_EFF_MAT_5);
		break;
	default:
		break;
	}
}

static void cifisp_ie_en(const struct xgold_isp_dev *isp_dev)
{
	const struct cifisp_ie_config *pconfig = &(isp_dev->ie_config);
	enum cif_isp20_image_effect effect;

	switch (pconfig->effect) {
	case V4L2_COLORFX_SEPIA:
	case V4L2_COLORFX_SET_CBCR:
		effect = CIF_ISP20_IE_SEPIA;
		break;
	case V4L2_COLORFX_BW:
		effect = CIF_ISP20_IE_BW;
		break;
	case V4L2_COLORFX_NEGATIVE:
		effect = CIF_ISP20_IE_NEGATIVE;
		break;
	case V4L2_COLORFX_EMBOSS:
		effect = CIF_ISP20_IE_EMBOSS;
		break;
	case V4L2_COLORFX_SKETCH:
		effect = CIF_ISP20_IE_SKETCH;
		break;
	case V4L2_COLORFX_AQUA:
		effect = CIF_ISP20_IE_C_SEL;
		break;
	case V4L2_COLORFX_NONE:
	default:
		effect = CIF_ISP20_IE_NONE;
		break;
	}

	if (effect < CIF_ISP20_IE_NONE) {
		cifisp_iowrite32OR(CIF_ICCL_IE_CLK, CIF_ICCL);
		cifisp_iowrite32(CIF_IMG_EFF_CTRL_ENABLE |
			effect << 1, CIF_IMG_EFF_CTRL);
		cifisp_iowrite32OR(CIF_IMG_EFF_CTRL_CFG_UPD, CIF_IMG_EFF_CTRL);
	} else if (effect == CIF_ISP20_IE_NONE) {
		cifisp_iowrite32AND(~CIF_IMG_EFF_CTRL_ENABLE, CIF_IMG_EFF_CTRL);
		cifisp_iowrite32AND(~CIF_ICCL_IE_CLK, CIF_ICCL);
	}
}

/*****************************************************************************/
static void cifisp_ie_end(const struct xgold_isp_dev *isp_dev)
{
	/*Disable measurement */
	cifisp_iowrite32AND(~CIF_IMG_EFF_CTRL_ENABLE, CIF_IMG_EFF_CTRL);
	cifisp_iowrite32AND(~CIF_ICCL_IE_CLK, CIF_ICCL);
}

/*****************************************************************************/
static void cifisp_csm_config(const struct xgold_isp_dev *isp_dev,
				bool capture)
{
	/* The color effects cause corrupted color if full value range
	 * is enabled. Therefore, disable full range with color effects.
	 */
	if (!capture || cifisp_is_ie_active(isp_dev)) {
		/* Reduced range conversion */
		cifisp_iowrite32(0x21, CIF_ISP_CC_COEFF_0);
		cifisp_iowrite32(0x40, CIF_ISP_CC_COEFF_1);
		cifisp_iowrite32(0xd, CIF_ISP_CC_COEFF_2);
		cifisp_iowrite32(0x1ed, CIF_ISP_CC_COEFF_3);
		cifisp_iowrite32(0x1db, CIF_ISP_CC_COEFF_4);
		cifisp_iowrite32(0x38, CIF_ISP_CC_COEFF_5);
		cifisp_iowrite32(0x38, CIF_ISP_CC_COEFF_6);
		cifisp_iowrite32(0x1d1, CIF_ISP_CC_COEFF_7);
		cifisp_iowrite32(0x1f7, CIF_ISP_CC_COEFF_8);
		cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_CSM_Y_FULL_ENA,
			CIF_ISP_CTRL);
		cifisp_iowrite32AND(~CIF_ISP_CTRL_ISP_CSM_C_FULL_ENA,
			CIF_ISP_CTRL);
	} else {
		cifisp_iowrite32(0x26, CIF_ISP_CC_COEFF_0);
		cifisp_iowrite32(0x4b, CIF_ISP_CC_COEFF_1);
		cifisp_iowrite32(0xf, CIF_ISP_CC_COEFF_2);
		cifisp_iowrite32(0x1ea, CIF_ISP_CC_COEFF_3);
		cifisp_iowrite32(0x1d6, CIF_ISP_CC_COEFF_4);
		cifisp_iowrite32(0x40, CIF_ISP_CC_COEFF_5);
		cifisp_iowrite32(0x40, CIF_ISP_CC_COEFF_6);
		cifisp_iowrite32(0x1ca, CIF_ISP_CC_COEFF_7);
		cifisp_iowrite32(0x1f6, CIF_ISP_CC_COEFF_8);
		cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_CSM_Y_FULL_ENA,
			CIF_ISP_CTRL);
		cifisp_iowrite32OR(CIF_ISP_CTRL_ISP_CSM_C_FULL_ENA,
			CIF_ISP_CTRL);
	}
}

/* ================================QUEUE OPS ================== */
static int cifisp_stat_vbq_setup(struct videobuf_queue *vq,
				 unsigned int *cnt, unsigned int *size)
{
	*size = sizeof(struct cifisp_stat_buffer);

	return 0;
}

static void cifisp_stat_vbq_release(struct videobuf_queue *vq,
				    struct videobuf_buffer *vb)
{
	CIFISP_DPRINT(CIFISP_DEBUG, "Releasing buffer entry!\n");

	videobuf_waiton(vq, vb, 0, 0);

	videobuf_vmalloc_free(vb);

	CIFISP_DPRINT(CIFISP_DEBUG, "Releasing buffer exit!\n");
}

static int cifisp_stat_vbq_prepare(struct videobuf_queue *vq,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	int err = 0;

	vb->size = sizeof(struct cifisp_stat_buffer);
	vb->width = 0;
	vb->height = 0;
	vb->field = field;

	if (vb->state == VIDEOBUF_NEEDS_INIT)
		err = videobuf_iolock(vq, vb, NULL);

	if (!err)
		vb->state = VIDEOBUF_PREPARED;
	else
		cifisp_stat_vbq_release(vq, vb);

	return err;
}

static void cifisp_stat_vbq_queue(struct videobuf_queue *vq,
				  struct videobuf_buffer *vb)
{
	struct xgold_isp_dev *isp_dev = vq->priv_data;

	vb->state = VIDEOBUF_QUEUED;

	CIFISP_DPRINT(CIFISP_DEBUG, "Queueing stat buffer!\n");

	list_add_tail(&vb->queue, &isp_dev->stat);
}

/* Queue Ops */
static struct videobuf_queue_ops cifisp_stat_qops = {
	.buf_setup = cifisp_stat_vbq_setup,
	.buf_prepare = cifisp_stat_vbq_prepare,
	.buf_queue = cifisp_stat_vbq_queue,
	.buf_release = cifisp_stat_vbq_release,
};

/*================== IOCTL implementation ========================= */
static int cifisp_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	CIFISP_DPRINT(CIFISP_DEBUG,
		      " %s: %s: p->type %d p->count %d\n",
		      ISP_DEV_NAME, __func__, p->type, p->count);

	return videobuf_reqbufs(&isp_dev->vbq_stat, p);
}

static int cifisp_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	CIFISP_DPRINT(CIFISP_DEBUG,
		      " %s: %s: p->type %d p->index %d\n",
		      ISP_DEV_NAME, __func__, p->type, p->index);

	return videobuf_querybuf(&isp_dev->vbq_stat, p);
}

static int cifisp_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	CIFISP_DPRINT(CIFISP_DEBUG,
		      " %s: %s: p->type %d p->index %d\n",
		      ISP_DEV_NAME, __func__, p->type, p->index);

	return videobuf_qbuf(&isp_dev->vbq_stat, p);
}

/* ========================================================== */

static int cifisp_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	CIFISP_DPRINT(CIFISP_DEBUG,
		      " %s: %s: p->type %d p->index %d\n",
		      ISP_DEV_NAME, __func__, p->type, p->index);

	return videobuf_dqbuf(&isp_dev->vbq_stat, p,
			      file->f_flags & O_NONBLOCK);
}

static int cifisp_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	int ret = videobuf_streamon(&isp_dev->vbq_stat);

	if (ret == 0)
		isp_dev->streamon = true;

	CIFISP_DPRINT(CIFISP_DEBUG,
		      " %s: %s: ret %d\n", ISP_DEV_NAME, __func__, ret);

	return ret;
}

/* ========================================================== */
static int cifisp_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));
	int ret;

#ifdef STATS_WITH_WQ
	drain_workqueue(measurement_wq);
#endif
	ret = videobuf_streamoff(&isp_dev->vbq_stat);

	if (ret == 0)
		isp_dev->streamon = false;

	CIFISP_DPRINT(CIFISP_DEBUG,
		" %s: %s: ret %d\n", ISP_DEV_NAME, __func__, ret);

	return ret;
}

static int cifisp_g_ctrl(struct file *file, void *priv, struct v4l2_control *vc)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	switch (vc->id) {
	case V4L2_CID_CIFISP_BPC:
		return cifisp_bpc_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_BLS:
		return cifisp_bls_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_SDG:
		return cifisp_sdg_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_LSC:
		return cifisp_lsc_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_AWB_MEAS:
		return cifisp_awb_meas_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_AWB_GAIN:
		return cifisp_awb_gain_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_FLT:
		return cifisp_flt_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_BDM:
		return cifisp_bdm_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_CTK:
		return cifisp_ctk_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_GOC:
		return cifisp_goc_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_HST:
		return cifisp_hst_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_AEC:
		return cifisp_aec_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_CPROC:
		return cifisp_cproc_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_MACC:
		return cifisp_macc_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_TMAP:
		return cifisp_tmap_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_YCFLT:
		return cifisp_ycflt_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_AFC:
		return cifisp_afc_enable(isp_dev, _GET_, &vc->value);
	case V4L2_CID_CIFISP_IE:
		return cifisp_ie_enable(isp_dev, _GET_, &vc->value);
	default:
		return -EINVAL;
	}
}

static int cifisp_s_ctrl(struct file *file, void *priv, struct v4l2_control *vc)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	switch (vc->id) {
	case V4L2_CID_CIFISP_BPC:
		return cifisp_bpc_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_BLS:
		return cifisp_bls_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_SDG:
		return cifisp_sdg_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_LSC:
		return cifisp_lsc_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_AWB_MEAS:
		return cifisp_awb_meas_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_AWB_GAIN:
		return cifisp_awb_gain_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_FLT:
		return cifisp_flt_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_BDM:
		return cifisp_bdm_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_CTK:
		return cifisp_ctk_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_GOC:
		return cifisp_goc_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_HST:
		return cifisp_hst_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_AEC:
		return cifisp_aec_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_CPROC:
		return cifisp_cproc_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_MACC:
		return cifisp_macc_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_TMAP:
		return cifisp_tmap_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_YCFLT:
		return cifisp_ycflt_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_AFC:
		return cifisp_afc_enable(isp_dev, _SET_, &vc->value);
	case V4L2_CID_CIFISP_IE:
		return cifisp_ie_enable(isp_dev, _SET_, &vc->value);
	default:
		return -EINVAL;
	}
}

static long cifisp_ioctl_default(struct file *file,
	 void *fh, bool valid_prio, unsigned int cmd, void *arg)
{
	struct xgold_isp_dev *isp = video_get_drvdata(video_devdata(file));

	switch (cmd) {
	case CIFISP_IOC_G_BPC:
		return cifisp_bpc_param(isp, _GET_, arg);
	case CIFISP_IOC_S_BPC:
		return cifisp_bpc_param(isp, _SET_, arg);
	case CIFISP_IOC_G_BLS:
		return cifisp_bls_param(isp, _GET_, arg);
	case CIFISP_IOC_S_BLS:
		return cifisp_bls_param(isp, _SET_, arg);
	case CIFISP_IOC_G_SDG:
		return cifisp_sdg_param(isp, _GET_, arg);
	case CIFISP_IOC_S_SDG:
		return cifisp_sdg_param(isp, _SET_, arg);
	case CIFISP_IOC_G_LSC:
		return cifisp_lsc_param(isp, _GET_, arg);
	case CIFISP_IOC_S_LSC:
		return cifisp_lsc_param(isp, _SET_, arg);
	case CIFISP_IOC_G_AWB_MEAS:
		return cifisp_awb_meas_param(isp, _GET_, arg);
	case CIFISP_IOC_S_AWB_MEAS:
		return cifisp_awb_meas_param(isp, _SET_, arg);
	case CIFISP_IOC_G_AWB_GAIN:
		return cifisp_awb_gain_param(isp, _GET_, arg);
	case CIFISP_IOC_S_AWB_GAIN:
		return cifisp_awb_gain_param(isp, _SET_, arg);
	case CIFISP_IOC_G_FLT:
		return cifisp_flt_param(isp, _GET_, arg);
	case CIFISP_IOC_S_FLT:
		return cifisp_flt_param(isp, _SET_, arg);
	case CIFISP_IOC_G_BDM:
		return cifisp_bdm_param(isp, _GET_, arg);
	case CIFISP_IOC_S_BDM:
		return cifisp_bdm_param(isp, _SET_, arg);
	case CIFISP_IOC_G_CTK:
		return cifisp_ctk_param(isp, _GET_, arg);
	case CIFISP_IOC_S_CTK:
		return cifisp_ctk_param(isp, _SET_, arg);
	case CIFISP_IOC_G_GOC:
		return cifisp_goc_param(isp, _GET_, arg);
	case CIFISP_IOC_S_GOC:
		return cifisp_goc_param(isp, _SET_, arg);
	case CIFISP_IOC_G_HST:
		return cifisp_hst_param(isp, _GET_, arg);
	case CIFISP_IOC_S_HST:
		return cifisp_hst_param(isp, _SET_, arg);
	case CIFISP_IOC_G_AEC:
		return cifisp_aec_param(isp, _GET_, arg);
	case CIFISP_IOC_S_AEC:
		return cifisp_aec_param(isp, _SET_, arg);
	case CIFISP_IOC_G_CPROC:
		return cifisp_cproc_param(isp, _GET_, arg);
	case CIFISP_IOC_S_CPROC:
		return cifisp_cproc_param(isp, _SET_, arg);
	case CIFISP_IOC_G_MACC:
		return cifisp_macc_param(isp, _GET_, arg);
	case CIFISP_IOC_S_MACC:
		return cifisp_macc_param(isp, _SET_, arg);
	case CIFISP_IOC_G_TMAP:
		return cifisp_tmap_param(isp, _GET_, arg);
	case CIFISP_IOC_S_TMAP:
		return cifisp_tmap_param(isp, _SET_, arg);
	case CIFISP_IOC_G_YCFLT:
		return cifisp_ycflt_param(isp, _GET_, arg);
	case CIFISP_IOC_S_YCFLT:
		return cifisp_ycflt_param(isp, _SET_, arg);
	case CIFISP_IOC_G_AFC:
		return cifisp_afc_param(isp, _GET_, arg);
	case CIFISP_IOC_S_AFC:
		return cifisp_afc_param(isp, _SET_, arg);
	case CIFISP_IOC_G_IE:
		return cifisp_ie_param(isp, _GET_, arg);
	case CIFISP_IOC_S_IE:
		return cifisp_ie_param(isp, _SET_, arg);
	case CIFISP_IOC_G_LAST_CONFIG:
		return cifisp_last_capture_config(arg);
	default:
		return -EINVAL;
	}
}

static int cifisp_g_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	/* Dummy function needed to allow allocation of
	   buffers on this device */
	return 0;
}

/* ISP video device IOCTLs*/
static const struct v4l2_ioctl_ops cifisp_ioctl = {
	.vidioc_reqbufs = cifisp_reqbufs,
	.vidioc_querybuf = cifisp_querybuf,
	.vidioc_qbuf = cifisp_qbuf,
	.vidioc_dqbuf = cifisp_dqbuf,
	.vidioc_streamon = cifisp_streamon,
	.vidioc_streamoff = cifisp_streamoff,
	.vidioc_g_ctrl = cifisp_g_ctrl,
	.vidioc_s_ctrl = cifisp_s_ctrl,
	.vidioc_default = cifisp_ioctl_default,
	.vidioc_g_fmt_vid_cap = cifisp_g_fmt_vid_cap,
};

/* ======================================================== */

static unsigned int cifisp_poll(struct file *file,
				struct poll_table_struct *wait)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));
	unsigned int ret;

	ret = videobuf_poll_stream(file, &isp_dev->vbq_stat, wait);

	CIFISP_DPRINT(CIFISP_DEBUG,
		      "Polling on vbq_stat buffer %d\n", ret);

	return ret;
}

/* ======================================================== */
static int cifisp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct xgold_isp_dev *isp_dev = video_get_drvdata(video_devdata(file));

	return videobuf_mmap_mapper(&isp_dev->vbq_stat, vma);
}

static int cifisp_open(struct file *file)
{
	struct xgold_isp_dev *isp_dev =
		video_get_drvdata(video_devdata(file));

	CIFISP_DPRINT(CIFISP_DEBUG, "cifisp_open\n");

	if (isp_dev->open_count)
		return -EBUSY;
	isp_dev->open_count++;

	isp_dev->bpc_en = false;
	isp_dev->bls_en = false;
	isp_dev->sdg_en = false;
	isp_dev->lsc_en = false;
	isp_dev->awb_meas_en = false;
	isp_dev->awb_gain_en = false;
	isp_dev->flt_en = false;
	isp_dev->bdm_en = false;
	isp_dev->ctk_en = false;
	isp_dev->goc_en = false;
	isp_dev->hst_en = false;
	isp_dev->aec_en = false;
	isp_dev->cproc_en = false;
	isp_dev->macc_en = false;
	isp_dev->tmap_en = false;
	isp_dev->ycflt_en = false;
	isp_dev->afc_en = false;
	isp_dev->ie_en = false;

	memset(&isp_dev->bpc_config, 0, sizeof(isp_dev->bpc_config));
	memset(&isp_dev->bls_config, 0, sizeof(isp_dev->bls_config));
	memset(&isp_dev->sdg_config, 0, sizeof(isp_dev->sdg_config));
	memset(&isp_dev->lsc_config, 0, sizeof(isp_dev->lsc_config));
	memset(&isp_dev->awb_meas_config, 0, sizeof(isp_dev->awb_meas_config));
	memset(&isp_dev->awb_gain_config, 0, sizeof(isp_dev->awb_gain_config));
	memset(&isp_dev->flt_config, 0, sizeof(isp_dev->flt_config));
	memset(&isp_dev->bdm_config, 0, sizeof(isp_dev->bdm_config));
	memset(&isp_dev->ctk_config, 0, sizeof(isp_dev->ctk_config));
	memset(&isp_dev->goc_config, 0, sizeof(isp_dev->goc_config));
	memset(&isp_dev->hst_config, 0, sizeof(isp_dev->hst_config));
	memset(&isp_dev->aec_config, 0, sizeof(isp_dev->aec_config));
	memset(&isp_dev->cproc_config, 0, sizeof(isp_dev->cproc_config));
	memset(&isp_dev->macc_config, 0, sizeof(isp_dev->macc_config));
	memset(&isp_dev->tmap_config, 0, sizeof(isp_dev->tmap_config));
	memset(&isp_dev->ycflt_config, 0, sizeof(isp_dev->ycflt_config));
	memset(&isp_dev->afc_config, 0, sizeof(isp_dev->afc_config));
	memset(&isp_dev->ie_config, 0, sizeof(isp_dev->ie_config));

	isp_dev->isp_param_bpc_update_needed = false;
	isp_dev->isp_param_bls_update_needed = false;
	isp_dev->isp_param_sdg_update_needed = false;
	isp_dev->isp_param_lsc_update_needed = false;
	isp_dev->isp_param_awb_meas_update_needed = false;
	isp_dev->isp_param_awb_gain_update_needed = false;
	isp_dev->isp_param_flt_update_needed = false;
	isp_dev->isp_param_bdm_update_needed = false;
	isp_dev->isp_param_ctk_update_needed = false;
	isp_dev->isp_param_goc_update_needed = false;
	isp_dev->isp_param_hst_update_needed = false;
	isp_dev->isp_param_aec_update_needed = false;
	isp_dev->isp_param_cproc_update_needed = false;
	isp_dev->isp_param_macc_update_needed = false;
	isp_dev->isp_param_tmap_update_needed = false;
	isp_dev->isp_param_afc_update_needed = false;
	isp_dev->isp_param_ie_update_needed = false;

	isp_dev->active_lsc_width = 0;
	isp_dev->active_lsc_height = 0;

	isp_dev->streamon = false;
	isp_dev->active_meas = 0;

	isp_dev->ycflt_update = false;
	isp_dev->cif_ism_cropping = false;
	return 0;
}

static int cifisp_close(struct file *file)
{
	struct xgold_isp_dev *isp_dev =
		video_get_drvdata(video_devdata(file));

	CIFISP_DPRINT(CIFISP_DEBUG, "cifisp_close\n");

	videobuf_stop(&isp_dev->vbq_stat);
	videobuf_mmap_free(&isp_dev->vbq_stat);
	isp_dev->open_count--;

	return 0;
}

struct v4l2_file_operations cifisp_fops = {
	.mmap = cifisp_mmap,
	.unlocked_ioctl = video_ioctl2,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = video_ioctl2,
#endif
	.poll = cifisp_poll,
	.open = cifisp_open,
	.release = cifisp_close
};

static void cifisp_release(struct video_device *vdev)
{
	CIFISP_DPRINT(CIFISP_DEBUG, "cifisp_release\n");

	video_device_release(vdev);
#ifdef STATS_WITH_WQ
	destroy_workqueue(measurement_wq);
#endif
}

/************************************************************/
int register_cifisp_device(struct xgold_isp_dev *isp_dev,
	struct video_device *vdev_cifisp,
	struct v4l2_device *v4l2_dev,
	void __iomem *cif_reg_baseaddress)
{
	memset(isp_dev, 0, sizeof(struct xgold_isp_dev));

	isp_dev->base_addr = cif_reg_baseaddress;
	BUG_ON(!(isp_dev->base_addr));

	INIT_LIST_HEAD(&isp_dev->stat);
	spin_lock_init(&isp_dev->irq_lock);
	spin_lock_init(&isp_dev->config_lock);
	strlcpy(vdev_cifisp->name, ISP_DEV_NAME, sizeof(vdev_cifisp->name));
	vdev_cifisp->vfl_type = V4L2_CAP_VIDEO_CAPTURE;
	video_set_drvdata(vdev_cifisp, isp_dev);
	vdev_cifisp->ioctl_ops = &cifisp_ioctl;
	vdev_cifisp->fops = &cifisp_fops;
	vdev_cifisp->debug = V4L2_DEV_DEBUG_LEVEL;
	/* This might not release all resources,
	   but unregistering is anyway not going to happen. */
	vdev_cifisp->release = cifisp_release;
	mutex_init(&isp_dev->mutex);
	/* Provide a mutex to v4l2 core. It will be used
	   to protect all fops and v4l2 ioctls. */
	vdev_cifisp->lock = &isp_dev->mutex;
	vdev_cifisp->v4l2_dev = v4l2_dev;

	videobuf_queue_vmalloc_init(
		&isp_dev->vbq_stat,
		&cifisp_stat_qops,
		NULL,
		&isp_dev->irq_lock,
		V4L2_BUF_TYPE_VIDEO_CAPTURE,
		V4L2_FIELD_NONE,
		sizeof(struct videobuf_buffer),
		isp_dev,
		NULL);	/* ext_lock: NULL */

	if (video_register_device(vdev_cifisp, VFL_TYPE_GRABBER, -1) < 0) {
		dev_err(&(vdev_cifisp->dev),
			"could not register Video for Linux device\n");
		return -ENODEV;
	} else {
		CIFISP_DPRINT(CIFISP_DEBUG,
			"%s: CIFISP vdev minor =  %d\n",
			__func__, vdev_cifisp->minor);
	}

#ifdef STATS_WITH_WQ
	measurement_wq =
		alloc_workqueue("measurement_queue",
			WQ_HIGHPRI | WQ_MEM_RECLAIM, 1);

	if (!measurement_wq)
		return -ENOMEM;
#endif

	isp_dev->v_blanking_us = CIFISP_MODULE_DEFAULT_VBLANKING_TIME;

	return 0;
}

void unregister_cifisp_device(struct video_device *vdev_cifisp)
{
	if (!IS_ERR_OR_NULL(vdev_cifisp))
		video_unregister_device(vdev_cifisp);
}

static void cifisp_dump_reg(struct xgold_isp_dev *isp_dev, int level)
{
#ifdef CIFISP_DEBUG_REG
	if (isp_dev->bpc_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_BPC, level);

	if (isp_dev->lsc_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_LSC, level);

	if (isp_dev->bls_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_BLS, level);

	if (isp_dev->sdg_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_SDG, level);

	if (isp_dev->goc_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_GOC, level);

	if (isp_dev->bdm_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_BDM, level);

	if (isp_dev->flt_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_FLT, level);

	if (isp_dev->awb_meas_en || isp_dev->awb_gain_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_AWB, level);

	if (isp_dev->aec_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_AEC, level);

	if (isp_dev->ctk_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_CTK, level);

	if (isp_dev->cproc_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_CPROC, level);

	if (isp_dev->macc_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_MACC, level);

	if (isp_dev->tmap_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_TMAP, level);

	if (isp_dev->ycflt_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_YCFLT, level);

	if (isp_dev->afc_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_AFC, level);

	if (isp_dev->hst_en)
		cifisp_reg_dump(isp_dev, CIFISP_MODULE_HST, level);
#endif
}

/* Not called when the camera active, thus not isr protection. */
void cifisp_configure_isp(
		struct xgold_isp_dev *isp_dev,
		enum cif_isp20_pix_fmt in_pix_fmt,
		bool capture)
{
	CIFISP_DPRINT(CIFISP_DEBUG, "%s format 0x%x capture %d\n",
		__func__, in_pix_fmt, capture);

	mutex_lock(&isp_dev->mutex);

	if (CIF_ISP20_PIX_FMT_IS_RAW_BAYER(in_pix_fmt)) {
#ifndef CIFISP_DEBUG_DISABLE_BLOCKS
		if (isp_dev->bpc_en) {
			cifisp_bp_config(isp_dev);
			cifisp_bp_en(isp_dev);
			isp_dev->isp_param_bpc_update_needed = false;
		}

		if (isp_dev->lsc_en) {
			if (cifisp_lsc_config(isp_dev))
				isp_dev->isp_param_lsc_update_needed = false;
		}

		if (isp_dev->bls_en) {
			cifisp_bls_config(isp_dev);
			cifisp_bls_en(isp_dev);
			isp_dev->isp_param_bls_update_needed = false;
		}

		if (isp_dev->sdg_en) {
			cifisp_sdg_config(isp_dev);
			cifisp_sdg_en(isp_dev);
			isp_dev->isp_param_sdg_update_needed = false;
		}

		if (isp_dev->goc_en) {
			cifisp_goc_config(isp_dev);
			cifisp_goc_en(isp_dev);
			isp_dev->isp_param_goc_update_needed = false;
		}

		if (isp_dev->bdm_en) {
			cifisp_bdm_config(isp_dev);
			cifisp_bdm_en(isp_dev);
			isp_dev->isp_param_bdm_update_needed = false;
		}

		if (isp_dev->flt_en) {
			cifisp_flt_config(isp_dev);
			cifisp_flt_en(isp_dev);
			isp_dev->isp_param_flt_update_needed = false;
		}

		if (isp_dev->awb_gain_en) {
			cifisp_awb_gain_config(isp_dev);
			cifisp_awb_gain_en(isp_dev);
			isp_dev->isp_param_awb_gain_update_needed = false;
		}

		if (isp_dev->ctk_en) {
			cifisp_ctk_config(isp_dev);
			cifisp_ctk_en(isp_dev);
			isp_dev->isp_param_ctk_update_needed = false;
		}

		if (isp_dev->cproc_en) {
			cifisp_cproc_config(isp_dev, capture);
			cifisp_cproc_en(isp_dev);
			isp_dev->isp_param_cproc_update_needed = false;
		}

		isp_dev->ycflt_update = true;

		if (isp_dev->macc_en) {
			cifisp_macc_config(isp_dev);
			cifisp_macc_en(isp_dev);
			isp_dev->isp_param_macc_update_needed = false;
		}

		if (isp_dev->tmap_en) {
			cifisp_tmap_config(isp_dev);
			cifisp_tmap_en(isp_dev);
			isp_dev->isp_param_tmap_update_needed = false;
		}

		if (isp_dev->ie_en) {
			cifisp_ie_config(isp_dev);
			cifisp_ie_en(isp_dev);
			isp_dev->isp_param_ie_update_needed = false;
		}

		cifisp_csm_config(isp_dev, capture);
#endif

		if (isp_dev->afc_en) {
			cifisp_afc_config(isp_dev);
			cifisp_afc_en(isp_dev);
			isp_dev->isp_param_afc_update_needed = false;
		}

		if (isp_dev->awb_meas_en) {
			cifisp_awb_meas_config(isp_dev);
			cifisp_awb_meas_en(isp_dev);
			isp_dev->isp_param_awb_meas_update_needed = false;
		}

		if (isp_dev->aec_en) {
			cifisp_aec_config(isp_dev);
			cifisp_aec_en(isp_dev);
			isp_dev->isp_param_aec_update_needed = false;
		}

		if (isp_dev->hst_en) {
			cifisp_hst_config(isp_dev);
			cifisp_hst_en(isp_dev);
			isp_dev->isp_param_hst_update_needed = false;
		}
	} else {
		/* Disable modules for yuv */
		cifisp_bp_end(isp_dev);
		isp_dev->bpc_en = false;

		cifisp_lsc_end(isp_dev);
		isp_dev->lsc_en = false;

		cifisp_bls_end(isp_dev);
		isp_dev->bls_en = false;

		cifisp_sdg_end(isp_dev);
		isp_dev->sdg_en = false;

		cifisp_goc_end(isp_dev);
		isp_dev->goc_en = false;

		cifisp_bdm_end(isp_dev);
		isp_dev->bdm_en = false;

		cifisp_flt_end(isp_dev);
		isp_dev->flt_en = false;

		cifisp_awb_meas_end(isp_dev);
		isp_dev->awb_meas_en = false;

		cifisp_awb_gain_end(isp_dev);
		isp_dev->awb_gain_en = false;

		cifisp_aec_end(isp_dev);
		isp_dev->aec_en = false;

		cifisp_ctk_end(isp_dev);
		isp_dev->ctk_en = false;

		/* cproc can be used for yuv in reduced range */
		if (isp_dev->cproc_en) {
			cifisp_cproc_config(isp_dev, false);
			cifisp_cproc_en(isp_dev);
			isp_dev->isp_param_cproc_update_needed = false;
		}

		cifisp_ycflt_end(isp_dev);
		isp_dev->ycflt_en = false;
		isp_dev->ycflt_update = false;

		cifisp_macc_end(isp_dev);
		isp_dev->macc_en = false;

		cifisp_tmap_end(isp_dev);
		isp_dev->tmap_en = false;

		cifisp_hst_end(isp_dev);
		isp_dev->hst_en = false;

		cifisp_afc_end(isp_dev);
		isp_dev->afc_en = false;

		/* ie can be used for yuv */
		if (isp_dev->ie_en) {
			cifisp_ie_config(isp_dev);
			cifisp_ie_en(isp_dev);
			isp_dev->isp_param_ie_update_needed = false;
		} else {
			cifisp_ie_end(isp_dev);
		}
	}

	cifisp_dump_reg(isp_dev, CIFISP_DEBUG);

	mutex_unlock(&isp_dev->mutex);

}

void cifisp_v_start(struct xgold_isp_dev *isp_dev,
	const struct timeval *timestamp)
{
	/* Called in an interrupt context. */
	isp_dev->frame_id += 2;
	isp_dev->frame_start_tv = *timestamp;
}

bool cifisp_is_ie_active(const struct xgold_isp_dev *isp_dev)
{
	if (isp_dev->ie_en &&
		isp_dev->ie_config.effect != V4L2_COLORFX_NONE)
		return true;
	else
		return false;
}


/* Not called when the camera active, thus not isr protection. */
void cifisp_disable_isp(struct xgold_isp_dev *isp_dev)
{
	CIFISP_DPRINT(CIFISP_DEBUG, "%s\n", __func__);

	mutex_lock(&isp_dev->mutex);

	cifisp_bp_end(isp_dev);
	isp_dev->bpc_en = false;

	cifisp_lsc_end(isp_dev);
	isp_dev->lsc_en = false;

	cifisp_bls_end(isp_dev);
	isp_dev->bls_en = false;

	cifisp_sdg_end(isp_dev);
	isp_dev->sdg_en = false;

	cifisp_goc_end(isp_dev);
	isp_dev->goc_en = false;

	cifisp_bdm_end(isp_dev);
	isp_dev->bdm_en = false;

	cifisp_flt_end(isp_dev);
	isp_dev->flt_en = false;

	cifisp_awb_meas_end(isp_dev);
	isp_dev->awb_meas_en = false;

	cifisp_awb_gain_end(isp_dev);
	isp_dev->awb_gain_en = false;

	cifisp_aec_end(isp_dev);
	isp_dev->aec_en = false;

	cifisp_ctk_end(isp_dev);
	isp_dev->ctk_en = false;

	cifisp_cproc_end(isp_dev);
	isp_dev->cproc_en = false;

	cifisp_macc_end(isp_dev);
	isp_dev->macc_en = false;

	cifisp_tmap_end(isp_dev);
	isp_dev->tmap_en = false;

	cifisp_ycflt_end(isp_dev);
	isp_dev->ycflt_en = false;
	isp_dev->ycflt_update = false;

	cifisp_hst_end(isp_dev);
	isp_dev->hst_en = false;

	cifisp_afc_end(isp_dev);
	isp_dev->afc_en = false;

	cifisp_ie_end(isp_dev);
	isp_dev->ie_en = false;

	mutex_unlock(&isp_dev->mutex);
}

static void cif_isp_send_measurement(struct work_struct *work)
{
	struct meas_readout_work *meas_work =
		(struct meas_readout_work *)work;
	struct xgold_isp_dev *isp_dev = meas_work->isp_dev;

	if (isp_dev->streamon) {
		int bufs_needed = 0, index = 0, cleanup = 0;
		unsigned long lock_flags = 0;
		struct videobuf_buffer *vb_array[3] = {NULL, NULL, NULL};
		struct videobuf_buffer *vb, *n;
		unsigned int active_meas;

		active_meas = isp_dev->active_meas;

		if (active_meas & CIF_ISP_AWB_DONE)
			bufs_needed++;
		if (active_meas & CIF_ISP_AFM_FIN)
			bufs_needed++;
		if (active_meas & CIF_ISP_EXP_END)
			bufs_needed++;

		spin_lock_irqsave(&isp_dev->irq_lock, lock_flags);

		list_for_each_entry_safe(vb, n, &isp_dev->stat, queue) {
			list_del(&vb->queue);
			vb->state = VIDEOBUF_ACTIVE;

			vb_array[index] = vb;
			bufs_needed--;

			if (bufs_needed == 0)
				break;

			index++;
		}

		spin_unlock_irqrestore(&isp_dev->irq_lock, lock_flags);

		if (bufs_needed == 0) {
			if ((index >= 0) &&
				(active_meas & CIF_ISP_AWB_DONE)) {
				vb = vb_array[index];
				cifisp_get_awb_meas(isp_dev,
					videobuf_to_vmalloc(vb));
				if (isp_dev->hst_en)
					cifisp_get_hst_meas(isp_dev,
						videobuf_to_vmalloc(vb));
				index--;
			}
			if ((index >= 0) &&
				(active_meas & CIF_ISP_AFM_FIN)) {
				vb = vb_array[index];
				cifisp_get_afc_meas(isp_dev,
					videobuf_to_vmalloc(vb));
				index--;
			}
			if ((index >= 0) &&
				(active_meas & CIF_ISP_EXP_END)) {
				vb = vb_array[index];
				cifisp_get_aec_meas(isp_dev,
					videobuf_to_vmalloc(vb));
				cifisp_bls_get_meas(isp_dev,
					videobuf_to_vmalloc(vb));
			}

#ifdef STATS_WITH_WQ
			if (isp_dev->frame_id == meas_work->frame_id) {
#else
			/*
			V_START was cleared in the corresponding ISR.
			If it is now up, it means a new frame has started
			while in this ISR. Do not deliver stats.
			*/
			if (!(cifisp_ioread32(CIF_ISP_RIS) & CIF_ISP_V_START)) {
#endif
				int i;
				for (i = 0; i < 3; i++) {
					if (vb_array[i] != NULL) {
						vb_array[i]->ts =
						isp_dev->frame_start_tv;
						vb_array[i]->field_count =
							meas_work->frame_id;
						vb_array[i]->state =
							VIDEOBUF_DONE;
						wake_up(&vb_array[i]->done);
					}
				}

				CIFISP_DPRINT(CIFISP_DEBUG,
					"Measurement done\n");
			} else {
				cleanup = 1;
				CIFISP_DPRINT(CIFISP_DEBUG,
					"Measurement late\n");
			}
		} else {
			cleanup = 1;
			CIFISP_DPRINT(CIFISP_DEBUG,
				"Not enough measurement bufs\n");
		}

		if (cleanup) {
			int i;
			spin_lock_irqsave(&isp_dev->irq_lock, lock_flags);
			for (i = 0; i < 3; i++) {
				if (vb_array[i] != NULL) {
					vb_array[i]->state = VIDEOBUF_QUEUED;
					list_add_tail(&vb_array[i]->queue,
					&isp_dev->stat);
				}
			}
			spin_unlock_irqrestore(&isp_dev->irq_lock, lock_flags);
		}
	}

#ifdef STATS_WITH_WQ
	kfree((void *)work);
#endif
}

int cifisp_isp_isr(struct xgold_isp_dev *isp_dev, u32 isp_mis)
{
#ifdef LOG_ISR_EXE_TIME
	ktime_t in_t = ktime_get();
#endif

	if (isp_mis & (CIF_ISP_DATA_LOSS | CIF_ISP_PIC_SIZE_ERROR))
		return 0;

	if (isp_mis & CIF_ISP_FRAME) {
		u32 isp_ris = cifisp_ioread32(CIF_ISP_RIS);
		int time_left = (int)isp_dev->v_blanking_us;
		unsigned long lock_flags = 0;

		cifisp_iowrite32(
			CIF_ISP_AWB_DONE|CIF_ISP_AFM_FIN|CIF_ISP_EXP_END,
			CIF_ISP_ICR);

		CIFISP_DPRINT(CIFISP_DEBUG, "isp_ris 0x%x\n", isp_ris);

		spin_lock_irqsave(&isp_dev->config_lock, lock_flags);

		if (!isp_dev->isp_param_awb_meas_update_needed &&
			!isp_dev->isp_param_afc_update_needed &&
			!isp_dev->isp_param_aec_update_needed &&
			isp_dev->active_meas &&
			((isp_dev->active_meas & isp_ris) ==
			isp_dev->active_meas)) {
			/* First handle measurements */
#ifdef STATS_WITH_WQ
			struct meas_readout_work *work;
			work = (struct meas_readout_work *)
			kmalloc(sizeof(struct meas_readout_work), GFP_ATOMIC);

			if (work) {
				INIT_WORK((struct work_struct *)work,
					cif_isp_send_measurement);
				work->isp_dev = isp_dev;
				work->frame_id = isp_dev->frame_id;

				if (!queue_work(measurement_wq,
					(struct work_struct *)work)) {
					CIFISP_DPRINT(CIFISP_ERROR,
					"Could not schedule work\n");
					kfree((void *)work);
				}
			} else {
				CIFISP_DPRINT(CIFISP_ERROR,
					"Could not allocate work\n");
			}
#else
			struct meas_readout_work work;
			work.isp_dev = isp_dev;
			work.frame_id = isp_dev->frame_id;

			cif_isp_send_measurement((struct work_struct *)&work);
#endif
		}

		CIFISP_DPRINT(CIFISP_DEBUG,
			"time-left 0:%d\n", time_left);

		/* Then update  changed configs. Some of them involve
		   lot of register writes. Do those only one per frame.

		   Do the updates in the order of the processing flow.*/
#ifndef CIFISP_DEBUG_DISABLE_BLOCKS
		if (isp_dev->isp_param_bpc_update_needed ||
			isp_dev->isp_param_bls_update_needed ||
			isp_dev->isp_param_sdg_update_needed ||
			isp_dev->isp_param_lsc_update_needed ||
			isp_dev->isp_param_awb_gain_update_needed ||
			isp_dev->isp_param_bdm_update_needed ||
			isp_dev->isp_param_flt_update_needed ||
			isp_dev->isp_param_ctk_update_needed ||
			isp_dev->isp_param_goc_update_needed ||
			isp_dev->isp_param_cproc_update_needed ||
			isp_dev->isp_param_macc_update_needed ||
			isp_dev->isp_param_tmap_update_needed ||
			isp_dev->isp_param_ie_update_needed) {

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 1:%d %d\n",
				time_left,
				isp_dev->isp_param_bpc_update_needed);

			if (isp_dev->isp_param_bpc_update_needed) {
				/*update bpc config */
				cifisp_bp_config(isp_dev);

				if (isp_dev->bpc_en)
					cifisp_bp_en(isp_dev);
				else
					cifisp_bp_end(isp_dev);

				isp_dev->isp_param_bpc_update_needed = false;

				time_left -= CIFISP_MODULE_BPC_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 2:%d %d\n",
				time_left,
				isp_dev->isp_param_bls_update_needed);

			if (isp_dev->isp_param_bls_update_needed &&
				time_left >= CIFISP_MODULE_BLS_PROC_TIME) {
				/*update bls config */
				cifisp_bls_config(isp_dev);

				if (isp_dev->bls_en)
					cifisp_bls_en(isp_dev);
				else
					cifisp_bls_end(isp_dev);

				isp_dev->isp_param_bls_update_needed = false;

				time_left -= CIFISP_MODULE_BLS_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 3:%d %d\n",
				time_left,
				isp_dev->isp_param_sdg_update_needed);

			if (isp_dev->isp_param_sdg_update_needed &&
				time_left >= CIFISP_MODULE_SDG_PROC_TIME) {
				/*update sdg config */
				cifisp_sdg_config(isp_dev);

				if (isp_dev->sdg_en)
					cifisp_sdg_en(isp_dev);
				else
					cifisp_sdg_end(isp_dev);

				isp_dev->isp_param_sdg_update_needed = false;

				time_left -= CIFISP_MODULE_SDG_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 4:%d %d\n",
				time_left,
				isp_dev->isp_param_lsc_update_needed);

			if (isp_dev->isp_param_lsc_update_needed &&
				time_left >= CIFISP_MODULE_LSC_PROC_TIME) {
				/*update lsc config */
				bool res = true;
				if (isp_dev->lsc_en) {
					if (!cifisp_lsc_config(isp_dev))
						res = false;
				} else
					cifisp_lsc_end(isp_dev);

				if (res)
					isp_dev->isp_param_lsc_update_needed =
						false;

				time_left -= CIFISP_MODULE_LSC_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 5:%d %d\n",
				time_left,
				isp_dev->isp_param_awb_gain_update_needed);

			if (isp_dev->isp_param_awb_gain_update_needed &&
				time_left >= CIFISP_MODULE_AWB_GAIN_PROC_TIME) {
				/*update awb gains */
				cifisp_awb_gain_config(isp_dev);

				if (isp_dev->awb_gain_en)
					cifisp_awb_gain_en(isp_dev);
				else
					cifisp_awb_gain_end(isp_dev);

				isp_dev->isp_param_awb_gain_update_needed =
					false;

				time_left -= CIFISP_MODULE_AWB_GAIN_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 6:%d %d\n",
				time_left,
				isp_dev->isp_param_bdm_update_needed);

			if (isp_dev->isp_param_bdm_update_needed &&
				time_left >= CIFISP_MODULE_BDM_PROC_TIME) {
				/*update bdm config */
				cifisp_bdm_config(isp_dev);

				if (isp_dev->bdm_en)
					cifisp_bdm_en(isp_dev);
				else
					cifisp_bdm_end(isp_dev);

				isp_dev->isp_param_bdm_update_needed = false;

				time_left -= CIFISP_MODULE_BDM_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 7:%d %d\n",
				time_left,
				isp_dev->isp_param_flt_update_needed);

			if (isp_dev->isp_param_flt_update_needed &&
				time_left >= CIFISP_MODULE_FLT_PROC_TIME) {
				/*update filter config */
				cifisp_flt_config(isp_dev);

				if (isp_dev->flt_en)
					cifisp_flt_en(isp_dev);
				else
					cifisp_flt_end(isp_dev);

				isp_dev->isp_param_flt_update_needed = false;

				time_left -= CIFISP_MODULE_FLT_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 9:%d %d\n",
				time_left,
				isp_dev->isp_param_ctk_update_needed);

			if (isp_dev->isp_param_ctk_update_needed &&
				time_left >= CIFISP_MODULE_CTK_PROC_TIME) {
				/*update ctk config */
				cifisp_ctk_config(isp_dev);

				if (isp_dev->ctk_en)
					cifisp_ctk_en(isp_dev);
				else
					cifisp_ctk_end(isp_dev);

				isp_dev->isp_param_ctk_update_needed = false;

				time_left -= CIFISP_MODULE_CTK_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 10:%d %d\n",
				time_left,
				isp_dev->isp_param_goc_update_needed);

			if (isp_dev->isp_param_goc_update_needed &&
				time_left >= CIFISP_MODULE_GOC_PROC_TIME) {
				/*update goc config */
				cifisp_goc_config(isp_dev);

				if (isp_dev->goc_en)
					cifisp_goc_en(isp_dev);
				else
					cifisp_goc_end(isp_dev);

				isp_dev->isp_param_goc_update_needed = false;

				time_left -= CIFISP_MODULE_GOC_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 11:%d %d\n",
				time_left,
				isp_dev->isp_param_cproc_update_needed);

			if (isp_dev->isp_param_cproc_update_needed &&
				time_left >= CIFISP_MODULE_CPROC_PROC_TIME) {
				/*update cprc config */
				cifisp_cproc_config(isp_dev, false);

				if (isp_dev->cproc_en)
					cifisp_cproc_en(isp_dev);
				else
					cifisp_cproc_end(isp_dev);

				isp_dev->isp_param_cproc_update_needed = false;

				time_left -= CIFISP_MODULE_CPROC_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 12:%d %d\n",
				time_left,
				isp_dev->isp_param_macc_update_needed);

			if (isp_dev->isp_param_macc_update_needed &&
				time_left >= CIFISP_MODULE_MACC_PROC_TIME) {
				/*update macc config */
				cifisp_macc_config(isp_dev);

				if (isp_dev->macc_en)
					cifisp_macc_en(isp_dev);
				else
					cifisp_macc_end(isp_dev);

				isp_dev->isp_param_macc_update_needed = false;

				time_left -= CIFISP_MODULE_MACC_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 13:%d %d\n",
				time_left,
				isp_dev->isp_param_tmap_update_needed);

			if (isp_dev->isp_param_tmap_update_needed &&
				time_left >= CIFISP_MODULE_TMAP_PROC_TIME) {
				/*update tmap config */
				cifisp_tmap_config(isp_dev);

				if (isp_dev->tmap_en)
					cifisp_tmap_en(isp_dev);
				else
					cifisp_tmap_end(isp_dev);

				isp_dev->isp_param_tmap_update_needed = false;

				time_left -= CIFISP_MODULE_TMAP_PROC_TIME;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 14:%d %d\n",
				time_left,
				isp_dev->isp_param_ie_update_needed);

			if (isp_dev->isp_param_ie_update_needed &&
				time_left >= CIFISP_MODULE_IE_PROC_TIME) {
				/*update ie config */
				cifisp_ie_config(isp_dev);

				if (isp_dev->ie_en)
					cifisp_ie_en(isp_dev);
				else
					cifisp_ie_end(isp_dev);

				isp_dev->isp_param_ie_update_needed = false;

				time_left -= CIFISP_MODULE_IE_PROC_TIME;
			}
		} else {
#endif
			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 15:%d %d\n",
				time_left,
				isp_dev->isp_param_awb_meas_update_needed);

			if (isp_dev->isp_param_awb_meas_update_needed) {
				/*update awb config */
				cifisp_awb_meas_config(isp_dev);

				if (isp_dev->awb_meas_en)
					cifisp_awb_meas_en(isp_dev);
				else
					cifisp_awb_meas_end(isp_dev);

				isp_dev->isp_param_awb_meas_update_needed =
					false;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 16:%d %d\n",
				time_left,
				isp_dev->isp_param_afc_update_needed);

			if (isp_dev->isp_param_afc_update_needed) {
				/*update afc config */
				cifisp_afc_config(isp_dev);

				if (isp_dev->afc_en)
					cifisp_afc_en(isp_dev);
				else
					cifisp_afc_end(isp_dev);

				isp_dev->isp_param_afc_update_needed =
					false;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 17:%d %d\n",
				time_left,
				isp_dev->isp_param_hst_update_needed);

			if (isp_dev->isp_param_hst_update_needed) {
				/*update hst config */
				cifisp_hst_config(isp_dev);

				if (isp_dev->hst_en)
					cifisp_hst_en(isp_dev);
				else
					cifisp_hst_end(isp_dev);

				isp_dev->isp_param_hst_update_needed = false;
			}

			CIFISP_DPRINT(CIFISP_DEBUG,
				"time-left 18:%d %d\n",
				time_left,
				isp_dev->isp_param_aec_update_needed);

			if (isp_dev->isp_param_aec_update_needed) {
				/*update aec config */
				cifisp_aec_config(isp_dev);

				if (isp_dev->aec_en)
					cifisp_aec_en(isp_dev);
				else
					cifisp_aec_end(isp_dev);

				isp_dev->isp_param_aec_update_needed =
					false;
			}
#ifndef CIFISP_DEBUG_DISABLE_BLOCKS
		}
#endif

		spin_unlock_irqrestore(&isp_dev->config_lock, lock_flags);

		cifisp_dump_reg(isp_dev, CIFISP_DEBUG);
	}
#ifdef LOG_ISR_EXE_TIME
	if (isp_mis & (CIF_ISP_EXP_END | CIF_ISP_AWB_DONE | CIF_ISP_FRAME)) {
		unsigned int diff_us =
		    ktime_to_us(ktime_sub(ktime_get(), in_t));

		if (diff_us > g_longest_isr_time)
			g_longest_isr_time = diff_us;

		pr_info("isp_isr time %d %d\n", diff_us, g_longest_isr_time);
	}
#endif

	return 0;
}

static void cifisp_param_dump(const void *config, unsigned int module)
{
	switch (module) {
	case CIFISP_MODULE_AWB_GAIN:{
			struct cifisp_awb_gain_config *pconfig =
			    (struct cifisp_awb_gain_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AWB Gain Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, "g_g: %d\n",
				      pconfig->gain_green_r);
			CIFISP_DPRINT(CIFISP_DEBUG, "g_b: %d\n",
				      pconfig->gain_green_b);
			CIFISP_DPRINT(CIFISP_DEBUG, "r: %d\n",
				      pconfig->gain_red);
			CIFISP_DPRINT(CIFISP_DEBUG, "b: %d\n",
				      pconfig->gain_blue);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AWB Gain Parameters - END ####\n",
				      ISP_DEV_NAME);
		}
		break;
	case CIFISP_MODULE_BPC:{
			struct cifisp_bpc_config *pconfig =
			    (struct cifisp_bpc_config *)config;
			struct cifisp_bp_correction_config *pcor_config =
			    &pconfig->corr_config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: BPC Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " corr_type: %d\n",
				      pcor_config->corr_type);
			CIFISP_DPRINT(CIFISP_DEBUG, " corr_rep: %d\n",
				      pcor_config->corr_rep);
			CIFISP_DPRINT(CIFISP_DEBUG, " corr_mode: %d\n",
				      pcor_config->corr_mode);
			CIFISP_DPRINT(CIFISP_DEBUG, " abs_hot_thres: %d\n",
				      pcor_config->abs_hot_thres);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " abs_dead_thres: %d\n",
				      pcor_config->abs_dead_thres);
			CIFISP_DPRINT(CIFISP_DEBUG, " dev_hot_thres: %d\n",
				      pcor_config->dev_hot_thres);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " dev_dead_thres: %d\n",
				      pcor_config->dev_dead_thres);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: BPC Parameters - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_BLS:{
			struct cifisp_bls_config *pconfig =
			    (struct cifisp_bls_config *)config;
			struct cifisp_bls_fixed_val *pval = &pconfig->fixed_val;

			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: BLS Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " enable_auto: %d\n",
				      pconfig->enable_auto);
			CIFISP_DPRINT(CIFISP_DEBUG, " en_windows: %d\n",
				      pconfig->en_windows);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window1.h_offs: %d\n",
				      pconfig->bls_window1.h_offs);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window1.v_offs: %d\n",
				      pconfig->bls_window1.v_offs);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window1.h_size: %d\n",
				      pconfig->bls_window1.h_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window1.v_size: %d\n",
				      pconfig->bls_window1.v_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window2.h_offs: %d\n",
				      pconfig->bls_window2.h_offs);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window2.v_offs: %d\n",
				      pconfig->bls_window2.v_offs);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window2.h_size: %d\n",
				      pconfig->bls_window2.h_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " bls_window2.v_size: %d\n",
				      pconfig->bls_window2.v_size);
			CIFISP_DPRINT(CIFISP_DEBUG, " bls_samples: %d\n",
				      pconfig->bls_samples);
			CIFISP_DPRINT(CIFISP_DEBUG, " fixed_A: %d\n",
				      pval->fixed_a);
			CIFISP_DPRINT(CIFISP_DEBUG, " fixed_B: %d\n",
				      pval->fixed_b);
			CIFISP_DPRINT(CIFISP_DEBUG, " fixed_C: %d\n",
				      pval->fixed_c);
			CIFISP_DPRINT(CIFISP_DEBUG, " fixed_D: %d\n",
				      pval->fixed_d);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: BLS Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;
	case CIFISP_MODULE_LSC:{
			int i;
			struct cifisp_lsc_config *pconfig =
			    (struct cifisp_lsc_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### LSC Parameters - BEGIN ####\n");
			CIFISP_DPRINT(CIFISP_DEBUG,	"sect size\n");
			for (i = 0; i < CIFISP_LSC_SIZE_TBL_SIZE; i++)
				CIFISP_DPRINT(CIFISP_DEBUG,
					"x 0x%x y 0x%x\n",
					*(pconfig->x_size_tbl + i),
					*(pconfig->y_size_tbl + i));
			CIFISP_DPRINT(CIFISP_DEBUG,	"\ngrad\n");

			for (i = 0; i < CIFISP_LSC_GRAD_TBL_SIZE; i++)
				CIFISP_DPRINT(CIFISP_DEBUG,
					"x 0x%x y 0x%x\n",
					*(pconfig->x_grad_tbl + i),
					*(pconfig->y_grad_tbl + i));

			CIFISP_DPRINT(CIFISP_DEBUG,	"\nsample\n");

			for (i = 0; i < CIFISP_LSC_DATA_TBL_SIZE; i++)
				CIFISP_DPRINT(CIFISP_DEBUG,
					"r 0x%x g 0x%x b 0x%x\n",
					*(pconfig->r_data_tbl + i),
					*(pconfig->g_data_tbl + i),
					*(pconfig->b_data_tbl + i));

			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### LSC Parameters - END ####\n");
		}
		break;
	case CIFISP_MODULE_FLT:{
			struct cifisp_flt_config *pconfig =
			    (struct cifisp_flt_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: FLT Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_mask_sharp0: %d\n",
				      pconfig->flt_mask_sharp0);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_mask_sharp1: %d\n",
				      pconfig->flt_mask_sharp1);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_mask_diag: %d\n",
				      pconfig->flt_mask_diag);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_mask_blur_max: %d\n",
				      pconfig->flt_mask_blur_max);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_mask_blur: %d\n",
				      pconfig->flt_mask_blur);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_mask_lin: %d\n",
				      pconfig->flt_mask_lin);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_mask_orth: %d\n",
				      pconfig->flt_mask_orth);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_mask_v_diag: %d\n",
				      pconfig->flt_mask_v_diag);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_mask_h_diag: %d\n",
				      pconfig->flt_mask_h_diag);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_lum_weight: %d\n",
				      pconfig->flt_lum_weight);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_blur_th0: %d\n",
				      pconfig->flt_blur_th0);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_blur_th1: %d\n",
				      pconfig->flt_blur_th1);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_sharp0_th: %d\n",
				      pconfig->flt_sharp0_th);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_sharp1_th: %d\n",
				      pconfig->flt_sharp1_th);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_chrom_h_mode: %d\n",
				      pconfig->flt_chrom_h_mode);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_chrom_v_mode: %d\n",
				      pconfig->flt_chrom_v_mode);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " flt_diag_sharp_mode: %d\n",
				      pconfig->flt_diag_sharp_mode);
			CIFISP_DPRINT(CIFISP_DEBUG, " flt_mode: %d\n",
				      pconfig->flt_mode);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: FLT Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_BDM:{
			struct cifisp_bdm_config *pconfig =
			    (struct cifisp_bdm_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: BDM Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " demosaic_th: %d\n",
				      pconfig->demosaic_th);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: BDM Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_SDG:{
			struct cifisp_sdg_config *pconfig =
			    (struct cifisp_sdg_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: SDG Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " RED -Curve parameters\n");
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y0: %d\n",
				      pconfig->curve_r.gamma_y0);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y1: %d\n",
				      pconfig->curve_r.gamma_y1);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y2: %d\n",
				      pconfig->curve_r.gamma_y2);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y3: %d\n",
				      pconfig->curve_r.gamma_y3);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y4: %d\n",
				      pconfig->curve_r.gamma_y4);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y5: %d\n",
				      pconfig->curve_r.gamma_y5);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y6: %d\n",
				      pconfig->curve_r.gamma_y6);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y7: %d\n",
				      pconfig->curve_r.gamma_y7);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y8: %d\n",
				      pconfig->curve_r.gamma_y8);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y9: %d\n",
				      pconfig->curve_r.gamma_y9);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y10: %d\n",
				      pconfig->curve_r.gamma_y10);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y11: %d\n",
				      pconfig->curve_r.gamma_y11);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y12: %d\n",
				      pconfig->curve_r.gamma_y12);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y13: %d\n",
				      pconfig->curve_r.gamma_y13);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y14: %d\n",
				      pconfig->curve_r.gamma_y14);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y15: %d\n",
				      pconfig->curve_r.gamma_y15);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y16: %d\n",
				      pconfig->curve_r.gamma_y16);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " GREEN -Curve parameters\n");
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y0: %d\n",
				      pconfig->curve_g.gamma_y0);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y1: %d\n",
				      pconfig->curve_g.gamma_y1);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y2: %d\n",
				      pconfig->curve_g.gamma_y2);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y3: %d\n",
				      pconfig->curve_g.gamma_y3);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y4: %d\n",
				      pconfig->curve_g.gamma_y4);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y5: %d\n",
				      pconfig->curve_g.gamma_y5);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y6: %d\n",
				      pconfig->curve_g.gamma_y6);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y7: %d\n",
				      pconfig->curve_g.gamma_y7);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y8: %d\n",
				      pconfig->curve_g.gamma_y8);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y9: %d\n",
				      pconfig->curve_g.gamma_y9);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y10: %d\n",
				      pconfig->curve_g.gamma_y10);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y11: %d\n",
				      pconfig->curve_g.gamma_y11);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y12: %d\n",
				      pconfig->curve_g.gamma_y12);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y13: %d\n",
				      pconfig->curve_g.gamma_y13);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y14: %d\n",
				      pconfig->curve_g.gamma_y14);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y15: %d\n",
				      pconfig->curve_g.gamma_y15);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y16: %d\n",
				      pconfig->curve_g.gamma_y16);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " BLUE -Curve parameters\n");
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y0: %d\n",
				      pconfig->curve_b.gamma_y0);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y1: %d\n",
				      pconfig->curve_b.gamma_y1);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y2: %d\n",
				      pconfig->curve_b.gamma_y2);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y3: %d\n",
				      pconfig->curve_b.gamma_y3);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y4: %d\n",
				      pconfig->curve_b.gamma_y4);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y5: %d\n",
				      pconfig->curve_b.gamma_y5);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y6: %d\n",
				      pconfig->curve_b.gamma_y6);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y7: %d\n",
				      pconfig->curve_b.gamma_y7);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y8: %d\n",
				      pconfig->curve_b.gamma_y8);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y9: %d\n",
				      pconfig->curve_b.gamma_y9);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y10: %d\n",
				      pconfig->curve_b.gamma_y10);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y11: %d\n",
				      pconfig->curve_b.gamma_y11);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y12: %d\n",
				      pconfig->curve_b.gamma_y12);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y13: %d\n",
				      pconfig->curve_b.gamma_y13);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y14: %d\n",
				      pconfig->curve_b.gamma_y14);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y15: %d\n",
				      pconfig->curve_b.gamma_y15);
			CIFISP_DPRINT(CIFISP_DEBUG, " gamma_y16: %d\n",
				      pconfig->curve_b.gamma_y16);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: SDG Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_GOC:{
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: GOC Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: GOC Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_CTK:{
			struct cifisp_ctk_config *pconfig =
			    (struct cifisp_ctk_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: CTK Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff0: %d\n",
				      pconfig->coeff0);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff1: %d\n",
				      pconfig->coeff1);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff2: %d\n",
				      pconfig->coeff2);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff3: %d\n",
				      pconfig->coeff3);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff4: %d\n",
				      pconfig->coeff4);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff5: %d\n",
				      pconfig->coeff5);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff6: %d\n",
				      pconfig->coeff6);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff7: %d\n",
				      pconfig->coeff7);
			CIFISP_DPRINT(CIFISP_DEBUG, " coeff8: %d\n",
				      pconfig->coeff8);
			CIFISP_DPRINT(CIFISP_DEBUG, " ct_offset_r: %d\n",
				      pconfig->ct_offset_r);
			CIFISP_DPRINT(CIFISP_DEBUG, " ct_offset_g: %d\n",
				      pconfig->ct_offset_g);
			CIFISP_DPRINT(CIFISP_DEBUG, " ct_offset_b: %d\n",
				      pconfig->ct_offset_b);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: CTK Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_AWB:{
			struct cifisp_awb_meas_config *pconfig =
			    (struct cifisp_awb_meas_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AWB Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " awb_mode: %d\n",
				      pconfig->awb_mode);
			CIFISP_DPRINT(CIFISP_DEBUG, " max_y: %d\n",
				      pconfig->max_y);
			CIFISP_DPRINT(CIFISP_DEBUG, " min_y: %d\n",
				      pconfig->min_y);
			CIFISP_DPRINT(CIFISP_DEBUG, " max_csum: %d\n",
				      pconfig->max_csum);
			CIFISP_DPRINT(CIFISP_DEBUG, " min_c: %d\n",
				      pconfig->min_c);
			CIFISP_DPRINT(CIFISP_DEBUG, " frames: %d\n",
				      pconfig->frames);
			CIFISP_DPRINT(CIFISP_DEBUG, " awb_ref_cr: %d\n",
				      pconfig->awb_ref_cr);
			CIFISP_DPRINT(CIFISP_DEBUG, " awb_ref_cb: %d\n",
				      pconfig->awb_ref_cb);
			CIFISP_DPRINT(CIFISP_DEBUG, " gb_sat: %d\n",
				      pconfig->gb_sat);
			CIFISP_DPRINT(CIFISP_DEBUG, " gr_sat: %d\n",
				      pconfig->gr_sat);
			CIFISP_DPRINT(CIFISP_DEBUG, " r_sat: %d\n",
				      pconfig->b_sat);
			CIFISP_DPRINT(CIFISP_DEBUG, " grid_h_dim: %d\n",
				      pconfig->grid_h_dim);
			CIFISP_DPRINT(CIFISP_DEBUG, " grid_v_dim: %d\n",
				      pconfig->grid_v_dim);
			CIFISP_DPRINT(CIFISP_DEBUG, " grid_h_dist: %d\n",
				      pconfig->grid_h_dist);
			CIFISP_DPRINT(CIFISP_DEBUG, " grid_v_dist: %d\n",
				      pconfig->grid_v_dist);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " enable_ymax_cmp: %d\n",
				      pconfig->enable_ymax_cmp);
			CIFISP_DPRINT(CIFISP_DEBUG, " rgb_meas_pnt: %d\n",
				      pconfig->rgb_meas_pnt);
			CIFISP_DPRINT(CIFISP_DEBUG, " AWB Window size\n");
			CIFISP_DPRINT(CIFISP_DEBUG, " h_offs: %d\n",
				      pconfig->awb_wnd.h_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " v_offs: %d\n",
				      pconfig->awb_wnd.v_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " h_size: %d\n",
				      pconfig->awb_wnd.h_size);
			CIFISP_DPRINT(CIFISP_DEBUG, " v_size: %d\n",
				      pconfig->awb_wnd.v_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AWB Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_HST:{
			struct cifisp_hst_config *pconfig =
			    (struct cifisp_hst_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: HST Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " mode: %d\n",
				      pconfig->mode);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " histogram_predivider: %d\n",
				      pconfig->histogram_predivider);
			CIFISP_DPRINT(CIFISP_DEBUG, " HST Window size\n");
			CIFISP_DPRINT(CIFISP_DEBUG, " h_offs: %d\n",
				      pconfig->meas_window.h_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " v_offs: %d\n",
				      pconfig->meas_window.v_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " h_size: %d\n",
				      pconfig->meas_window.h_size);
			CIFISP_DPRINT(CIFISP_DEBUG, " v_size: %d\n",
				      pconfig->meas_window.v_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: HST Parameters - END ####\n",
				      ISP_DEV_NAME);

		} break;

	case CIFISP_MODULE_AEC:{
			struct cifisp_aec_config *pconfig =
			    (struct cifisp_aec_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AEC Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " autostop: %d\n",
				      pconfig->autostop);
			CIFISP_DPRINT(CIFISP_DEBUG, " AEC Window size\n");
			CIFISP_DPRINT(CIFISP_DEBUG, " h_offs: %d\n",
				      pconfig->meas_window.h_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " v_offs: %d\n",
				      pconfig->meas_window.v_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " h_size: %d\n",
				      pconfig->meas_window.h_size);
			CIFISP_DPRINT(CIFISP_DEBUG, " v_size: %d\n",
				      pconfig->meas_window.v_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AEC Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;

	case CIFISP_MODULE_CPROC:{
			struct cifisp_cproc_config *pconfig =
			    (struct cifisp_cproc_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: CPROC Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " contrast: %d\n",
				      pconfig->contrast);
			CIFISP_DPRINT(CIFISP_DEBUG, " hue: %d\n",
				      pconfig->hue);
			CIFISP_DPRINT(CIFISP_DEBUG, " sat: %d\n",
				      pconfig->sat);
			CIFISP_DPRINT(CIFISP_DEBUG, " brightness: %d\n",
				      pconfig->brightness);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: CPROC Parameters - END ####\n",
				      ISP_DEV_NAME);
		} break;
	case CIFISP_MODULE_YCFLT:{
			struct cifisp_ycflt_config *pconfig =
			    (struct cifisp_ycflt_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: YCFLT Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG, " ctrl: %d\n",
					pconfig->ctrl);
			CIFISP_DPRINT(CIFISP_DEBUG, " chr_ss_ctrl: %d\n",
				      pconfig->chr_ss_ctrl);
			CIFISP_DPRINT(CIFISP_DEBUG, " chr_ss_fac: %d\n",
				      pconfig->chr_ss_fac);
			CIFISP_DPRINT(CIFISP_DEBUG, " chr_ss_offs: %d\n",
				      pconfig->chr_ss_offs);
			CIFISP_DPRINT(CIFISP_DEBUG, " chr_nr_ctrl: %d\n",
				      pconfig->chr_nr_ctrl);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " lum_eenr_edge_gain: %d\n",
				      pconfig->lum_eenr_edge_gain);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " lum_eenr_corner_gain: %d\n",
				      pconfig->lum_eenr_corner_gain);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " lum_eenr_fc_crop_neg: %d\n",
				      pconfig->lum_eenr_fc_crop_neg);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " lum_eenr_fc_crop_pos: %d\n",
				      pconfig->lum_eenr_fc_crop_pos);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " lum_eenr_fc_gain_neg: %d\n",
				      pconfig->lum_eenr_fc_gain_neg);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " lum_eenr_fc_gain_pos: %d\n",
				      pconfig->lum_eenr_fc_gain_pos);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: YCFLT Parameters - END ####\n",
				      ISP_DEV_NAME);
			break;
		}
	case CIFISP_MODULE_AFC:{
			struct cifisp_afc_config *pconfig =
			    (struct cifisp_afc_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				      "#### %s: AFC Parameters - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " window A %d %d %d %d\n",
				      pconfig->afm_win[0].h_offs,
				      pconfig->afm_win[0].v_offs,
				      pconfig->afm_win[0].h_size,
				      pconfig->afm_win[0].v_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " window B %d %d %d %d\n",
				      pconfig->afm_win[1].h_offs,
				      pconfig->afm_win[1].v_offs,
				      pconfig->afm_win[1].h_size,
				      pconfig->afm_win[1].v_size);
			CIFISP_DPRINT(CIFISP_DEBUG,
				      " window C %d %d %d %d\n",
				      pconfig->afm_win[2].h_offs,
				      pconfig->afm_win[2].v_offs,
				      pconfig->afm_win[2].h_size,
				      pconfig->afm_win[2].v_size);
			CIFISP_DPRINT(CIFISP_DEBUG, " thres: %d\n",
				      pconfig->thres);
			CIFISP_DPRINT(CIFISP_DEBUG, " var_shift: %d\n",
				      pconfig->var_shift);
			break;
		}
	case CIFISP_MODULE_IE: {
			struct cifisp_ie_config *pconfig =
			    (struct cifisp_ie_config *)config;
			CIFISP_DPRINT(CIFISP_DEBUG,
				"effect %d, %x, %x, %x, %x, %x, %x %d\n",
				pconfig->effect, pconfig->color_sel,
				pconfig->eff_mat_1, pconfig->eff_mat_2,
				pconfig->eff_mat_3, pconfig->eff_mat_4,
				pconfig->eff_mat_5, pconfig->eff_tint);
			break;
		}
	default:
		CIFISP_DPRINT(CIFISP_DEBUG,
			      "####%s: Invalid Module ID ####\n", ISP_DEV_NAME);
		break;
	}
}

#ifdef CIFISP_DEBUG_REG
static void cifisp_reg_dump(const struct xgold_isp_dev *isp_dev,
			    unsigned int module, int level)
{
	switch (module) {
	case CIFISP_MODULE_BPC:
		CIFISP_DPRINT(level, "#### BPC Registers - BEGIN ####\n");
		CIFISP_DPRINT(level,
			      " CIF_ISP_BP_CTRL: %d\n",
			      cifisp_ioread32(CIF_ISP_BP_CTRL));
		CIFISP_DPRINT(level,
			      " CIF_ISP_BP_CFG1: %d\n",
			      cifisp_ioread32(CIF_ISP_BP_CFG1));
		CIFISP_DPRINT(level,
			      " CIF_ISP_BP_CFG2: %d\n",
			      cifisp_ioread32(CIF_ISP_BP_CFG2));
		CIFISP_DPRINT(level, "#### BPC Registers - END ####\n");
		break;
	case CIFISP_MODULE_BLS:
		CIFISP_DPRINT(level, "#### BLS Registers - BEGIN ####\n");
		CIFISP_DPRINT(level, " CIF_ISP_BLS_CTRL: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_CTRL));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_SAMPLES: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_SAMPLES));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_H1_START: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_H1_START));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_H1_STOP: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_H1_STOP));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_H1_START: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_H1_START));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_V1_START: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_V1_START));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_V1_STOP: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_V1_STOP));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_H2_START: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_H2_START));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_H2_STOP: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_H2_STOP));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_V2_START: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_V2_START));
		CIFISP_DPRINT(level, " CIF_ISP_BLS_V2_STOP: %d\n",
			      cifisp_ioread32(CIF_ISP_BLS_V2_STOP));
		CIFISP_DPRINT(level, "#### BLS Registers - END ####\n");
		break;
	case CIFISP_MODULE_LSC:
		CIFISP_DPRINT(level, "#### LSC Registers - BEGIN ####\n");
		CIFISP_DPRINT(level, "#### LSC Registers - END ####\n");
		break;
	case CIFISP_MODULE_FLT:{
			CIFISP_DPRINT(level,
				"#### %s: FLT Registers - BEGIN ####\n",
				ISP_DEV_NAME);
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MODE: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MODE));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_SHARP_0: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_SHARP_0));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_SHARP_1: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_SHARP_1));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_SHARP_DIAG: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_SHARP_DIAG));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_BLUR_MAX: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_BLUR_MAX));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_BLUR: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_BLUR));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_LIN: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_LIN));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_LIN_ORTH: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_LIN_ORTH));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_DIAG: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_DIAG));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_MASK_H_DIAG: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_MASK_H_DIAG));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_BLUR_TH0: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_BLUR_TH0));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_BLUR_TH1: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_BLUR_TH1));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_SHARP0_TH: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_SHARP0_TH));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_SHARP1_TH: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_SHARP1_TH));
			CIFISP_DPRINT(level,
				" CIF_ISP_FILT_LUM_WEIGHT: %d\n",
				cifisp_ioread32(CIF_ISP_FILT_LUM_WEIGHT));
			CIFISP_DPRINT(level,
				"#### %s: FLT Registers - END ####\n",
				ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_BDM:{
			CIFISP_DPRINT(level,
				      "#### %s: BDM Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " CIF_ISP_DEMOSAIC: %d\n",
				      cifisp_ioread32(CIF_ISP_DEMOSAIC));
			CIFISP_DPRINT(level,
				      "#### %s: BDM Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_SDG:{
			CIFISP_DPRINT(level,
				      "#### %s: SDG Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_DX_LO: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_DX_LO));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_DX_HI: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_DX_HI));

			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y0: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y0));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y1: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y1));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y2: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y2));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y3: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y3));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y4: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y4));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y5: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y5));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y6: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y6));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y7: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y7));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y8: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y8));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y9: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y9));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y10: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y10));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y11: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y11));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y12: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y12));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y13: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y13));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y14: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y14));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y15: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y15));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_R_Y16: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_R_Y16));

			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y0: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y0));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y1: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y1));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y2: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y2));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y3: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y3));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y4: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y4));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y5: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y5));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y6: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y6));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y7: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y7));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y8: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y8));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y9: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y9));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y10: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y10));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y11: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y11));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y12: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y12));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y13: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y13));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y14: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y14));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y15: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y15));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_G_Y16: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_G_Y16));

			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y0: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y0));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y1: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y1));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y2: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y2));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y3: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y3));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y4: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y4));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y5: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y5));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y6: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y6));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y7: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y7));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y8: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y8));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y9: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y9));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y10: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y10));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y11: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y11));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y12: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y12));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y13: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y13));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y14: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y14));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y15: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y15));
			CIFISP_DPRINT(level, " CIF_ISP_GAMMA_B_Y16: %d\n",
				      cifisp_ioread32(CIF_ISP_GAMMA_B_Y16));
			CIFISP_DPRINT(level,
				      "#### %s: SDG Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_GOC:{
			CIFISP_DPRINT(level,
				      "#### %s: GOC Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level,
				      "#### %s: GOC registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_CTK:{
			CIFISP_DPRINT(level,
				      "#### %s: CTK Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_0: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_0));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_1: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_1));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_2: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_2));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_3: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_3));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_4: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_4));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_5: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_5));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_6: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_6));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_7: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_7));
			CIFISP_DPRINT(level, " CIF_ISP_CT_COEFF_8: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_COEFF_8));
			CIFISP_DPRINT(level, " CIF_ISP_CT_OFFSET_R: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_OFFSET_R));
			CIFISP_DPRINT(level, " CIF_ISP_CT_OFFSET_G: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_OFFSET_G));
			CIFISP_DPRINT(level, " CIF_ISP_CT_OFFSET_B: %d\n",
				      cifisp_ioread32(CIF_ISP_CT_OFFSET_B));
			CIFISP_DPRINT(level,
				      "#### %s: CTK Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_AWB:{
			CIFISP_DPRINT(level,
				      "#### %s: AWB Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " CIF_ISP_AWB_PROP: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_PROP));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_GAIN_G: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_GAIN_G));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_GAIN_RB: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_GAIN_RB));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_THRESH_G: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_THRESH_G));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_THRESH_RB: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_THRESH_RB));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_REF: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_REF));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_THRESH_YC: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_THRESH_YC));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_GAIN_RB: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_PROP));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_OFFS: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_OFFS));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_WND_SIZE: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_WND_SIZE));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_GRID_DIM: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_GRID_DIM));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_GRID_DIST: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_GRID_DIST));
			CIFISP_DPRINT(level, " CIF_ISP_AWB_FRAMES: %x\n",
				      cifisp_ioread32(CIF_ISP_AWB_FRAMES));
			CIFISP_DPRINT(level,
				      "#### %s: AWB Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_HST:{
			CIFISP_DPRINT(level,
				      "#### %s: HST Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " CIF_ISP_HIST_PROP: %d\n",
				      cifisp_ioread32(CIF_ISP_HIST_PROP));
			CIFISP_DPRINT(level, " CIF_ISP_HIST_H_OFFS: %d\n",
				      cifisp_ioread32(CIF_ISP_HIST_H_OFFS));
			CIFISP_DPRINT(level, " CIF_ISP_HIST_H_SIZE: %d\n",
				      cifisp_ioread32(CIF_ISP_HIST_H_SIZE));
			CIFISP_DPRINT(level, " CIF_ISP_HIST_V_OFFS: %d\n",
				      cifisp_ioread32(CIF_ISP_HIST_V_OFFS));
			CIFISP_DPRINT(level, " CIF_ISP_HIST_V_SIZE: %d\n",
				      cifisp_ioread32(CIF_ISP_HIST_V_SIZE));
			CIFISP_DPRINT(level,
				      "#### %s: HST Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_AEC:{
			CIFISP_DPRINT(level,
				      "#### %s: AEC Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " CIF_ISP_EXP_CTRL: %d\n",
				      cifisp_ioread32(CIF_ISP_EXP_CTRL));
			CIFISP_DPRINT(level, " CIF_ISP_EXP_H_OFFSET: %d\n",
				      cifisp_ioread32(CIF_ISP_EXP_H_OFFSET));
			CIFISP_DPRINT(level, " CIF_ISP_EXP_V_OFFSET: %d\n",
				      cifisp_ioread32(CIF_ISP_EXP_V_OFFSET));
			CIFISP_DPRINT(level, " CIF_ISP_EXP_H_SIZE: %d\n",
				      cifisp_ioread32(CIF_ISP_EXP_H_SIZE));
			CIFISP_DPRINT(level, " CIF_ISP_EXP_V_SIZE: %d\n",
				      cifisp_ioread32(CIF_ISP_EXP_V_SIZE));
			CIFISP_DPRINT(level,
				      "#### %s: AEC Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_CPROC:{
			CIFISP_DPRINT(level,
				      "#### %s: CPROC Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " ctrl: %d\n",
				      cifisp_ioread32(CIF_C_PROC_CTRL));
			CIFISP_DPRINT(level, " contrast: %d\n",
				      cifisp_ioread32(CIF_C_PROC_CONTRAST));
			CIFISP_DPRINT(level, " hue: %d\n",
				      cifisp_ioread32(CIF_C_PROC_HUE));
			CIFISP_DPRINT(level, " sat: %d\n",
				      cifisp_ioread32(CIF_C_PROC_SATURATION));
			CIFISP_DPRINT(level, " brightness: %d\n",
				      cifisp_ioread32(CIF_C_PROC_BRIGHTNESS));
			CIFISP_DPRINT(level,
				      "#### %s: CPROC Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;
	case CIFISP_MODULE_YCFLT:{
			CIFISP_DPRINT(level,
				      "#### %s: YCFLT Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " chr_ss_ctrl: %d\n",
				      cifisp_ioread32(CIF_YC_FLT_CHR_SS_CTRL));
			CIFISP_DPRINT(level, " chr_ss_fac: %d\n",
				      cifisp_ioread32(CIF_YC_FLT_CHR_SS_FAC));
			CIFISP_DPRINT(level, " chr_ss_offs: %d\n",
				      cifisp_ioread32(CIF_YC_FLT_CHR_SS_OFFS));
			CIFISP_DPRINT(level, " chr_nr_ctrl: %d\n",
				      cifisp_ioread32(CIF_YC_FLT_CHR_NR_CTRL));
			CIFISP_DPRINT(level, " lum_eenr_edge_gain: %d\n",
				      cifisp_ioread32
				      (CIF_YC_FLT_LUM_EENR_EDGE_GAIN));
			CIFISP_DPRINT(level, " lum_eenr_corner_gain: %d\n",
				      cifisp_ioread32
				      (CIF_YC_FLT_LUM_ENNR_CORNER_GAIN));
			CIFISP_DPRINT(level, " lum_eenr_fc_crop_neg: %d\n",
				      cifisp_ioread32
				      (CIF_YC_FLT_LUM_ENNR_FC_CROP_NEG));
			CIFISP_DPRINT(level, " lum_eenr_fc_crop_pos: %d\n",
				      cifisp_ioread32
				      (CIF_YC_FLT_LUM_ENNR_FC_CROP_POS));
			CIFISP_DPRINT(level, " lum_eenr_fc_gain_neg: %d\n",
				      cifisp_ioread32
				      (CIF_YC_FLT_LUM_ENNR_FC_GAIN_NEG));
			CIFISP_DPRINT(level, " lum_eenr_fc_gain_pos: %d\n",
				      cifisp_ioread32
				      (CIF_YC_FLT_LUM_ENNR_FC_GAIN_POS));
			CIFISP_DPRINT(level,
				      "#### %s: YCFLT Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;
	case CIFISP_MODULE_AFC:{
			CIFISP_DPRINT(level,
				      "#### %s: AFC Registers - BEGIN ####\n",
				      ISP_DEV_NAME);
			CIFISP_DPRINT(level, " afm_ctr: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_CTRL));
			CIFISP_DPRINT(level, " afm_lt_a: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_LT_A));
			CIFISP_DPRINT(level, " afm_rb_a: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_RB_A));
			CIFISP_DPRINT(level, " afm_lt_b: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_LT_B));
			CIFISP_DPRINT(level, " afm_rb_b: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_RB_B));
			CIFISP_DPRINT(level, " afm_lt_c: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_LT_C));
			CIFISP_DPRINT(level, " afm_rb_c: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_RB_C));
			CIFISP_DPRINT(level, " afm_thres: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_THRES));
			CIFISP_DPRINT(level, " afm_var_shift: %d\n",
				      cifisp_ioread32(CIF_ISP_AFM_VAR_SHIFT));
			CIFISP_DPRINT(level,
				      "#### %s: YCFLT Registers - END ####\n",
				      ISP_DEV_NAME);
		}
		break;

	case CIFISP_MODULE_MACC:{
			CIFISP_DPRINT(level,
				      "#### MACC Registers - BEGIN ####\n");
			CIFISP_DPRINT(level,
				      "#### MACC Registers - END ####\n");
		}
		break;
	case CIFISP_MODULE_TMAP:{
			CIFISP_DPRINT(level,
				      "#### TMAP Registers - BEGIN ####\n");
			CIFISP_DPRINT(level,
				      "#### TMAP Registers - END ####\n");
		}
		break;

	default:
		CIFISP_DPRINT(level, "####%s: Invalid Module ID ####\n",
			      ISP_DEV_NAME);
		break;
	}
}
#endif
