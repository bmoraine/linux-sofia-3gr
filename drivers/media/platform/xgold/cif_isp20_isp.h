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

#ifndef _CIF_ISP20_ISP_H
#define _CIF_ISP20_ISP_H

#include <media/v4l2-common.h>
#include <media/videobuf-core.h>
#include <media/xgold-isp-ioctl.h>

/****************************************************************************
*                                                     ISP device struct
****************************************************************************/
enum cif_isp20_pix_fmt;

struct xgold_isp_dev {

	int open_count;

	bool bpc_en;
	bool bls_en;
	bool sdg_en;
	bool lsc_en;
	bool awb_meas_en;
	bool awb_gain_en;
	bool flt_en;
	bool bdm_en;
	bool ctk_en;
	bool goc_en;
	bool hst_en;
	bool aec_en;
	bool cproc_en;
	bool macc_en;
	bool tmap_en;
	bool ycflt_en;
	bool afc_en;
	bool ie_en;

	/* Purpose of mutex is to protect and serialize use
		of isp data structure and CIF API calls. */
	struct mutex mutex;
	/* Current ISP parameters */
	spinlock_t config_lock;
	struct cifisp_bpc_config bpc_config;
	struct cifisp_bls_config bls_config;
	struct cifisp_sdg_config sdg_config;
	struct cifisp_lsc_config lsc_config;
	struct cifisp_awb_meas_config awb_meas_config;
	struct cifisp_awb_gain_config awb_gain_config;
	struct cifisp_flt_config flt_config;
	struct cifisp_bdm_config bdm_config;
	struct cifisp_ctk_config ctk_config;
	struct cifisp_goc_config goc_config;
	struct cifisp_hst_config hst_config;
	struct cifisp_aec_config aec_config;
	struct cifisp_cproc_config cproc_config;
	struct cifisp_macc_config macc_config;
	struct cifisp_tmap_config tmap_config;
	struct cifisp_ycflt_config ycflt_config;
	struct cifisp_ycflt_config ycflt_config_ism_on;
	struct cifisp_afc_config afc_config;
	struct cifisp_ie_config ie_config;

	bool isp_param_bpc_update_needed;
	bool isp_param_bls_update_needed;
	bool isp_param_sdg_update_needed;
	bool isp_param_lsc_update_needed;
	bool isp_param_awb_meas_update_needed;
	bool isp_param_awb_gain_update_needed;
	bool isp_param_flt_update_needed;
	bool isp_param_bdm_update_needed;
	bool isp_param_ctk_update_needed;
	bool isp_param_goc_update_needed;
	bool isp_param_hst_update_needed;
	bool isp_param_aec_update_needed;
	bool isp_param_cproc_update_needed;
	bool isp_param_macc_update_needed;
	bool isp_param_tmap_update_needed;
	bool isp_param_afc_update_needed;
	bool isp_param_ie_update_needed;

	bool ycflt_update;
	bool cif_ism_cropping;

	/* input resolution needed for LSC param check */
	unsigned int input_width;
	unsigned int input_height;
	unsigned int active_lsc_width;
	unsigned int active_lsc_height;

	/* ISP statistics related */
	spinlock_t irq_lock;
	struct videobuf_queue vbq_stat;
	struct list_head stat;
	void __iomem *base_addr;    /* registers base address */

	bool streamon;
	unsigned int v_blanking_us;

	unsigned int frame_id;
	unsigned int active_meas;
	struct timeval frame_start_tv;
};

int register_cifisp_device(
	struct xgold_isp_dev *isp_dev,
	struct video_device *vdev_cifisp,
	struct v4l2_device *v4l2_dev,
	void __iomem *cif_reg_baseaddress);
void unregister_cifisp_device(struct video_device *vdev_cifisp);
void cifisp_configure_isp(
	struct xgold_isp_dev *isp_dev,
	enum cif_isp20_pix_fmt in_pix_fmt,
	bool capture);
void cifisp_disable_isp(struct xgold_isp_dev *isp_dev);
int cifisp_isp_isr(struct xgold_isp_dev *isp_dev, u32 isp_mis);
void cifisp_ycflt_config(const struct xgold_isp_dev *isp_dev);
void cifisp_ycflt_en(const struct xgold_isp_dev *isp_dev);
void cifisp_ycflt_end(const struct xgold_isp_dev *isp_dev);
void cifisp_v_start(struct xgold_isp_dev *isp_dev,
	const struct timeval *timestamp);
bool cifisp_is_ie_active(const struct xgold_isp_dev *isp_dev);
#endif
