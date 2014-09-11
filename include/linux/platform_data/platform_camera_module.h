/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef PLATFORM_CAMERA_MODULE_H
#define PLATFORM_CAMERA_MODULE_H
#include <linux/videodev2.h>

#define PLTFRM_CAMERA_MODULE_REG_TYPE_DATA 0
#define PLTFRM_CAMERA_MODULE_REG_TYPE_TIMEOUT 1

extern const char *PLTFRM_CAMERA_MODULE_PIN_PD;
extern const char *PLTFRM_CAMERA_MODULE_PIN_DVDD;
extern const char *PLTFRM_CAMERA_MODULE_PIN_FLASH;
extern const char *PLTFRM_CAMERA_MODULE_PIN_TORCH;

enum pltfrm_camera_module_pin_state {
	PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE = 0,
	PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE = 1
};

struct pltfrm_camera_module_reg {
	u32 flag;
	u16 reg;
	u16 val;
};

struct pltfrm_camera_module_reg_table {
	u32 reg_table_num_entries;
	struct pltfrm_camera_module_reg *reg_table;
};

struct pltfrm_camera_module_timings {
	unsigned int coarse_integration_time_min;
	unsigned int coarse_integration_time_max_margin;
	unsigned int fine_integration_time_min;
	unsigned int fine_integration_time_max_margin;
	unsigned int fine_integration_time_def;
	unsigned int frame_length_lines;
	unsigned int line_length_pck;
	unsigned int vt_pix_clk_freq_hz;
	unsigned int crop_horizontal_start; /* Sensor crop start cord. (x0,y0)*/
	unsigned int crop_vertical_start;
	unsigned int crop_horizontal_end; /* Sensor crop end cord. (x1,y1)*/
	unsigned int crop_vertical_end;
	unsigned int sensor_output_width; /* input size to ISP */
	unsigned int sensor_output_height;
	unsigned int isp_input_width;
	unsigned int isp_input_height;
	unsigned char binning_factor_x; /* horizontal binning factor used */
	unsigned char binning_factor_y; /* vertical binning factor used */
	unsigned int sensor_type; /* 0 without ISP, 1 with ISP*/
};

#define INTEL_VIDIOC_PLTFRM_SENSOR_MODULE_TIMINGS \
	_IOR('v', BASE_VIDIOC_PRIVATE + 1, struct pltfrm_camera_module_timings*)

int pltfrm_camera_module_set_pm_state(
	struct v4l2_subdev *sd,
	enum device_pm_state state);

int pltfrm_camera_module_set_pin_state(
	struct v4l2_subdev *sd,
	const char *pin,
	enum pltfrm_camera_module_pin_state state);

int pltfrm_camera_module_s_power(
	struct v4l2_subdev *sd,
	int on);

int pltfrm_camera_module_patch_config(
	struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *frm_fmt,
	struct v4l2_subdev_frame_interval *frm_intrvl);

struct v4l2_subdev *pltfrm_camera_module_get_af_ctrl(
	struct v4l2_subdev *sd);

int pltfrm_camera_module_init(
	struct v4l2_subdev *sd,
	void **pldata);

void pltfrm_camera_module_release(
	struct v4l2_subdev *sd);

int pltfrm_camera_module_read_reg(struct v4l2_subdev *sd,
	u16 data_length,
	u16 reg,
	u32 *val);

int pltfrm_camera_module_write_reg(struct v4l2_subdev *sd,
	u16 reg, u8 val);

int pltfrm_camera_module_write_reglist(struct v4l2_subdev *sd,
	const struct pltfrm_camera_module_reg reglist[], int len);

const char *pltfrm_dev_string(struct v4l2_subdev *sd);

#define pltfrm_camera_module_pr_debug(dev, fmt, arg...) \
	pr_debug("%s.%s: " fmt, \
		pltfrm_dev_string(dev), __func__, ## arg)
#define pltfrm_camera_module_pr_info(dev, fmt, arg...) \
	pr_info("%s.%s: " fmt, \
		pltfrm_dev_string(dev), __func__, ## arg)
#define pltfrm_camera_module_pr_warn(dev, fmt, arg...) \
	pr_warn("%s.%s WARN: " fmt, \
		pltfrm_dev_string(dev), __func__, ## arg)
#define pltfrm_camera_module_pr_err(dev, fmt, arg...) \
	pr_err("%s.%s(%d) ERR: " fmt, \
		pltfrm_dev_string(dev), __func__, __LINE__, \
		## arg)

#endif
