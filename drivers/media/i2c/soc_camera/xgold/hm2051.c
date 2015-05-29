/*
 * Support for Himax HM2051 8M camera sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <media/v4l2-device.h>
#include <media/v4l2-controls_intel.h>
#include <linux/io.h>
#include <linux/time.h>
#include <linux/platform_data/platform_camera_module.h>
#include "hm2051.h"

/* i2c read/write stuff */
static int hm2051_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

	if (data_length != HM2051_8BIT && data_length != HM2051_16BIT
					&& data_length != HM2051_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = 0;
	/* high byte comes first */
	if (data_length == HM2051_8BIT)
		*val = (u8)data[0];
	else if (data_length == HM2051_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int hm2051_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int hm2051_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != HM2051_8BIT && data_length != HM2051_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == HM2051_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* HM2051_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = hm2051_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * hm2051_write_reg_array - Initializes a list of HM2051 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __hm2051_flush_reg_array, __hm2051_buf_reg_array() and
 * __hm2051_write_reg_is_consecutive() are internal functions to
 * hm2051_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __hm2051_flush_reg_array(struct i2c_client *client,
				    struct hm2051_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return hm2051_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __hm2051_buf_reg_array(struct i2c_client *client,
				  struct hm2051_write_ctrl *ctrl,
				  const struct hm2051_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case HM2051_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case HM2051_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= HM2051_MAX_WRITE_BUF_SIZE)
		return __hm2051_flush_reg_array(client, ctrl);

	return 0;
}

static int __hm2051_write_reg_is_consecutive(struct i2c_client *client,
					     struct hm2051_write_ctrl *ctrl,
					     const struct hm2051_reg *next)
{
	return 0;

	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg;
}

static int hm2051_write_reg_array(struct i2c_client *client,
				  const struct hm2051_reg *reglist)
{
	const struct hm2051_reg *next = reglist;
	struct hm2051_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != HM2051_TOK_TERM; next++) {
		switch (next->type & HM2051_TOK_MASK) {
		case HM2051_TOK_DELAY:
			err = __hm2051_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__hm2051_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __hm2051_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			}
			err = __hm2051_buf_reg_array(client, &ctrl, next);
			if (err) {
				dev_err(&client->dev, "%s: write reg[0x%X, 0x%X] error, aborted\n",
						__func__, next->reg, next->val);
				return err;
			}
			break;
		}
	}

	return __hm2051_flush_reg_array(client, &ctrl);
}
static int hm2051_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (HM2051_FOCAL_LENGTH_NUM << 16) | HM2051_FOCAL_LENGTH_DEM;
	return 0;
}

static int hm2051_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	*val = (HM2051_F_NUMBER_DEFAULT_NUM << 16) | HM2051_F_NUMBER_DEM;
	return 0;
}

static int hm2051_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (HM2051_F_NUMBER_DEFAULT_NUM << 24) |
		(HM2051_F_NUMBER_DEM << 16) |
		(HM2051_F_NUMBER_DEFAULT_NUM << 8) | HM2051_F_NUMBER_DEM;
	return 0;
}

static int hm2051_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	*val = hm2051_res[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int hm2051_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	*val = hm2051_res[dev->fmt_idx].bin_factor_y;

	return 0;
}

int hm2051_g_vblanking(struct v4l2_subdev *sd, int *val)
{
	*val = 2000;
	pr_debug("%s, vblanking %d\n", __func__, *val);

	return 0;
}

static int hm2051_g_VTS(struct v4l2_subdev *sd, u16 *vts)
{
	u16 RegH, RegL;
	u16 uRdcfgMode;
	u16 uVTS = 0, uVTS_Thrsh = HM2051_FULL_FRAME_VTS_THRESHOLD;
	u16 uIntg;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int ret = 0;

	ret = hm2051_read_reg(client,
		HM2051_8BIT, HM2051_RDCFG_REG, &uRdcfgMode);
	if (IS_ERR_VALUE(ret))
		goto err;
	uRdcfgMode &= HM2051_RDCFG_MODE_MASK;

	ret = hm2051_read_reg(client,
		HM2051_8BIT, HM2051_REG_BLANKING_ROW_H, &RegH);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = hm2051_read_reg(client,
		HM2051_8BIT, HM2051_REG_BLANKING_ROW_L, &RegL);
	if (IS_ERR_VALUE(ret))
		goto err;


	if (uRdcfgMode  == RDCFG_MODE_1616_736)
		uVTS_Thrsh = HM2051_CROP_FRAME_VTS_THRESHOLD;

	uVTS = uVTS_Thrsh += (RegH << 8)  + RegL;

	/* check intergration time */
	ret = hm2051_read_reg(client, HM2051_8BIT, HM2051_EXPOSURE_H, &RegH);
	if (IS_ERR_VALUE(ret))
		goto err;

	ret = hm2051_read_reg(client, HM2051_8BIT, HM2051_EXPOSURE_L, &RegL);
	if (IS_ERR_VALUE(ret))
		goto err;

	uIntg = (RegH << 8) + RegL;
	if (uIntg > (uVTS_Thrsh-2))
		uVTS = uIntg + 4;
	*vts = uVTS;

	return 0;

err:
	pltfrm_camera_module_pr_err(sd,
		"failed with error (%d)\n", ret);
	*vts = hm2051_res[dev->fmt_idx].lines_per_frame;
	return ret;
}

static int hm2051_g_HTS(struct v4l2_subdev *sd, u16 *hts)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u16 Hblank;

	ret = hm2051_read_reg(client,
		HM2051_8BIT, HM2051_REG_BLANKING_COLUMN, &Hblank);
	if (IS_ERR_VALUE(ret))
		goto err;
	*hts = hm2051_res[dev->fmt_idx].pixels_per_line + 16*Hblank;

	return 0;

err:
	pltfrm_camera_module_pr_err(sd,
		"failed with error (%d)\n", ret);
	*hts = hm2051_res[dev->fmt_idx].pixels_per_line;
	return ret;
}

static int hm2051_get_intg_factor(struct i2c_client *client,
				void *arg,
				const struct hm2051_resolution *res)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct isp_supplemental_sensor_mode_data *buf =
		(struct isp_supplemental_sensor_mode_data *) arg;

#if 0
	const unsigned int ext_clk_freq_hz = 26000000;
	u16 vt_pix_clk_div;
	u16 vt_sys_clk_div;
	u16 pre_pll_clk_div;
	u16 pll_multiplier;
	u32 vt_pix_clk_freq_mhz;
	u32 div;
	u16 reg_val;
	int ret;

	if (info == NULL)
		return -EINVAL;

	/* pixel clock calculattion */
  /* TODO cal this
	ret =  hm2051_read_reg(client, HM2051_8BIT,
				HM2051_VT_SYS_CLK_DIV, &vt_sys_clk_div);
	if (ret)
		return ret;

	ret =  hm2051_read_reg(client, HM2051_8BIT,
				HM2051_PRE_PLL_CLK_DIV, &pre_pll_clk_div);
	if (ret)
		return ret;

	ret =  hm2051_read_reg(client, HM2051_8BIT,
				HM2051_PLL_MULTIPLIER, &pll_multiplier);

	if (ret)
		return ret;

  vt_pix_clk_freq_mhz = ext_clk_freq_hz/((pre_pll_clk_div & 0x1F)+ 1);
  pr_info("marky :%s: vt_pix_clk_freq_mhz=%d -> MClk=%d/DivClk=%d ",
	__func__, vt_pix_clk_freq_mhz, ext_clk_freq_hz,
	((pre_pll_clk_div & 0x1F)+ 1));

  if ((pre_pll_clk_div & 0x20) == 1) {
       vt_pix_clk_freq_mhz *= (2* (pll_multiplier&0x7F) + 1);
       pr_info("marky : %s: vt_pix_clk_freq_mhz=%d  pll_multiplier = %d",
		__func__, vt_pix_clk_freq_mhz, (2* (pll_multiplier&0x7F) + 1));

  }

  if ((vt_sys_clk_div & 0x08) == 0) {
       vt_pix_clk_freq_mhz /= 2;
       pr_info("marky : %s CK2CFG[3] = 0 vt_pix_clk_freq_mhz /= 2",__func__);
  }

  pr_info("marky :%s: vt_pix_clk_freq_mhz=%d",__func__, vt_pix_clk_freq_mhz);
  dev->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
  */

	dev->vt_pix_clk_freq_mhz = res->pix_clk_freq;
	buf->vt_pix_clk_freq_mhz = res->pix_clk_freq;

	/* get integration time */

	buf->coarse_integration_time_min =
				HM2051_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
				HM2051_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min =
				HM2051_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
				HM2051_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = HM2051_FINE_INTG_TIME_MIN;

	buf->frame_length_lines = res->lines_per_frame;
	buf->line_length_pck = res->pixels_per_line;
	buf->read_mode = res->bin_mode;

	/* get the cropping and output resolution to ISP for this mode. */
	buf->output_width = res->width;
	buf->output_height = res->height;
	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = res->bin_factor_x;
	buf->binning_factor_y = res->bin_factor_y;
	buf->crop_horizontal_start = res->horizontal_start;
	buf->crop_horizontal_end = res->horizontal_end;
	buf->crop_vertical_start = res->vertical_start;
	buf->crop_vertical_end = res->vertical_end;

	pr_info("%s: output_width %d output_height %d\n",
		__func__, buf->output_width, buf->output_height);
#else

	u16 vts = res->lines_per_frame;
	u16 hts = res->pixels_per_line;

	dev->vt_pix_clk_freq_mhz = res->pix_clk_freq;
	buf->vt_pix_clk_freq_hz = res->pix_clk_freq;

	/* get integration time */

	buf->coarse_integration_time_min = HM2051_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
				HM2051_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = HM2051_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
				HM2051_FINE_INTG_TIME_MAX_MARGIN;

	/*VTS*/
	hm2051_g_VTS(sd, &vts);
	buf->frame_length_lines = vts;

	/*HTS*/
	hm2051_g_HTS(sd, &hts);
	buf->line_length_pck = hts;

	/* get the cropping and output resolution to ISP for this mode. */
	buf->sensor_output_width = res->width;
	buf->sensor_output_height = res->height;
	buf->binning_factor_x = res->bin_factor_x;
	buf->binning_factor_y = res->bin_factor_y;
	buf->crop_horizontal_start = res->horizontal_start;
	buf->crop_horizontal_end = res->horizontal_end;
	buf->crop_vertical_start = res->vertical_start;
	buf->crop_vertical_end = res->vertical_end;

	pr_debug("%s: output_width %d output_height %d\n",
		__func__, buf->sensor_output_width, buf->sensor_output_height);
#endif
	return 0;
}

static long __hm2051_set_adgain(struct v4l2_subdev *sd,
					int gain, int digitgain) {
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0, iGainBase = 32, iMaxGainX100 = 6169;
	int iGainX100 = (gain*100)/iGainBase;
	int iAgainReg = 0 , iFineAgainReg = 0, iFineAgainBase = 16;
	int iAgainRegPowX100 = 100, iMaxAgainReg = 3, iMaxAgainRegPowX100 = 800;
	int iDgainReg = 0;
	int iMaxDgainX100 = 398, iMinDgainX100 = 100, iDGainBase = 0x40;
	int iTemp;

	if (gain < 0)
		return -EINVAL;

	/* gain = Analog gain * Digitgain*/
	/* [8*(1+15/16)]*[(3.98)] = [31/2]*[3.98] */
	/* Analog gain */
	iGainX100 = iGainX100 > iMaxGainX100 ?
		iMaxGainX100 : (iGainX100 < 100  ? 100 : iGainX100);

	iTemp = iGainX100 / 200;
	while (iTemp > 0) {
		iAgainReg += 1;
		iTemp >>= 1;
		iAgainRegPowX100 *= 2;
	}

	iAgainReg = iAgainReg > iMaxAgainReg ? iMaxAgainReg : iAgainReg;
	iAgainRegPowX100 =  iAgainRegPowX100 > iMaxAgainRegPowX100 ?
				iMaxAgainRegPowX100 : iAgainRegPowX100;
	iTemp = (iFineAgainBase * iGainX100) /
		iAgainRegPowX100 - iFineAgainBase;

	iFineAgainReg = iTemp  < 0 ? 0 :
		(iTemp >= iFineAgainBase ? iFineAgainBase - 1 : iTemp);
	gain = (iAgainReg) + (iFineAgainReg << 4);
	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_AGAIN, gain);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_AGAIN);
		return ret;
	}
	/* Digitgain*/
	/*Digitgain = gain / Analog gain*/
	iGainX100 = (iGainX100 * 100) / (iAgainRegPowX100 +
			(iAgainRegPowX100 * iFineAgainReg) / iFineAgainBase);
	iGainX100 = iGainX100 < iMinDgainX100 ?
		iMinDgainX100 : (iGainX100 > iMaxDgainX100 ?
				iMaxDgainX100 : iGainX100);
	iDgainReg = (iGainX100 * iDGainBase) / 100;
	gain = iDgainReg = (iDgainReg < iDGainBase ? iDGainBase : iDgainReg);

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_DGAIN, gain);

	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_DGAIN);
		return ret;
	}

	return ret;
}

static long __hm2051_set_exposure(struct v4l2_subdev *sd, int coarse_itg,
				 int gain, int digitgain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	u16 vts, hts;
	int ret = 0;
	u16 img_out_conf = 0;

	hts = hm2051_res[dev->fmt_idx].pixels_per_line;
	vts = hm2051_res[dev->fmt_idx].lines_per_frame;

	if (vts < coarse_itg)
		vts = (u16) coarse_itg + HM2051_INTEGRATION_TIME_MARGIN;

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_BLANKING_ROW_H,
		((vts - hm2051_res[dev->fmt_idx].height) & 0xFF00) >> 8);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_REG_BLANKING_ROW_H);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_REG_BLANKING_ROW_L,
		(vts - hm2051_res[dev->fmt_idx].height) & 0x00FF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_REG_BLANKING_ROW_L);
		return ret;
	}

	/* set exposure */
	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_EXPOSURE_H, (coarse_itg >> 8) & 0xFF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_EXPOSURE_H);
		return ret;
	}
	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_EXPOSURE_L, coarse_itg & 0xFF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_EXPOSURE_L);
		return ret;
	}
	/* set AD gain */
	ret = __hm2051_set_adgain(sd, gain, digitgain);
	if (ret) {
		dev_err(&client->dev, "%s: Set adgain error, aborted\n",
			__func__);
		return ret;
	}

	/* Add for sofia blanking sync */
	ret = hm2051_read_reg(client, HM2051_8BIT,
		HM2051_IMG_OUT_CONF, &img_out_conf);

	if (ret)
		dev_err(&client->dev, "HM2051_IMG_OUT_CONF err\n");

	if (coarse_itg <
		(dev->sensor_blanking - 2 +
		hm2051_res[dev->fmt_idx].lines_per_frame)) {
		img_out_conf |= HM2051_FIXED_FRAME_RATE_MASK;
		hm2051_write_reg(client, HM2051_8BIT,
			HM2051_IMG_OUT_CONF, img_out_conf);
	} else {
		img_out_conf &= (~HM2051_FIXED_FRAME_RATE_MASK);
		hm2051_write_reg(client, HM2051_8BIT,
			HM2051_IMG_OUT_CONF, img_out_conf);
	}
	/* End Add for sofia blanking sync */

	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_COMMAND_UPDATE, 1);
	if (ret) {
		dev_err(&client->dev, "%s: Write HM2051_COMMAND_UPDATE %x error, aborted\n",
			__func__, HM2051_COMMAND_UPDATE);
		return ret;
	}
	return ret;
}

static int hm2051_set_exposure(struct v4l2_subdev *sd, int exposure,
	int gain, int digitgain)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __hm2051_set_exposure(sd, exposure, gain, digitgain);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_s_ctrl_exposure(struct v4l2_subdev *sd,
			       int exposure)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	dev->coarse_itg = exposure;

	return __hm2051_set_exposure(sd, dev->coarse_itg, dev->gain, 0x40);
}

static int hm2051_s_ctrl_gain(struct v4l2_subdev *sd,
			       int gain)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	dev->gain = gain;

	return __hm2051_set_exposure(sd, dev->coarse_itg, dev->gain, 0x40);
}

static long hm2051_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	u16 coarse_itg = exposure->integration_time[0];
	u16 analog_gain = exposure->gain[0];
	u16 digital_gain = exposure->gain[1];
#if 0
	/* we should not accept the invalid value below */
	if (analog_gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}
#endif
	return hm2051_set_exposure(sd, coarse_itg, analog_gain, digital_gain);
}

static long hm2051_g_sensor_mode_data(struct v4l2_subdev *sd,
			       void *arg)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	ret = hm2051_get_intg_factor(client, arg,
		&hm2051_res[dev->fmt_idx]);
	if (ret)
		dev_err(&client->dev, "failed to get integration_factor\n");
	return ret;
}

static long hm2051_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return hm2051_s_exposure(sd, arg);
	case INTEL_VIDIOC_SENSOR_MODE_DATA:
		return hm2051_g_sensor_mode_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int hm2051_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg_v, reg_v2;
	int ret;
	/* get exposure */
	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_EXPOSURE_L,
					&reg_v);
	if (ret)
		goto err;

	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_EXPOSURE_H,
					&reg_v2);
	if (ret)
		goto err;

	*value = reg_v + ((u32)reg_v2 << 8);
err:
	return ret;
}

struct hm2051_control hm2051_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "gain",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.tweak = hm2051_s_ctrl_gain,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.tweak = hm2051_s_ctrl_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = hm2051_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = HM2051_FOCAL_LENGTH_DEFAULT,
			.maximum = HM2051_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = HM2051_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = hm2051_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = HM2051_F_NUMBER_DEFAULT,
			.maximum = HM2051_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = HM2051_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = hm2051_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = HM2051_F_NUMBER_RANGE,
			.maximum =  HM2051_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = HM2051_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = hm2051_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = HM2051_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2051_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = HM2051_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2051_g_bin_factor_y,
	},
	{
		.qc = {
			.id = INTEL_V4L2_CID_VBLANKING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical blanking",
			.minimum = 0,
			.maximum = 5000,
			.step = 1,
			.default_value = 2000,
			.flags = 0,
		},
		.query = hm2051_g_vblanking,
	},
};
#define N_CONTROLS (ARRAY_SIZE(hm2051_controls))

static struct hm2051_control *hm2051_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (hm2051_controls[i].qc.id == id)
			return &hm2051_controls[i];
	return NULL;
}

static int hm2051_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct hm2051_control *ctrl = hm2051_find_control(qc->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* imx control set/get */
static int hm2051_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2051_control *s_ctrl;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = hm2051_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2051_control *octrl = hm2051_find_control(ctrl->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_s_ext_ctrls(struct v4l2_subdev *sd,
				 struct v4l2_ext_controls *ctrls)
{
	int ret = 0;

	/* Handles only exposure and gain together special case. */
	if (ctrls->count == 1 &&
		(ctrls->controls[0].id == V4L2_CID_HFLIP ||
		ctrls->controls[0].id == V4L2_CID_VFLIP))
		ret = 0;
	else if (ctrls->count == 2 &&
		(ctrls->controls[0].id == V4L2_CID_GAIN &&
		ctrls->controls[1].id == V4L2_CID_EXPOSURE))
		ret = hm2051_set_exposure(sd, ctrls->controls[1].value,
						ctrls->controls[0].value, 0x40);
	else if (ctrls->count == 2 &&
		(ctrls->controls[1].id == V4L2_CID_GAIN &&
		ctrls->controls[0].id == V4L2_CID_EXPOSURE))
		ret = hm2051_set_exposure(sd, ctrls->controls[0].value,
						ctrls->controls[1].value, 0x40);
	else
		ret = -EINVAL;

	if (IS_ERR_VALUE(ret))
		pltfrm_camera_module_pr_err(sd,
			"failed with error (%d)\n", ret);

	return ret;
}

static int hm2051_init(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	pr_debug("%s\n", __func__);
	mutex_lock(&dev->input_lock);

	/* restore settings */
	hm2051_res = hm2051_res_preview;


	N_RES = N_RES_PREVIEW;

	mutex_unlock(&dev->input_lock);

	return 0;
}


static int power_up(struct v4l2_subdev *sd)
{
/*	struct hm2051_device *dev = to_hm2051_sensor(sd); */
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

#if 0
	if (NULL == dev->platform_data) {
		dev_err(&client->dev,
			"no camera_sensor_platform_data");
		return -ENODEV;
	}

	pr_info("%s\n", __func__);
	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* according to DS, at least 5ms is needed between DOVDD and PWDN */
	usleep_range(5000, 6000);

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 1);
		if (ret)
			goto fail_power;
	}

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* according to DS, 20ms is needed between PWDN and i2c access */
	msleep(20);

	return 0;

fail_clk:
	dev->platform_data->gpio_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
#else

	/* pull low reset first */
	ret = pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	/* power control */
	ret = pltfrm_camera_module_s_power(sd, 1);
	if (ret) {
		dev_err(&client->dev, "power_ctrl failed\n");
		goto fail_power;
	}

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_DVDD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	/* Reset control */
	ret |= pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

	/* Pull low suspend */
	ret |= pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

	if (ret) {
		pltfrm_camera_module_pr_err(sd, "failed with error %d\n", ret);
		goto fail_power;
	}

	/* delay time for first i2c command */
	msleep(20);

	pltfrm_camera_module_pr_debug(sd, "sensor power-up done.\n");
	return ret;

fail_power:
	pltfrm_camera_module_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;

#endif
}

static int power_down(struct v4l2_subdev *sd)
{
/*	struct hm2051_device *dev = to_hm2051_sensor(sd); */
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

#if 0
	if (NULL == dev->platform_data) {
		dev_err(&client->dev,
			"no camera_sensor_platform_data");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 0);
		if (ret)
			dev_err(&client->dev, "gpio failed 2\n");
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");
#else


	/* pull low reset first */
	ret = pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	msleep(20);

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_DVDD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

	/* power control */
	ret = pltfrm_camera_module_s_power(sd, 0);


	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

#endif
	return ret;
}

static int hm2051_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (on == 0)
		return power_down(sd);
	else {
		ret = power_up(sd);

	ret = hm2051_write_reg_array(client, hm2051_global_setting);
	if (ret) {
		dev_err(&client->dev, "hm2051 write register err(global).\n");
		return ret;
	}

		if (!ret)
			return hm2051_init(sd);
	}
	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 800
static int distance(struct hm2051_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << 13)/w);
	unsigned int h_ratio;
	int match;

	if (h == 0)
		return -1;
	h_ratio = ((res->height << 13) / h);
	if (h_ratio == 0)
		return -1;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct hm2051_resolution *tmp_res = NULL;


	for (i = 0; i < N_RES; i++) {
		tmp_res = &hm2051_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != hm2051_res[i].width)
			continue;
		if (h != hm2051_res[i].height)
			continue;

		return i;
	}

	return -1;
}

static int hm2051_try_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;
	idx = nearest_resolution_index(fmt->width,
					fmt->height);

	/* return the largest resolution */
	if (idx == -1)
		idx = 0;

	fmt->width = hm2051_res[idx].width;
	fmt->height = hm2051_res[idx].height;
	fmt->code = HM2051_BAYER_ORDER;

	return 0;
}

/* TODO: remove it. */
static int startup(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (ret) {
		dev_err(&client->dev, "hm2051 reset err.\n");
		return ret;
	}
#if 0
	ret = hm2051_write_reg_array(client, hm2051_global_setting);
	if (ret) {
		dev_err(&client->dev, "hm2051 write register err(global).\n");
		return ret;
	}
#endif
	ret = hm2051_write_reg_array(client, hm2051_res[dev->fmt_idx].regs);
	if (ret) {
		dev_err(&client->dev, "hm2051 write register err(fmt_idx).\n");
		return ret;
	}

	return ret;
}

static int hm2051_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
#if 0
	struct camera_mipi_info *hm2051_info = NULL;

	hm2051_info = v4l2_get_subdev_hostdata(sd);
	if (hm2051_info == NULL)
		return -EINVAL;
#endif
	mutex_lock(&dev->input_lock);
	ret = hm2051_try_mbus_fmt(sd, fmt);
	if (ret == -1) {
		dev_err(&client->dev, "try fmt fail\n");
		goto err;
	}

	dev->fmt_idx = get_resolution_index(fmt->width,
					      fmt->height);
	if (dev->fmt_idx == -1) {
		dev_err(&client->dev, "get resolution fail\n");
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	ret = startup(sd);
	if (ret)
		dev_err(&client->dev, "hm2051 startup err\n");

	/* Add for sofia blanking sync */
	dev->sensor_blanking = 0x33;

#if 0
	ret = hm2051_get_intg_factor(client, hm2051_info,
		&hm2051_res[dev->fmt_idx]);
	if (ret) {
		dev_err(&client->dev, "failed to get integration_factor\n");
		goto err;
	}
#endif

err:
	mutex_unlock(&dev->input_lock);
	return ret;
}
static int hm2051_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (!fmt)
		return -EINVAL;

	fmt->width = hm2051_res[dev->fmt_idx].width;
	fmt->height = hm2051_res[dev->fmt_idx].height;
	fmt->code = HM2051_BAYER_ORDER;

	return 0;
}

static int hm2051_detect(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

#if 1
	/* Add by marky to check sensor ids */
	msleep(100);
	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_SC_CMMN_CHIP_ID_H, &high);


	if (ret) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}

	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_SC_CMMN_CHIP_ID_L, &low);
	id = ((((u16) high) << 8) | (u16) low);

	if (id != HM2051_ID) {
		dev_err(&client->dev, "sensor ID error 0x%x\n", id);
		return -ENODEV;
	}
#endif

	dev_dbg(&client->dev, "detect hm2051 success\n");
	return 0;
}

static int hm2051_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	mutex_lock(&dev->input_lock);
  #if ULPM_PROCEDURE
	ret = hm2051_write_reg_array(client, enable ?
			hm2051_stream_on : hm2051_stream_off);
#else
	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_SW_STREAM,
				enable ? HM2051_START_STREAMING :
				HM2051_STOP_STREAMING);
#endif
	if (enable == 0) {
		int pclk;
		int wait_ms = 0;
		u16 vts = hm2051_res[dev->fmt_idx].lines_per_frame;
		u16 hts = hm2051_res[dev->fmt_idx].pixels_per_line;

		pclk = hm2051_res[dev->fmt_idx].pix_clk_freq / 1000;

		if (!pclk)
			goto err;

		ret = hm2051_g_VTS(sd, &vts);
		if (IS_ERR_VALUE(ret))
			goto err;

		ret = hm2051_g_HTS(sd, &hts);
		if (IS_ERR_VALUE(ret))
			goto err;

		wait_ms = (hts * vts) / pclk;
		dev_dbg(&client->dev,
			"stream off wait%d=%d*%d/%d\n",
			wait_ms, hts, vts, pclk);

		/* wait for a frame period to make sure that there is
			no pending frame left. */

		msleep(wait_ms + 1);
	}

	mutex_unlock(&dev->input_lock);

	return ret;
err:
	msleep(133);
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* hm2051 enum frame size, frame intervals */
static int hm2051_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = hm2051_res[index].width;
	fsize->discrete.height = hm2051_res[index].height;
	fsize->reserved[0] = hm2051_res[index].used;

	return 0;
}

static int hm2051_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = hm2051_res[index].width;
	fival->height = hm2051_res[index].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = hm2051_res[index].fps;
	fival->pixel_format = HM2051_BAYER_ORDER;

	return 0;
}

static int hm2051_enum_mbus_fmt(struct v4l2_subdev *sd,
				unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	*code = HM2051_BAYER_ORDER;

	return 0;
}

static int hm2051_s_config(struct v4l2_subdev *sd,
			   int irq, void *platform_data)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&dev->input_lock);
	/* power off the module, then power on it in future
	 * as first power on by board may not fulfill the
	 * power on sequqence needed by the module
	 */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "hm2051 power-off err.\n");
		goto fail_power_off;
	}

	ret = power_up(sd);
	if (ret) {
		dev_err(&client->dev, "hm2051 power-up err.\n");
		goto fail_power_on;
	}

#if 0
	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;
#endif

	/* config & detect sensor */
	ret = hm2051_detect(client);
	if (ret) {
		dev_err(&client->dev, "hm2051_detect err s_config.\n");
		goto fail_csi_cfg;
	}

	/* turn off sensor, after probed */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "hm2051 power-off err.\n");
		goto fail_csi_cfg;
	}
	mutex_unlock(&dev->input_lock);

	return 0;

fail_csi_cfg:
fail_power_on:
	power_down(sd);
	dev_err(&client->dev, "sensor power-gating failed\n");
fail_power_off:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int hm2051_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	if (!param)
		return -EINVAL;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_err(&client->dev,  "unsupported buffer type.\n");
		return -EINVAL;
	}

	memset(param, 0, sizeof(*param));
	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (dev->fmt_idx >= 0 && dev->fmt_idx < N_RES) {
		param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		param->parm.capture.timeperframe.numerator = 1;
		param->parm.capture.capturemode = dev->run_mode;
		param->parm.capture.timeperframe.denominator =
			hm2051_res[dev->fmt_idx].fps;
	}
	return 0;
}

static int hm2051_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		hm2051_res = hm2051_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		hm2051_res = hm2051_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		hm2051_res = hm2051_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int hm2051_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	interval->interval.numerator = 1;
	interval->interval.denominator = hm2051_res[dev->fmt_idx].fps;

	return 0;
}

static int hm2051_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	return 0;
}

static int hm2051_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;

	code->code = HM2051_BAYER_ORDER;
	return 0;
}

static int hm2051_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = hm2051_res[index].width;
	fse->min_height = hm2051_res[index].height;
	fse->max_width = hm2051_res[index].width;
	fse->max_height = hm2051_res[index].height;

	return 0;

}

static struct v4l2_mbus_framefmt *
__hm2051_get_pad_format(struct hm2051_device *sensor,
			struct v4l2_subdev_fh *fh, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,
			"__hm2051_get_pad_format err. pad %x\n", pad);
		return NULL;
	}

	switch (which) {
#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
#endif
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int hm2051_get_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct hm2051_device *snr = to_hm2051_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__hm2051_get_pad_format(snr, fh, fmt->pad, fmt->which);
	if (!format)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int hm2051_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct hm2051_device *snr = to_hm2051_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int hm2051_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	mutex_lock(&dev->input_lock);
	*frames = hm2051_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_sensor_ops hm2051_sensor_ops = {
	.g_skip_frames = hm2051_g_skip_frames,
};

static const struct v4l2_subdev_video_ops hm2051_video_ops = {
	.s_stream = hm2051_s_stream,
	.g_parm = hm2051_g_parm,
	.s_parm = hm2051_s_parm,
	.enum_framesizes = hm2051_enum_framesizes,
	.enum_frameintervals = hm2051_enum_frameintervals,
	.enum_mbus_fmt = hm2051_enum_mbus_fmt,
	.try_mbus_fmt = hm2051_try_mbus_fmt,
	.g_mbus_fmt = hm2051_g_mbus_fmt,
	.s_mbus_fmt = hm2051_s_mbus_fmt,
	.g_frame_interval = hm2051_g_frame_interval,
	.s_frame_interval = hm2051_s_frame_interval,
};

static const struct v4l2_subdev_core_ops hm2051_core_ops = {
	.s_power = hm2051_s_power,
	.queryctrl = hm2051_queryctrl,
	.g_ctrl = hm2051_g_ctrl,
	.s_ctrl = hm2051_s_ctrl,
	.s_ext_ctrls = hm2051_s_ext_ctrls,
	.ioctl = hm2051_ioctl,
};

static const struct v4l2_subdev_pad_ops hm2051_pad_ops = {
	.enum_mbus_code = hm2051_enum_mbus_code,
	.enum_frame_size = hm2051_enum_frame_size,
	.get_fmt = hm2051_get_pad_format,
	.set_fmt = hm2051_set_pad_format,
};

static const struct v4l2_subdev_ops hm2051_ops = {
	.core = &hm2051_core_ops,
	.video = &hm2051_video_ops,
	.pad = &hm2051_pad_ops,
	.sensor = &hm2051_sensor_ops,
};

static int hm2051_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	dev_dbg(&client->dev, "hm2051_remove...\n");

	pltfrm_camera_module_release(sd);
	devm_kfree(&(client->dev), dev->platform_data);
	v4l2_device_unregister_subdev(sd);
	kfree(dev);

	return 0;
}

static int hm2051_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hm2051_device *dev;
	int ret = 0;
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &hm2051_ops);


#if 1
	ret = pltfrm_camera_module_init(&(dev->sd),
			(void **)&(dev->platform_data));
	if (ret) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"failed with error %d\n", ret);
		hm2051_remove(client);
		return ret;
	}

	ret = hm2051_s_config(&dev->sd, client->irq,
				   client->dev.platform_data);
	if (ret) {
		pr_info("[Progress][%s] Probe fails\n", HM2051_NAME);
		hm2051_remove(client);
	}

	return ret;

#else
	if (client->dev.platform_data) {
		ret = hm2051_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret)
			goto out_free;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = HM2051_BAYER_ORDER;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		hm2051_remove(client);
	return ret;
out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
#endif
}

static struct of_device_id hm2051_of_match[] = {
	{.compatible = "himax," HM2051_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, hm2051_id);
static struct i2c_driver hm2051_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = HM2051_NAME,
		.of_match_table = hm2051_of_match
	},
	.probe = hm2051_probe,
	.remove = hm2051_remove,
	.id_table = hm2051_id,
};

#if 0
static int init_hm2051(void)
{
	return i2c_add_driver(&hm2051_driver);
}

static void exit_hm2051(void)
{

	i2c_del_driver(&hm2051_driver);
}

module_init(init_hm2051);
module_exit(exit_hm2051);
#else
module_i2c_driver(hm2051_driver);
#endif

MODULE_AUTHOR("Chris Kan <chris.kan@intel.com>");
MODULE_DESCRIPTION("A low-level driver for Himax hm2051 sensors");
MODULE_LICENSE("GPL");
