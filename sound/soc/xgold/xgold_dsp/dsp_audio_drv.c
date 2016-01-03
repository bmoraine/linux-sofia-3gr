/*
 * Component: Intel XGOLD DSP Driver
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


#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <sound/soc.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device_pm.h>
#include <linux/reboot.h>

#include "xgold_machine.h"
#include "dsp_audio_platform.h"
#include "dsp_audio_driverif.h"
#include "aud_lib_dsp_internal.h"
#include "dsp_audio_internal.h"


#define MAX_DSP_CMD_LEN_BYTES		0x70
#define PROP_DSP_INT_IMSC		"intel,dsp-imsc"
#define PROP_DSP_INT_ICR		"intel,dsp-icr"
#define PROP_DSP_INT_MIS		"intel,dsp-mis"
#define PROP_DSP_CF			"intel,dsp-cf"
#define PROP_DSP_REMAIN_PCM		"intel,dsp-pcm-offset"
#define PROP_DSP_BUF_SIZE_UL_OFFSET	"intel,dsp-buf-size-ul-offset"
#define PROP_DSP_SM_AUD_BUF_DL_OFFSET	"intel,dsp-sm-aud-buf-dl-offset"
#define PROP_DSP_SM_AUD_BUF_DL2_OFFSET	"intel,dsp-sm-aud-buf-dl2-offset"
#define PROP_DSP_SM_AUD_BUF_UL_OFFSET	"intel,dsp-sm-aud-buf-ul-offset"
#define PROP_DSP_CMD_GAIN_CONST		"intel,dsp-cmd-gain_const"
#define PROP_DSP_CMD_SWM_PCM_OUT	"intel,dsp-cmd-swm_pcmout"
#define PROP_DSP_CMD_SWM_AFE_OUT	"intel,dsp-cmd-swm_afeout"
#define PROP_DSP_CMD_SET_IIR_PCM_LEFT	"intel,dsp-cmd-pcm_iir_left"
#define PROP_DSP_CMD_SET_IIR_PCM_RIGHT	"intel,dsp-cmd-pcm_iir_right"
#define PROP_DSP_CMD_SET_IIR_AFE_TX	"intel,dsp-cmd-afe_tx_iir"
#define PROP_DSP_CMD_SET_GAIN		"intel,dsp-cmd-pcm_gain"
#define PROP_DSP_CMD_SET_MIX_MATRIX	"intel,dsp-cmd-mix_matrix"
#define PROP_DSP_SM_HW_PROBE_A_OFFSET	"intel,dsp-sm-buf_sm_hw_probe_a_offset"
#define PROP_DSP_SM_HW_PROBE_B_OFFSET	"intel,dsp-sm-buf_sm_hw_probe_b_offset"
#define PROP_DSP_SM_SPEECH_BUFFER_1	"intel,dsp-sm-buf-speech-buffer_1"
#define PROP_DSP_SM_SPEECH_BUFFER_2	"intel,dsp-sm-buf-speech-buffer_2"
#define PROP_DSP_SM_SPEECH_BUFFER_3	"intel,dsp-sm-buf-speech-buffer_3"
#define PROP_DSP_SM_SPEECH_BUFFER_4	"intel,dsp-sm-buf-speech-buffer_4"
#define PROP_DSP_SM_SPEECH_BUFFER_5	"intel,dsp-sm-buf-speech-buffer_5"
#define PROP_DSP_SM_SPEECH_BUFFER_6	"intel,dsp-sm-buf-speech-buffer_6"
#define PROP_DSP_CMD_ENA_I2S2_TX	"intel,dsp-cmd-i2s2_tx"
#define PROP_DSP_CMD_ENA_I2S2_RX	"intel,dsp-cm-i2s2_rx"



#define	xgold_err(fmt, arg...) \
		pr_err("snd: dsp: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: dsp: "fmt, ##arg)

#define	xgold_dsp_log(fmt, arg...) \
		pr_debug("snd: dsp: "fmt, ##arg)

struct rw_semaphore dsp_audio_cb_rwsem;

/* List of interrupt callback registrations */
struct dsp_audio_cb_list {
	void (*p_func)(void *);
	void *dev;
} g_dsp_audio_cb_list[DSP_LISR_CB_END] = {{0,} };

/* dsp device */
/*FIXME: remove global variables */
static struct dsp_audio_device *g_dsp_audio_dev;
static struct dsp_common_data *p_dsp_common_data;
static LIST_HEAD(list_dsp);

static inline int i2s_set_pinctrl_state(struct dsp_i2s_device *i2s_dev,
		struct pinctrl_state *state)
{
	int ret = 0;

	if (!i2s_dev) {
		xgold_err("%s: Unable to retrive i2s device data\n", __func__);
		return -EINVAL;
	}
	if (!IS_ERR_OR_NULL(i2s_dev->pinctrl)) {
		if (!IS_ERR_OR_NULL(state)) {
			ret = pinctrl_select_state(i2s_dev->pinctrl, state);
			if (ret)
				dev_err(i2s_dev->dev, "%s %d: could not set pins\n",
						__func__, __LINE__);
		}
	}
	return ret;
}

static void dsp_set_i2s_power_state(struct dsp_i2s_device *i2s_dev,
			bool state)
{
	int ret = 0;
	xgold_debug("%s: state %d\n", __func__, state);

	if (state == 1) {
#ifdef CONFIG_PLATFORM_DEVICE_PM
		/* Enable I2S Power and clock domains */
		if (i2s_dev->pm_platdata) {
			ret = device_state_pm_set_state_by_name(
				i2s_dev->dev,
				i2s_dev->pm_platdata->pm_state_D0_name);
			if (ret < 0)
				xgold_err("%s: failed to set PM state error %d\n",
					__func__, ret);
			udelay(5);
		}
#endif
		/* Enable XGOLD I2S pins */
		ret = i2s_set_pinctrl_state(i2s_dev, i2s_dev->pins_default);
		if (ret < 0)
			xgold_err("%s: failed to set pinctrl state %d\n",
				__func__, ret);
	} else {

		/* Disable i2s pins */
		ret = i2s_set_pinctrl_state(i2s_dev, i2s_dev->pins_inactive);

		if (ret < 0)
			xgold_err("%s: failed to set pinctrl state %d\n",
			__func__, ret);
#ifdef CONFIG_PLATFORM_DEVICE_PM
		/* Disable I2S Power and clock domains */
		if (i2s_dev->pm_platdata) {
			ret = device_state_pm_set_state_by_name(
				i2s_dev->dev,
				i2s_dev->pm_platdata->pm_state_D3_name);
			if (ret < 0)
				xgold_err("%s: failed to set PM state error %d",
				__func__, ret);
		}
#endif
	}
}

/* Function to register the callbacks for interrupt type of enum dsp_lisr_cb */
int register_dsp_audio_lisr_cb(enum dsp_lisr_cb lisr_type,
	void (*p_func)(void *), void *dev)
{
	if (lisr_type >= DSP_LISR_CB_END)
		return -EINVAL;

	xgold_debug("%segistering lisr_cb: %d\n",
			(NULL == p_func ? "De-r" : "R"), lisr_type);
	down_write(&dsp_audio_cb_rwsem);
	g_dsp_audio_cb_list[lisr_type].p_func = p_func;
	g_dsp_audio_cb_list[lisr_type].dev = dev;
	up_write(&dsp_audio_cb_rwsem);
	return 0;
}

/* Function to invoke the registered callbacks for interrupts */
static void dsp_audio_hisr_cb_handler(enum dsp_irq_no intr_no,
				      void *dsp_dev)
{
	int comm_flag = 0;
	int i = 0;
	static int debug_log_cnt = 9;
	bool comm_flag_clear[DSP_IRQ_COMM_FLAG_END] = {false};
	const struct dsp_lisr_cb_conf *p_dsp_aud_lisr_cb;

	debug_log_cnt++;
	if (debug_log_cnt == 10)
		xgold_debug("enter func - intr no: %d\n", intr_no);

	down_read(&dsp_audio_cb_rwsem);
	p_dsp_aud_lisr_cb = dsp_audio_get_lisr_cb(intr_no);

	/* find the set communication flag and call the registered
	 * callback function */
	if (p_dsp_aud_lisr_cb == NULL) {
		xgold_debug("p_dsp_aud_lisr_cb == NULL\n");
		up_read(&dsp_audio_cb_rwsem);
		return;
	}

	while (p_dsp_aud_lisr_cb[i].lisr_cb_type != DSP_LISR_CB_END) {
		/* only read comm flag if a cb function has been registered
		 * for the current cb type */
		if (g_dsp_audio_cb_list
			[p_dsp_aud_lisr_cb[i].lisr_cb_type].p_func != NULL) {
			comm_flag = dsp_read_audio_dsp_communication_flag(
				(struct dsp_audio_device *)dsp_dev,
				p_dsp_aud_lisr_cb[i].comm_flag_no);
			if (debug_log_cnt == 10)
				xgold_debug(
					"%d type %d flag_no %d comm_flag %d\n",
					i, p_dsp_aud_lisr_cb[i].lisr_cb_type,
					p_dsp_aud_lisr_cb[i].comm_flag_no,
					comm_flag);
			/* TODO:
			 * Support interrupts w/o comm flag (lte voice memo)
			 * maybe use
			 * p_dsp_aud_lisr_cb[i].lisr_cb_type.comm_flag_no
			 * with wildcard flag ... */
			if (comm_flag) {
				/* mark comm flag to be cleared after while
				 * loop ends */
				comm_flag_clear[p_dsp_aud_lisr_cb[i].
					comm_flag_no] = true;
				if (debug_log_cnt == 10)
					xgold_debug("call lisr_cb_type: %d\n",
							p_dsp_aud_lisr_cb[i].
							lisr_cb_type);
				(g_dsp_audio_cb_list
				 [p_dsp_aud_lisr_cb[i].lisr_cb_type].
				 p_func) (g_dsp_audio_cb_list
					[p_dsp_aud_lisr_cb[i].
					lisr_cb_type].dev);
			}
		}
		i++;
	}

	/* clear the relevant comm flags now */
	for (i = 0; i < DSP_IRQ_COMM_FLAG_END; i++) {
		if (true == comm_flag_clear[i]) {
			dsp_reset_audio_dsp_communication_flag(
				(struct dsp_audio_device *)dsp_dev,
				i);
			if (debug_log_cnt == 10)
				xgold_debug("cleared comm flag: %d\n", i);
		}
	}

	if (debug_log_cnt == 10) {
		xgold_debug("exit func\n");
		debug_log_cnt = 0;
	}

	up_read(&dsp_audio_cb_rwsem);
}

/* LISR for DSP_INT1 */
static irqreturn_t dsp_audio_int1_lisr(int irq, void *dsp_dev)
{
	xgold_debug("enter func\n");
	/* clear the interrupt */
	dsp_audio_irq_ack((struct dsp_audio_device *)dsp_dev, DSP_IRQ_1);

	return IRQ_WAKE_THREAD;
}

/* LISR for DSP_INT2 */
static irqreturn_t dsp_audio_int2_lisr(int irq, void *dsp_dev)
{
	xgold_debug("enter func\n");
	/* clear the interrupt */
	dsp_audio_irq_ack((struct dsp_audio_device *)dsp_dev, DSP_IRQ_2);

	return IRQ_WAKE_THREAD;
}

/* HISR for DSP_INT1 */
static irqreturn_t dsp_audio_int1_hisr(int irq, void *dev)
{
	xgold_debug("enter func\n");
	dsp_audio_hisr_cb_handler(DSP_IRQ_1, dev);
	return IRQ_HANDLED;
}

/* HISR for DSP_INT2 */
static irqreturn_t dsp_audio_int2_hisr(int irq, void *dev)
{
	xgold_debug("enter func\n");
	dsp_audio_hisr_cb_handler(DSP_IRQ_2, dev);
	return IRQ_HANDLED;
}

/* LISR for DSP_INT3 */
static irqreturn_t dsp_audio_int3_lisr(int irq, void *dsp_dev)
{
	xgold_debug("enter func\n");
	/* clear the interrupt */
	dsp_audio_irq_ack((struct dsp_audio_device *)dsp_dev, DSP_IRQ_3);

	return IRQ_WAKE_THREAD;
}

/* HISR for DSP_INT3 */
static irqreturn_t dsp_audio_int3_hisr(int irq, void *dev)
{
	xgold_debug("enter func\n");
	dsp_audio_hisr_cb_handler(DSP_IRQ_3, dev);
	return IRQ_HANDLED;
}

/* LISR for DSP_FBA_INT2 */
static irqreturn_t dsp_audio_fba_int2_lisr(int irq, void *dsp_dev)
{
	xgold_debug("enter func\n");
	/* clear the interrupt */
	dsp_audio_irq_ack((struct dsp_audio_device *)dsp_dev, DSP_FBA_IRQ_2);

	return IRQ_WAKE_THREAD;
}

/* HISR for DSP_FBA_INT2 */
static irqreturn_t dsp_audio_fba_int2_hisr(int irq, void *dev)
{
	xgold_debug("enter func\n");
	dsp_audio_hisr_cb_handler(DSP_FBA_IRQ_2, dev);
	return IRQ_HANDLED;
}

/* open call for the device */
static int dsp_audio_dev_open(void)
{
	int retval = 0;

	xgold_debug("dsp_audio_dev_open, runtime_pm_get\n");
	/* ToDo: handle run time PM */
#if 0
	pm_runtime_get_sync(g_dsp_audio_dev->pdev->dev);
#endif
	return retval;
}

/* close call for the device */
static int dsp_audio_dev_close(void)
{
	int retval = 0;

	xgold_debug("dsp_audio_dev_close, runtime_pm_put\n");
	/* ToDo: handle run time PM */
#if 0
	pm_runtime_put(g_dsp_audio_dev->pdev->dev);
#endif

	return retval;
}

/* Check if the command is to turn on */
/* This is a variant function for xg632.
 * Consider creating a variant file for future platforms */
static void dsp_audio_mark_scheduler_status(struct dsp_aud_cmd_data *p_cmd_data)
{
	#define PARM_ON 1
	#define PARM_UPDATE 2
	#define PARM_ON_UPDATE 3

	unsigned short *data_trace_ptr;
	xgold_debug("%s:\n", __func__);

	if ((NULL != p_cmd_data) && (NULL != p_cmd_data->p_data)) {
		data_trace_ptr = (unsigned short *)p_cmd_data->p_data;
		/* Check if the command is to start dsp scheduler */
		if (DSP_AUDIO_CMD_VB_HW_AFE == p_cmd_data->command_id) {
			if ((PARM_ON == data_trace_ptr[0]) ||
				(PARM_UPDATE == data_trace_ptr[0]) ||
				(PARM_ON_UPDATE == data_trace_ptr[0])) {
				g_dsp_audio_dev->dsp_sched_start = 1;
				xgold_debug("dsp scheduler marked as started\n");
			} else {
				g_dsp_audio_dev->dsp_sched_start = 0;
				xgold_debug("dsp scheduler marked as stopped\n");
			}
		}
	}
}

static int dsp_audio_xgold_set_pcm_path(bool pcm_dir)
{
	int ret_val = 0;
	unsigned cmd_len;
	unsigned short *p_dsp_cmd;

	p_dsp_cmd = kzalloc(MAX_DSP_CMD_LEN_BYTES, GFP_KERNEL);

	xgold_debug(" pcm_dir %d\n", pcm_dir);

	if (p_dsp_cmd == NULL) {
		xgold_err("Failed to allocate memory for DSP cmds\n");
		return -ENOMEM;
	}

	/* Program Gain constant in firmware */
	of_find_property(g_dsp_audio_dev->dev->of_node,
			PROP_DSP_CMD_GAIN_CONST,
			&cmd_len);

	ret_val =
		of_property_read_u16_array(g_dsp_audio_dev->dev->of_node,
					PROP_DSP_CMD_GAIN_CONST,
					p_dsp_cmd,
					cmd_len/2);

	if (ret_val != 0) {
		xgold_err("read %s property failed with error %d\n",
		PROP_DSP_CMD_GAIN_CONST, ret_val);
	} else
		(void)dsp_audio_cmd(*p_dsp_cmd,
						*(p_dsp_cmd + 1),
						p_dsp_cmd + 2);

	if (pcm_dir == 0) {
		/* Program PCM_IN->AFE_OUT SWM connection */
		of_find_property(g_dsp_audio_dev->dev->of_node,
			PROP_DSP_CMD_SWM_AFE_OUT,
			&cmd_len);

		ret_val =
			of_property_read_u16_array(
					g_dsp_audio_dev->dev->of_node,
					PROP_DSP_CMD_SWM_AFE_OUT,
					p_dsp_cmd,
					cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_SWM_AFE_OUT, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

		/*Switch off IIR filters in PCM Left path */
		of_find_property(g_dsp_audio_dev->dev->of_node,
			PROP_DSP_CMD_SET_IIR_PCM_LEFT,
			&cmd_len);

		ret_val =
			of_property_read_u16_array(
						g_dsp_audio_dev->dev->of_node,
						PROP_DSP_CMD_SET_IIR_PCM_LEFT,
						p_dsp_cmd,
						cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_SET_IIR_PCM_LEFT, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

		/*Switch off IIR filters in PCM RIGHT path */
		of_find_property(g_dsp_audio_dev->dev->of_node,
				PROP_DSP_CMD_SET_IIR_PCM_RIGHT,
				&cmd_len);

		ret_val =
			of_property_read_u16_array(
						g_dsp_audio_dev->dev->of_node,
						PROP_DSP_CMD_SET_IIR_PCM_RIGHT,
						p_dsp_cmd,
						cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_SET_IIR_PCM_RIGHT, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

		/*Enable I2S2 in playback path */
		of_find_property(g_dsp_audio_dev->dev->of_node,
				PROP_DSP_CMD_ENA_I2S2_TX,
				&cmd_len);

		ret_val =
			of_property_read_u16_array(
						g_dsp_audio_dev->dev->of_node,
						PROP_DSP_CMD_ENA_I2S2_TX,
						p_dsp_cmd,
						cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_ENA_I2S2_TX, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

	} else {

		/* Program AFE_IN->PCM_OUT SWM connection */
		of_find_property(g_dsp_audio_dev->dev->of_node,
			PROP_DSP_CMD_SWM_PCM_OUT,
			&cmd_len);

		ret_val =
			of_property_read_u16_array(
						g_dsp_audio_dev->dev->of_node,
						PROP_DSP_CMD_SWM_PCM_OUT,
						p_dsp_cmd,
						cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_SWM_PCM_OUT, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

		/*Switch off IIR filters in AFE TX path */
		of_find_property(g_dsp_audio_dev->dev->of_node,
				PROP_DSP_CMD_SET_IIR_AFE_TX,
				&cmd_len);

		ret_val =
			of_property_read_u16_array(
						g_dsp_audio_dev->dev->of_node,
						PROP_DSP_CMD_SET_IIR_AFE_TX,
						p_dsp_cmd,
						cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_SET_IIR_AFE_TX, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

		/*Enable I2S2 RX*/
		of_find_property(g_dsp_audio_dev->dev->of_node,
				PROP_DSP_CMD_ENA_I2S2_RX,
				&cmd_len);

		ret_val =
			of_property_read_u16_array(
						g_dsp_audio_dev->dev->of_node,
						PROP_DSP_CMD_ENA_I2S2_RX,
						p_dsp_cmd,
						cmd_len/2);

		if (ret_val != 0) {
			xgold_err("read %s property failed with error %d\n",
			PROP_DSP_CMD_ENA_I2S2_RX, ret_val);
		} else
			(void)dsp_audio_cmd(*p_dsp_cmd,
							*(p_dsp_cmd + 1),
							p_dsp_cmd + 2);

	}

	/*Unmute all gain cells in DSP*/
	of_find_property(g_dsp_audio_dev->dev->of_node,
			PROP_DSP_CMD_SET_GAIN,
			&cmd_len);

	ret_val =
		of_property_read_u16_array(
					g_dsp_audio_dev->dev->of_node,
					PROP_DSP_CMD_SET_GAIN,
					p_dsp_cmd,
					cmd_len/2);

	if (ret_val != 0) {
		xgold_err("read %s property failed with error %d\n",
		PROP_DSP_CMD_SET_GAIN, ret_val);
	} else
		(void)dsp_audio_cmd(*p_dsp_cmd,
						*(p_dsp_cmd + 1),
						p_dsp_cmd + 2);

	/*Set mix Matrix in DSP*/
	of_find_property(g_dsp_audio_dev->dev->of_node,
		PROP_DSP_CMD_SET_MIX_MATRIX,
		&cmd_len);

	ret_val =
		of_property_read_u16_array(
				g_dsp_audio_dev->dev->of_node,
				PROP_DSP_CMD_SET_MIX_MATRIX,
				p_dsp_cmd,
				cmd_len/2);

	if (ret_val != 0) {
		xgold_err("read %s property failed with error %d\n",
		PROP_DSP_CMD_SET_MIX_MATRIX, ret_val);
	} else
		(void)dsp_audio_cmd(*p_dsp_cmd,
						*(p_dsp_cmd + 1),
						p_dsp_cmd + 2);

	kfree(p_dsp_cmd);
	return ret_val;
}



/* device controls handler */
static int dsp_audio_dev_set_controls(struct dsp_audio_device *dsp_dev,
		enum dsp_audio_controls cmd, void *arg)
{
	int ret_val = 0;
	bool *power_state = NULL;
	struct dsp_rw_shm_data *p_rw_shm = NULL;
	struct dsp_aud_cmd_data *p_cmd_data = NULL;
	unsigned short *data_trace_ptr;
	unsigned short i;

	xgold_debug("In :%s, cmd:%d\n", __func__, cmd);

	if ((NULL == arg && cmd <= DSP_AUDIO_CONTROL_WRITE_SHM) ||
			(NULL == arg &&
			 cmd == DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC) ||
			(NULL == arg && cmd == DSP_AUDIO_POWER_REQ))
		return -EINVAL;
	if (cmd != DSP_AUDIO_POWER_REQ && !pm_runtime_active(dsp_dev->dev))
		return 0;

	switch (cmd) {
	case DSP_AUDIO_CONTROL_SEND_CMD:
		/* control to send the command */
		p_cmd_data = (struct dsp_aud_cmd_data *) arg;

		xgold_dsp_log("dsp_log: cmd_id: %d, cmd_len:%d\n",
			      p_cmd_data->command_id,
			      p_cmd_data->command_len);

		xgold_dsp_log("dsp_log: parameters: ");
		data_trace_ptr = (unsigned short *)p_cmd_data->p_data;
		for (i = 0; i < (p_cmd_data->command_len + 1) / 2; i += 2)
			xgold_dsp_log(" %X - %X\n", data_trace_ptr[i],
					data_trace_ptr[i + 1]);

		xgold_dsp_log("\n");

		if(system_state != SYSTEM_RUNNING) {
			WARN(1, "DSP_AUDIO_CONTROL_SEND_CMD while system is %d\n", system_state);
			break;
		}

		ret_val = (int)dsp_audio_cmd(
				p_cmd_data->command_id,
				p_cmd_data->command_len,
				p_cmd_data->p_data);

		/* Mark the DSP scheduler status */
		dsp_audio_mark_scheduler_status(p_cmd_data);
		break;

	case DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC:
		/* control to send the commands in atomic context */
		p_cmd_data = (struct dsp_aud_cmd_data *) arg;

		xgold_dsp_log("dsp_log: cmd_id: %d, cmd_len:%d\n",
			      p_cmd_data->command_id,
			      p_cmd_data->command_len);

		xgold_dsp_log("dsp_log: parameters: ");
		data_trace_ptr = (unsigned short *)p_cmd_data->p_data;
		for (i = 0; i < (p_cmd_data->command_len + 1) / 2; i += 2)
			xgold_dsp_log(" %X - %X\n", data_trace_ptr[i],
					data_trace_ptr[i + 1]);

		xgold_dsp_log("\n");

		ret_val = (int)dsp_audio_cmd_pipe_1(
				p_cmd_data->command_id,
				p_cmd_data->command_len,
				p_cmd_data->p_data);

		/* Mark the DSP scheduler status */
		dsp_audio_mark_scheduler_status(p_cmd_data);
		break;


	case DSP_AUDIO_CONTROL_READ_SHM:
		/* control to read data from the shared memory */
		p_rw_shm = (struct dsp_rw_shm_data *) arg;
		ret_val = (int)dsp_audio_read_shm(
				dsp_dev,
				p_rw_shm->p_data,
				p_rw_shm->word_offset,
				p_rw_shm->len_in_bytes);
		break;

	case DSP_AUDIO_CONTROL_WRITE_SHM:
		/* control to write data to the shared memory */
		p_rw_shm = (struct dsp_rw_shm_data *) arg;
		ret_val = (int)dsp_audio_write_shm(
				dsp_dev,
				p_rw_shm->p_data,
				p_rw_shm->word_offset,
				p_rw_shm->len_in_bytes);
		break;

	case DSP_AUDIO_CONTROL_SET_PLAY_PATH:
		/* control to setup PCM play path */
		xgold_debug("DSP_AUDIO_CONTROL_SET_PLAY_PATH\n");
		ret_val = dsp_audio_xgold_set_pcm_path(0);
		break;

	case DSP_AUDIO_CONTROL_SET_REC_PATH:
		/* control to setup PCM record path */
		ret_val = dsp_audio_xgold_set_pcm_path(1);
		break;

	case DSP_AUDIO_POWER_REQ:
		/*control to request DSP power */
		power_state = (bool *)arg;

		xgold_debug("DSP power request %d\n", *power_state);

		if (*power_state == 1) {
			ret_val = pm_runtime_get_sync(dsp_dev->dev);

			if (ret_val < 0) {
				xgold_err("%s: Power req error for sba %d\n",
					__func__, ret_val);
				return ret_val;
			}

			if (XGOLD_DSP_XG742_SBA == dsp_dev->id) {
				ret_val = pm_runtime_get_sync(
					dsp_dev->p_dsp_common_data->fba_dev);

				if (ret_val < 0) {
					xgold_err("%s: Power req error for fba %d\n",
					__func__, ret_val);
					return ret_val;
				}
			}
		} else {
			ret_val = pm_runtime_put_sync_suspend(dsp_dev->dev);

			if (ret_val < 0) {
				xgold_err("%s: Power req error for sba %d\n",
						__func__, ret_val);
				return ret_val;
			}
			if (XGOLD_DSP_XG742_SBA == dsp_dev->id) {
				ret_val = pm_runtime_put_sync_suspend(
				dsp_dev->p_dsp_common_data->fba_dev);

				if (ret_val < 0) {
					xgold_err("%s: Power req error for fba %d\n",
							__func__, ret_val);
					return ret_val;
				}
			}
		}
		break;

	default:
		break;
	}

	return ret_val;
}

enum dsp_err_code dsp_audio_intr_activate(
			enum dsp_irq_no irq_no)
{
	enum dsp_err_code ret = DSP_SUCCESS;
	struct dsp_audio_device *dsp_dev = NULL;

	list_for_each_entry(dsp_dev, &list_dsp, node) {
		switch (dsp_dev->id) {
		case XGOLD_DSP_XG642:
			ret = dsp_audio_irq_activate(dsp_dev,
					irq_no);
		break;
		case XGOLD_DSP_XG742_FBA:
			if (irq_no >= DSP_FBA_IRQ_0)
				ret = dsp_audio_irq_activate(dsp_dev,
						irq_no);
			else
				ret = DSP_ERR_INVALID_REQUEST;
		break;
		case XGOLD_DSP_XG742_SBA:
			if (irq_no < DSP_IRQ_4)
				ret = dsp_audio_irq_activate(dsp_dev,
						irq_no);
			else
				ret = DSP_ERR_INVALID_REQUEST;
		break;

		default:
		break;
		}
	}
	return ret;
}

enum dsp_err_code dsp_audio_intr_deactivate(
			enum dsp_irq_no irq_no)
{
	struct dsp_audio_device *dsp_dev = NULL;
	enum dsp_err_code ret = DSP_SUCCESS;

	list_for_each_entry(dsp_dev, &list_dsp, node) {
		switch (dsp_dev->id) {
		case XGOLD_DSP_XG642:
			ret = dsp_audio_irq_deactivate(dsp_dev, irq_no);
		break;
		case XGOLD_DSP_XG742_FBA:
			if (irq_no >= DSP_FBA_IRQ_0)
				ret = dsp_audio_irq_deactivate(dsp_dev,
						irq_no);
			else
				ret = DSP_ERR_INVALID_REQUEST;
		break;
		case XGOLD_DSP_XG742_SBA:
			if (irq_no < DSP_IRQ_4)
				ret = dsp_audio_irq_deactivate(dsp_dev,
						irq_no);
			else
				ret = DSP_ERR_INVALID_REQUEST;
		break;

		default:
		break;
		}
	}
	return ret;
}


/* dsp device operations */
static struct dsp_ops dsp_dev_ops = {
	.open = dsp_audio_dev_open,
	.set_controls = dsp_audio_dev_set_controls,
	.irq_activate = dsp_audio_intr_activate,
	.irq_deactivate = dsp_audio_intr_deactivate,
	.close = dsp_audio_dev_close,
};

/* static functions */
enum dsp_interrupt {
	PLAYBACK_INT,
	PLAYBACK2_INT,
	RECORD_INT,
	TONE_INT,
	I2S_INT,
	AUDIO_OV_USB,
	HW_PROBE,
	SPEECH_PROBES,
	DSP_INTERRUPT_END
};

static int xgold_dsp_get_interrupt_index(const char *name)
{
	if (!strcmp(name, "dsp_record"))
		return RECORD_INT;
	else if (!strcmp(name, "dsp_playback"))
		return PLAYBACK_INT;
	else if (!strcmp(name, "dsp_playback2"))
		return PLAYBACK2_INT;
	else if (!strcmp(name, "dsp_tone"))
		return TONE_INT;
	else if (!strcmp(name, "dsp_hw_probe"))
		return HW_PROBE;
	else if (!strcmp(name, "dsp_speech_probe"))
		return SPEECH_PROBES;
	else
		return -EINVAL;
}

/* DSP interrupts */
#define DSP_INT_ID_LENGTH	9
#define DSP_INT_ID_MASK		((1 << DSP_INT_ID_LENGTH) - 1)
#define DSP_INT_GET_ID(val)	(val & DSP_INT_ID_MASK)

#define DSP_INT_IMSC_OFFSET	DSP_INT_ID_LENGTH
#define DSP_INT_IMSC_LENGTH	5
#define DSP_INT_IMSC_MASK	((1 << DSP_INT_IMSC_LENGTH) - 1)
#define DSP_INT_GET_IMSC(val) \
	((val >> DSP_INT_IMSC_OFFSET) & DSP_INT_IMSC_MASK)

#define DSP_INT_ICR_OFFSET	(DSP_INT_IMSC_OFFSET + DSP_INT_IMSC_LENGTH)
#define DSP_INT_ICR_LENGTH	5
#define DSP_INT_ICR_MASK	((1 << DSP_INT_ICR_LENGTH) - 1)
#define DSP_INT_GET_ICR(val) \
	((val >> DSP_INT_ICR_OFFSET) & DSP_INT_ICR_MASK)

#define DSP_INT_MIS_OFFSET	(DSP_INT_ICR_OFFSET + DSP_INT_ICR_LENGTH)
#define DSP_INT_MIS_LENGTH	5
#define DSP_INT_MIS_MASK	((1 << DSP_INT_MIS_LENGTH) - 1)
#define DSP_INT_GET_MIS(val) \
	((val >> DSP_INT_MIS_OFFSET) & DSP_INT_MIS_MASK)

#define DSP_INT_CF_OFFSET	(DSP_INT_MIS_OFFSET+DSP_INT_MIS_LENGTH)
#define DSP_INT_CF_LENGTH	5
#define DSP_INT_CF_MASK		((1 << DSP_INT_CF_LENGTH) - 1)
#define DSP_INT_GET_CF(val) \
	((val >> DSP_INT_CF_OFFSET) & DSP_INT_CF_MASK)

#define DSP_MAKE_INTERRUPT(_id, _imsc, _icr, _mis, _cf) \
	((_id & DSP_INT_ID_MASK) | \
	((_imsc & DSP_INT_IMSC_MASK) << DSP_INT_IMSC_OFFSET) | \
	((_icr & DSP_INT_ICR_MASK) << DSP_INT_ICR_OFFSET) | \
	((_mis & DSP_INT_MIS_MASK) << DSP_INT_MIS_OFFSET) | \
	((_cf & DSP_INT_CF_MASK) << DSP_INT_CF_OFFSET))

static int xgold_dsp_fill_interrupt_array(struct device_node *np,
		struct dsp_audio_device *dsp)
{
	unsigned out_imsc[8], out_icr[8], out_mis[8], out_cf[8];
	int irq_num, ret, i;

	if (!np) {
		xgold_err("Could not parse interrupts\n");
		return -EINVAL;
	}

	irq_num = of_irq_count(np);
	ret = of_property_read_u32_array(np, PROP_DSP_INT_IMSC,
			out_imsc, irq_num + 1);	/* Reg address + interrupts */
	if (ret) {
		xgold_err("Could not find property %s\n",
			PROP_DSP_INT_IMSC);
		return ret;
	}
	dsp->imsc = out_imsc[0];

	ret = of_property_read_u32_array(np, PROP_DSP_INT_ICR,
			out_icr, irq_num + 1);	/* Reg address + interrupts */
	if (ret) {
		xgold_err("Could not find property %s\n",
			PROP_DSP_INT_ICR);
		return ret;
	}
	dsp->icr = out_icr[0];

	ret = of_property_read_u32_array(np, PROP_DSP_INT_MIS,
			out_mis, irq_num + 1);	/* Reg address + interrupts */
	if (ret) {
		xgold_err("Could not find property %s\n",
			PROP_DSP_INT_MIS);
		return ret;
	}
	dsp->mis = out_mis[0];

	ret = of_property_read_u32_array(np, PROP_DSP_CF,
			out_cf, irq_num);	/* all interrupts */
	if (ret) {
		xgold_err("Could not find property %s\n",
			PROP_DSP_CF);
		return ret;
	}

	for (i = 0; i < irq_num; i++) {
		int index;
		const char *name;

		ret = irq_of_parse_and_map(np, i);
		if (!ret) {
			xgold_err("Cannot map irq index %d\n", i);
			return -EINVAL;
		}

		if (of_property_read_string_index
		    (np, "interrupt-names", i, &name)) {
			xgold_err("Cannot get name of interrupt %d\n", i);
			return -EINVAL;
		}

		index = xgold_dsp_get_interrupt_index(name);
		if (index < 0) {
			xgold_err("invalid name %s\n", name);
			return -EINVAL;
		}

		dsp->interrupts[index] =
			DSP_MAKE_INTERRUPT(ret, out_imsc[i + 1], out_icr[i + 1],
			out_mis[i + 1], out_cf[i]);
	}

	return 0;
}

static int xgold_dsp_init_reg_array(struct device_node *np,
				    struct xgold_dsp_reg *dsp_reg,
				    const char *propname, unsigned nr)
{
	int ret, i;
	unsigned out_values[nr * 3];

	ret = of_property_read_u32_array(np, propname, out_values, nr * 3);

	if (ret) {
		pr_devel("Could not find property %s\n", propname);
		for (i = 0; i < nr; i++)
			dsp_reg[i].reg = NULL;

		return ret;
	}

	for (i = 0; i < nr; i++) {
		dsp_reg->reg = ioremap(out_values[0], 0x4);
		dsp_reg->shift = (unsigned char)out_values[1];
		dsp_reg->width = (unsigned char)out_values[2];
	}

	return 0;

}

static void xgold_dsp_fill_shm_offset(struct device_node *np,
		struct dsp_audio_device *dsp)
{
	int ret = 0;

	/* PCM remaining block offset */
	ret = of_property_read_u32_array(np,
			PROP_DSP_REMAIN_PCM,
			dsp->p_dsp_common_data->pcm_offset, 2);
	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_REMAIN_PCM);

	/* Buffer size UL offset */
	ret = of_property_read_u32(np,
			PROP_DSP_BUF_SIZE_UL_OFFSET,
			&dsp->p_dsp_common_data->buf_size_ul_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_BUF_SIZE_UL_OFFSET);

	/* Aud SM buffer DL offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_AUD_BUF_DL_OFFSET,
			&dsp->p_dsp_common_data->buf_sm_dl_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_AUD_BUF_DL_OFFSET);

	/* Aud SM buffer DL2 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_AUD_BUF_DL2_OFFSET,
			&dsp->p_dsp_common_data->buf_sm_dl2_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_AUD_BUF_DL2_OFFSET);

	/* Aud SM buffer UL offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_AUD_BUF_UL_OFFSET,
			&dsp->p_dsp_common_data->buf_sm_ul_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_AUD_BUF_UL_OFFSET);

	/* Aud SM HW probe A offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_HW_PROBE_A_OFFSET,
			&dsp->p_dsp_common_data->buf_sm_hw_probe_a_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_HW_PROBE_A_OFFSET);

	/* Aud SM HW probe B offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_HW_PROBE_B_OFFSET,
			&dsp->p_dsp_common_data->buf_sm_hw_probe_b_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_HW_PROBE_B_OFFSET);

	/* Aud SM SPEECH PROBE 1 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_SPEECH_BUFFER_1,
			&dsp->p_dsp_common_data->buf_sm_speech_probe_a_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_SPEECH_BUFFER_1);

	/* Aud SM SPEECH PROBE 2 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_SPEECH_BUFFER_2,
			&dsp->p_dsp_common_data->buf_sm_speech_probe_b_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_SPEECH_BUFFER_2);

	/* Aud SM SPEECH PROBE 3 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_SPEECH_BUFFER_3,
			&dsp->p_dsp_common_data->buf_sm_speech_probe_c_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_SPEECH_BUFFER_3);

	/* Aud SM SPEECH PROBE 4 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_SPEECH_BUFFER_4,
			&dsp->p_dsp_common_data->buf_sm_speech_probe_d_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_SPEECH_BUFFER_4);

	/* Aud SM SPEECH PROBE 5 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_SPEECH_BUFFER_5,
			&dsp->p_dsp_common_data->buf_sm_speech_probe_e_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_SPEECH_BUFFER_5);

	/* Aud SM SPEECH PROBE 6 offset */
	ret = of_property_read_u32(np,
			PROP_DSP_SM_SPEECH_BUFFER_6,
			&dsp->p_dsp_common_data->buf_sm_speech_probe_f_offset);

	if (ret)
		xgold_debug("Could not find property %s\n",
			PROP_DSP_SM_SPEECH_BUFFER_6);

}
static int xgold_dsp_init_reg(struct device_node *np,
			      struct xgold_dsp_reg *dsp_reg,
			      const char *propname)
{
	return xgold_dsp_init_reg_array(np, dsp_reg, propname, 1);
}

#define PROP_DSP_CLC		"intel,dsp-clc"
#define PROP_DSP_RST		"intel,dsp-rst"
#define PROP_DSP_ID		"intel,dsp,id"
#define PROP_UCCF		"intel,uccf"
#define PROP_MCU2DSP		"intel,mcu2dsp"
#define PROP_DSP_FIRMWARE	"intel,dsp-fw"
#define PROP_DSP_SUPPLY		"dsp"


/* TODO move all DTS configuration handling to separate file ? */
static int dsp_audio_of_parse(struct device *dev, struct dsp_audio_device *dsp)
{
	struct device_node *np = dev->of_node;
	const char *name;
	int ret;

	/* DSP identification */
	ret = of_property_read_string(np, PROP_DSP_ID, &name);
	if (ret)
		xgold_err("cannot get DSP name id\n");
	else if (!strcmp(name, "XG642")) {
		dsp->id = XGOLD_DSP_XG642;
		g_dsp_audio_dev = dsp;
	} else if (!strcmp(name, "XG742_FBA")) {
		dsp->id = XGOLD_DSP_XG742_FBA;
		dsp->p_dsp_common_data->fba_dev = dev;
	} else if (!strcmp(name, "XG742_SBA")) {
		dsp->id = XGOLD_DSP_XG742_SBA;
		g_dsp_audio_dev = dsp;
	} else {
		xgold_err("name id '%s' doesn't match\n", name);
		ret = -EINVAL;
	}

	dsp->p_dsp_common_data->num_dsp++;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	if (XGOLD_DSP_XG742_FBA != dsp->id && XGOLD_DSP_XG742_SBA != dsp->id) {
		struct dsp_clk *dsp_clk;
		struct clk *clk;
		int i = 0, n = 0, clk_num = 0;
		/* Clocks */
		xgold_debug("DSP clocks initialization\n");
		if (!of_find_property(np, "clocks", &n))
			return -EINVAL;

		n /= sizeof(struct clk *);
		xgold_debug("%s: %d clocks to be added\n", __func__, n);
		INIT_LIST_HEAD(&dsp->clk_list);
		clk = of_clk_get_by_name(np, "clk_dsp");
		if (IS_ERR(clk)) {
			xgold_err("clk %s not found\n", "clk_dsp");
			return PTR_ERR(clk);
		}
		dsp_clk = kzalloc(sizeof(struct dsp_clk), GFP_KERNEL);
		if (!dsp_clk)
			return -ENOMEM;

		dsp_clk->clk = clk;
		list_add_tail(&dsp_clk->node, &dsp->clk_list);
		clk_num++;
		i++;

		while (i < n) {
			clk = of_clk_get(np, i++);
			if (IS_ERR(clk))
				continue;

			dsp_clk = kzalloc(sizeof(struct dsp_clk), GFP_KERNEL);
			if (!dsp_clk)
				return -ENOMEM;

			dsp_clk->clk = clk;
			list_add_tail(&dsp_clk->node, &dsp->clk_list);
			clk_num++;
		}
		xgold_debug("%d clocks attributed to this dsp\n", clk_num);

		/* Regulator */
		dsp->regulator = regulator_get(dev, PROP_DSP_SUPPLY);
		if (IS_ERR(dsp->regulator)) {
			xgold_err("read %s-supply property failed\n",
				PROP_DSP_SUPPLY);
			dsp->regulator = NULL;
			goto skip_regulator;
		}
		ret = regulator_enable(dsp->regulator);
		if (ret)
			xgold_err("Unable to switch on DSP power domain\n");
skip_regulator:
	}
#endif /* CONFIG_PLATFORM_DEVICE_PM_VIRT */

	/* FW patch and length */
	of_find_property(np, PROP_DSP_FIRMWARE, &dsp->patch_length);
	dsp->patch_length /= 2;
	dsp->patch = kzalloc(sizeof(unsigned short) *
			dsp->patch_length, GFP_KERNEL);
	ret = of_property_read_u16_array(np, PROP_DSP_FIRMWARE,
			dsp->patch, dsp->patch_length);
	if (ret != 0) {
		xgold_err("read %s property failed with error %d\n",
			PROP_DSP_FIRMWARE, ret);
	}

	/* UCCF */
	ret = of_property_read_u32_array(np, PROP_UCCF, (u32 *) &dsp->uccf, 3);
	if (ret != 0) {
		xgold_err("read %s property failed with error code %d\n",
			PROP_UCCF, ret);
	}

	/* MCU2DSP */
	ret = of_property_read_u32(np, PROP_MCU2DSP, &dsp->mcu2dsp);
	if (ret) {
		xgold_err("read %s property failed with error code %d\n",
			PROP_MCU2DSP, ret);
	}

	/* Interrupt */
	ret = xgold_dsp_fill_interrupt_array(np, dsp);
	if (ret)
		xgold_err("Interrupt array assignement failed\n");

	/* SHM offset */
	xgold_dsp_fill_shm_offset(np, dsp);

	/* Regulator */
	dsp->regulator = regulator_get(dev, PROP_DSP_SUPPLY);
	if (IS_ERR(dsp->regulator)) {
		xgold_err("read %s-supply property failed\n",
			PROP_DSP_SUPPLY);
		dsp->regulator = NULL;
		goto skip_regulator;
	}
	ret = regulator_enable(dsp->regulator);
	if (ret)
		xgold_err("Unable to switch on DSP power domain\n");

skip_regulator:
	/* Reset */
	dsp->rst_ctl = reset_control_get(dev, "dsp");
	if (IS_ERR(dsp->rst_ctl)) {
		dsp->rst_ctl = NULL;

		/* If no reset control, rst field must be enterred */
		/* This is the case for XG22x */
		ret = xgold_dsp_init_reg(np, &dsp->rst, PROP_DSP_RST);
		if (ret != 0) {
			xgold_err("read %s property failed with error code %d\n",
				PROP_DSP_RST, ret);
		}
	}

	/* CLC */
	ret = of_property_read_u32(np, PROP_DSP_CLC, &dsp->clc);
	if (ret) {
		xgold_err("read %s property failed with error code %d\n",
			PROP_DSP_CLC, ret);
	}

/* Optional properties */
	/* Intenal Reset (RSTMODS) */
	ret = of_property_read_u32(np, "intel,dsp-rstmods", &dsp->rstmods);
	if (ret)
		dsp->rstmods = -1;

	return 0;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int dsp_clock_init(struct dsp_audio_device *dsp)
{
	struct dsp_clk *dsp_clk;
	int ret;

	list_for_each_entry(dsp_clk, &dsp->clk_list, node) {
		ret = clk_prepare_enable(dsp_clk->clk);
		if (ret)
			return ret;
	}
	return 0;
}
#else
static int dsp_clock_init(struct dsp_audio_device *dsp)
{
	int ret = 0;
	/* BUG_ON(!dsp->pm_platdata); */
	if (dsp->pm_platdata) {
		ret = device_state_pm_set_state_by_name(dsp->dev,
				dsp->pm_platdata->pm_state_D0_name);
		pr_info("%s set state return %d\n", __func__, ret);
	}
	return ret;
}
#endif

static inline int dsp_writel(unsigned int value, void *addr)
{
	xgold_debug("@0x%p wr 0x%X\n", addr, value);
	writel(value, addr);
	return 0;
}

static void __dsp_write_clc(struct dsp_audio_device *dsp, unsigned val)
{
	dsp_writel(val, dsp->shm_regs + dsp->clc);
}

static void __dsp_write_rstmods(struct dsp_audio_device *dsp, unsigned val)
{
	dsp_writel(val, dsp->shm_regs + dsp->rstmods);
}

static void dsp_reset(struct dsp_audio_device *dsp)
{
	if (!dsp->p_dsp_common_data->rst_done) {
		reset_control_assert(dsp->rst_ctl);
		udelay(200);
		reset_control_deassert(dsp->rst_ctl);
	}

	__dsp_write_clc(dsp, 0x100);

	__dsp_write_rstmods(dsp, 1);

	dsp->p_dsp_common_data->rst_done = 1;
}
/* Low level func */
static void dsp_set_uccf(struct dsp_audio_device *dsp, unsigned val)
{
	iowrite32(val, dsp->shm_regs + dsp->uccf.set);
}

static unsigned dsp_read_uccf(struct dsp_audio_device *dsp)
{
	return ioread32(dsp->shm_regs + dsp->uccf.get);
}

static void go_cmd(struct dsp_audio_device *dsp, unsigned id)
{
	unsigned mcu2dsp;

	dsp_set_uccf(dsp, BIT(id));
	/*FIXME:SpinLock */
	mcu2dsp = ioread32(dsp->shm_regs + dsp->mcu2dsp);
	mcu2dsp &= ~BIT(id);
	iowrite32(mcu2dsp, dsp->shm_regs + dsp->mcu2dsp);
	#ifdef CONFIG_X86_INTEL_XGOLD_VP
	udelay(5);
	#endif
	mcu2dsp |= BIT(id);
	iowrite32(mcu2dsp, dsp->shm_regs + dsp->mcu2dsp);
	#ifdef CONFIG_X86_INTEL_XGOLD_VP
	udelay(20);
	#endif
}

static int poll_cmd(struct dsp_audio_device *dsp, unsigned id)
{
	unsigned dsp_cf;
	int retries = 100;

	while (retries--) {
		dsp_cf = dsp_read_uccf(dsp);
		if (!(dsp_cf & BIT(id)))
			return 0;
		udelay(50);
	}
	return -1;
}
static unsigned short dsp_read_reg(
	struct dsp_audio_device *dsp,
	unsigned offset)
{
	return ioread16(dsp->shm_mem + (offset * 2));
}


#define OFFSET_SM_FW_VERSION                       0
#define OFFSET_SM_HW_VERSION                       1
#define OFFSET_SM_CUSTOMER_INTERFACE_VERSION       2
#define OFFSET_SM_STARTUP_DEBUG_VERSION            3

static void dsp_show_info(struct dsp_audio_device *dsp)
{
	char *name = "DSPXG642";
	switch (dsp->id) {
	case XGOLD_DSP_XG642:
		name = "DSPXG642";
		break;
	case XGOLD_DSP_XG742_FBA:
		name = "DSPXG742_FBA";
		break;
	case XGOLD_DSP_XG742_SBA:
		name = "DSPXG742_SBA";
		break;
	default:
		break;
	}
	xgold_debug("%s: Firmware version 0x%04x\n", name,
		dsp_read_reg(dsp, OFFSET_SM_FW_VERSION));
	xgold_debug("%s: Hardware version 0x%04x\n", name,
		dsp_read_reg(dsp, OFFSET_SM_HW_VERSION));
	xgold_debug("%s: Customer itf version 0x%04x\n", name,
		dsp_read_reg(dsp, OFFSET_SM_CUSTOMER_INTERFACE_VERSION));
	xgold_debug("%s: Startup dbg version version 0x%04x\n", name,
		dsp_read_reg(dsp, OFFSET_SM_STARTUP_DEBUG_VERSION));
}

static int dsp_patch(struct dsp_audio_device *dsp, unsigned offset)
{
#define DSP_BOOTCMD_PLOAD	0x0
#define DSP_BOOTCMD_DLOAD	0x1
#define DSP_BOOTCMD_BRANCH	0x2
#define DSP_BOOTCMD_PREAD	0x3
#define DSP_BOOTCMD_DREAD	0x4
	int ret = 0;
	unsigned len;
	unsigned short *patch;

	patch = dsp->patch;
	len = dsp->patch_length;

	while (len) {
		unsigned short dsp_cmd, length;

		dsp_cmd = *patch;
		if (dsp_cmd == DSP_BOOTCMD_BRANCH)
			length = 2;
		else
			length = patch[2] + 3;	/* cmd,addr,length */

		memcpy((void *)(dsp->shm_mem + offset * 2), patch,
		       (length * 2));
		patch += length;

		len -= length;
		go_cmd(dsp, 0);
		ret = poll_cmd(dsp, 0);
		if (ret < 0)
			return ret;
	};

	dsp_show_info(dsp);
	return 0;
}

/* DSP commands */
#define DSP_CMD_DATA_OFF_OFFSET 8
#define DSP_CMD_DATA_LENGTH_OFFSET 20
#define DSP_CMD_RW_OFFSET 31

#define DSP_MAKE_CMD(x, y, z, w) \
		((w << DSP_CMD_RW_OFFSET) | \
		(z << DSP_CMD_DATA_LENGTH_OFFSET) | \
		(y << DSP_CMD_DATA_OFF_OFFSET) | x)
#define DSP_GET_RW_CODE(x) \
		(x >> DSP_CMD_RW_OFFSET)
#define DSP_GET_CMD_DATA_LENGTH(x) \
		((x >> DSP_CMD_DATA_LENGTH_OFFSET) & 0x7FF)
#define DSP_GET_CMD_DATA_OFFSET(x) \
		((x >> DSP_CMD_DATA_OFF_OFFSET) & 0xFFF)
#define DSP_GET_CMD_ID(x) (x & 0xFF)

struct xgold_dsp_cmd {
	struct list_head queue;
	unsigned id;
	void *cmd;
	void *result;
	unsigned status;
};

struct xgold_dsp_pipe {
	struct list_head pipe;
	struct xgold_dsp_cmd *cmd_active;
	char id; /* 0 .. 1 .. 2 */
	unsigned short *cmd_addr;
};

static int dsp_init(struct dsp_audio_device *dsp, unsigned mcu_cmd_offset,
		       unsigned mcu_cmd_length)
{
	int i, ret = 0;

	INIT_LIST_HEAD(&dsp->pipe);
	INIT_LIST_HEAD(&dsp->done);
	INIT_LIST_HEAD(&dsp->free);

	for (i = 0; i < 3; i++) {
		struct xgold_dsp_pipe *pipe;
		pipe = kmalloc(sizeof(struct xgold_dsp_pipe), GFP_KERNEL);
		if (pipe == NULL) {
			xgold_err("Failed to allocate memory for pipe\n");
			return -ENOMEM;
		}
		list_add_tail(&pipe->pipe, &dsp->pipe);
		pipe->cmd_active = NULL;
		pipe->id = i;
		pipe->cmd_addr = (void __iomem *)(dsp->shm_mem +
			(2 * (mcu_cmd_offset + mcu_cmd_length * i)));
	}

	for (i = 0; i < 20; i++) {
		struct xgold_dsp_cmd *cmd;
		cmd = kmalloc(sizeof(struct xgold_dsp_cmd), GFP_KERNEL);
		list_add_tail(&cmd->queue, &dsp->free);
	}

	return ret;
}
enum dsp_cmd {
	VB_HW_AFE = DSP_MAKE_CMD(2, 0, 0, 0),
	VB_HW_BT = DSP_MAKE_CMD(3, 0, 0, 0),
	VB_HW_DIGMIC = DSP_MAKE_CMD(4, 0, 0, 0),
	VB_HW_FMR = DSP_MAKE_CMD(5, 0, 0, 0),
	IDLE = DSP_MAKE_CMD(14, 0, 0, 0),
	DTX_ON = DSP_MAKE_CMD(18, 0, 0, 0),
	VB_SET_SPEECH_PATH = DSP_MAKE_CMD(26, 0, 0, 0),
	VB_SET_SWM_AFE_OUT = DSP_MAKE_CMD(27, 0, 0, 0),
	VB_SET_SWM_SPEECH_OUT = DSP_MAKE_CMD(28, 0, 0, 0),
	VB_SET_SWM_HF_OUT = DSP_MAKE_CMD(29, 0, 0, 0),
	VB_SET_SWM_BT_OUT = DSP_MAKE_CMD(30, 0, 0, 0),
	VB_SET_SWM_PCM_OUT = DSP_MAKE_CMD(32, 0, 0, 0),
	VB_SET_GAIN = DSP_MAKE_CMD(33, 0, 0, 0),
	VB_RX_CNI = DSP_MAKE_CMD(35, 0, 0, 0),
	VB_SET_GAIN_TIMECONST = DSP_MAKE_CMD(36, 0, 0, 0),
	VB_ANA = DSP_MAKE_CMD(37, 0, 0, 0),
	VB_SET_FIR = DSP_MAKE_CMD(38, 0, 0, 0),
	VB_SET_IIR = DSP_MAKE_CMD(39, 0, 0, 0),
	VB_RESET = DSP_MAKE_CMD(40, 0, 0, 0),
	/*VB_START_TONE = DSP_MAKE_CMD(41, DSP_CMD_START_TONE_SHM_OFFSET,
				     DSP_CMD_START_TONE_LENGTH, 1), */
	VB_STOP_TONE = DSP_MAKE_CMD(42, 0, 0, 0),
	VB_READ_DURATION = DSP_MAKE_CMD(43, 0, 0, 0),
	VB_SPLC_EP = DSP_MAKE_CMD(44, 0, 0, 0),
	VB_SPLC_LS = DSP_MAKE_CMD(45, 0, 0, 0),
	VB_UL_ENL = DSP_MAKE_CMD(46, 0, 0, 0),
	VB_AEC = DSP_MAKE_CMD(47, 0, 0, 0),
	VB_NR_UL = DSP_MAKE_CMD(48, 0, 0, 0),
	VB_AGC = DSP_MAKE_CMD(49, 0, 0, 0),
	TTY_CTM = DSP_MAKE_CMD(51, 0, 0, 0),
	VB_NR_DL = DSP_MAKE_CMD(55, 0, 0, 0),
	PPL_DRP = DSP_MAKE_CMD(56, 0, 0, 0),
	PPL_PARAM_FILTER = DSP_MAKE_CMD(57, 0, 0, 0),
	PCM1_PLAY = DSP_MAKE_CMD(58, 0, 0, 0),
	PCM_REC = DSP_MAKE_CMD(59, 0, 0, 0),
	SPEECH_UL_ENCODER = DSP_MAKE_CMD(60, 0, 0, 0),
	SPEECH_DL_DECODER = DSP_MAKE_CMD(61, 0, 0, 0),
	/*WRITE_DATA = DSP_MAKE_CMD(62, DSP_CMD_WRITE_DATA_SHM_OFFSET,
				  DSP_CMD_WRITE_DATA_LENGTH, 1),
	READ_DATA = DSP_MAKE_CMD(63, DSP_CMD_READ_DATA_SHM_OFFSET,
				 DSP_CMD_READ_DATA_LENGTH, 0), */
	WRITE_PROG = DSP_MAKE_CMD(64, 0, 0, 1),	/* FIXME check SHM offset
						   and length */
	READ_PROG = DSP_MAKE_CMD(65, 0, 0, 0),	/* FIXME check SHM offset
						   and length */
	SPEECH_PROBE = DSP_MAKE_CMD(66, 0, 0, 0),
	VB_SET_SWM_PROBE_OUT = DSP_MAKE_CMD(68, 0, 0, 0),
	VB_SET_BIQUAD_DUAL_COMP = DSP_MAKE_CMD(69, 0, 0, 0),
	VB_DUAL_BAND_COMP = DSP_MAKE_CMD(70, 0, 0, 0),
	VB_SET_SWM_SNS_OUT = DSP_MAKE_CMD(75, 0, 0, 0),
	VB_SET_SWM_MIX_MATRIX = DSP_MAKE_CMD(77, 0, 0, 0),
	VB_SER = DSP_MAKE_CMD(78, 0, 0, 0),
	VB_SET_SWM_FMR_OUT = DSP_MAKE_CMD(79, 0, 0, 0),
	VB_TX_CNI = DSP_MAKE_CMD(80, 0, 0, 0),
	TRACE_CONFIG = DSP_MAKE_CMD(84, 0, 0, 0),
	VB_START = DSP_MAKE_CMD(85, 0, 0, 0),
	VB_HW_I2S1 = DSP_MAKE_CMD(86, 0, 0, 0),
	VB_SET_SWM_I2S1_OUT = DSP_MAKE_CMD(87, 0, 0, 0),
	VB_SYNC = DSP_MAKE_CMD(88, 0, 0, 0),
	CHANGE_VB_TIMING = DSP_MAKE_CMD(89, 0, 0, 0),
	GSM_ON = DSP_MAKE_CMD(90, 0, 0, 0),
	UMTS_ON = DSP_MAKE_CMD(91, 0, 0, 0),
	LOOPBACK_ON = DSP_MAKE_CMD(92, 0, 0, 0),
	VB_SET_REFERENCE_LINE = DSP_MAKE_CMD(94, 0, 0, 0),
	VB_SET_DELAY_LINE = DSP_MAKE_CMD(95, 0, 0, 0),
	CLOCK_SCALING = DSP_MAKE_CMD(98, 0, 0, 0),
	VB_FMR_SILENT_DETECT = DSP_MAKE_CMD(99, 0, 0, 0),
	VB_HW_PROBE = DSP_MAKE_CMD(100, 0, 0, 0),
	VB_BWX = DSP_MAKE_CMD(104, 0, 0, 0),
	VB_GMM = DSP_MAKE_CMD(105, 0, 0, 0),
	PCM2_PLAY = DSP_MAKE_CMD(106, 0, 0, 0),
	VB_GLC = DSP_MAKE_CMD(107, 0, 0, 0),
	VB_BMF = DSP_MAKE_CMD(111, 0, 0, 0),
};

struct s_dsp_cmd_change_vb_timing {
	unsigned short id;
	unsigned short cmd_switch;
};

struct s_dsp_cmd_vb_hw_afe {
	unsigned short id;
	unsigned short cmd_switch;
	unsigned short ratesw;
};

static unsigned dsp_pop_pipes(struct dsp_audio_device *dsp)
{
	struct list_head *pipe_head;
	struct xgold_dsp_pipe *pipe;
	unsigned ret = 0;
	unsigned dsp_cf = dsp_read_uccf(dsp);

	list_for_each(pipe_head, &dsp->pipe) {
		pipe = list_entry(pipe_head, struct xgold_dsp_pipe, pipe);
		if (!(dsp_cf & BIT(pipe->id))) {
			struct xgold_dsp_cmd *cmd;
			unsigned length;
			cmd = pipe->cmd_active;
			if (cmd == NULL)
				continue;

			cmd->status = *pipe->cmd_addr;

			/* Read command */
			if (!DSP_GET_RW_CODE(cmd->id)) {
				length = 2 * DSP_GET_CMD_DATA_LENGTH(cmd->id);
				if (length) {
					void *offset = dsp->shm_mem + (2 *
					     DSP_GET_CMD_DATA_OFFSET(cmd->id));
					memcpy(cmd->result, offset,
					       length);
				}
			}

			list_add_tail(&cmd->queue, &dsp->done);
			pipe->cmd_active = NULL;
			ret |= BIT(pipe->id);
		}
	}
	return ret;
}

static int poll_for_free_pipe(struct dsp_audio_device *dsp)
{
	int retries = 10000;
	int free_pipes;
	while (retries--) {
		free_pipes = dsp_pop_pipes(dsp);
		if (free_pipes)
			return free_pipes;
	}
	return 0;
}

/* DSP Command functions */
static int dsp_wait_for_cmd_completion(struct dsp_audio_device *dsp,
				      struct xgold_dsp_cmd *cmd)
{
	struct list_head *ptr;
	int ret;
find:
	list_for_each(ptr, &dsp->done) {
		struct xgold_dsp_cmd *curr;
		curr = list_entry(ptr, struct xgold_dsp_cmd, queue);
		if (curr == cmd)
			goto found;
	}
	ret = poll_for_free_pipe(dsp);
	if (ret)
		goto find;
	else
		return -ETIME;
found:
	list_move_tail(&cmd->queue, &dsp->free);
	if (cmd->status == ((unsigned short)(~(DSP_GET_CMD_ID(cmd->id))) + 1))
		return 0;
	else
		return -EIO;
}

static struct xgold_dsp_cmd *dsp_post_command(struct dsp_audio_device *dsp,
						unsigned id,
						void *param,
						void *result,
						unsigned mcu_cmd_length)
{
	struct list_head *pipe_head;
	struct xgold_dsp_pipe *pipe;
	int ret;
	struct xgold_dsp_cmd *cmd;
find:
	list_for_each(pipe_head, &dsp->pipe) {
		pipe = list_entry(pipe_head, struct xgold_dsp_pipe, pipe);
		if (pipe->cmd_active == NULL)
			goto found;
	}
	ret = poll_for_free_pipe(dsp);
	if (ret)
		goto find;
	else
		return NULL;
found:
	cmd = list_first_entry(&dsp->free, struct xgold_dsp_cmd, queue);
	memcpy((void *)pipe->cmd_addr, param, mcu_cmd_length * 2);
	*pipe->cmd_addr = DSP_GET_CMD_ID(id);
	pipe->cmd_active = cmd;
	cmd->id = id;

	if (!DSP_GET_RW_CODE(id))	/* Read command */
		cmd->result = result;
	else			/* Write command */
		memcpy((void *)dsp->shm_mem + 2 * DSP_GET_CMD_DATA_OFFSET(id),
		       result, 2 * DSP_GET_CMD_DATA_LENGTH(id));

	list_del_init(&cmd->queue);
	go_cmd(dsp, pipe->id);
	return cmd;
}

#define SM_MCU_CMD_LENGHT 33
static int dsp_do_command(
	struct dsp_audio_device *dsp,
	unsigned id, void *param,
	void *result)
{
	int ret;

	struct xgold_dsp_cmd *cmd =
		dsp_post_command(dsp, id, param, result, SM_MCU_CMD_LENGHT);
	ret = dsp_wait_for_cmd_completion(dsp, cmd);

	/* dsp_save_cmd(id, param, ret); */
	return ret;
}

static int dsp_start_audio_sched(struct dsp_audio_device *dsp)
{
	struct s_dsp_cmd_change_vb_timing vb_timing;
	int ret;

	memset(&vb_timing, 0, sizeof(struct s_dsp_cmd_change_vb_timing));
	vb_timing.cmd_switch = 0; /* Default timing */
	ret = dsp_do_command(dsp, CHANGE_VB_TIMING, &vb_timing, NULL);
	if (ret) {
		xgold_err("VB_TIMING command generated error\n");
		return ret;
	}
	xgold_debug("VB_TIMING command sent\n");

	return ret;
}

static inline short get_pcm_cmd_mode(unsigned nbchan)
{
	switch (nbchan) {
	case 1:
		return 0;
	case 2:
		return 3;
	}
	return -1;
}

static inline short get_pcm_cmd_rate(unsigned rate)
{
	switch (rate) {
	case 8000:
		return 0;
	case 11025:
		return 1;
	case 12000:
		return 2;
	case 16000:
		return 3;
	case 22050:
		return 4;
	case 24000:
		return 5;
	case 32000:
		return 6;
	case 44100:
		return 7;
	case 48000:
		return 8;
	}
	return -1;
}
enum dsp_stream_req_type {
	STREAM_PCM_REQ_TYPE_IRQ = 0,
	STREAM_PCM_REQ_TYPE_DMA,
};

struct s_dsp_cmd_pcm_play {
	unsigned short id;
	unsigned short cmd_switch;
	unsigned short mode;
	unsigned short rate;
	unsigned short request_type;
};

int dsp_start_audio_hwafe(void)
{
	struct s_dsp_cmd_vb_hw_afe afe;
	struct dsp_audio_device *dsp_dev = NULL;
	int ret = 0;

	memset(&afe, 0, sizeof(struct s_dsp_cmd_vb_hw_afe));
	afe.cmd_switch = 1;	/* On & Update */
	afe.ratesw = 0;		/* Hard Code 8 Khz */
	list_for_each_entry(dsp_dev, &list_dsp, node) {
		switch (dsp_dev->id) {
		case XGOLD_DSP_XG642:
			ret = dsp_do_command(dsp_dev,
					VB_HW_AFE, &afe, NULL);
			break;
		case XGOLD_DSP_XG742_SBA:
		case XGOLD_DSP_XG742_FBA:
			ret = dsp_audio_cmd(85, 0, NULL);
		default:
			break;
		}
	}
	if (ret)
		xgold_err("VB_HW_AFE command generated error\n");
	xgold_debug("VB_HW_AFE command sent\n");

	return ret;
}
EXPORT_SYMBOL(dsp_start_audio_hwafe);

int dsp_stop_audio_hwafe(void)
{
	struct s_dsp_cmd_vb_hw_afe afe;
	struct dsp_audio_device *dsp_dev = NULL;
	int ret = 0;

	memset(&afe, 0, sizeof(struct s_dsp_cmd_vb_hw_afe));
	afe.cmd_switch = 0;	/* off */
	afe.ratesw = 0;		/* Hard Code 8 Khz */

	list_for_each_entry(dsp_dev, &list_dsp, node) {
		switch (dsp_dev->id) {
		case XGOLD_DSP_XG642:
			ret = dsp_do_command(dsp_dev,
					VB_HW_AFE, &afe, NULL);
			break;
		case XGOLD_DSP_XG742_SBA:
		case XGOLD_DSP_XG742_FBA:
			dsp_audio_cmd(14, 0, NULL);
		default:
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL(dsp_stop_audio_hwafe);

static void send_idle_cmd(enum dsp_id id)
{
	struct dsp_audio_device *dsp_dev = NULL;
	xgold_debug("-->%s\n", __func__);

	list_for_each_entry(dsp_dev, &list_dsp, node) {
		if (id == dsp_dev->id) {
			dsp_add_audio_msg_2_dsp(dsp_dev,
					DSP_AUDIO_CMD_IDLE,
					DSP_AUDIO_CMD_ID_LEN,
					NULL);
			break;
		}
	}
	xgold_debug("<-- %s\n", __func__);
}

#define OFFSET_SM_BOOT_DATA		OFFSET_SM_CUSTOMER_INTERFACE_VERSION
#define OFFSET_SM_MCU_CMD_0             4

/* DSP boot and patch ram download */
static int dsp_audio_boot(struct dsp_audio_device *dsp)
{
	int ret;
	dsp_clock_init(dsp);
	dsp_reset(dsp);
	ret = dsp_patch(dsp, OFFSET_SM_BOOT_DATA);
	if (ret)
		xgold_err("error during FW download\n");
	return ret;
}

#ifdef CONFIG_PM
static int dsp_audio_suspend(struct device *dev)
{
	int ret = 0;
	struct dsp_audio_device *dsp_dev;
	xgold_debug("-->%s\n", __func__);

	if (!pm_runtime_active(dev)) {
		dsp_dev = dev_get_drvdata(dev);

		xgold_debug("%s: Turning off dsp\n", __func__);
		ret = device_state_pm_set_state_by_name(dev,
				dsp_dev->pm_platdata->pm_state_D3_name);

		dsp_dev->p_dsp_common_data->rst_done = 0;
	}

	else if ((true == g_dsp_audio_dev->pb_running) ||
		(true == g_dsp_audio_dev->rec_running)) {
		/* Audio DSP doesn't support PM suspend/resume while
		 * there is active audio streaming, so we return error
		 * when there is active audio streaming if system is
		 * asked to enter suspended mode.
		 * Userspace application needs to close the audio before
		 * suspend the system */
		return -EBUSY;
	}

	if (ret < 0)
		xgold_err("%s: Failed with error %d\n",	__func__, ret);

	xgold_debug("<-- %s\n", __func__);

	return ret;
}

static int dsp_audio_resume(struct device *dev)
{
	int ret = 0;

	struct dsp_audio_device *dsp_dev;
	xgold_debug("-->%s\n", __func__);

	if (!pm_runtime_active(dev)) {
		dsp_dev = dev_get_drvdata(dev);

		xgold_debug("%s: Booting audio dsp", __func__);
		/* Request clock/voltage for DSP */
		ret = device_state_pm_set_state_by_name(dev,
			dsp_dev->pm_platdata->pm_state_D0_name);

		if (ret < 0)
			xgold_err("%s: Failed with error %d\n",
				__func__, ret);

		ret = dsp_audio_boot(dsp_dev);

		if (ret < 0)
			xgold_err("%s: Boot Failed with error %d\n",
				__func__, ret);

		/* Send IDLE command to initialize FW properly */
		/* TODO: During first boot, IDLE is sent from userspace.
		   Send IDLE from kernel during probe */
		if (ret == 0)
			send_idle_cmd(dsp_dev->id);

		/* Suspend DSP to Memory retention mode after dsp boot*/
		ret = device_state_pm_set_state_by_name(dev,
				dsp_dev->pm_platdata->pm_state_D0i3_name);

		if (ret < 0)
			xgold_err("%s: Failed with error %d\n",
				__func__, ret);
	}

	xgold_debug("<-- %s\n", __func__);

	return ret;
}
#else
#define dsp_audio_suspend NULL
#define dsp_audio_resume NULL
#endif


#ifdef CONFIG_PM_RUNTIME
static int dsp_audio_runtime_suspend(struct device *dev)
{
	struct dsp_audio_device *dsp_dev;
	int ret = 0;

	xgold_debug("%s: called\n", __func__);

	dsp_dev = dev_get_drvdata(dev);

	if (dsp_dev != NULL)
		ret = device_state_pm_set_state_by_name(dev,
				dsp_dev->pm_platdata->pm_state_D0i3_name);

	if (ret < 0)
		xgold_err("%s : failed with error %d", __func__, ret);

	return ret;
}

static int dsp_audio_runtime_resume(struct device *dev)
{
	struct dsp_audio_device *dsp_dev;
	int ret = 0;

	xgold_debug("%s: Called\n", __func__);

	dsp_dev = dev_get_drvdata(dev);

	if (!dsp_dev)
		return 0;

	if (dsp_dev->pm_platdata)
		ret = device_state_pm_set_state_by_name(dev,
				dsp_dev->pm_platdata->pm_state_D0_name);
	if (ret < 0)
		xgold_err("%s : failed with error %d", __func__, ret);

	if (XGOLD_DSP_XG642 == dsp_dev->id) {
		/* Activate HW probe, pcm rec and pcm play interrupt */
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_1);
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_2);
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_3);
		/* Activate voice codec interrupt*/
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_4);
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_5);
	} else if (XGOLD_DSP_XG742_FBA == dsp_dev->id) {
		/* Activate FBA DSP interrupt 0 for VOLTE call */
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_FBA_IRQ_0);
		/* Activate HW probe, pcm rec and pcm play interrupt */
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_1);
		(void)dsp_dev->p_dsp_common_data->
			ops->irq_activate(DSP_IRQ_2);
	}

	return ret;
}

static int dsp_audio_runtime_idle(struct device *dev)
{
	struct dsp_audio_device *dsp_dev;
	int ret = 0;

	xgold_debug("%s: called\n", __func__);

	dsp_dev = dev_get_drvdata(dev);

	if (dsp_dev != NULL)
		ret = device_state_pm_set_state_by_name(dev,
				dsp_dev->pm_platdata->pm_state_D0i3_name);

	if (ret < 0)
		xgold_err("%s : failed with error %d", __func__, ret);

	return ret;
}
#endif

static const struct dev_pm_ops dsp_audio_pm = {
	SET_RUNTIME_PM_OPS(dsp_audio_runtime_suspend, dsp_audio_runtime_resume,
			dsp_audio_runtime_idle)
	SET_SYSTEM_SLEEP_PM_OPS(dsp_audio_suspend,
			dsp_audio_resume)
};

/* DSP API */
static int get_dsp_pcm_channels(unsigned int channels)
{
	if (channels == 1)
		return 0;
	else if (channels == 2)
		return 3;
	else
		return -1;
}

static int get_dsp_pcm_rate(unsigned int rate)
{
	switch (rate) {
	case 8000:
		return 0;
	case 11025:
		return 1;
	case 12000:
		return 2;
	case 16000:
		return 3;
	case 22050:
		return 4;
	case 24000:
		return 5;
	case 32000:
		return 6;
	case 44100:
		return 7;
	case 48000:
		return 8;
	}
	return -1;
}

struct dsp_audio_device *of_dsp_register_client(
		struct device *dev, struct device_node *np)
{
	struct dsp_audio_device *dsp;

	list_for_each_entry(dsp, &list_dsp, node) {
		if (dsp->dev->of_node == np)
			return dsp;

	}

	return NULL;
}
EXPORT_SYMBOL(of_dsp_register_client);

int dsp_pcm_play(struct dsp_audio_device *dsp, enum xgold_pcm_stream_type type,
		unsigned int channels, unsigned int rate, bool dma_mode,
		unsigned short buffer_mode,
		unsigned short dma_req_interval_time,
		unsigned short buffer_size)
{
	struct T_AUD_DSP_CMD_PCM_PLAY_PAR pcm_par = { 0 };
	struct dsp_aud_cmd_data cmd_data;

	pcm_par.setting = 1; /* Init & Go */
	pcm_par.mode = get_dsp_pcm_channels(channels);
	pcm_par.rate = get_dsp_pcm_rate(rate);
	pcm_par.req = (dma_mode == true) ? 1 : 0;
	if (dsp->id == XGOLD_DSP_XG642) {
		pcm_par.buffer_mode = buffer_mode;
		pcm_par.dma_req_interval_time = dma_req_interval_time;
		pcm_par.buffer_size = buffer_size;
	}

	xgold_debug("PCM %s cmd mode %d rate %d buffer_mode %d dma_interval_time %d buffer_size :%d req %s\n",
			(type == STREAM_PLAY) ? "PLAY1" : "PLAY2",
			pcm_par.mode, pcm_par.rate, pcm_par.buffer_mode,
			pcm_par.dma_req_interval_time,
			pcm_par.buffer_size,
			(dma_mode == true) ? "DMA" : "PIO");

	cmd_data.command_id = (type == STREAM_PLAY) ?
		DSP_AUD_PCM1_PLAY : DSP_AUD_PCM2_PLAY;
	cmd_data.command_len = sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR);
	cmd_data.p_data = (u16 *)&pcm_par;

	dsp->p_dsp_common_data->ops->set_controls(
			dsp, DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC, &cmd_data);

	g_dsp_audio_dev->pb_running = true;

	return 0;
}

int dsp_pcm_rec(struct dsp_audio_device *dsp, unsigned int channels,
		unsigned int rate, bool dma_mode, unsigned int path_select)
{
	struct T_AUD_DSP_CMD_PCM_REC_PAR pcm_rec_par = { 0 };
	struct dsp_aud_cmd_data cmd_data;

	pcm_rec_par.setting = 3; /* Init & Go */
	pcm_rec_par.mode = get_dsp_pcm_channels(channels);
	pcm_rec_par.rate = get_dsp_pcm_rate(rate);
	pcm_rec_par.req = (dma_mode == true) ? 1 : 0;
	pcm_rec_par.path_select = path_select;

	xgold_debug("PCM_REC setting %d, cmd mode %d, rate %d, path_select %d",
			pcm_rec_par.setting, pcm_rec_par.mode,
			pcm_rec_par.rate, pcm_rec_par.path_select);

	cmd_data.command_id = DSP_AUD_PCM_REC;
	cmd_data.command_len = sizeof(struct T_AUD_DSP_CMD_PCM_REC_PAR);
	cmd_data.p_data = (u16 *)&pcm_rec_par;

	dsp->p_dsp_common_data->ops->set_controls(
			dsp, DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC, &cmd_data);

	g_dsp_audio_dev->rec_running = true;

	return 0;
}

int dsp_pcm_feed(struct dsp_audio_device *dsp, enum xgold_pcm_stream_type type,
		unsigned int channels, unsigned int rate,
		unsigned short buffer_mode,
		unsigned short dma_req_interval_time,
		unsigned short buffer_size)
{
	struct T_AUD_DSP_CMD_PCM_PLAY_PAR pcm_par = { 0 };
	struct dsp_aud_cmd_data cmd_data;

	pcm_par.setting = 2; /* Feed */
	pcm_par.mode = get_dsp_pcm_channels(channels);
	pcm_par.rate = get_dsp_pcm_rate(rate);
	pcm_par.req = 0;
	if (dsp->id == XGOLD_DSP_XG642) {
		pcm_par.buffer_mode = buffer_mode;
		pcm_par.dma_req_interval_time = dma_req_interval_time;
		pcm_par.buffer_size = buffer_size;
	}

	cmd_data.command_id = (STREAM_PLAY == type) ?
		DSP_AUD_PCM1_PLAY : DSP_AUD_PCM2_PLAY;
	cmd_data.command_len = sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR);
	cmd_data.p_data = (u16 *)&pcm_par;

	dsp->p_dsp_common_data->ops->set_controls(
			dsp,
			DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC,
			&cmd_data);

	return 0;
}

int dsp_pcm_stop(struct dsp_audio_device *dsp, enum xgold_pcm_stream_type type)
{
	struct T_AUD_DSP_CMD_PCM_PLAY_PAR pcm_par = { 0 };
	struct T_AUD_DSP_CMD_PCM_REC_PAR pcm_rec_par = { 0 };
	struct T_AUD_DSP_CMD_HW_PROBE hw_probe_par = { 0 };
	struct dsp_aud_cmd_data cmd_data;

	switch (type) {
	case STREAM_PLAY:
	case STREAM_PLAY2:
		pcm_par.setting = 0; /* Off */
		cmd_data.command_id = (type == STREAM_PLAY) ?
			DSP_AUD_PCM1_PLAY : DSP_AUD_PCM2_PLAY;
		cmd_data.command_len =
			sizeof(struct T_AUD_DSP_CMD_PCM_PLAY_PAR);
		cmd_data.p_data = (u16 *)&pcm_par;
		dsp->p_dsp_common_data->ops->set_controls(
				dsp,
				DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC,
				&cmd_data);
		g_dsp_audio_dev->pb_running = false;
		break;

	case STREAM_REC:
		pcm_rec_par.setting = 0;
		cmd_data.command_id = DSP_AUD_PCM_REC;
		cmd_data.command_len =
			sizeof(struct T_AUD_DSP_CMD_PCM_REC_PAR);
		cmd_data.p_data = (u16 *)&pcm_rec_par;
		dsp->p_dsp_common_data->ops->set_controls(
				dsp,
				DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC,
				&cmd_data);
		g_dsp_audio_dev->rec_running = false;
		break;

	case HW_PROBE_B:
	case HW_PROBE_A:
		hw_probe_par.setting = 0x0;
		cmd_data.command_id = DSP_AUD_HW_PROBE;
		cmd_data.command_len =
			sizeof(struct T_AUD_DSP_CMD_HW_PROBE);
		cmd_data.p_data = (u16 *)&hw_probe_par;

		dsp->p_dsp_common_data->ops->set_controls(
				dsp,
				DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC,
				&cmd_data);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

int dsp_cmd_hw_probe(
	struct dsp_audio_device *dsp,
	struct xgold_runtime_data *xgold_rtd)
{
	struct T_AUD_DSP_CMD_HW_PROBE hw_probe_par = { 0 };
	struct dsp_aud_cmd_data cmd_data;
	enum xgold_pcm_stream_type type =
		xgold_rtd->stream_type;
	enum xgold_hw_probe_stream_type hwp_id =
		((type == HW_PROBE_A) ? HW_PROBE_POINT_A : HW_PROBE_POINT_B);
	struct xgold_pcm_hw_probe_status *hwp_status =
		&xgold_rtd->pcm->hw_probe_status[hwp_id];

	hw_probe_par.setting = ((hwp_status->active == true) ? 0x1 : 0x0);
	hw_probe_par.sm_interface = (type == HW_PROBE_A) ? 0x1 : 0x2;
	hw_probe_par.probe_index = hwp_status->hw_probe_sel;
	hw_probe_par.mix_flag = 0x1;
	/* Injection gain should be 0dB if injection is used,
	 * and otherwise mute.
	 * Injection is not supported at the moment, so mute the gain.
	 */
	hw_probe_par.injection_gain = (U16)DSP_GAIN_MUTE_VALUE;

	cmd_data.command_id = DSP_AUD_HW_PROBE;
	cmd_data.command_len =
		sizeof(struct T_AUD_DSP_CMD_HW_PROBE);
	cmd_data.p_data = (u16 *)&hw_probe_par;

	dsp->p_dsp_common_data->ops->set_controls(
			dsp, DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC, &cmd_data);

	xgold_debug("hardware probe configured: setting: %d,\n",
		hw_probe_par.setting);
	xgold_debug("sm_interface: %d, probe_index: %d\n",
		hw_probe_par.sm_interface, hw_probe_par.probe_index);
	xgold_debug("mix_flag: %d, injection_gain: %d\n",
		hw_probe_par.mix_flag, hw_probe_par.injection_gain);

	return 0;
}

void dsp_cmd_afe_streaming_off(struct dsp_audio_device *dsp)
{
	struct dsp_aud_cmd_data dsp_cmd;
	struct T_AUD_DSP_CMD_VB_HW_AFE_PAR afe_hw_cmd = { 0 };

	if (dsp->id == XGOLD_DSP_XG642) {
		if (pm_runtime_active(dsp->dev)) {
			dsp_cmd.command_id = DSP_AUDIO_CMD_VB_HW_AFE;
			dsp_cmd.command_len =
				sizeof(struct T_AUD_DSP_CMD_VB_HW_AFE_PAR);
			dsp_cmd.p_data = (u16 *)&afe_hw_cmd;
			xgold_debug("%s: Send AFE OFF command\n", __func__);
			dsp->p_dsp_common_data->ops->set_controls(
				dsp,
				DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC,
				&dsp_cmd);
		}
	}
}

static int dsp_audio_reboot(struct notifier_block *notifier,
		unsigned long event, void *data)
{
	struct dsp_audio_device *dsp = g_dsp_audio_dev;
	struct T_AUD_DSP_CMD_VB_HW_AFE_PAR afe_hw_cmd = { 0 };

	if (dsp->dsp_sched_start) {
		xgold_err("%s: Scheduler is on. Turning it off\n", __func__);
		dsp->dsp_sched_start = 0;
		dsp_audio_cmd(DSP_AUDIO_CMD_VB_HW_AFE,
				sizeof(struct T_AUD_DSP_CMD_VB_HW_AFE_PAR),
				(u16 *)&afe_hw_cmd);

		if (dsp->pm_platdata)
			device_state_pm_set_state_by_name(dsp->dev,
					dsp->pm_platdata->pm_state_D3_name);
	}
	return NOTIFY_OK;
}
static struct notifier_block dsp_audio_reboot_notifier = {
	.notifier_call = dsp_audio_reboot
};

/* device probe function */
static int dsp_audio_drv_probe(struct platform_device *pdev)
{
	struct dsp_audio_device *dsp_dev;
	unsigned interrupt;
	struct resource *res;
	int ret = 0;
	int i;

	dsp_dev = devm_kzalloc(&pdev->dev, sizeof(struct dsp_audio_device),
						GFP_KERNEL);

	xgold_debug("line: %d\n", __LINE__);

	if (!dsp_dev) {
		xgold_err("Failed to allocate dsp audio device\n");
		return -ENOMEM;
	}
	dsp_dev->dev = &pdev->dev;

	/* Prepare DSP common data */
	if (!p_dsp_common_data) {
		p_dsp_common_data = kzalloc(sizeof(struct dsp_common_data),
							GFP_KERNEL);

		if (!p_dsp_common_data) {
			xgold_err("%s: Failed to allocate common dsp data\n",
					__func__);
			return -ENOMEM;
		}
	}
	dsp_dev->p_dsp_common_data = p_dsp_common_data;

	/* Allocate the memory for I2s devices control structure */
	for (i = 0; i < XGOLD_I2S_END; i++) {
		if (!dsp_dev->p_dsp_common_data->p_i2s_dev[i]) {
			dsp_dev->p_dsp_common_data->p_i2s_dev[i] =
				kzalloc((sizeof(struct dsp_i2s_device)),
						GFP_KERNEL);

			if (!dsp_dev->p_dsp_common_data->p_i2s_dev[i]) {
				xgold_err("%s: Failed to allocate I2S dev %d\n",
						__func__, i);
				return -ENOMEM;
			}
		}
	}

	dsp_dev->pm_platdata = of_device_state_pm_setup(pdev->dev.of_node);
	platform_set_drvdata(pdev, dsp_dev);

	if (IS_ERR(dsp_dev->pm_platdata)) {
		dev_warn(&pdev->dev, "Missing pm platdata properties\n");
		/* FIXME: for legacy only. Should never be NULL ! */
		dsp_dev->pm_platdata = NULL;
	}

	if (dsp_dev->pm_platdata)
		ret = platform_device_pm_set_class(pdev,
				dsp_dev->pm_platdata->pm_user_name);

	/* SHM REGS */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "shm-regs");
	if (!res) {
		xgold_err("Cannot get resource %s\n", "shm-regs");
		ret = -ENOENT;
		goto out;
	}

	if (!devm_request_mem_region
	    (&pdev->dev, res->start, resource_size(res), res->name)) {
		xgold_err("Request shm-regs region failed\n");
		ret = -EBUSY;
		goto out;
	}

	dsp_dev->shm_regs =
		devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!dsp_dev->shm_regs) {
		xgold_err("shm-regs ioremap failed\n");
		ret = -EBUSY;
		goto out;
	}
	pr_info("%s: ioremap for %pR returned %p\n", __func__, res,
			dsp_dev->shm_regs);

	/* SHM Memory */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "shm-mem");
	if (!res) {
		xgold_err("Cannot get resource %s\n", "shm-mem");
		ret = -ENOENT;
		goto out;
	}

	if (!devm_request_mem_region(&pdev->dev,
		res->start, resource_size(res), res->name)) {

		xgold_err("Request shm-mem region failed\n");
		ret = -EBUSY;
		goto out;
	}

	dsp_dev->shm_mem = devm_ioremap(&pdev->dev,
		res->start, resource_size(res));

	if (!dsp_dev->shm_mem) {
		xgold_err("shm-mem ioremap failed\n");
		ret = -EBUSY;
		goto out;
	}

	dsp_dev->shm_mem_phys = res->start;

	pr_info("%s: ioremap for %pR returned %p\n", __func__, res,
			dsp_dev->shm_mem);

	ret = dsp_audio_of_parse(&pdev->dev, dsp_dev);
	if (ret) {
		xgold_err("dsp dts file parsing failed\n");
		goto out;
	}

	if (dsp_dev->id == XGOLD_DSP_XG742_FBA)
		pr_info("dsp_id: %s\n", "XGOLD_DSP_XG742_FBA");
	if (dsp_dev->id == XGOLD_DSP_XG742_SBA)
		pr_info("dsp_id: %s\n", "XGOLD_DSP_XG742_SBA");

	/* Enable dsp power when in native mode */
	if (dsp_dev->pm_platdata) {
		ret = platform_device_pm_set_state_by_name(pdev,
				dsp_dev->pm_platdata->pm_state_D0_name);
		if (ret < 0)
			xgold_err("%s: failed to set PM state error %d\n",
			__func__, ret);
	}

	list_add_tail(&dsp_dev->node, &list_dsp);

	/* FIXME: those parameters must come from dts */
	dsp_init(dsp_dev, OFFSET_SM_MCU_CMD_0, SM_MCU_CMD_LENGHT);
	ret = dsp_audio_boot(dsp_dev);

	/* register DSP_INT1 interrupt handler */
	if (dsp_dev->interrupts[PLAYBACK_INT]) {
		interrupt = dsp_dev->interrupts[PLAYBACK_INT];
		ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
				dsp_audio_int1_lisr, dsp_audio_int1_hisr,
				IRQF_TRIGGER_RISING, "dsp_int1",
				dsp_dev);
		if (ret < 0) {
			xgold_err("FAILED to attach DSP_INT1 %d\n", ret);
			goto out;
		}
	} else if ((dsp_dev->id == XGOLD_DSP_XG742_SBA) &&
			(dsp_dev->interrupts[HW_PROBE])) {
		interrupt = dsp_dev->interrupts[HW_PROBE];
		ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
			dsp_audio_int1_lisr, dsp_audio_int1_hisr,
			IRQF_TRIGGER_RISING, "dsp_int1",
			dsp_dev);
		if (ret < 0) {
			xgold_err("FAILED to attach DSP_INT1 %d\n", ret);
			goto out;
		}
	}

	/* register DSP_INT2 interrupt handler */
	if (dsp_dev->interrupts[RECORD_INT]) {
		interrupt = dsp_dev->interrupts[RECORD_INT];
		ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
				dsp_audio_int2_lisr,
				dsp_audio_int2_hisr,
				IRQF_TRIGGER_RISING, "dsp_int2",
				dsp_dev);
		if (ret < 0) {
			xgold_err("FAILED to attach DSP_INT2 %d\n", ret);
			goto out;
		}
	}

	/* register DSP_INT3 interrupt handler */
	if ((dsp_dev->id == XGOLD_DSP_XG642) &&
		(dsp_dev->interrupts[HW_PROBE])) {
		interrupt = dsp_dev->interrupts[HW_PROBE];
		ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
			dsp_audio_int3_lisr,
			dsp_audio_int3_hisr,
			IRQF_TRIGGER_RISING, "dsp_int3",
			dsp_dev);

		if (ret < 0) {
			xgold_err("FAILED to attach DSP_INT3 %d\n", ret);
			goto out;
		}
	}

	/* register FBA_INT2 interrupt handler */
	if (dsp_dev->interrupts[SPEECH_PROBES]) {
		interrupt = dsp_dev->interrupts[SPEECH_PROBES];
		ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
				dsp_audio_fba_int2_lisr,
				dsp_audio_fba_int2_hisr,
				IRQF_TRIGGER_RISING, "dsp_fba_int2",
				dsp_dev);
		if (ret < 0) {
			xgold_err("\n FAILED to attach FBA DSP_INT2 %d\n", ret);
			goto out;
		}
	}

	init_rwsem(&dsp_audio_cb_rwsem);

	/* Register dsp with client only after all dsp's in the
	 * system have initialised, because the number of dsp's
	 * in the system is transparent to the higher layers */
	/*FIXME: What about other DSP types ? */
	if (dsp_dev->p_dsp_common_data->num_dsp == 2 ||
			dsp_dev->id == XGOLD_DSP_XG642) {

		g_dsp_audio_dev->name = "DSP AUDIO DEV";
		g_dsp_audio_dev->p_dsp_common_data->ops = &dsp_dev_ops;

		g_dsp_audio_dev->p_dsp_common_data->i2s_set_power_state =
			dsp_set_i2s_power_state;

		dsp_audio_init(&list_dsp);

		/* FIXME: native for other DSP types ? */
		if (dsp_dev->id == XGOLD_DSP_XG642 &&
				audio_native_mode)
			dsp_start_audio_sched(dsp_dev);
	}

	/* initializaion for playback and record */
	g_dsp_audio_dev->pb_running = false;
	g_dsp_audio_dev->rec_running = false;

	pr_info("DSP initialization done %d\n", ret);
	register_reboot_notifier(&dsp_audio_reboot_notifier);

out:

	/* BU_HACK memory retention mode not working on ES 1.0
	 * keep the DSP on always */
	if (dsp_dev->pm_platdata)
		platform_device_pm_set_state_by_name(pdev,
				dsp_dev->pm_platdata->pm_state_D0i3_name);

	pm_runtime_enable(dsp_dev->dev);
	return ret;
}

/* device remove function */
static int dsp_audio_drv_remove(struct platform_device *pdev)
{
	/*FIXME: kfree*/
	return 0;
}

static void dsp_audio_drv_shutdown(struct platform_device *pdev)
{
	struct dsp_audio_device *dsp = platform_get_drvdata(pdev);
	struct T_AUD_DSP_CMD_VB_HW_AFE_PAR afe_hw_cmd = { 0 };
	struct dsp_aud_cmd_data cmd_data;
	int power_control = 0;
	cmd_data.command_id = DSP_AUDIO_CMD_VB_HW_AFE;
	cmd_data.command_len = sizeof(struct T_AUD_DSP_CMD_VB_HW_AFE_PAR);
	cmd_data.p_data = (u16 *)&afe_hw_cmd;

	if (dsp->dsp_sched_start) {
		xgold_err("%s: Scheduler is on. Turning it off\n", __func__);
		if (pm_runtime_active(dsp->dev)) {
			dsp->p_dsp_common_data->ops->set_controls(
				dsp,
				DSP_AUDIO_CONTROL_SEND_CMD_ATOMIC,
				&cmd_data);
		}
	}
	dsp_audio_dev_set_controls(dsp, DSP_AUDIO_POWER_REQ, &power_control);
}

static struct of_device_id xgold_snd_dsp_of_match[] = {
	{ .compatible = "intel,xgold-snd-dsp", },
	{ },
};

/* dsp platform driver */
static struct platform_driver dsp_audio_driver = {
	.driver = {
		   .name = "intel-dsp",
		   .owner = THIS_MODULE,
		   .of_match_table = xgold_snd_dsp_of_match,
		   .pm = &dsp_audio_pm,
	},
	.shutdown = dsp_audio_drv_shutdown,
	.probe = dsp_audio_drv_probe,
	.remove = dsp_audio_drv_remove,
};

static int __init dsp_audio_drv_init(void)
{
	int ret = 0;

	xgold_debug("dsp audio driver loaded\n");
	ret = platform_driver_register(&dsp_audio_driver);
	if (ret)
		xgold_err("dsp audio platform register failed\n");
	return ret;
}

static void __exit dsp_audio_drv_exit(void)
{
	platform_driver_unregister(&dsp_audio_driver);

	xgold_debug("dsp audio driver unloaded\n");
	return;
}

module_init(dsp_audio_drv_init);
module_exit(dsp_audio_drv_exit);

MODULE_ALIAS("dsp audio");
MODULE_DESCRIPTION("XGOLD Audio DSP Driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL V2");
MODULE_DEVICE_TABLE(of, xgold_snd_dsp_of_match);
