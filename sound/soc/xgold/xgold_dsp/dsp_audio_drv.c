/*
 * Component: XGOLD DSP Audio Driver
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

#include "xgold_machine.h"
#include "dsp_audio_platform.h"
#include "dsp_audio_driverif.h"
#include "aud_lib_dsp_internal.h"

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

/* Function to register the callbacks for interrupt type of enum dsp_lisr_cb */
int register_dsp_audio_lisr_cb(enum dsp_lisr_cb lisr_type,
	void (*p_func) (void *), void *dev)
{
	if (lisr_type >= DSP_LISR_CB_END)
		return -EINVAL;

	xgold_debug("Registering %d type\n", lisr_type);
	down_write(&dsp_audio_cb_rwsem);
	g_dsp_audio_cb_list[lisr_type].p_func = p_func;
	g_dsp_audio_cb_list[lisr_type].dev = dev;
	up_write(&dsp_audio_cb_rwsem);
	return 0;
}

/* Function to invoke the registered callbacks for interrupts */
static void dsp_audio_lisr_cb_handler(enum dsp_irq_no intr_no,
				      void *dev)
{
	int comm_flag = 0;
	int i = 0;
	const struct dsp_lisr_cb_conf *p_dsp_aud_lisr_db;

	xgold_debug("in %s intr no: %d\n", __func__, intr_no);

	down_read(&dsp_audio_cb_rwsem);
	p_dsp_aud_lisr_db = dsp_audio_get_lisr_db(intr_no);

/* find the set communication flag and call the registered callback function */
	if (p_dsp_aud_lisr_db != NULL) {
		while (p_dsp_aud_lisr_db[i].lisr_cb_type !=
			DSP_LISR_CB_END) {
			comm_flag =
			    dsp_read_audio_dsp_communication_flag
			    (p_dsp_aud_lisr_db[i].comm_flag_no);
			if (comm_flag) {
				dsp_reset_audio_dsp_communication_flag
				    (p_dsp_aud_lisr_db[i].comm_flag_no);
				if (g_dsp_audio_cb_list
				    [p_dsp_aud_lisr_db[i].lisr_cb_type].
				    p_func != NULL) {
					(g_dsp_audio_cb_list
					 [p_dsp_aud_lisr_db[i].lisr_cb_type].
					 p_func) (g_dsp_audio_cb_list
						  [p_dsp_aud_lisr_db[i].
						   lisr_cb_type].dev);
				}
			}
			i++;
		}
	}
	up_read(&dsp_audio_cb_rwsem);
}

/* LISR for DSP_INT1 */
static irqreturn_t dsp_audio_int1_lisr(int irq, void *dev)
{
	xgold_debug("In %s\n", __func__);

	/* clear the interrupt */
	dsp_audio_irq_ack(DSP_IRQ_1);

	return IRQ_WAKE_THREAD;
}

/* LISR for DSP_INT2 */
static irqreturn_t dsp_audio_int2_lisr(int irq, void *dev)
{
	xgold_debug("In %s\n", __func__);

	/* clear the interrupt */
	dsp_audio_irq_ack(DSP_IRQ_2);

	return IRQ_WAKE_THREAD;
}

/* HISR for DSP_INT1 */
static irqreturn_t dsp_audio_int1_hisr(int irq, void *dev)
{
	xgold_debug("In %s\n", __func__);
	dsp_audio_lisr_cb_handler(DSP_IRQ_1, dev);
	return IRQ_HANDLED;
}

/* HISR for DSP_INT2 */
static irqreturn_t dsp_audio_int2_hisr(int irq, void *dev)
{
	xgold_debug("In %s\n", __func__);
	dsp_audio_lisr_cb_handler(DSP_IRQ_2, dev);
	return IRQ_HANDLED;
}

/*FIXME remove static variable */
/* dsp device */
static struct dsp_audio_device *g_dsp_audio_dev;

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
	Consider creating a variant file for future platforms */
static void dsp_audio_mark_scheduler_status(struct dsp_aud_cmd_data *p_cmd_data)
{
	#define PARM_ON 1
	#define PARM_ON_UPDATE 3

	unsigned short *data_trace_ptr;
	xgold_debug("%s:\n", __func__);

	if ((NULL != p_cmd_data) && (NULL != p_cmd_data->p_data)) {
		data_trace_ptr = (unsigned short *)p_cmd_data->p_data;
		/* Check if the command is to start dsp scheduler */
		if (DSP_AUDIO_CMD_VB_HW_AFE == p_cmd_data->command_id) {
			if ((PARM_ON == data_trace_ptr[0]) ||
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

/* device controls handler */
static int dsp_audio_dev_set_controls(enum dsp_audio_controls cmd, void *arg)
{
	int ret_val = 0;
	struct dsp_rw_shm_data *p_rw_shm = NULL;
	struct dsp_aud_cmd_data *p_cmd_data = NULL;

	unsigned short *data_trace_ptr;
	unsigned short i;

	xgold_debug("In :%s, cmd:%d\n", __func__, cmd);

	if (!arg)
		return -EINVAL;

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
		ret_val = (int)dsp_audio_cmd(
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
			p_rw_shm->p_data,
			p_rw_shm->word_offset,
			p_rw_shm->len_in_bytes);
		break;

	case DSP_AUDIO_CONTROL_WRITE_SHM:
		/* control to write data to the shared memory */
		p_rw_shm = (struct dsp_rw_shm_data *) arg;
		ret_val = (int)dsp_audio_write_shm(
			p_rw_shm->p_data,
			p_rw_shm->word_offset,
			p_rw_shm->len_in_bytes);
		break;

	default:
		break;
	}

	return ret_val;
}

/* dsp device operations */
static struct dsp_ops dsp_dev_ops = {
	.open = dsp_audio_dev_open,
	.set_controls = dsp_audio_dev_set_controls,
	.close = dsp_audio_dev_close,
};

/* runtime PM operations of dsp device */
static int dsp_audio_runtime_suspend(struct device *dev)
{
	xgold_debug("dsp_audio_runtime_suspend called\n");
	return 0;
}

static int dsp_audio_runtime_resume(struct device *dev)
{
	xgold_debug("dsp_audio_runtime_resume called\n");
	return 0;
}

static int dsp_audio_runtime_idle(struct device *dev)
{
	xgold_debug("dsp_audio_runtime_idle called\n");
	return 0;
}

static const struct dev_pm_ops dsp_audio_pm = {
	.suspend = dsp_audio_runtime_suspend,
	.resume = dsp_audio_runtime_resume,
	.runtime_suspend = dsp_audio_runtime_suspend,
	.runtime_resume = dsp_audio_runtime_resume,
	.runtime_idle = dsp_audio_runtime_idle,
};

/* static functions */
enum dsp_interrupt {
	PLAYBACK_INT,
	PLAYBACK2_INT,
	RECORD_INT,
	TONE_INT,
	I2S_INT,
	AUDIO_OV_USB,
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
	else
		return -EINVAL;
}

#define PROP_DSP_INT_IMSC		"intel,dsp-imsc"
#define PROP_DSP_INT_ICR		"intel,dsp-icr"
#define PROP_DSP_INT_MIS		"intel,dsp-mis"
#define PROP_DSP_CF			"intel,dsp-cf"
#define PROP_DSP_REMAIN_PCM		"intel,dsp-pcm-offset"

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

static int xgold_dsp_fill_interrupt_array(struct dsp_audio_device *dsp)
{
	struct device_node *np = dsp->dev->of_node;
	unsigned out_imsc[8];
	unsigned out_icr[8];
	unsigned out_mis[8];
	unsigned out_cf[8];
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
		dsp_reg->reg = (void *)out_values[0];
		dsp_reg->shift = (unsigned char)out_values[1];
		dsp_reg->width = (unsigned char)out_values[2];
	}

	return 0;

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

static int dsp_audio_of_parse(struct device *dev, struct dsp_audio_device *dsp)
{
	struct device_node *np = dev->of_node;
	const char *name;
	int ret;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
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
#endif /* CONFIG_PLATFORM_DEVICE_PM_VIRT */

	if (dsp->native_mode) {
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
	ret = xgold_dsp_fill_interrupt_array(dsp);
	if (ret)
		xgold_err("Interrupt array assignement failed\n");

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

	/* PCM remaining block offset */
	ret = of_property_read_u32_array(np, PROP_DSP_REMAIN_PCM,
			dsp->pcm_offset, 2);
	if (ret) {
		xgold_err("Could not find property %s\n", PROP_DSP_REMAIN_PCM);
		return ret;
	}

	/* DSP identification */
	ret = of_property_read_string(np, PROP_DSP_ID, &name);
	if (ret)
		xgold_err("cannot get DSP name id\n");

	if (!strcmp(name, "XG631"))
		/* ret = of_xgold631_dsp_register(dsp, np);
		if (ret)
			goto err_dsp_id;*/

		dsp->id = XGOLD_DSP_XG631;
	else if (!strcmp(name, "XG632"))
		dsp->id = XGOLD_DSP_XG632;
	else if (!strcmp(name, "XG642"))
		dsp->id = XGOLD_DSP_XG642;
	else {
		xgold_err("name id '%s' doesn't match\n", name);
		ret = -EINVAL;
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

static void __dsp_write_imsc(struct dsp_audio_device *dsp, unsigned val)
{
	dsp_writel(val, dsp->shm_regs + dsp->imsc);
}

static void dsp_reset(struct dsp_audio_device *dsp)
{
	reset_control_assert(dsp->rst_ctl);
	udelay(200);
	reset_control_deassert(dsp->rst_ctl);
	__dsp_write_clc(dsp, 0x100);

	/* switch in RUN mode */
	__dsp_write_rstmods(dsp, 1);
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
	mcu2dsp |= BIT(id);
	iowrite32(mcu2dsp, dsp->shm_regs + dsp->mcu2dsp);
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

#define DSP_NAME "DSPXG631"

#define OFFSET_SM_FW_VERSION                       0
#define OFFSET_SM_HW_VERSION                       1
#define OFFSET_SM_CUSTOMER_INTERFACE_VERSION       2
#define OFFSET_SM_STARTUP_DEBUG_VERSION            3
static void dsp_show_info(struct dsp_audio_device *dsp)
{

	xgold_debug("%s: Firmware version 0x%04x\n", DSP_NAME,
		dsp_read_reg(dsp, OFFSET_SM_FW_VERSION));
	xgold_debug("%s: Hardware version 0x%04x\n", DSP_NAME,
		dsp_read_reg(dsp, OFFSET_SM_HW_VERSION));
	xgold_debug("%s: Customer itf version 0x%04x\n", DSP_NAME,
		dsp_read_reg(dsp, OFFSET_SM_CUSTOMER_INTERFACE_VERSION));
	xgold_debug("%s: Startup dbg version version 0x%04x\n", DSP_NAME,
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

static struct xgold_dsp_cmd  *dsp_post_command(struct dsp_audio_device *dsp,
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
	int ret;

	memset(&afe, 0, sizeof(struct s_dsp_cmd_vb_hw_afe));
	afe.cmd_switch = 1;	/* On & Update */
	afe.ratesw = 0;		/* Hard Code 8 Khz */

	ret = dsp_do_command(g_dsp_audio_dev, VB_HW_AFE, &afe, NULL);
	if (ret)
		xgold_err("VB_HW_AFE command generated error\n");
	xgold_debug("VB_HW_AFE command sent\n");

	return ret;
}
EXPORT_SYMBOL(dsp_start_audio_hwafe);

#define OFFSET_SM_BOOT_DATA		OFFSET_SM_CUSTOMER_INTERFACE_VERSION
#define OFFSET_SM_MCU_CMD_0             4

/* device probe function */
static int dsp_audio_drv_probe(struct platform_device *pdev)
{
	struct dsp_audio_device *dsp;
	unsigned imsc, interrupt;
	struct resource *res;
	int ret;

	dsp = devm_kzalloc(&pdev->dev, sizeof(struct dsp_audio_device),
			GFP_KERNEL);
	if (!dsp)
		return -ENOMEM;

	dsp->dev = &pdev->dev;
	dsp->dsp_sched_start = 0;
	dsp->native_mode = audio_native_mode;
	platform_set_drvdata(pdev, dsp);
	dsp->pm_platdata = of_device_state_pm_setup(pdev->dev.of_node);
	if (IS_ERR(dsp->pm_platdata)) {
		dev_warn(&pdev->dev, "Missing pm platdata properties\n");
		/* FIXME: for legacy only. Should never be NULL ! */
		dsp->pm_platdata = NULL;
	}

	if (dsp->pm_platdata)
		ret = platform_device_pm_set_class(pdev,
				dsp->pm_platdata->pm_user_name);

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

	dsp->shm_regs =
		devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!dsp->shm_regs) {
		xgold_err("shm-regs ioremap failed\n");
		ret = -EBUSY;
		goto out;
	}
	pr_info("%s: ioremap for %X size %X returned %p\n", __func__,
			res->start, resource_size(res), dsp->shm_regs);

	/* SHM Memory */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "shm-mem");
	if (!res) {
		xgold_err("Cannot get resource %s\n", "shm-mem");
		ret = -ENOENT;
		goto out;
	}

	if (!devm_request_mem_region
	    (&pdev->dev, res->start, resource_size(res), res->name)) {
		xgold_err("Request shm-mem region failed\n");
		ret = -EBUSY;
		goto out;
	}

	dsp->shm_mem = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!dsp->shm_mem) {
		xgold_err("shm-mem ioremap failed\n");
		ret = -EBUSY;
		goto out;
	}
	dsp->shm_mem_phys = res->start;
	pr_info("%s: ioremap for %X size %X returned %p\n", __func__,
			res->start, resource_size(res), dsp->shm_mem);

	ret = dsp_audio_of_parse(&pdev->dev, dsp);
	if (ret) {
		xgold_err("dsp dts file parsing failed\n");
		goto out;
	}

	imsc = ioread32(dsp->shm_regs + dsp->imsc);

	/* register DSP_INT1 interrupt handler */
	interrupt = dsp->interrupts[PLAYBACK_INT];
	ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
			dsp_audio_int1_lisr, dsp_audio_int1_hisr,
			IRQF_DISABLED, "dsp_int1", dsp);
	if (ret < 0) {
		xgold_err("\n FAILED to attach DSP_INT1 %d\n", ret);
		goto out;
	}
	imsc |= BIT(DSP_INT_GET_IMSC(interrupt));

	/* register DSP_INT2 interrupt handler */
	interrupt = dsp->interrupts[RECORD_INT];
	ret = request_threaded_irq(DSP_INT_GET_ID(interrupt),
			dsp_audio_int2_lisr, dsp_audio_int2_hisr,
			IRQF_DISABLED, "dsp_int2", dsp);
	if (ret < 0) {
		xgold_err("\n FAILED to attach DSP_INT2 %d\n", ret);
		goto out;
	}
	imsc |= BIT(DSP_INT_GET_IMSC(interrupt));

	init_rwsem(&dsp_audio_cb_rwsem);

	dsp->name = "DSP AUDIO DEV",
	dsp->ops = &dsp_dev_ops,
	g_dsp_audio_dev = dsp; /* FIXME do not use global static variable */
	init_rwsem(&dsp_audio_cb_rwsem);
	register_audio_dsp(dsp);
	dsp_audio_init();

	if (dsp->native_mode) {
		dsp_init(dsp, OFFSET_SM_MCU_CMD_0, SM_MCU_CMD_LENGHT);
		ret = dsp_clock_init(dsp);

		if (ret)
			xgold_err("error during pm init\n");

		dsp_reset(dsp);
		__dsp_write_imsc(dsp, imsc);
		ret = dsp_patch(dsp, OFFSET_SM_BOOT_DATA);

		if (ret)
			xgold_err("error during FW download\n");

		dsp_start_audio_sched(dsp);
	}

	pr_info("DSP initialization done %d\n", ret);

out:
	return ret;
}

/* device remove function */
static int dsp_audio_drv_remove(struct platform_device *pdev)
{
	struct dsp_audio_device *dsp = platform_get_drvdata(pdev);
	unregister_audio_dsp(dsp);

	return 0;
}

static void dsp_audio_drv_shutdown(struct platform_device *pdev)
{
	struct T_AUD_DSP_CMD_VB_HW_AFE_PAR afe_hw_cmd = { 0 };
	xgold_debug("dsp_audio_drv_shutdown\n");

	if (g_dsp_audio_dev->dsp_sched_start) {
		xgold_err("%s: Scheduler is on. Turning it off\n", __func__);
		dsp_audio_cmd(DSP_AUDIO_CMD_VB_HW_AFE,
				sizeof(struct T_AUD_DSP_CMD_VB_HW_AFE_PAR),
				(u16 *)&afe_hw_cmd);
	}
}

static struct of_device_id xgold_snd_dsp_of_match[] = {
	{ .compatible = "intel,xgold-snd-dsp", },
	{ },
};

/* dsp platform driver */
static struct platform_driver dsp_audio_driver = {
	.driver = {
		   .name = "dsp_audio",
		   .owner = THIS_MODULE,
		   .of_match_table = xgold_snd_dsp_of_match,
		   },
	.probe = dsp_audio_drv_probe,
	.remove = dsp_audio_drv_remove,
	.shutdown = dsp_audio_drv_shutdown,
#if 0 /* TODO: enable power management */
	.driver = {
		   .pm = &dsp_audio_pm,
		   },
#endif
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
