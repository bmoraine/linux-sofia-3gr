/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

/******************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <linux/module.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/i2c.h>
#include <linux/platform_data/i2c-xgold.h>

#include "i2c-xgold.h"

/******************************************************************************
 * DEFINES
 *****************************************************************************/
#define XGOLD_I2C_TIMEOUT			50
#define XGOLD_I2C_SLAVE_ADDR_7_BIT_MASK		0x7f
#define XGOLD_I2C_SLAVE_ADDR_10_BIT_MASK	0x3ff

#define I2C_MSG_WR(_flag_) \
	(0 == (_flag_ & I2C_M_RD))

#define I2C_MSG_RD(_flag_) \
	(I2C_M_RD == (_flag_ & I2C_M_RD))

#define I2C_MSG_ADR10(_flag_) \
	(I2C_M_TEN == (_flag_ & I2C_M_TEN))

#define I2C_PKT_FIRST_DATA_LENGTH_IN_BYTES \
	(XGOLD_I2C_BYTES_IN_REG - 1)

/* slave address constructors */
#define I2C_PKT_ADR(_flag_, _address_) \
	((_address_ << 1) | (_flag_ & I2C_M_RD))

#define reg_write(_r_, _a_) {\
	/* i2c_debug("wr @%p -- 0x%08x\n",_a_, _r_); */\
	iowrite32(_r_, _a_); }


/******************************************************************************
 * TYPE DEFINITIONS
 *****************************************************************************/
enum xgold_i2c_drv_state {
	XGOLD_I2C_LISTEN = 0,
	XGOLD_I2C_TRANSMIT,
	XGOLD_I2C_RECEIVE,
	XGOLD_I2C_RESTART,
	XGOLD_I2C_CONFIGURE,
};


/******************************************************************************
 * FUNCTION PROTOTYPE
 *****************************************************************************/


/******************************************************************************
 * PLATFORM DEVICE PM
 ******************************************************************************/

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_i2c_set_pm_state(
		struct device *, struct device_state_pm_state *);
static struct device_state_pm_state *xgold_i2c_get_initial_state(
		struct device *);

static struct device_state_pm_ops i2c_pm_ops = {
	.set_state = xgold_i2c_set_pm_state,
	.get_initial_state = xgold_i2c_get_initial_state,
};

/* I2C PM states & class */
static struct device_state_pm_state i2c_pm_states[] = {
	{ .name = "disable", }, /* D3 */
	{ .name = "enable", }, /* D0 */
	{ .name = "enable_psv", },
};

DECLARE_DEVICE_STATE_PM_CLASS(i2c);

/* PM states index */
#define XGOLD_I2C_D0		1
#define XGOLD_I2C_D3		0

static int xgold_i2c_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	int id = device_state_pm_get_state_id(dev, state->name);
	int ret;

	pr_info("%s: pm state %s\n", __func__, state->name);
	switch (id) {
	case XGOLD_I2C_D0:
		clk_prepare_enable(platdata->clk_bus);
		clk_prepare_enable(platdata->clk_kernel);

		/* Enable regulator if any */
		if (platdata->regulator) {
			ret = regulator_enable(platdata->regulator);
			if (ret)
				return ret;
		}

		if (platdata->regulator2) {
			ret = regulator_enable(platdata->regulator2);
			if (ret)
				return ret;
		}
		break;

	case XGOLD_I2C_D3:
		clk_disable_unprepare(platdata->clk_kernel);
		clk_disable_unprepare(platdata->clk_bus);

		if (platdata->regulator)
			regulator_disable(platdata->regulator);

		if (platdata->regulator2)
			regulator_disable(platdata->regulator2);

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static struct device_state_pm_state *xgold_i2c_get_initial_state(
		struct device *dev)
{
	return &i2c_pm_states[XGOLD_I2C_D3];
}

#endif /* CONFIG_PLATFORM_DEVICE_PM_VIRT */


/******************************************************************************
 * DEVICE TREE DECODE
 ******************************************************************************/
#ifdef CONFIG_OF

#define OF_KERNEL_CLK		"clk_kernel"
#define OF_AHB_CLK		"clk_ahb"
#define OF_CFG_B400		"intel,i2c,b400"
#define OF_CFG_B100		"intel,i2c,b100"
#define OF_I2C_DMA_REQ		"intel,i2c,dma"

static struct xgold_i2c_platdata *xgold_i2c_of_get_platdata(struct device *dev)
{
	struct xgold_i2c_platdata *platdata;
	struct device_node *np;
	int ret = 0, i, n;
	char id;

	platdata = devm_kzalloc(dev, sizeof(*platdata), GFP_KERNEL);
	if (!platdata)
		return NULL;

/* Required properties */

	/* Get device node pointer */
	np = dev->of_node;

	if (!np) {
		i2c_err("Can't find i2c matching node\n");
		return NULL;
	}

	/* FIXME: use another property */
	id = *(np->name + strlen(np->name) - 1) - 0x30;
	WARN_ON(id < 0 || id > 9);
	platdata->id = id;

	dev->init_name = np->name;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	/* clock */
	platdata->clk_kernel = of_clk_get_by_name(np, OF_KERNEL_CLK);
	if (IS_ERR(platdata->clk_kernel)) {
		i2c_err("Clk %s not found\n", OF_KERNEL_CLK);
	}

	platdata->clk_bus = of_clk_get_by_name(np, OF_AHB_CLK);
	if (IS_ERR(platdata->clk_bus)) {
		i2c_err("Clk %s not found\n", OF_AHB_CLK);
	}
#endif

	/* Interrupts */
	platdata->irq_num = n = of_irq_count(np);
	if (!n) {
		i2c_err("Error parsing interrupts in %s\n", np->name);
		return NULL;
	}

	for (i = n; i > 0; i--) {
		ret = irq_of_parse_and_map(np, i - 1);
		if (!ret) {
			i2c_err("cannot map irq index %d", i);
			return NULL;
		}
	}

	platdata->irq = ret; /* save only the first interrupt id */

	/* FDIV and TIMINGS data for 100kbps baud rate */
	if (of_property_read_u32_array(np, OF_CFG_B100,
		platdata->cfg_baud[I2C_XGOLD_B100], 4) < 0) {

		i2c_err("Error parsing %s propery of node %s\n",
			OF_CFG_B100, np->name);
		return NULL;
	}

	/* FDIV and TIMINGS data for 400kbps baud rate */
	if (of_property_read_u32_array(np, OF_CFG_B400,
		platdata->cfg_baud[I2C_XGOLD_B400], 4) < 0) {

		i2c_err("Error parsing %s propery of node %s\n",
			OF_CFG_B400, np->name);
		return NULL;
	}

	/* pinctrl */
	platdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(platdata->pinctrl))
		goto skip_pinctrl;

	platdata->pins_default = pinctrl_lookup_state(platdata->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(platdata->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	platdata->pins_sleep = pinctrl_lookup_state(platdata->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(platdata->pins_sleep))
		dev_err(dev, "could not get sleep pinstate\n");

	platdata->pins_inactive = pinctrl_lookup_state(platdata->pinctrl,
			"inactive");
	if (IS_ERR(platdata->pins_inactive))
		dev_err(dev, "could not get inactive pinstate\n");

skip_pinctrl:
	/* pm */
	platdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(platdata->pm_platdata)) {
		dev_err(dev, "Error during device state pm init\n");
		return NULL;
	}

/* Optional properties */
	/* DMA request */
	if (of_property_read_string(np, OF_I2C_DMA_REQ, &platdata->dma_txrx))
		platdata->dma_txrx = NULL;

	/* Reset */
	platdata->rst = reset_control_get(dev, "i2c");
	if (IS_ERR(platdata->rst))
		platdata->rst = NULL;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	/* Regulator */
	platdata->regulator = regulator_get(dev, "i2c");
	if (IS_ERR(platdata->regulator))
		platdata->regulator = NULL;

	platdata->regulator2 = regulator_get(dev, "i2c2");
	if (IS_ERR(platdata->regulator2))
		platdata->regulator2 = NULL;
#endif

	return platdata;
}
#endif /* CONFIG_OF */

static inline int xgold_i2c_set_pinctrl_state(struct device *dev,
		struct pinctrl_state *state)
{
	struct xgold_i2c_platdata *pdata = dev_get_platdata(dev);
	int ret = 0;

	if (!pdata) {
		dev_err(dev, "Unable to retrieve i2c platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}



/******************************************************************************
 * FUNCTION IMPLEMENTATION
 *****************************************************************************/

/* DMA engine API usage */
static void xgold_i2c_dma_finish(struct xgold_i2c_algo_data *data)
{
	if (I2C_MSG_RD(data->flags)) {
		dma_unmap_sg(data->i2c_dev->dev, &data->sg_io, 1,
				DMA_FROM_DEVICE);
	} else {
		dma_unmap_sg(data->i2c_dev->dev, &data->sg_io, 1,
				DMA_TO_DEVICE);
		kfree(data->dma_buf);
	}
}

static void xgold_i2c_dma_callback_txrx(void *param)
{
	struct xgold_i2c_algo_data *data = param;

	if (dma_async_is_tx_complete(data->dmach, data->dma_cookie, NULL, NULL)
			== DMA_COMPLETE) {
		i2c_debug("DMA transfer complete\n");
	} else {
		data->cmd_err = -EIO;
		i2c_debug("DMA error\n");
	}

	xgold_i2c_dma_finish(data);
	if (data->buf_len == 0)
		complete(&data->cmd_complete);
	else
		data->buf_len = 0;

	i2c_debug("<-- %s\n", __func__);
}

static int xgold_i2c_dma_config(struct xgold_i2c_algo_data *data,
				unsigned int flags)
{
	struct dma_slave_config config;
	int ret;

	if (!data->dmach)
		return -EPERM;

	if (I2C_MSG_RD(flags)) {
		config.direction = DMA_FROM_DEVICE;
		config.src_addr = (dma_addr_t)(data->regs + I2C_RXD_OFFSET);
		config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config.src_maxburst = 4;
		config.device_fc = true;
	} else {
		config.direction = DMA_TO_DEVICE;
		config.dst_addr = (dma_addr_t)(data->regs + I2C_TXD_OFFSET);
		config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		config.dst_maxburst = 4;
		config.device_fc = true;
	}

	ret = dmaengine_slave_config(data->dmach, &config);

	return ret;
}

static int xgold_i2c_dma_setup_xfer(struct xgold_i2c_algo_data *data)
{
	struct dma_async_tx_descriptor *desc;

	if (I2C_MSG_RD(data->flags)) {
		/* Queue the read buffer */
		sg_init_one(&data->sg_io, data->buf,
				round_up(data->buf_len, 4));
		dma_map_sg(data->i2c_dev->dev, &data->sg_io, 1,
				DMA_FROM_DEVICE);
		desc = dmaengine_prep_slave_sg(data->dmach, &data->sg_io, 1,
				DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);

		if (!desc) {
			i2c_err("Failed to get DMA desc for RD buffer\n");
			goto read_buff_dma_fail;
		}
	} else {
		__u8 *addr = (__u8 *)&data->addr;
		data->dma_buf = kmalloc(sizeof(char) * (data->buf_len + 1),
				GFP_KERNEL);
		if (!data->dma_buf)
			return -ENOMEM;

		data->dma_buf[0] = I2C_PKT_ADR(data->flags, addr[0]);
		memcpy(data->dma_buf + 1, data->buf, data->buf_len);

		/*
		 * Prepare DMA buffer.
		 *
		 * Must use round_up to force the pl08x driver to select
		 * S/D WITDH as 32bits (Word), and not Half-Word nor Byte,
		 * which would prevent the I2C to send odds amount of data.
		 */
		sg_init_one(&data->sg_io, data->dma_buf,
			round_up(data->buf_len + 1, 4));

		dma_map_sg(data->i2c_dev->dev, &data->sg_io, 1, DMA_TO_DEVICE);
		desc = dmaengine_prep_slave_sg(data->dmach, &data->sg_io, 1,
			DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);

		if (!desc) {
			i2c_err("Failed to get DMA data write descriptor\n");
			goto write_init_dma_fail;
		}
	}

	/* Set the DMA callback */
	desc->callback = xgold_i2c_dma_callback_txrx;
	desc->callback_param = data;

	/* Start transfer */
	data->dma_cookie = dmaengine_submit(desc);
	dma_async_issue_pending(data->dmach);

	return 0;

read_buff_dma_fail:
	dma_unmap_sg(data->i2c_dev->dev, &data->sg_io, 1, DMA_FROM_DEVICE);
	dmaengine_terminate_all(data->dmach);
	return -EINVAL;

write_init_dma_fail:
	dma_unmap_sg(data->i2c_dev->dev, &data->sg_io, 1, DMA_TO_DEVICE);
	dmaengine_terminate_all(data->dmach);
	return -EINVAL;
}

/*****************************************************************************
 * Function:... xgold_i2c_get_client_baud()
 * Description:  Linux I2C subsystem does not support Baud rate
 *				selection by Client, I2C_MSG flag = 0x7c11 are
 *				reserved, 8 and 9 bits are free. 8th bit will
 *				be set by client for baudrate 100 kbps, 9th
 *				bit will be set by client for baudrate 400
 *				kbps, If none of the bits are set, default
 *				baud rate will be set.
 *****************************************************************************/
inline int xgold_i2c_get_client_baud(unsigned int flags)
{
	if (I2C_MSG_B100(flags))
		return I2C_XGOLD_B100;

	return I2C_XGOLD_B400;
}

/******************************************************************************
 * Function:... xgold_i2c_xmit_word()
 * Description: (Slave Address + Client data) are written
 *              in I2C_TXD register
 *****************************************************************************/
static void xgold_i2c_xmit_word(struct xgold_i2c_algo_data *data)
{
	int copy, to_copy, i = 0;
	unsigned int word = 0;
	__u8 *ptr = (__u8 *) &word;
	__u8 *addr = (__u8 *) &data->addr;

	to_copy = data->buf_len;
	to_copy += (data->addr_sent == 0) ? 1 : 0;
	copy = (to_copy < 4) ? to_copy : 4;

	if (data->addr_sent == 0) {
		ptr[i++] = I2C_PKT_ADR(data->flags, addr[0]);
		data->addr_sent = 1;
	}

	if (I2C_MSG_WR(data->flags)) {
		memcpy((ptr + i), data->buf, copy - i);
		data->buf += copy - i;
		data->buf_len -= (copy - i);
	}

	i2c_debug("send word %x\n", word);
	reg_write(word, data->regs + I2C_TXD_OFFSET);
}

/******************************************************************************
 * Function:... xgold_i2c_recv_word()
 * Description: Adapter receives/copies data from I2C bus to
 * buffer
******************************************************************************/
static void xgold_i2c_recv_word(struct xgold_i2c_algo_data *data)
{
	int copy = data->buf_len < 4 ? data->buf_len : 4;

	unsigned long word = ioread32(data->regs + I2C_RXD_OFFSET);
	__u8 *ptr = (__u8 *) &word;

	i2c_debug("rcvd word %x\n", (unsigned int)word);
	memcpy(data->buf, ptr, copy);
	data->buf += copy;
	data->buf_len -= copy;
}

/******************************************************************************
 * Function:... xgold_i2c_set_baud()
 * Description: This to check to set the baud rate. Baud rate
 * is supplied as part of flags parameter.
******************************************************************************/
static void xgold_i2c_set_baud(struct i2c_adapter *adap,
					unsigned int flags)
{
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(adap);
	struct xgold_i2c_platdata *pdata = dev_get_platdata(data->i2c_dev->dev);
	int cbaud;
	u32 reg;

	/* The below API  would configure i2c adapter either for
	 * 400 kbps or 100 kbps and the baud rate depends on flags
	 * parameter or default baud rate If flags is not set to
	 * any of the above, then the default baudrate would be set
	 * at 100 kbps the API would return I2C_XGOLD_B100 for 100
	 * kbps and I2C_XGOLD_B400 for 400 kbps.
	 */
	cbaud = xgold_i2c_get_client_baud(flags);

	if (cbaud != data->current_baud) {
		i2c_debug("%s baud rate = %skHz\n", __func__,
			 cbaud == I2C_XGOLD_B100 ? "100" : "400");
		data->state = XGOLD_I2C_CONFIGURE;

		/* turning OFF I2C */
		reg_write(I2C_RUN_CTRL_RUN_DIS,
			data->regs + I2C_RUN_CTRL_OFFSET);

		/* FDIV CFG register */
		reg = (pdata->cfg_baud[cbaud][FDIV_INC] <<
				I2C_FDIV_CFG_INC_OFFSET) |
			(pdata->cfg_baud[cbaud][FDIV_DEC] <<
				I2C_FDIV_CFG_DEC_OFFSET);

		reg_write(reg, data->regs + I2C_FDIV_CFG_OFFSET);

		/* TIM CFG register */
		reg = (cbaud == I2C_XGOLD_B400) ?
			I2C_TIM_CFG_FS_SCL_LOW_MASK : 0;

		reg |= (pdata->cfg_baud[cbaud][TIM_SDA] <<
				I2C_TIM_CFG_SDA_DEL_HD_DAT_OFFSET) |
			(pdata->cfg_baud[cbaud][TIM_HS_SDA] <<
				I2C_TIM_CFG_HS_SDA_DEL_HD_DAT_OFFSET);
		reg_write(reg, data->regs + I2C_TIM_CFG_OFFSET);

		reg_write(I2C_RUN_CTRL_RUN_EN,
			data->regs + I2C_RUN_CTRL_OFFSET);
		data->current_baud = cbaud;
	}

	return;
}

static inline int xgold_i2c_invalid_address(const struct i2c_msg *msg)
{
	return (msg->addr > XGOLD_I2C_SLAVE_ADDR_7_BIT_MASK);
}


/******************************************************************************
 * Function:... xgold_i2c_bus_status()
 * Description: will return i2c adapter bus status
 *****************************************************************************/
static inline unsigned long xgold_i2c_bus_status(
		struct xgold_i2c_algo_data *data)
{
	return ioread32(data->regs + I2C_BUS_STAT_OFFSET)
		& I2C_BUS_STAT_BS_MASK;
}

/******************************************************************************
 * Function:... xgold_i2c_dmae()
 * Description:
 *****************************************************************************/
static inline void xgold_i2c_dmae(struct xgold_i2c_algo_data *data, bool en)
{
	unsigned dmae = (en == true) ?
		XGOLD_I2C_DMAE_ENABLE : XGOLD_I2C_DMAE_DISABLE;

	reg_write(dmae, data->regs + I2C_DMAE_OFFSET);
}

/******************************************************************************
 * Function:... xgold_i2c_err_handler()
 * Description: Error handler to handle error conditions
 *****************************************************************************/
static irqreturn_t xgold_i2c_err_handler(void *dev)
{
	struct i2c_adapter *adap = (struct i2c_adapter *)dev;
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(adap);
	unsigned long err_irqss = ioread32(data->regs + I2C_ERR_IRQSS_OFFSET);

	if (!(err_irqss))
		return IRQ_NONE;

	i2c_err("%s, ERR_INT 0x%X recvd.\n", __func__,
			(unsigned int)err_irqss);

	data->cmd_err = -EIO;
	reg_write(err_irqss, data->regs + I2C_ERR_IRQSC_OFFSET);

	return IRQ_HANDLED;
}

/*****************************************************************************
 * Function:... xgold_i2c_p_handler()
 * Description: protocol handler to handle I2C protocols
 *****************************************************************************/
static irqreturn_t xgold_i2c_p_handler(void *dev)
{
	struct i2c_adapter *adap = (struct i2c_adapter *)dev;
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(adap);
	unsigned long irqss =
		ioread32(data->regs + I2C_P_IRQSS_OFFSET) & XGOLD_I2C_P_MASK;

	if (!(irqss))
		return IRQ_NONE;

	i2c_debug("%s irqss=0x%08x\n", __func__, (unsigned int)irqss);
	if (irqss & I2C_P_IRQSS_NACK_INTRS_PEND) {
		/* Not Acknowledged */
		i2c_debug("M13: NACK recvd.\n");
		data->cmd_err = -EREMOTEIO;
		complete(&data->cmd_complete);
		reg_write(I2C_P_IRQSC_NACK, data->regs + I2C_P_IRQSC_OFFSET);

	} else if (irqss & I2C_P_IRQSS_RX_INTRS_PEND) {
		/* Receiving data */
		i2c_debug("M2: switching TX to RX.\n");
		data->state = XGOLD_I2C_RECEIVE;
		reg_write(I2C_P_IRQSC_RX_INTRS_CLR,
				data->regs + I2C_P_IRQSC_OFFSET);

	} else if (irqss & I2C_P_IRQSS_TX_END_INTRS_PEND) {
		/* End of transfer */
		if (!data->cmd_err && data->state == XGOLD_I2C_RECEIVE &&
			data->buf_len > 0 && !data->dmach) {

			i2c_debug("WARN : Flush RX FIFO %d bytes remaining\n",
				data->buf_len);

			/*
			 * WARNING: Due to race condition, it may happen that
			 * the TX_END interrupt occurs while the LSREQ has not
			 * been treated. Thus some bytes remain in the FIFO.
			 * The FIFO flush must be executed before to ack the
			 * TX_END interrupt. Return NONE now, so the FIFO flush
			 * can be executed */
			return IRQ_NONE;
		}

		i2c_debug("M3 or M20: flags=%d, count=%d, device.state= %d\n",
				data->flags, data->buf_len, data->state);

		reg_write(I2C_P_IRQSC_TX_END_INTRS_CLR,
				data->regs + I2C_P_IRQSC_OFFSET);

		if (data->dma_mode == false)
			complete(&data->cmd_complete);
		else if (data->buf_len == 0)
			complete(&data->cmd_complete);
		else
			data->buf_len = 0;

	} else if (irqss & I2C_P_IRQSS_AL_INTRS_PEND) {
		/* Arbitration lost */
		i2c_err("M5: AL recvd.\n");
		data->cmd_err = -EACCES;
		complete(&data->cmd_complete);
		reg_write(I2C_P_IRQSC_AL_INTRS_CLR,
				data->regs + I2C_P_IRQSC_OFFSET);

	} else if (irqss & (I2C_P_IRQSS_AM_INTRS_PEND |
				I2C_P_IRQSS_GC_INTRS_PEND |
				I2C_P_IRQSS_MC_INTRS_PEND)) {
		/* Adress Match */
		i2c_debug("M4: %x ACK recvd.\n", (unsigned int)irqss);
		reg_write(I2C_P_IRQSC_AM_INTRS_CLR | I2C_P_IRQSC_GC_INTRS_CLR |
				I2C_P_IRQSC_MC_INTRS_CLR,
				data->regs + I2C_P_IRQSC_OFFSET);
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * Function:... xgold_i2c_irq_handler()
 * Description: I2C Interrupt handler
 *****************************************************************************/
static irqreturn_t xgold_i2c_irq_handler(int irq, void *dev)
{
	struct i2c_adapter *adap = (struct i2c_adapter *)dev;
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(adap);
	unsigned long mis = ioread32(data->regs + I2C_MIS_OFFSET);
	irqreturn_t ret = IRQ_NONE;

	while (mis != I2C_MIS_NO_INT_PEND) {
		if (mis & I2C_MIS_I2C_ERR_INT_INT_PEND) {
			/* Error interrupt Received */
			ret = xgold_i2c_err_handler(dev);
			i2c_debug("%s: error interrupt ret = %d",
					__func__, ret);
		}

		if (mis & (I2C_MIS_LSREQ_INT_INT_PEND |
					I2C_MIS_SREQ_INT_INT_PEND)) {
			/* SREQ/LSREQ Received */
			i2c_debug("SREQ_INT/LSREQ_INT recvd\n");
			if (data->state == XGOLD_I2C_TRANSMIT)
				xgold_i2c_xmit_word(data);
			else
				xgold_i2c_recv_word(data);

			reg_write(I2C_ICR_LSREQ_INT_CLR_INT |
					I2C_ICR_SREQ_INT_CLR_INT,
					data->regs + I2C_ICR_OFFSET);

			ret = IRQ_HANDLED;
		}

		if (mis & (I2C_MIS_LBREQ_INT_INT_PEND |
					I2C_MIS_BREQ_INT_INT_PEND)) {
			/* BREQ/LBREQ Received */
			int i;
			/* FIXME: check FIFO stage ?!? */
			i2c_debug("BREQ_INT/LBREQ_INT recvd\n");
			if (data->state == XGOLD_I2C_TRANSMIT) {
				for (i = 0; i < data->txbs; ++i)
					xgold_i2c_xmit_word(data);
			} else {
				for (i = 0; i < data->rxbs; ++i)
					xgold_i2c_recv_word(data);
			}

			reg_write(I2C_ICR_LBREQ_INT_CLR_INT |
					I2C_ICR_BREQ_INT_CLR_INT,
					data->regs + I2C_ICR_OFFSET);

			ret = IRQ_HANDLED;
		}

		if (mis & XGOLD_I2C_IMSC_P_BIT) {
			ret = xgold_i2c_p_handler(dev);
			i2c_debug("%s protocol interrupt ret %d\n",
					__func__, ret);
		}

		mis = ioread32(data->regs + I2C_MIS_OFFSET);
	}

	return ret;
}

/******************************************************************************
 * Function:... xgold_i2c_xfer_msg()
 * Description:
 * return 0 if no failure. Return cmd_err code otherwise
 *****************************************************************************/
static int xgold_i2c_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(adap);
	unsigned long flag;
	int ret = 0;
	u32 reg;

	if (xgold_i2c_invalid_address(msg)) {
		i2c_err("Invalid address.\n");
		ret = -EINVAL;
		return ret;
	}

	spin_lock_irqsave(&data->lock, flag);
	reinit_completion(&data->cmd_complete);

	/* Update the state to transmit, since slave address
	 * has to be transmitted first */
	data->state = XGOLD_I2C_TRANSMIT;
	data->msg = msg;
	data->flags = msg->flags;
	data->buf = msg->buf;
	data->buf_len = msg->len;
	data->addr = msg->addr;
	data->addr_sent = 0;
	data->cmd_err = 0;
	spin_unlock_irqrestore(&data->lock, flag);

	/*
	 * PIO/DMA mode selection:
	 * Current boundary to select betwen PIO/DMA mode is set to 32 bytes.
	 * Transfers shorter that 16B are transfered in PIO mode, while longer
	 * transfers use DMA mode.
	 * This limit is fixed as for reception, RXOVF (overflow) could be
	 * observed if interrupt latency is too long when receiving more that 32
	 * bytes. Fix limit to half of this value.
	 */
	data->dma_mode = (data->dmach && data->buf_len > 32) ? true : false;

	if (I2C_MSG_RD(data->flags)) {
		unsigned long word = 0;
		__u8 *ptr = (__u8 *)&word;
		__u8 *addr = (__u8 *)&data->addr;
		ptr[0] = I2C_PKT_ADR(data->flags, addr[0]);

		i2c_debug("--> RD dev=0x%x, %d bytes, flags=%d, mode %s\n",
			data->addr, data->buf_len, data->flags,
			(data->dma_mode == true) ? "DMA" : "PIO");

		if (data->dma_mode == true) {
			ret = xgold_i2c_dma_config(data, data->flags);
			if (ret) {
				data->cmd_err = ret;
				goto out;
			}

			ret = xgold_i2c_dma_setup_xfer(data);
			if (ret) {
				data->cmd_err = ret;
				goto out;
			}
		}

		xgold_i2c_dmae(data, data->dma_mode);

		/* Mask undesired interrupt LSREQ */
		reg = ioread32(data->regs + I2C_IMSC_OFFSET);
		reg &= ~I2C_IMSC_LSREQ_INT_MASK;
		reg_write(reg, data->regs + I2C_IMSC_OFFSET);

		/* Program data lengths */
		reg_write(data->buf_len, data->regs + I2C_MRPS_CTRL_OFFSET);
		reg_write(1 , data->regs + I2C_TPS_CTRL_OFFSET);

		/*
		 * Wait for last burst request before sending the slave
		 * address
		 */
		while ((ioread32(data->regs + I2C_RIS_OFFSET) &
					I2C_RIS_LSREQ_INT_INT_PEND) == 0)
			;

		/* Must spinlock the address send to avoid RXOVF issue */
		spin_lock_irqsave(&data->lock, flag);

		/* Send slave address */
		reg_write(word, data->regs + I2C_TXD_OFFSET);
		data->addr_sent = 1;

		/*
		 * Clear LSREQ interrupt to let the FIFO state going
		 * into ready state before RX occurs
		 */
		reg_write(I2C_ICR_LSREQ_INT_CLR_INT,
				data->regs + I2C_ICR_OFFSET);
		reg = ioread32(data->regs + I2C_IMSC_OFFSET);
		reg &= ~I2C_IMSC_LSREQ_INT_MASK;
		reg |= I2C_IMSC_LSREQ_INT_INT_EN;
		reg_write(reg, data->regs + I2C_IMSC_OFFSET);

		/* Put the adpater in receive mode */
		data->state = XGOLD_I2C_RECEIVE;

		spin_unlock_irqrestore(&data->lock, flag);

		/* Wait for transfer completion */
		ret = wait_for_completion_timeout(&data->cmd_complete,
				data->timeout);

		if (data->dma_mode == true && (ret == 0 || data->cmd_err) &&
				dma_async_is_tx_complete(data->dmach,
				data->dma_cookie, NULL, NULL) != DMA_COMPLETE) {
			i2c_debug("An error occured %#X. Terminate all dma\n",
				data->cmd_err);
			dmaengine_terminate_all(data->dmach);
			xgold_i2c_dma_finish(data);
		} else if (ret == 0) {
			i2c_err("timeout waiting end of cmd RD\n");
			data->cmd_err = -EIO;
		}

		i2c_debug("<-- RD dev=0x%x, %d bytes val=0x%x\n",
				data->addr, data->buf_len, data->buf[0]);
	} else {
		i2c_debug("--> WR dev=0x%x, len=%d, reg=0x%x, flags=%d, mode %s\n",
			data->addr, data->buf_len, data->buf[0], data->flags,
			(data->dma_mode == true) ? "DMA" : "PIO");

		if (data->dma_mode == true) {
			ret = xgold_i2c_dma_config(data, data->flags);
			if (ret) {
				data->cmd_err = ret;
				goto out;
			}

			ret = xgold_i2c_dma_setup_xfer(data);
			if (ret) {
				data->cmd_err = ret;
				goto out;
			}
		}

		xgold_i2c_dmae(data, data->dma_mode);

		reg_write(data->buf_len + 1,
				data->regs + I2C_TPS_CTRL_OFFSET);

		ret = wait_for_completion_timeout(&data->cmd_complete,
				data->timeout);

		if (data->dma_mode == true && (ret == 0 || data->cmd_err) &&
				dma_async_is_tx_complete(data->dmach,
				data->dma_cookie, NULL, NULL) != DMA_COMPLETE) {

			i2c_debug("An error occured %#X. Terminate all dma\n",
				data->cmd_err);
			dmaengine_terminate_all(data->dmach);
			xgold_i2c_dma_finish(data);
		} else if (ret == 0) {
			i2c_err("timeout waiting end of cmd WR\n");
			data->cmd_err = -EIO;
		}

		i2c_debug("<-- WR dev=0x%x done\n", data->addr);
	}

out:
	if (data->dma_mode) {
		xgold_i2c_dmae(data, false);
		if (dma_async_is_tx_complete(data->dmach, data->dma_cookie,
					NULL, NULL) != DMA_COMPLETE) {

			dmaengine_terminate_all(data->dmach);
			xgold_i2c_dma_finish(data);
		}
	}

	if (data->cmd_err) {
		i2c_debug("An error occured %#X\n", data->cmd_err);
		return data->cmd_err;
	}

	return 0;
}

/*****************************************************************************
 * Function:... xgold_i2c_xfer()
 * Description:  msgs has the parameters needed for transfer
 *				i.e. data pointer, number of bytes, address of
 *				the slave device, flags. num will have the
 *				number of transfers
 *****************************************************************************/
static int xgold_i2c_xfer(struct i2c_adapter *adap,
		struct i2c_msg msgs[], int num)
{
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(adap);
	int ret = 0;
	int i;

	spin_lock(&data->lock);
	if (data->state != XGOLD_I2C_LISTEN) {
		i2c_err("Transaction ongoing\n");
		spin_unlock(&data->lock);
		return -EAGAIN;
	}

	/* turning ON I2C */
	reg_write(I2C_RUN_CTRL_RUN_EN, data->regs + I2C_RUN_CTRL_OFFSET);
	spin_unlock(&data->lock);

	/* FIXME: while bus not free */
	if (xgold_i2c_bus_status(data) != XGOLD_I2C_BUS_FREE) {
		i2c_info("BUSY\n");
		i2c_info("Driver state %d, Bus state %u.\n", data->state,
				(unsigned int)xgold_i2c_bus_status(data));
		return -EBUSY;
	}

	/* set client specific baud */
	xgold_i2c_set_baud(adap, msgs[0].flags);

	/* handle messages list */
	for (i = 0; i < num; i++) {
		ret = xgold_i2c_xfer_msg(adap, &msgs[i]);
		if (ret)
			break;
	}

	if (!ret)
		/* no error, must return byte count */
		ret = num;

	spin_lock(&data->lock);
	data->state = XGOLD_I2C_LISTEN;
	/* turning OFF I2C */
	reg_write(I2C_RUN_CTRL_RUN_DIS, data->regs + I2C_RUN_CTRL_OFFSET);
	spin_unlock(&data->lock);

	return ret;
}

/******************************************************************************
 * Function:... xgold_i2c_func()
 * Description:  Returns the i2c functionality supported by
 * I2C-XGOLD adapter
 *****************************************************************************/
static u32 xgold_i2c_func(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
		I2C_FUNC_PROTOCOL_MANGLING;
}

static struct i2c_algorithm xgold_i2c_algo = {
	.master_xfer = xgold_i2c_xfer,
	.smbus_xfer = NULL, /* FIXME */
	.functionality = xgold_i2c_func,
};

/*****************************************************************************
* Function:... xgold_i2c_hw_init()
* Description:  Configure FIFO and ADDRESS_CFG of I2C adapter.
 *****************************************************************************/
static void xgold_i2c_hw_init(struct xgold_i2c_algo_data *data)
{
	struct xgold_i2c_platdata *pdata =
		dev_get_platdata(data->i2c_dev->dev);
	u32 reg;

	if (data->state != XGOLD_I2C_CONFIGURE) {
		i2c_err("%s controller is in wrong state\n", __func__);
		data->state = XGOLD_I2C_CONFIGURE;
	}

	i2c_debug("%s baud rate = %skbps\n", __func__,
		data->current_baud == I2C_XGOLD_B100 ? "100" : "400");

	/* Fixing kernel clk to 52 Mhz */
	reg_write(0x100, data->regs + I2C_CLC_OFFSET);

	/* turning OFF I2C */
	reg_write(I2C_RUN_CTRL_RUN_DIS, data->regs + I2C_RUN_CTRL_OFFSET);

	/* High Speed Disabled */
	reg_write(XGOLD_I2C_FDIV_HIGH_CFG_DISABLE,
			data->regs + I2C_FDIV_HIGH_CFG_OFFSET);

	/* FDIV CFG register */
	reg = (pdata->cfg_baud[data->current_baud][FDIV_INC] <<
			I2C_FDIV_CFG_INC_OFFSET) |
		(pdata->cfg_baud[data->current_baud][FDIV_DEC] <<
			I2C_FDIV_CFG_DEC_OFFSET);

	reg_write(reg, data->regs + I2C_FDIV_CFG_OFFSET);

	/* TIM CFG register */
	reg = (data->current_baud == I2C_XGOLD_B400) ?
		I2C_TIM_CFG_FS_SCL_LOW_MASK : 0;

	reg |= (pdata->cfg_baud[data->current_baud][TIM_SDA] <<
			I2C_TIM_CFG_SDA_DEL_HD_DAT_OFFSET) |
		(pdata->cfg_baud[data->current_baud][TIM_HS_SDA] <<
			I2C_TIM_CFG_HS_SDA_DEL_HD_DAT_OFFSET);
	reg_write(reg, data->regs + I2C_TIM_CFG_OFFSET);

	/* I2C Address Configuration Register */
	reg = (0x44 << I2C_ADDR_CFG_ADR_OFFSET) |
		I2C_ADDR_CFG_TBAM_AM7 | I2C_ADDR_CFG_MNS_EN |
		I2C_ADDR_CFG_SONA_EN | I2C_ADDR_CFG_SOPE_EN;
	reg_write(reg, data->regs + I2C_ADDR_CFG_OFFSET);

	/* FIFO config */
	reg = I2C_FIFO_CFG_RXBS_RXBS4 | I2C_FIFO_CFG_TXBS_TXBS4 |
		I2C_FIFO_CFG_RXFA_RXFA1 | I2C_FIFO_CFG_TXFA_TXFA1 |
		I2C_FIFO_CFG_RXFC_RXFC | I2C_FIFO_CFG_TXFC_TXFC;
	reg_write(reg, data->regs + I2C_FIFO_CFG_OFFSET);

	data->rxbs = 1 << (I2C_FIFO_CFG_RXBS_RXBS4 >> I2C_FIFO_CFG_RXBS_OFFSET);
	data->txbs = 1 << (I2C_FIFO_CFG_TXBS_TXBS4 >> I2C_FIFO_CFG_TXBS_OFFSET);

	/* Set MRPS to unlimited receive */
	reg_write(0, data->regs + I2C_MRPS_CTRL_OFFSET);

	/* Disable DMAE interrupts */
	reg_write(XGOLD_I2C_DMAE_DISABLE, data->regs + I2C_DMAE_OFFSET);

	/* Clear pending protocol interrupt */
	reg_write(XGOLD_I2C_ERR_MASK, data->regs + I2C_ERR_IRQSC_OFFSET);
	reg_write(XGOLD_I2C_P_MASK_ALL, data->regs + I2C_P_IRQSC_OFFSET);
	reg_write(XGOLD_I2C_IMSC_MASK, data->regs + I2C_ICR_OFFSET);

	/* Set Interrupt Mask */
	reg_write(XGOLD_I2C_ERR_MASK, data->regs + I2C_ERR_IRQSM_OFFSET);
	reg_write(XGOLD_I2C_P_MASK, data->regs + I2C_P_IRQSM_OFFSET);
	reg_write(XGOLD_I2C_IMSC_MASK, data->regs + I2C_IMSC_OFFSET);

	data->state = XGOLD_I2C_LISTEN;
}

/*****************************************************************************
 * Function:... xgold_i2c_hw_stop()
 * Description:  STOP the run bit of I2C adapter
 ****************************************************************************/
static void xgold_i2c_hw_stop(struct xgold_i2c_algo_data *data)
{
	/* Turn off everything */
	data->state = XGOLD_I2C_CONFIGURE;
	reg_write(I2C_RUN_CTRL_RUN_DIS, data->regs + I2C_RUN_CTRL_OFFSET);

	/* Disable the kernel clock */
	reg_write(0x0, data->regs + I2C_CLC_OFFSET);
}

/*****************************************************************************
 * Function:... xgold_i2c_probe()
 * Description:  I2C adapter requests resources like memory,
 * irq lines. update Adapter's information.
 *****************************************************************************/
static int xgold_i2c_core_probe(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct i2c_adapter *adap;
	struct xgold_i2c_algo_data *algo_data;
	struct xgold_i2c_platdata *platdata;
	int ret, i;

	i2c_info("Enterring %s\n", __func__);
	platdata = dev->platform_data;

	/* Initialize algo data */
	algo_data = &i2c_dev->algo_data;
	algo_data = &i2c_dev->algo_data;
	algo_data->state = XGOLD_I2C_CONFIGURE;
	algo_data->timeout = XGOLD_I2C_TIMEOUT;
	algo_data->default_baud = I2C_XGOLD_B100;
	algo_data->current_baud = I2C_XGOLD_B100;
	algo_data->i2c_dev = i2c_dev;
	init_completion(&algo_data->cmd_complete);
	spin_lock_init(&algo_data->lock);

	/* Initialize the i2c adapter data */
	adap = &i2c_dev->adapter;
	i2c_set_adapdata(adap, algo_data);
	adap->owner = THIS_MODULE;
	strcpy(adap->name, dev_name(dev));
	adap->algo = &xgold_i2c_algo;
	adap->dev.parent = dev;
	adap->nr = platdata->id;
	adap->dev.of_node = dev->of_node;

	/* Initialize I2C controller */
	if (platdata->rst)
		reset_control_reset(platdata->rst);

	ret = device_state_pm_set_class(dev,
			platdata->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(dev, "Error while setting the pm class\n");
		goto err_pm_class;
	}

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D0_name);

	if (ret)
		dev_err(dev, "Set state return %d\n", ret);

	/* pcl */
	xgold_i2c_set_pinctrl_state(dev, platdata->pins_default);

	xgold_i2c_hw_init(algo_data);

	/* Request irq */
	for (i = 0; i < platdata->irq_num; i++) {
		ret = request_irq(platdata->irq + i, xgold_i2c_irq_handler,
					IRQF_DISABLED, adap->name, adap);
		if (ret) {
			i2c_err("Cannot allocate IRQ %d", platdata->irq+i);
			goto free_irqs;
		}
	}

	/* Register I2C adapter */
	ret = i2c_add_numbered_adapter(adap);
	i2c_debug("xgold: i2c_add_numbered_adapter= %d", ret);
	if (ret < 0) {
		i2c_err("failed to add bus to i2c core\n");
		goto err_add_adapter;
	}

	return 0;

err_add_adapter:
	if (algo_data->dmach)
		dma_release_channel(algo_data->dmach);
free_irqs:
	while (i >= 0) {
		free_irq(platdata->irq+i, adap);
		i--;
	}

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D3_name);

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_put(platdata->clk_kernel);
	clk_put(platdata->clk_bus);
#endif

err_pm_class:
	return ret;
}

/******************************************************************************
 * Function:... xgold_i2c_core_remove()
 * Description:
 *****************************************************************************/
static int xgold_i2c_core_remove(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(&i2c_dev->adapter);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	int j;

	dev_set_drvdata(dev, NULL);

	xgold_i2c_set_pinctrl_state(dev, platdata->pins_inactive);

	if (data->dmach) {
		dmaengine_terminate_all(data->dmach);
		dma_release_channel(data->dmach);
	}

	for (j = 0; j < platdata->irq_num; j++)
		free_irq(platdata->irq + j, &i2c_dev->adapter);

	i2c_del_adapter(&i2c_dev->adapter);
	xgold_i2c_hw_stop(data);

	device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D3_name);

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	clk_put(platdata->clk_kernel);
	clk_put(platdata->clk_bus);
#endif

	if (platdata->regulator)
		regulator_put(platdata->regulator);
	if (platdata->regulator2)
		regulator_put(platdata->regulator2);

	iounmap((void __iomem *)data->regs);
	return 0;
}


static struct xgold_i2c_ops xgold_i2c_core_ops = {
	.probe = xgold_i2c_core_probe,
	.remove = xgold_i2c_core_remove,
};

#if defined CONFIG_PM
/*****************************************************************************
 * Function:... xgold_i2c_core_suspend()
 * Description:
 ****************************************************************************/
static int xgold_i2c_core_suspend(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(&i2c_dev->adapter);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	int ret = 0;

	i2c_info("suspend %s\n", dev->init_name);

	xgold_i2c_hw_stop(data);

	xgold_i2c_set_pinctrl_state(dev, platdata->pins_sleep);

	return ret;
}

/*****************************************************************************
 * Function:... xgold_i2c_core_resume()
 * Description:
 ****************************************************************************/
static int xgold_i2c_core_resume(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	struct xgold_i2c_algo_data *data = i2c_get_adapdata(&i2c_dev->adapter);
	struct xgold_i2c_platdata *platdata = dev_get_platdata(dev);
	int ret;

	i2c_info("resume %s\n", dev->init_name);

	ret = device_state_pm_set_state_by_name(dev,
			platdata->pm_platdata->pm_state_D0_name);

	if (ret) {
		i2c_err("Error during state transition %s\n",
				platdata->pm_platdata->pm_state_D0_name);
		return ret;
	}

	xgold_i2c_set_pinctrl_state(dev, platdata->pins_default);

	xgold_i2c_hw_init(data);

	return 0;
}

static const struct dev_pm_ops xgold_i2c_core_pm_ops = {
	.suspend = xgold_i2c_core_suspend,
	.resume = xgold_i2c_core_resume,
};
#endif

struct xgold_i2c_dev *xgold_i2c_init_driver(struct device *dev)
{
	struct xgold_i2c_dev *i2c_dev;
	struct xgold_i2c_platdata *platdata;

	i2c_dev = devm_kzalloc(dev, sizeof(struct xgold_i2c_dev), GFP_KERNEL);
	if (i2c_dev == NULL)
		return ERR_PTR(-ENOMEM);

	i2c_dev->dev = dev;
	i2c_dev->core_ops = &xgold_i2c_core_ops;
#ifdef CONFIG_PM
	i2c_dev->pm_ops = &xgold_i2c_core_pm_ops;
#else
	i2c_dev->pm_ops = NULL;
#endif
	dev_set_drvdata(dev, i2c_dev);

#ifdef CONFIG_OF
	platdata = xgold_i2c_of_get_platdata(dev);
	if (!platdata) {
		i2c_err("Error during dts file parsing\n");
		return ERR_PTR(-EINVAL);
	}

	dev->platform_data = platdata;
#else
	if (!dev_get_platdata(dev)) {
		i2c_err("No platdata\n");
		return ERR_PTR(-EINVAL);
	}
#endif

	return i2c_dev;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int __init xgold_i2c_core_init(void)
{
	return device_state_pm_add_class(&i2c_pm_class);
}

arch_initcall(xgold_i2c_core_init);
#endif

MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");
