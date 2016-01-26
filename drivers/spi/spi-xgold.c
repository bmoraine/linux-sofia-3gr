/*
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
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
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/amba/pl08x.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/platform_data/spi-xgold.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include "spi-xgold.h"

#undef VERBOSE
#define XGOLD_SPI_FUNC_IN	pr_info("----->%s\n", __func__)
#define XGOLD_SPI_FUNC_OUT	pr_info("<-----%s\n", __func__)

#define OF_KERNEL_CLK		"kernel"
#define OF_AHB_CLK		"register"
#define OF_SPI_DMA_TXREQ	"intel,spi,dmatx"
#define OF_SPI_DMA_RXREQ	"intel,spi,dmarx"
#define OF_SPI_NUM_CHIPSELECT	"intel,spi,numcs"
#define OF_SPI_HW_TYPE		"intel,spi,hw_type"
#define OF_SPI_BUS_NUM		"intel,spi,bus_num"

#define CS_IS_GPIO(spi) (spi->cs_gpio != -ENOENT)

struct xgold_usif_hwinfo {
	unsigned short swcid;
	unsigned short ts_rev;
	unsigned char rx_fifo_width;
	unsigned char tx_fifo_width;
	unsigned char rx_burst_size;	/* in words */
	unsigned char tx_burst_size;	/* in words */
	unsigned char rps_width;
	unsigned char mod_id;
	unsigned char rev;
};

struct xgold_spi_ctl_drv {
	struct spi_master *master;
	struct xgold_usif_hwinfo hwinfo;
	struct list_head queue;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	spinlock_t lock;
	struct completion done;
	struct completion xfer_completion;
	void __iomem *base;
	unsigned cur_mode, cur_bpw;
	unsigned cur_speed;
	unsigned long clk_spi_rate;
	struct spi_transfer *current_xfer;
	struct spi_message *current_msg;
	unsigned char *current_rxbuf;
	unsigned char *current_txbuf;
	unsigned char packet_size;
	unsigned long current_txlen;
	unsigned long current_rxlen;
	unsigned state;
	struct device *dev;
	unsigned dma_rxmode;
	unsigned dma_txmode;
	int pio_mode;
	unsigned int irq_num;
	unsigned int spi_irq[USIF_IRQS_NR];
	struct xgold_spi_platdata *platdata;
	/* DMA */
	struct dma_chan *dmach[2];
	struct scatterlist sg_io[2];
	dma_cookie_t dma_cookie[2];
};

enum xgold_spi_dma {
	XGOLD_SPI_DMA_RX = 0,
	XGOLD_SPI_DMA_TX
};

/*
  Used to hold the current state of the controller
*/
struct xgold_spi_ctl_state {
	unsigned timeout;
	unsigned speed_hz;
	unsigned char bits_per_word;
	unsigned char mode;
};


static struct of_device_id xgold_spi_of_match[] = {
	{.compatible = "intel,usif-spi",},
	{},
};

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
static int xgold_spi_set_pm_state(struct device *,
		struct device_state_pm_state *);
static struct device_state_pm_state *xgold_spi_get_initial_state(
		struct device *);

static struct device_state_pm_ops xgold_spi_pm_ops = {
	.set_state = xgold_spi_set_pm_state,
	.get_initial_state = xgold_spi_get_initial_state,
};

/* USIF PM states & class */
static struct device_state_pm_state xgold_spi_pm_states[] = {
	{ .name = "enable_26M" },
	{ .name = "enable_26M_hperf" },
	{ .name = "enable_96M" },
	{ .name = "enable_96M_hperf" },
	{ .name = "enable_104M", },
	{ .name = "enable_104M_hperf", }, /* D0 */
	{ .name = "disable", }, /* D0i3, D3 */
};

DECLARE_DEVICE_STATE_PM_CLASS(xgold_spi);
#endif

static inline void xgold_spi_set_configuration_mode(struct xgold_spi_ctl_drv
						    *ctl_drv)
{
	unsigned int reg = 0;
	reg |= (USIF_CLC_RUN_STOP | USIF_CLC_MOD_EN_EN_REQ);
	iowrite32(reg, USIF_CLC(ctl_drv->base));
}

static inline void xgold_spi_set_run_mode(struct xgold_spi_ctl_drv *ctl_drv)
{
	unsigned int reg = 0;
	reg |= (USIF_CLC_RUN_RUN | USIF_CLC_MOD_EN_EN_REQ);
	iowrite32(reg, USIF_CLC(ctl_drv->base));
}

#define USIF_SPI_HANDLE(_X_) \
{ \
	if (mis & USIF_RIS_##_X_##_MASK) { \
		xgold_spi_debug(XGOLD_SPI_ISR,\
				"%s: %s: Got IRQ to deal with : %s\n",\
						DRIVER_NAME, __func__, #_X_); \
		iowrite32(USIF_RIS_##_X_##_MASK, USIF_ICR(ctl_drv->base)); \
	} \
}

#ifdef VERBOSE

static void dump_xfer_buf(const char *prefix, const void *buf, unsigned size)
{
	int rowsize = 16;
	int groupsize = 1;
	size_t len = (size > 256) ? 256 : size;
	bool ascii = false;
	const u8 *ptr = buf;
	int i, linelen, remaining = len;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];

	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
				   linebuf, sizeof(linebuf), ascii);

		pr_debug("%s%s\n", prefix, linebuf);
	}
}

static void dump_tx_xfer(struct spi_transfer *xfer)
{
	dump_xfer_buf("TX--->:", xfer->tx_buf, xfer->len);
}

static void dump_rx_xfer(struct spi_transfer *xfer)
{
	dump_xfer_buf("RX--->:", xfer->rx_buf, xfer->len);
}

static void dump_xfer(struct spi_transfer *xfer)
{

	pr_info("SPI XFER  :%p  , Length : %d\n", xfer, xfer->len);
	pr_info("\t TX Buf :%p , TX DMA :%p\n", xfer->tx_buf,
	       (void *)xfer->tx_dma);
	pr_info("\t RX Buf :%p , RX DMA :%p\n", xfer->rx_buf,
	       (void *)xfer->rx_dma);
	pr_info(
	"\tCS change:%d, bits/w :%d, delay : %d us, speed : %d Hz\n",
	       xfer->cs_change, xfer->bits_per_word, xfer->delay_usecs,
	       xfer->speed_hz);

}
#endif


#ifdef CONFIG_OF
static struct xgold_spi_platdata *xgold_spi_get_platdata(
		struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct xgold_spi_ctl_drv *ctl_drv;
	struct xgold_spi_platdata *platdata;
	struct device_node *nspi;
	int ret;

	if (!master)
		return ERR_PTR(-EINVAL);

	ctl_drv = spi_master_get_devdata(master);

	platdata = devm_kzalloc(&pdev->dev, sizeof(struct xgold_spi_platdata),
			GFP_KERNEL);
	if (!platdata)
		return ERR_PTR(-ENOMEM);

	/* Get device node pointer */
	nspi = pdev->dev.of_node;
	if (!nspi) {
		dev_err(&pdev->dev, "Can't find spi matching node\n");
		return ERR_PTR(-ENODEV);
	}
	master->dev.of_node = nspi;

	/* dev name: FIXME: XR: Not sure it is needed */
	/* pdev->dev.init_name = nspi->name; */

	platdata->clock_spi = of_clk_get_by_name(nspi, OF_KERNEL_CLK);
	if (IS_ERR(platdata->clock_spi)) {
		dev_err(&pdev->dev, "Clk %s not found: fixed to 104MHz\n",
				OF_KERNEL_CLK);
		ctl_drv->clk_spi_rate = 104 * 1000 * 1000;
	}

	platdata->clock_ahb = of_clk_get_by_name(nspi, OF_AHB_CLK);
	if (IS_ERR(platdata->clock_ahb))
		dev_err(&pdev->dev, "Clk %s not found\n", OF_AHB_CLK);

	ret = of_property_read_u32(nspi, OF_SPI_NUM_CHIPSELECT,
						&platdata->num_chipselect);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get num_chipcs\n");
		return ERR_PTR(-EINVAL);
	}

	ret = of_property_read_u32(nspi, OF_SPI_HW_TYPE, &platdata->hw_type);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get hw_type\n");
		return ERR_PTR(-EINVAL);
	}

	ret = of_property_read_u32(nspi, OF_SPI_BUS_NUM, &pdev->id);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get bus number\n");
		return ERR_PTR(-EINVAL);
	}

	platdata->pm_platdata = of_device_state_pm_setup(nspi);
	if (IS_ERR(platdata->pm_platdata)) {
		dev_err(&pdev->dev, "Error during device state pm init\n");
		return ERR_PTR(-EINVAL);
	}

	/* DMA request */
	if (of_property_read_string(nspi, OF_SPI_DMA_TXREQ,
					&platdata->dma_tx_name)
		|| of_property_read_string(nspi, OF_SPI_DMA_RXREQ,
					&platdata->dma_rx_name)) {
		platdata->dma_rx_name = NULL;
		platdata->dma_tx_name = NULL;
	}

	pdev->dev.platform_data = platdata;

	return platdata;
}
#else
static struct xgold_spi_platdata *
		xgold_spi_get_platdata(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(master);
	struct xgold_spi_platdata *platdata = dev_get_platdata(&pdev->dev);

#ifdef CONFIG_HAVE_CLK
	if (platdata->clk_spi_name)
		platdata->clock_spi = clk_get(NULL, platdata->clk_spi_name);

	if (platdata->clk_ahb_name)
		platdata->clock_ahb = clk_get(NULL, platdata->clk_ahb_name);
#endif

	return platdata;
}

#endif

static int xgold_spi_init_ctrl_from_platdata(struct platform_device *pdev)
{
	int ret = 0, irq, i;
	struct resource *res_reg;
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(master);
	struct device *dev = &pdev->dev;
	struct xgold_spi_platdata *platdata = dev_get_platdata(&pdev->dev);

	res_reg = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			XGOLD_SPI_REG_RES_NAME);

	if (!res_reg)
		return -EINVAL;

#ifndef CONFIG_NKERNEL
	if (!request_mem_region(res_reg->start, resource_size(res_reg),
					dev_name(&pdev->dev)))
		return -EPERM;
#endif

	ctl_drv->base = ioremap(res_reg->start, resource_size(res_reg));
	if (!ctl_drv->base) {
		ret = -EINVAL;
		goto spi_release_region;
	}

	if (platdata->dma_tx_name && platdata->dma_rx_name)
		ctl_drv->pio_mode = false;
	else
		ctl_drv->pio_mode = true;

	for (i = 0;; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			break;
		ctl_drv->spi_irq[i] = irq;
	};

	ctl_drv->irq_num = i;

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

	platdata->pins_reset = pinctrl_lookup_state(platdata->pinctrl,
			"reset");
	if (IS_ERR(platdata->pins_reset))
		dev_err(dev, "could not get reset pinstate\n");

skip_pinctrl:
	return 0;

spi_release_region:
	release_mem_region(res_reg->start, resource_size(res_reg));
	return ret;

}

void xgold_spi_write_fifo(struct xgold_spi_ctl_drv *ctl_drv, int len)
{
	unsigned *data = (unsigned *)ctl_drv->current_txbuf;
	while (len > 0) {
		iowrite32(*data++, USIF_TXD(ctl_drv->base));
		len -= 4;
	}
}

static irqreturn_t spi_irq_handler(int irq, void *dev_id)
{
	struct xgold_spi_ctl_drv *ctl_drv = (struct xgold_spi_ctl_drv *)dev_id;
	struct spi_transfer *xfer = ctl_drv->current_xfer;
	void *fifo_rx = (void *)(((unsigned)ctl_drv->base) + USIF_RXD_OFFSET);
	unsigned payload_tx, payload_rx, remaining;
	unsigned int reg;
#define MAX(a , b) (a > b ? a : b)
#define MIN(a , b) (a > b ? b : a)
	unsigned mis;

/*	XGOLD_SPI_FUNC_IN; */

	mis = ioread32(USIF_MIS(ctl_drv->base));

	if (mis & USIF_MASK_ISR_ERR) {
		USIF_SPI_HANDLE(MC)
		USIF_SPI_HANDLE(SLIP)
		USIF_SPI_HANDLE(CRC)
		USIF_SPI_HANDLE(PHE)
		USIF_SPI_HANDLE(FE)
		USIF_SPI_HANDLE(PE)
		USIF_SPI_HANDLE(TXOF)
		USIF_SPI_HANDLE(TXUR)
		USIF_SPI_HANDLE(RXOF)
		USIF_SPI_HANDLE(RXUR)
	}

	while ((mis = ioread32(USIF_MIS(ctl_drv->base))) &
			(USIF_SPI_TXFIFO_MASK | USIF_SPI_RXFIFO_MASK)) {
		/* RX SINGLE REQUEST */
		if (mis & USIF_SPI_RXFIFO_MASK) {
			unsigned rxffs = 0;

			if (!ctl_drv->current_rxbuf) {
				dev_err(ctl_drv->dev, "Unexpected RX interrupt\n");
				goto skip_rx_memcpy;
			}

			/*
			   RX FIFO is word aligned,
			   so we can rely on the RXFFS number,
			   .i.e. is the exact number of received characters
			 */
			reg = ioread32(USIF_FIFO_STAT(ctl_drv->base));
			rxffs = USIF_FIFO_STAT_RXFFS(reg);
			if (ctl_drv->packet_size == 4) {
				payload_rx = rxffs * 4;
				memcpy(ctl_drv->current_rxbuf, fifo_rx,
				       payload_rx);
				ctl_drv->current_rxbuf += payload_rx;
				ctl_drv->current_rxlen += payload_rx;
			} else {
				unsigned i = 0;
				for (i = 0; i < rxffs; i++) {
					unsigned data =
					    ioread32(USIF_RXD(ctl_drv->base));
					if (ctl_drv->packet_size == 2) {
						*((unsigned short *)ctl_drv->
						  current_rxbuf) =
			 (unsigned short)data;
					} else {
						*((unsigned char *)ctl_drv->
						  current_rxbuf) =
			  (unsigned char)data;
					}
					ctl_drv->current_rxbuf +=
					    ctl_drv->packet_size;
					ctl_drv->current_rxlen +=
					    ctl_drv->packet_size;
				}
			}
			/* Trig end of transfer in RX only mode */
			if (!ctl_drv->current_txbuf &&
				ctl_drv->current_rxlen == xfer->len)
				complete(&ctl_drv->done);

skip_rx_memcpy:
			iowrite32(mis & USIF_SPI_RXFIFO_MASK,
					USIF_ICR(ctl_drv->base));
		}

		/* TX SINGLE REQUEST */
		if (mis & USIF_SPI_TXFIFO_MASK) {
			unsigned fifo_room_tx, txffs = 0;
			reg = ioread32(USIF_FIFO_STAT(ctl_drv->base));
			txffs = USIF_FIFO_STAT_TXFFS(reg);
			fifo_room_tx =
			    (ctl_drv->hwinfo.tx_fifo_width - txffs) * 4;
			remaining = xfer->len - ctl_drv->current_txlen;
			payload_tx = MIN(remaining, fifo_room_tx);
			 /* When doing both TX and RX in parallel
			  * sometimes get a RX_OVF error.
			  * To prevent this, the TX FIFO must not be completelly
			  * filled up, but only amount of data according to
			  * burst size (BS4 * 4Bytes).
			  */
			if (ctl_drv->current_rxbuf) {
				/* Try to fill up 2 bursts */
				payload_tx = MIN(payload_tx, 4 * 4);
			}
			xgold_spi_write_fifo(ctl_drv, payload_tx);
			ctl_drv->current_txbuf += payload_tx;
			ctl_drv->current_txlen += payload_tx;
			iowrite32(mis & USIF_SPI_TXFIFO_MASK,
					USIF_ICR(ctl_drv->base));
		}
	}

	if (mis & USIF_MASK_ISR_STA) {
		USIF_SPI_HANDLE(TMO)
		USIF_SPI_HANDLE(FRM_PAUSE)
		if (mis & USIF_RIS_TX_FIN_MASK) {
			iowrite32(USIF_RIS_TX_FIN_MASK,
					USIF_ICR(ctl_drv->base));
			reg = ioread32(USIF_FIFO_STAT(ctl_drv->base));
			if (USIF_FIFO_STAT_TXFFS(reg))
				BUG();
			complete(&ctl_drv->done);
		}
	}

/*	XGOLD_SPI_FUNC_OUT; */
	return IRQ_HANDLED;
}

/*
  CS Handling might be not compliant with SPI API specification,
  as explained in struct spi_transfer declaration in linux/spi.h
  see cs_change explanation
*/

static inline void xgold_spi_set_cs_gpio(struct xgold_spi_ctl_drv *ctl_drv,
			     struct spi_device *spi, bool active)
{
	unsigned gpio_value;


	if (spi->mode & SPI_CS_HIGH)
		gpio_value = (active == true) ? 1 : 0;
	else
		gpio_value = (active == false) ? 1 : 0;

	pr_info("SPI set cs %s, gpio %d\n",
			(active ? "ACTIVE" : "INACTIVE"),
			gpio_value);

	gpio_set_value(spi->cs_gpio, gpio_value);
}

static inline void xgold_spi_set_cs_usif(struct xgold_spi_ctl_drv *ctl_drv,
			     struct spi_device *spi, bool active)
{
	unsigned reg;

	xgold_spi_set_configuration_mode(ctl_drv);
	reg = ioread32(USIF_CS_CFG(ctl_drv->base));
	reg &= ~USIF_CS_CFG_CSO_MASK;

	if (active)
		reg |= ((~(BIT(spi->chip_select))) & 0xFF)
				<< USIF_CS_CFG_CSO_OFFSET;
	else
		reg |= (BIT(spi->chip_select) & 0xFF)
				<< USIF_CS_CFG_CSO_OFFSET;

	iowrite32(reg, USIF_CS_CFG(ctl_drv->base));
	/* XXX: Not to enable clock here */
	/* xgold_spi_set_run_mode(ctl_drv); */
}


static inline void enable_cs(struct xgold_spi_ctl_drv *ctl_drv,
			     struct spi_device *spi)
{

	if (CS_IS_GPIO(spi))
		xgold_spi_set_cs_gpio(ctl_drv, spi, true);
	else
		xgold_spi_set_cs_usif(ctl_drv, spi, true);

}

static inline void disable_cs(struct xgold_spi_ctl_drv *ctl_drv,
			      struct spi_device *spi)
{
	if (CS_IS_GPIO(spi))
		xgold_spi_set_cs_gpio(ctl_drv, spi, false);
	else
		xgold_spi_set_cs_usif(ctl_drv, spi, false);

}

static void xgold_spi_dma_finishtx(struct xgold_spi_ctl_drv *ctl_drv)
{
	unsigned int reg;
	struct spi_message *msg = ctl_drv->current_msg;

	/* disable tx request */
	reg = ioread32(USIF_IMSC(ctl_drv->base));
	reg &= ~USIF_SPI_TXFIFO_MASK;
	iowrite32(reg, USIF_IMSC(ctl_drv->base));

	if (!msg->is_dma_mapped)
		dma_unmap_sg(ctl_drv->dev, &ctl_drv->sg_io[XGOLD_SPI_DMA_TX],
				1, DMA_TO_DEVICE);
}

static void xgold_spi_dma_finishrx(struct xgold_spi_ctl_drv *ctl_drv)
{
	unsigned int reg;
	struct spi_message *msg = ctl_drv->current_msg;

	/*disable rx request */
	reg = ioread32(USIF_IMSC(ctl_drv->base));
	reg &= ~USIF_SPI_RXFIFO_MASK;
	iowrite32(reg, USIF_IMSC(ctl_drv->base));

	if (!msg->is_dma_mapped)
		dma_unmap_sg(ctl_drv->dev, &ctl_drv->sg_io[XGOLD_SPI_DMA_RX],
				1, DMA_FROM_DEVICE);
}

static void xgold_spi_dma_callback_rx(void *param)
{
	struct xgold_spi_ctl_drv *ctl_drv = param;
	struct dma_chan *dmach = ctl_drv->dmach[XGOLD_SPI_DMA_RX];
	dma_cookie_t dma_cookie = ctl_drv->dma_cookie[XGOLD_SPI_DMA_RX];
	unsigned long flags;

	spin_lock_irqsave(&ctl_drv->lock, flags);
	if (dma_async_is_tx_complete(dmach, dma_cookie, NULL, NULL) ==
			DMA_COMPLETE) {
		ctl_drv->state &= ~RXBUSY;
		dev_dbg(ctl_drv->dev, "spi rx complete");
		xgold_spi_dma_finishrx(ctl_drv);
	}
	/* If the other done */
	if (!(ctl_drv->state & TXBUSY))
		complete(&ctl_drv->xfer_completion);

	spin_unlock_irqrestore(&ctl_drv->lock, flags);
}

static void xgold_spi_dma_callback_tx(void *param)
{
	struct xgold_spi_ctl_drv *ctl_drv = param;
	struct dma_chan *dmach = ctl_drv->dmach[XGOLD_SPI_DMA_TX];
	dma_cookie_t dma_cookie = ctl_drv->dma_cookie[XGOLD_SPI_DMA_TX];
	unsigned long flags;

	spin_lock_irqsave(&ctl_drv->lock, flags);
	if (dma_async_is_tx_complete(dmach, dma_cookie, NULL, NULL) ==
			DMA_COMPLETE) {
		ctl_drv->state &= ~TXBUSY;
		dev_dbg(ctl_drv->dev, "spi tx complete");
		xgold_spi_dma_finishtx(ctl_drv);
	}
	/* If the other done */
	if (!(ctl_drv->state & RXBUSY))
		complete(&ctl_drv->xfer_completion);

	spin_unlock_irqrestore(&ctl_drv->lock, flags);
}

static int xgold_spi_start_transfer_dma(struct spi_message *msg,
					struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *desctx, *descrx;
	struct spi_device *spi = msg->spi;
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(spi->master);
	struct dma_chan *dmach_rx = ctl_drv->dmach[XGOLD_SPI_DMA_RX];
	struct dma_chan *dmach_tx = ctl_drv->dmach[XGOLD_SPI_DMA_TX];
	struct scatterlist *sg_rx = &ctl_drv->sg_io[XGOLD_SPI_DMA_RX];
	struct scatterlist *sg_tx = &ctl_drv->sg_io[XGOLD_SPI_DMA_TX];
	unsigned int reg, imsc, mode;
	int ret;

	if (xfer == NULL)
		return -EINVAL;

	imsc = mode = 0;

	if (xfer->tx_buf == NULL)
		goto dma_skip_tx;

	ctl_drv->state |= TXBUSY;
	sg_init_one(sg_tx, xfer->tx_buf, round_up(xfer->len, 4));

	if (msg->is_dma_mapped) {
		sg_tx->dma_address = xfer->tx_dma;
	} else {
		ret = dma_map_sg(ctl_drv->dev, sg_tx, 1, DMA_TO_DEVICE);
		if (ret == 0) {
			dev_err(ctl_drv->dev, "Mapping TX buffers failed!\n");
			return -EINVAL;
		}
	}

	desctx = dmaengine_prep_slave_sg(dmach_tx, sg_tx, 1,
			DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
	if (!desctx) {
		dev_err(ctl_drv->dev,
			"Failed to get DMA data write descriptor\n");
		goto write_init_dma_fail;
	}

	/* Set the DMA tx callback */
	desctx->callback = xgold_spi_dma_callback_tx;
	desctx->callback_param = ctl_drv;
	/* Start tx transfer */
	ctl_drv->dma_cookie[XGOLD_SPI_DMA_TX] = dmaengine_submit(desctx);
	dma_async_issue_pending(dmach_tx);
	/* enable tx request */
	imsc |= USIF_SPI_TXFIFO_MASK;
	/* enable tx */
	mode |= USIF_MODE_CFG_TXEN_EN;

dma_skip_tx:
	if (xfer->rx_buf == NULL)
		goto dma_skip_rx;

	ctl_drv->state |= RXBUSY;
	sg_init_one(sg_rx, xfer->rx_buf, round_up(xfer->len, 4));
	if (msg->is_dma_mapped) {
		sg_rx->dma_address = xfer->rx_dma;
	} else {
		ret = dma_map_sg(ctl_drv->dev, sg_rx, 1, DMA_FROM_DEVICE);
		if (ret == 0) {
			dev_err(ctl_drv->dev, "Mapping RX buffers failed!\n");
			return -EINVAL;
		}
	}

	descrx = dmaengine_prep_slave_sg(dmach_rx, sg_rx, 1,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!descrx) {
		dev_err(ctl_drv->dev,
			"Failed to get DMA data read descriptor\n");
		goto read_init_dma_fail;
	}
	/* Set the DMA rx callback */
	descrx->callback = xgold_spi_dma_callback_rx;
	descrx->callback_param = ctl_drv;
	/* Start rx transfer */
	ctl_drv->dma_cookie[XGOLD_SPI_DMA_TX] = dmaengine_submit(descrx);
	dma_async_issue_pending(dmach_rx);
	/* enable rx request */
	imsc |= USIF_SPI_RXFIFO_MASK;
	/* enable rx */
	mode |= USIF_MODE_CFG_RXEN_EN;

dma_skip_rx:
	xgold_spi_set_configuration_mode(ctl_drv);
	/* clear tx and rx fifo request */
	iowrite32(USIF_SPI_RXFIFO_MASK | USIF_SPI_TXFIFO_MASK,
		  USIF_ICR(ctl_drv->base));
	/* enable request */
	reg = ioread32(USIF_IMSC(ctl_drv->base));
	reg &= ~USIF_IMSC_CTRL;
	reg |= imsc;
	iowrite32(reg, USIF_IMSC(ctl_drv->base));
	/* enable tx, rx. Check if half-duplex is required */
	reg = ioread32(USIF_MODE_CFG(ctl_drv->base));
	reg &= ~(USIF_MODE_CFG_TXEN_MASK | USIF_MODE_CFG_RXEN_MASK |
			USIF_MODE_CFG_HDEN_MASK | USIF_MODE_CFG_SCFRC_MASK);
	if (xfer->tx_buf == NULL && xfer->rx_buf != NULL) {
		/* RX only */
		mode |= USIF_MODE_CFG_HDEN_MASK | USIF_MODE_CFG_SCFRC_MASK;
	} else if (xfer->tx_buf != NULL && xfer->rx_buf == NULL) {
		/* RX only */
		mode |= USIF_MODE_CFG_HDEN_MASK;
	}
	reg |= mode;
	iowrite32(reg, USIF_MODE_CFG(ctl_drv->base));
	xgold_spi_set_run_mode(ctl_drv);

	return 0;

read_init_dma_fail:
	dma_unmap_sg(ctl_drv->dev, sg_rx, 1, DMA_FROM_DEVICE);
	dmaengine_terminate_all(dmach_rx);
	if (xfer->tx_buf == NULL)
		return -EINVAL;
write_init_dma_fail:
	dma_unmap_sg(ctl_drv->dev, sg_tx, 1, DMA_TO_DEVICE);
	dmaengine_terminate_all(dmach_tx);
	return -EINVAL;
}

static int xgold_spi_start_transfer_pio(struct spi_device *spi,
					struct spi_transfer *xfer)
{
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(spi->master);
	unsigned packet_len = 0;
	unsigned int reg, imsc, mode;

	imsc = mode = 0;
	ctl_drv->packet_size =
	    (spi->bits_per_word > 16 ? 4 : (spi->bits_per_word > 8 ? 2 : 1));
	ctl_drv->current_txlen = ctl_drv->current_rxlen = 0;
	ctl_drv->current_rxbuf = ctl_drv->current_txbuf = NULL;
	packet_len = xfer->len / ctl_drv->packet_size;
	if (xfer->len % ctl_drv->packet_size)
		packet_len++;

	if (xfer->tx_buf == NULL)
		goto pio_skip_tx;

	ctl_drv->current_txbuf = (unsigned char *)xfer->tx_buf;
	/* enable tx request */
	imsc |= USIF_SPI_TX_MASK;
	/* enable tx */
	mode |= USIF_MODE_CFG_TXEN_EN;

pio_skip_tx:
	if (xfer->rx_buf == NULL)
		goto pio_skip_rx;

	ctl_drv->current_rxbuf = xfer->rx_buf;
	/* enable rx request */
	imsc |= USIF_SPI_RX_MASK;
	/* enable rx */
	mode |= USIF_MODE_CFG_RXEN_EN;

pio_skip_rx:
	xgold_spi_set_configuration_mode(ctl_drv);

	/* FIX: kernel panic at spi-xgold.c : spi_irq_handler : BUG()
	 *
	 * As documented in SoFIA_3G_R_SoC_V1.0_BBHW_UM.pdf
	 * 4.19.3.6 Interrupt and DMA Registers
	 * Page: 2591
	 *
	 * Before enabling a source in the IMSC it is good practice to always first
	 * clear the corresponding bit in the RIS via ICR before enabling a source in the
	 * IMSC.
	 */
	iowrite32(USIF_RIS_TX_FIN_MASK, USIF_ICR(ctl_drv->base));

	/* enable request */
	imsc |= USIF_MASK_ISR_STA | USIF_MASK_ISR_ERR;
	iowrite32(imsc, USIF_IMSC(ctl_drv->base));
	/* enable tx, rx. Check if half-duplex is required */
	reg = ioread32(USIF_MODE_CFG(ctl_drv->base));
	reg &= ~(USIF_MODE_CFG_TXEN_MASK | USIF_MODE_CFG_RXEN_MASK |
			USIF_MODE_CFG_HDEN_MASK | USIF_MODE_CFG_SCFRC_MASK);
	if (xfer->tx_buf == NULL && xfer->rx_buf != NULL) {
		/* RX only */
		mode |= USIF_MODE_CFG_HDEN_MASK | USIF_MODE_CFG_SCFRC_MASK;
	} else if (xfer->tx_buf != NULL && xfer->rx_buf == NULL) {
		/* RX only */
		mode |= USIF_MODE_CFG_HDEN_MASK;
	}
	reg |= mode;
	iowrite32(reg, USIF_MODE_CFG(ctl_drv->base));
	xgold_spi_set_run_mode(ctl_drv);
	return 0;
}

static int wait_for_xfer(struct xgold_spi_ctl_drv *ctl_drv,
			 struct spi_transfer *xfer)
{
	unsigned long val;
	int ms;

	/* millisecs to xfer 'len' bytes @ 'cur_speed' */
	ms = xfer->len * 8 * 1000 / ctl_drv->cur_speed;
	ms += 10;		/* some tolerance */

	if (!ctl_drv->pio_mode) {
		val = msecs_to_jiffies(ms) + 10;
		val =
		    wait_for_completion_timeout(&ctl_drv->xfer_completion, val);
		if (!val)
			return -EIO;
		return 0;
	}
	val = msecs_to_jiffies(ms) + 10;
	val = wait_for_completion_timeout(&ctl_drv->done, val);
	if (!val)
		return -EIO;
	return 0;
}

static void xgold_spi_config_cs(struct xgold_spi_ctl_drv *ctl_drv,
						struct spi_device *spi)
{
	int chip_select = spi->chip_select;
	unsigned reg, val;

	if (CS_IS_GPIO(spi))
		return;

	/* FIXME: Do we need to set the CS_CFG if cs
		is a gpio to deselect previous devices ?
	 */

	/*
	 * Chip Select configuration
	 * We assume that typically a CS is active low.
	 * The CSOINV will take care to deal with CS active high.
	 */
	reg = ioread32(USIF_CS_CFG(ctl_drv->base));
	val = USIF_CS_CFG_CSOINV(reg);
	if (ctl_drv->cur_mode & SPI_CS_HIGH)
		val |= BIT(chip_select);
	else
		val &= ~BIT(chip_select);

	reg &= ~USIF_CS_CFG_CSOINV_MASK;
	reg |= (val << USIF_CS_CFG_CSOINV_OFFSET);
	iowrite32(reg, USIF_CS_CFG(ctl_drv->base));

	reg = ioread32(USIF_CS_CFG(ctl_drv->base)) & (~USIF_CS_CFG_CSEN_MASK);
	if (ctl_drv->cur_mode & SPI_NO_CS)
		reg |= USIF_CS_CFG_CSEN_DIS;
	else
		reg |= USIF_CS_CFG_CSEN_EN;
	iowrite32(reg, USIF_CS_CFG(ctl_drv->base));

}

static void xgold_spi_config(struct xgold_spi_ctl_drv *ctl_drv,
					struct spi_device *spi)
{
	u32 val, reg;
	unsigned mode;

	/* Set the USIF in configuration mode */
	xgold_spi_set_configuration_mode(ctl_drv);

	mode = ioread32(USIF_MODE_CFG(ctl_drv->base));
	mode &=
	    ~(USIF_MODE_CFG_SCPH_MASK | USIF_MODE_CFG_SCPOL_MASK |
	      USIF_MODE_CFG_TX_IDLE_MASK);
	/* Set the mode */
	if (ctl_drv->cur_mode & SPI_CPOL)
		mode |= USIF_MODE_CFG_SCPOL_HIGH;
	else
		mode |= USIF_MODE_CFG_SCPOL_LOW;
	if (ctl_drv->cur_mode & SPI_CPHA)
		mode |= USIF_MODE_CFG_SCPH_LEAD;
	else
		mode |= USIF_MODE_CFG_SCPH_TRAIL;
	if (ctl_drv->cur_mode & SPI_TX_IDLE_HIGH)
		mode |= USIF_MODE_CFG_TX_IDLE_HIGH;
	else
		mode |= USIF_MODE_CFG_TX_IDLE_LOW;
	iowrite32(mode, USIF_MODE_CFG(ctl_drv->base));

	mode = ioread32(USIF_PRTC_CFG(ctl_drv->base));
	mode &=
	    ~(USIF_PRTC_CFG_HD_MASK | USIF_PRTC_CFG_PAR_MASK |
	      USIF_PRTC_CFG_ODD_MASK);
	if (ctl_drv->cur_mode & SPI_LSB_FIRST)
		mode |= USIF_PRTC_CFG_HD_LSB;
	else
		mode |= USIF_PRTC_CFG_HD_MSB;

	if (ctl_drv->cur_mode & (SPI_PARITY_EVEN | SPI_PARITY_ODD))
		mode |= (1 << USIF_PRTC_CFG_PAR_OFFSET);
	else
		mode |= (0 << USIF_PRTC_CFG_PAR_OFFSET);

	if (ctl_drv->cur_mode & SPI_PARITY_ODD)
		mode |= USIF_PRTC_CFG_ODD_ODD;
	else
		mode |= USIF_PRTC_CFG_ODD_EVEN;
	iowrite32(mode, USIF_PRTC_CFG(ctl_drv->base));

	reg = ioread32(USIF_FDIV_CFG(ctl_drv->base));
	reg &= ~(USIF_FDIV_CFG_INC_MASK | USIF_FDIV_CFG_DEC_MASK);
	reg |= (0 << USIF_FDIV_CFG_INC_OFFSET) |
		(0 << USIF_FDIV_CFG_DEC_OFFSET);
	iowrite32(reg, USIF_FDIV_CFG(ctl_drv->base));

	val = ioread32(USIF_PRTC_CFG(ctl_drv->base));
	val &= ~USIF_PRTC_CFG_CLEN_MASK;
	/* Set Character lenght and FIFO alignment */
	switch (ctl_drv->cur_bpw) {
	case 32:
		val |= USIF_PRTC_CFG_CLEN_N32;
		break;
	case 16:
		val |= (16 << USIF_PRTC_CFG_CLEN_OFFSET);
		break;
	default:
		val |= (8 << USIF_PRTC_CFG_CLEN_OFFSET);
		break;
	}
	iowrite32(val, USIF_PRTC_CFG(ctl_drv->base));

	val = ioread32(USIF_FIFO_CFG(ctl_drv->base));
	val &= ~(USIF_FIFO_CFG_TXFA_MASK | USIF_FIFO_CFG_RXFA_MASK);
	if (ctl_drv->cur_bpw > 16) {
		val |= USIF_FIFO_CFG_TXFA_TXFA4;
	} else {
		if (ctl_drv->cur_bpw > 8)
			val |= USIF_FIFO_CFG_TXFA_TXFA2;
		else
			val |= USIF_FIFO_CFG_TXFA_TXFA1;
	}

	/* Must be byte aligned in DMA mode */
	if (ctl_drv->pio_mode)
		val |= USIF_FIFO_CFG_RXFA_RXFA4;
	else
		val |= USIF_FIFO_CFG_RXFA_RXFA1;

	iowrite32(val, USIF_FIFO_CFG(ctl_drv->base));
	val = ioread32(USIF_FIFO_CFG(ctl_drv->base));
#if 0
	val |= USIF_FIFO_CFG_RXFA_RXFA4 | USIF_FIFO_CFG_TXFA_TXFA4;
	iowrite32(val, USIF_FIFO_CFG(ctl_drv->base));
#endif

	/* Configure Clock */
	val = ioread32(USIF_BC_CFG(ctl_drv->base));
	val &= ~USIF_BC_CFG_BCRV_MASK;
	val |=
	    (((ctl_drv->clk_spi_rate / ctl_drv->cur_speed / 4 -
	       1)) << USIF_BC_CFG_BCRV_OFFSET);

	iowrite32(val, USIF_BC_CFG(ctl_drv->base));

	/* Initial configuration of Chip Select Mode */
	xgold_spi_config_cs(ctl_drv, spi);

	/* Set USIF in run mode */
	xgold_spi_set_run_mode(ctl_drv);
}

#define XFER_DMAADDR_INVALID DMA_BIT_MASK(32)

static void xgold_spi_abort_msg(struct xgold_spi_ctl_drv *ctl_drv,
		struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct spi_transfer *xfer = ctl_drv->current_xfer;
	unsigned long flags;

	spin_lock_irqsave(&ctl_drv->lock, flags);
	dev_err(&spi->dev, "I/O Error: rx-%d tx-%d res:rx-%c tx-%c len-%d\n",
			xfer->rx_buf ? 1 : 0, xfer->tx_buf ? 1 : 0,
			(ctl_drv->state & RXBUSY) ? 'f' : 'p',
			(ctl_drv->state & TXBUSY) ? 'f' : 'p',
			xfer->len);

	if (!ctl_drv->pio_mode) {
		if (ctl_drv->state & TXBUSY) {
			dmaengine_terminate_all(
					ctl_drv->dmach[XGOLD_SPI_DMA_TX]);
			xgold_spi_dma_finishtx(ctl_drv);
		}

		if ((xfer->rx_buf != NULL) && (ctl_drv->state & RXBUSY)) {
			dmaengine_terminate_all(
					ctl_drv->dmach[XGOLD_SPI_DMA_RX]);
			xgold_spi_dma_finishrx(ctl_drv);
		}
	}

	spin_unlock_irqrestore(&ctl_drv->lock, flags);
}

static void handle_msg(struct xgold_spi_ctl_drv *ctl_drv,
		       struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct spi_transfer *xfer, *prev = NULL;
	int status = 0;
	u32 speed, reg;
	u8 bpw;

	/* If Master's (controller) state differs from that needed by Slave */
	if (ctl_drv->cur_speed != spi->max_speed_hz
	    || ctl_drv->cur_mode != spi->mode
	    || ctl_drv->cur_bpw != spi->bits_per_word) {
		ctl_drv->cur_bpw = spi->bits_per_word;
		ctl_drv->cur_speed = spi->max_speed_hz;
		ctl_drv->cur_mode = spi->mode;
		xgold_spi_config(ctl_drv, spi);
	}

	dev_dbg(ctl_drv->dev, "Pushing msg %p\n", msg);
	enable_cs(ctl_drv, spi);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		unsigned long flags;
		unsigned payload = xfer->len * 8 / ctl_drv->cur_bpw;

		if ((prev != NULL) && (prev->cs_change))
			enable_cs(ctl_drv, spi);

		init_completion(&ctl_drv->xfer_completion);
		init_completion(&ctl_drv->done);

		/* Only BPW and Speed may change across transfers */
		bpw = xfer->bits_per_word ? : spi->bits_per_word;
		speed = xfer->speed_hz ? : spi->max_speed_hz;

		if (bpw != ctl_drv->cur_bpw || speed != ctl_drv->cur_speed) {
			ctl_drv->cur_bpw = bpw;
			ctl_drv->cur_speed = speed;
			xgold_spi_config(ctl_drv, spi);
		}

		spin_lock_irqsave(&ctl_drv->lock, flags);

		ctl_drv->current_msg = msg;
		ctl_drv->current_xfer = xfer;

		/* Pending only which is to be done */
		ctl_drv->state &= ~RXBUSY;
		ctl_drv->state &= ~TXBUSY;

		/* Start the signals */
		if (ctl_drv->pio_mode)
			xgold_spi_start_transfer_pio(spi, xfer);
		else
			xgold_spi_start_transfer_dma(msg, xfer);

		spin_unlock_irqrestore(&ctl_drv->lock, flags);

		if (xfer->len % ctl_drv->packet_size)
			payload++;

		if (xfer->rx_buf != NULL)
			iowrite32(payload, USIF_MRPS_CTRL(ctl_drv->base));

		if (xfer->tx_buf != NULL)
			iowrite32(payload, USIF_TPS_CTRL(ctl_drv->base));

		status = wait_for_xfer(ctl_drv, xfer);

		xgold_spi_set_configuration_mode(ctl_drv);
		reg = ioread32(USIF_MODE_CFG(ctl_drv->base));
		reg &= ~(USIF_MODE_CFG_TXEN_MASK | USIF_MODE_CFG_RXEN_MASK);
		reg |= (USIF_MODE_CFG_RXEN_DIS | USIF_MODE_CFG_TXEN_DIS);
		iowrite32(reg, USIF_MODE_CFG(ctl_drv->base));
		xgold_spi_set_run_mode(ctl_drv);

#ifdef VERBOSE
		dump_xfer(xfer);
		dump_tx_xfer(xfer);
		dump_rx_xfer(xfer);
#endif

		if (status) {
			xgold_spi_abort_msg(ctl_drv, msg);
			goto out;
		}

		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);

		if (xfer->cs_change)
			disable_cs(ctl_drv, spi);
		prev = xfer;
		msg->actual_length += xfer->len;
	}

out:
	disable_cs(ctl_drv, spi);
	msg->status = status;
	dev_dbg(ctl_drv->dev, "Msg %p completed with status %d\n", msg, status);
	if (msg->complete)
		msg->complete(msg->context);

}

static int xgold_spi_dma_config(struct xgold_spi_ctl_drv *ctl_drv)
{

	struct dma_slave_config configtx, configrx;
	int ret = 0;

	if (!ctl_drv->dmach[XGOLD_SPI_DMA_RX] ||
			!ctl_drv->dmach[XGOLD_SPI_DMA_TX])
		return -EINVAL;

	/* tx */
	memset(&configtx, 0, sizeof(configtx));
	configtx.direction = DMA_TO_DEVICE;
	configtx.dst_addr = (dma_addr_t)USIF_TXD(ctl_drv->base);
	configtx.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	configtx.dst_maxburst = 4;
	configtx.device_fc = true;
	ret = dmaengine_slave_config(ctl_drv->dmach[XGOLD_SPI_DMA_TX],
			&configtx);
	if (ret)
		return ret;

	/* rx */
	memset(&configrx, 0, sizeof(configrx));
	configrx.direction = DMA_FROM_DEVICE;
	configrx.src_addr = (dma_addr_t)USIF_RXD(ctl_drv->base);
	configrx.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	configrx.src_maxburst = 4;
	configrx.device_fc = true;
	ret = dmaengine_slave_config(ctl_drv->dmach[XGOLD_SPI_DMA_RX],
			&configrx);

	return ret;
}

static void xgold_spi_work(struct work_struct *work)
{
	struct xgold_spi_ctl_drv *ctl_drv =
	    container_of(work, struct xgold_spi_ctl_drv, work);
	unsigned long flags;
	spin_lock_irqsave(&ctl_drv->lock, flags);
	while (!list_empty(&ctl_drv->queue) && !(ctl_drv->state & SUSPND)) {
		struct spi_message *msg;

		msg =
		    container_of(ctl_drv->queue.next, struct spi_message,
				 queue);
		list_del_init(&msg->queue);

		/* Set Xfer busy flag */
		ctl_drv->state |= SPIBUSY;

		spin_unlock_irqrestore(&ctl_drv->lock, flags);

		handle_msg(ctl_drv, msg);

		spin_lock_irqsave(&ctl_drv->lock, flags);

		ctl_drv->state &= ~SPIBUSY;
	}

	spin_unlock_irqrestore(&ctl_drv->lock, flags);
}

static int xgold_spi_reset(struct spi_device *spi, bool reset)
{
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(spi->master);
	struct xgold_spi_platdata *pdata = ctl_drv->platdata;
	struct pinctrl_state *pinstate;
	int ret = 0;

	if (!pdata) {
		dev_err(ctl_drv->dev, "Unable to retrieve usif platform data\n");
		return -EINVAL;
	}

	if (reset)
		pinstate = pdata->pins_reset;
	else
		pinstate = pdata->pins_default;

	if (!IS_ERR_OR_NULL(pinstate)) {
		ret = pinctrl_select_state(pdata->pinctrl, pinstate);
		if (ret)
			dev_err(ctl_drv->dev, "could not set pins\n");
	}
	return ret;
}

static int xgold_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(spi->master);
	unsigned long flags;
	xgold_spi_debug(XGOLD_SPI_ENTER, "%s: %s: ENTER\n", DRIVER_NAME,
			__func__);
#ifdef VERBOSE
	{
		struct spi_transfer *xfer;
		pr_debug("SPI transfer, DMA mapped: %s\n",
				msg->is_dma_mapped ? "true" : "false");
		list_for_each_entry(xfer, &msg->transfers, transfer_list) {
			dump_xfer(xfer);
			dump_tx_xfer(xfer);
		}
	}
#endif
	/* FIXME: Should check msg->is_dma_mapped */
	/* reject invalid messages and transfers */
	if (list_empty(&msg->transfers) || !msg->complete) {
		pr_info("NO MSG\n");
		return -EINVAL;
	}
	spin_lock_irqsave(&ctl_drv->lock, flags);
	if (ctl_drv->state & SUSPND) {
		spin_unlock_irqrestore(&ctl_drv->lock, flags);
		return -ESHUTDOWN;
	}
	msg->status = -EINPROGRESS;
	msg->actual_length = 0;
	list_add_tail(&msg->queue, &ctl_drv->queue);
	queue_work(ctl_drv->workqueue, &ctl_drv->work);
	spin_unlock_irqrestore(&ctl_drv->lock, flags);
	return 0;
}

static int xgold_spi_setup(struct spi_device *spi)
{
	struct xgold_spi_ctl_drv *ctl_drv;
	struct spi_message *msg;
	u32 bcrv, speed;
	unsigned long flags;
	int err = 0;

	ctl_drv = spi_master_get_devdata(spi->master);

	spin_lock_irqsave(&ctl_drv->lock, flags);

	list_for_each_entry(msg, &ctl_drv->queue, queue) {
		/* Is some mssg is already queued for this device */
		if (msg->spi == spi) {
			dev_err(&spi->dev,
				"setup: attempt while mssg in queue!\n");
			spin_unlock_irqrestore(&ctl_drv->lock, flags);
			return -EBUSY;
		}
	}

	if (ctl_drv->state & SUSPND) {
		spin_unlock_irqrestore(&ctl_drv->lock, flags);
		dev_err(&spi->dev,
			"setup: SPI-%d not active!\n", spi->master->bus_num);
		return -ESHUTDOWN;
	}

	spin_unlock_irqrestore(&ctl_drv->lock, flags);

	if (spi->bits_per_word != 8
	    && spi->bits_per_word != 16 && spi->bits_per_word != 32) {
		dev_err(&spi->dev, "setup: %dbits/wrd not supported!\n",
			spi->bits_per_word);
		err = -EINVAL;
		goto setup_exit;
	}

	disable_cs(ctl_drv, spi);

	speed = ctl_drv->clk_spi_rate / 4 / (0 + 1);

	if (spi->max_speed_hz > speed)
		spi->max_speed_hz = speed;

	bcrv = ctl_drv->clk_spi_rate / 4 / spi->max_speed_hz - 1;
	bcrv &= USIF_BC_CFG_BCRV_MASK;
	if (bcrv == USIF_BC_CFG_BCRV_MASK)
		bcrv--;

	speed = ctl_drv->clk_spi_rate / 4 / (bcrv + 1);
	if (spi->max_speed_hz < speed) {
		if (bcrv + 1 < USIF_BC_CFG_BCRV_MASK) {
			bcrv++;
		} else {
			err = -EINVAL;
			goto setup_exit;
		}
	}

	speed = ctl_drv->clk_spi_rate / 4 / (bcrv + 1);
	if (spi->max_speed_hz >= speed)
		spi->max_speed_hz = speed;
	else
		err = -EINVAL;
	if (ctl_drv->pio_mode) {
		if (spi->inter_character_pause) {
			xgold_spi_set_configuration_mode(ctl_drv);
			iowrite32(spi->inter_character_pause * 2,
				  USIF_ICTM_CFG(ctl_drv->base));
			xgold_spi_set_run_mode(ctl_drv);
		}
	}

setup_exit:

	/* setup() returns with device de-selected */
/*      disable_cs(ctl_drv, spi); */

	return err;

}

static void xgold_spi_cleanup(struct spi_device *spi)
{
	struct xgold_spi_ctl_state *ctl_state;
	ctl_state = spi_get_ctldata(spi);
	kfree(ctl_state);
	return;
}

static void xgold_spi_hwinit(struct xgold_spi_ctl_drv *ctl_drv,
			     struct device *dev)
{
	unsigned reg;
	/* Set the USIF in configuration mode */
	xgold_spi_set_configuration_mode(ctl_drv);

	/* Fix the kernel divider to 1 */
	reg = ioread32(USIF_CLC_CNT(ctl_drv->base)) & (~USIF_CLC_CNT_RMC_MASK);
	reg |= (0x01 << USIF_CLC_CNT_RMC_OFFSET);
	iowrite32(reg, USIF_CLC_CNT(ctl_drv->base));
	/*
	   Get info on the controller
	 */
	ctl_drv->hwinfo.swcid = ioread32(USIF_SWCID(ctl_drv->base));

	reg = ioread32(USIF_ID(ctl_drv->base));
	ctl_drv->hwinfo.mod_id = USIF_ID_MOD_ID(reg);
	ctl_drv->hwinfo.ts_rev = USIF_ID_TS_REV_NR(reg);
	ctl_drv->hwinfo.rev = USIF_ID_REV_NUMBER(reg);

	reg = ioread32(USIF_FIFO_ID(ctl_drv->base));
	ctl_drv->hwinfo.rx_fifo_width = USIF_FIFO_ID_RX_STAGE(reg);
	ctl_drv->hwinfo.tx_fifo_width = USIF_FIFO_ID_TX_STAGE(reg);
	ctl_drv->hwinfo.rps_width = USIF_FIFO_ID_RPS_STAGE(reg);
	dev_info(dev, "Module ID : %x,TOPSIN ID : %x, Revision : %x\n",
		 ctl_drv->hwinfo.mod_id, ctl_drv->hwinfo.ts_rev,
		 ctl_drv->hwinfo.rev);

	reg = ioread32(USIF_MODE_CFG(ctl_drv->base));
	reg &= ~(USIF_MODE_CFG_SYNC_MASK | USIF_MODE_CFG_MA_MASK);
	reg |= (USIF_MODE_CFG_SYNC_SYN | USIF_MODE_CFG_MA_MASTER);
	iowrite32(reg, USIF_MODE_CFG(ctl_drv->base));

	/* FIXME: XR: What are we doing below ?*/
	reg = ioread32(USIF_CS_CFG(ctl_drv->base));
	if (ctl_drv->platdata->hw_type == XGOLD6XXSPI) {
		reg &=
		    ~(USIF_CS_CFG_CSOFRM_MASK | USIF_CS_CFG_CSOCLK_MASK |
		      USIF_CS_CFG_EACS_MASK | USIF_CS_CFG_CSO_MASK |
		      USIF_CS_CFG_CSOINV_MASK | USIF_CS_CFG_CSEN_MASK);
		reg |=
		    (USIF_CS_CFG_CSOFRM_NO | USIF_CS_CFG_CSOCLK_NO |
		     USIF_CS_CFG_EACS_DIS | (0xff << USIF_CS_CFG_CSO_OFFSET) |
		     (0x00 << USIF_CS_CFG_CSOINV_OFFSET) | USIF_CS_CFG_CSEN_EN);
	} else {
		reg &=
		    ~(USIF_CS_CFG_CSOFRM_MASK | USIF_CS_CFG_CSOCLK_MASK |
		      USIF_CS_CFG_EACS_MASK | USIF_CS_CFG_CSO_MASK |
		      USIF_CS_CFG_CSEN_MASK);
		reg |=
		    (USIF_CS_CFG_CSOFRM_NO | USIF_CS_CFG_CSOCLK_NO |
		     USIF_CS_CFG_EACS_DIS | (0xff << USIF_CS_CFG_CSO_OFFSET) |
		     USIF_CS_CFG_CSEN_EN);
	}
	iowrite32(reg, USIF_CS_CFG(ctl_drv->base));

	/* fifo config  */
	reg = ioread32(USIF_FIFO_CFG(ctl_drv->base));
	reg &=
	    ~(USIF_FIFO_CFG_RXBS_MASK | USIF_FIFO_CFG_TXBS_MASK |
	      USIF_FIFO_CFG_TXFC_MASK | USIF_FIFO_CFG_RXFC_MASK);
	reg |=
	    (USIF_FIFO_CFG_RXBS_RXBS4 | USIF_FIFO_CFG_TXBS_TXBS4 |
	     USIF_FIFO_CFG_TXFC_TXFC | USIF_FIFO_CFG_RXFC_RXFC);
	iowrite32(reg, USIF_FIFO_CFG(ctl_drv->base));

	/* Shift clock initialization:
	 *	Kernel clock divided by 4
	 *	Do not use feedback clock
	 */
	reg = USIF_BC_CFG_SCDIV_QUART | (0 << USIF_BC_CFG_SCDEL_OFFSET);
	iowrite32(reg, USIF_BC_CFG(ctl_drv->base));
	iowrite32(0, USIF_FDIV_CFG(ctl_drv->base));

	/* Init Inter Character Time Out */
	iowrite32(0, USIF_ICTM_CFG(ctl_drv->base));

	/* Enable/Disable DMA requests */
	if (!ctl_drv->pio_mode) {
		reg = (USIF_DMAE_TX_BREQ_DMA_CTRL
			| USIF_DMAE_TX_LBREQ_DMA_CTRL
			| USIF_DMAE_TX_SREQ_DMA_CTRL
			| USIF_DMAE_TX_LSREQ_DMA_CTRL
			| USIF_DMAE_RX_BREQ_DMA_CTRL
			| USIF_DMAE_RX_LBREQ_DMA_CTRL
			| USIF_DMAE_RX_SREQ_DMA_CTRL
			| USIF_DMAE_RX_LSREQ_DMA_CTRL);
	} else {
		reg = (USIF_DMAE_TX_BREQ_CPU_CTRL
			| USIF_DMAE_TX_LBREQ_CPU_CTRL
			| USIF_DMAE_TX_SREQ_CPU_CTRL
			| USIF_DMAE_TX_LSREQ_CPU_CTRL
			| USIF_DMAE_RX_BREQ_CPU_CTRL
			| USIF_DMAE_RX_LBREQ_CPU_CTRL
			| USIF_DMAE_RX_SREQ_CPU_CTRL
			| USIF_DMAE_RX_LSREQ_CPU_CTRL);
	}
	iowrite32(reg, USIF_DMAE(ctl_drv->base));

	xgold_spi_set_run_mode(ctl_drv);
}

static int xgold_spi_dma_init(struct xgold_spi_ctl_drv *ctl_drv)
{
	dma_cap_mask_t masktx, maskrx;
	struct dma_chan *dmach;
	int ret = 0;

	/* TX */
	dma_cap_zero(masktx);
	dma_cap_set(DMA_SLAVE, masktx);
	dmach = dma_request_channel(masktx, pl08x_filter_id,
			(void *)ctl_drv->platdata->dma_tx_name);

	if (!dmach) {
		dev_err(ctl_drv->dev,
				"can't get spi tx \"%s\" channel\n",
				ctl_drv->platdata->dma_tx_name);
		return -EINVAL;
	}
	ctl_drv->dmach[XGOLD_SPI_DMA_TX] = dmach;

	/* RX */
	dma_cap_zero(maskrx);
	dma_cap_set(DMA_SLAVE, maskrx);
	dmach = dma_request_channel(maskrx, pl08x_filter_id,
			(void *)ctl_drv->platdata->dma_rx_name);
	if (!dmach) {
		dev_err(ctl_drv->dev,
				"can't get spi rx \"%s\" channel\n",
				ctl_drv->platdata->dma_rx_name);
		ret = -EINVAL;
		goto err_rx_dma;
	}
	ctl_drv->dmach[XGOLD_SPI_DMA_RX] = dmach;

	ret = xgold_spi_dma_config(ctl_drv);
	return ret;

err_rx_dma:
	dma_release_channel(ctl_drv->dmach[XGOLD_SPI_DMA_TX]);
	return ret;
}

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT

#define XGOLD_SPI_D0	5
#define XGOLD_SPI_D3	6
#define XGOLD_SPI_D0i3	6

static int xgold_spi_set_power_state(struct device *dev, int state)
{
	struct spi_master *master = spi_master_get(dev_get_drvdata(dev));
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(master);
	struct xgold_spi_platdata *platdata = ctl_drv->platdata;
	int ret = 0;

	switch (state) {
	case XGOLD_SPI_D0:
		/* Enable needed clocks */
		if (!IS_ERR(platdata->clock_ahb)) {
			if (clk_prepare_enable(platdata->clock_ahb)) {
				dev_err(dev, "Couldn't enable clock '%s'\n",
					OF_AHB_CLK);
				ret = -EBUSY;
				goto err_clock_ahb;
			}
		}

		if (!IS_ERR(platdata->clock_spi)) {
			clk_set_rate(platdata->clock_spi, 52000000);
			if (clk_prepare_enable(platdata->clock_spi)) {
				dev_err(dev,
					"Couldn't enable clock '%s'\n",
					OF_KERNEL_CLK);
				ret = -EBUSY;
				goto err_clock_spi;
			}
			ctl_drv->clk_spi_rate =
					clk_get_rate(platdata->clock_spi);
		}
		break;
	/*case XGOLD_SPI_D0i3:*/
	case XGOLD_SPI_D3:
		if (!IS_ERR(platdata->clock_spi))
			clk_disable_unprepare(platdata->clock_spi);

		if (!IS_ERR(platdata->clock_ahb))
			clk_disable_unprepare(platdata->clock_ahb);
		break;
	default:
		return 0;
	}
	return ret;

err_clock_spi:
	if (!IS_ERR(platdata->clock_ahb))
		clk_disable_unprepare(platdata->clock_ahb);
err_clock_ahb:
	return ret;
}

static int xgold_spi_set_pm_state(struct device *dev,
		struct device_state_pm_state *state)
{
	int id;

	if (!strcmp(state->name, xgold_spi_pm_states[XGOLD_SPI_D0].name))
		id = XGOLD_SPI_D0;
	else if (!strcmp(state->name, xgold_spi_pm_states[XGOLD_SPI_D0i3].name))
		id = XGOLD_SPI_D0i3;
	else if (!strcmp(state->name, xgold_spi_pm_states[XGOLD_SPI_D3].name))
		id = XGOLD_SPI_D0i3;
	else
		return -EINVAL;

	return xgold_spi_set_power_state(dev, id);
}

static struct device_state_pm_state *xgold_spi_get_initial_state(
		struct device *dev)
{
	return &xgold_spi_pm_states[XGOLD_SPI_D3];
}
#endif

static inline int xgold_spi_set_pinctrl_state(struct device *dev,
						struct pinctrl_state *state)
{

	int ret = 0;
	struct xgold_spi_platdata *pdata = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "Unable to retrieve usif platform data\n");
		return -EINVAL;
	}

	if (!IS_ERR_OR_NULL(state)) {
		ret = pinctrl_select_state(pdata->pinctrl, state);
		if (ret)
			dev_err(dev, "%d:could not set pins\n", __LINE__);
	}
	return ret;
}

static int xgold_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct xgold_spi_ctl_drv *ctl_drv;
	int i = 0, ret = 0;
	struct xgold_spi_platdata *platdata;

	dev_info(&pdev->dev, "Registering %s\n", dev_driver_string(&pdev->dev));
	master = spi_alloc_master(&pdev->dev, sizeof(*ctl_drv));
	if (master == NULL) {
		dev_err(&pdev->dev, "master allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);
	ctl_drv = spi_master_get_devdata(master);
	ctl_drv->master = master;
	spin_lock_init(&ctl_drv->lock);
	INIT_LIST_HEAD(&ctl_drv->queue);

	platdata = ctl_drv->platdata = xgold_spi_get_platdata(pdev);
	if (IS_ERR(platdata))
		return PTR_ERR(platdata);

	ret = xgold_spi_init_ctrl_from_platdata(pdev);

	if (ret)
		return ret;

	/* Pinctrl */
	ret = xgold_spi_set_pinctrl_state(&pdev->dev, platdata->pins_default);
	if (ret) {
		dev_err(&pdev->dev, "Error while setting the pins\n");
		goto out_release;

	}

#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = platform_device_pm_set_class(pdev,
			platdata->pm_platdata->pm_user_name);
	if (ret) {
		dev_err(&pdev->dev, "Error while setting the pm class\n");
		goto out_release;
	}

	ret = platform_device_pm_set_state_by_name(pdev,
			platdata->pm_platdata->pm_state_D0_name);
#else
	ret = xgold_spi_set_power_state(pdev, XGOLD_SPI_D0);
#endif
	if (ret) {
		dev_err(&pdev->dev, "Error while powering up device\n");
		goto out_release;
	}

	ctl_drv->workqueue =
	    create_singlethread_workqueue(dev_driver_string(&pdev->dev));
	if (ctl_drv->workqueue == NULL) {
		dev_err(&pdev->dev, "Unable to create workqueue\n");
		ret = -ENOMEM;
		goto out_disable;
	}
	ctl_drv->dev = &pdev->dev;
	ctl_drv->cur_bpw = 8;
	ctl_drv->cur_speed = 0;
	init_completion(&ctl_drv->xfer_completion);
	init_completion(&ctl_drv->done);
	INIT_WORK(&ctl_drv->work, xgold_spi_work);
	/*
	 * Bus Number Which has been Assigned to this SSP controller
	 * on this board
	 */
	master->bus_num = pdev->id;
	master->num_chipselect = platdata->num_chipselect;
	master->cs_gpios = platdata->cs_gpios;
	master->dma_alignment = 8;

#ifndef CONFIG_OF
	/* Initialize GPIO used as chip select */
	for (i = 0; i < master->num_chipselect; i++) {
		int cs_gpio = master->cs_gpios[i];
		dev_err(&pdev->dev,
				"Looking on gpio %#x for cs %d\n", cs_gpio, i);

		if (cs_gpio == -ENOENT)
			continue;

		ret = gpio_request(cs_gpio, platdata->pinctrl_cs_name);
		if (ret) {
			dev_err(&pdev->dev,
				"Error while requesting gpio %#x\n", cs_gpio);
			goto out_workqueue;
		}

		gpio_direction_output(cs_gpio, 1);
		gpio_set_value(cs_gpio, 1);
	}

#endif

	if (ctl_drv->platdata->hw_type == XGOLD6XXSPI)
		master->mode_bits =
		    SPI_PARITY_EVEN | SPI_PARITY_ODD | SPI_CPHA | SPI_CPOL |
		    SPI_LSB_FIRST | SPI_CS_HIGH | SPI_NO_CS;
	else
		master->mode_bits =
		    SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST | SPI_CS_HIGH;

	master->flags = 0;
	master->cleanup = xgold_spi_cleanup;
	master->setup = xgold_spi_setup;
	master->transfer = xgold_spi_transfer;
	master->reset = xgold_spi_reset;
	xgold_spi_debug(XGOLD_SPI_DEBUG, "%s: %s: bus num: %d\n",
			DRIVER_NAME, __func__, master->bus_num);

	/* register spi irq  by one handle */
	for (i = 0; i < ctl_drv->irq_num; i++) {
		ret =
		    request_irq(ctl_drv->spi_irq[i], spi_irq_handler,
				IRQF_SHARED, "xgold_spi", ctl_drv);
		if (ret) {
			dev_err(&pdev->dev, "failure in requesting spi IRQ\n");
			goto out_workqueue;
		}
	}
	xgold_spi_hwinit(ctl_drv, &pdev->dev);

	if (!ctl_drv->pio_mode) {
		ret = xgold_spi_dma_init(ctl_drv);
		if (ret) {
			dev_err(&pdev->dev, "init dma failure\n");
			goto out_irqs;
		}
	}

	ret = spi_register_master(master);
	if (ret) {
		dev_err(&pdev->dev, "spi register master error\n");
		goto out_master;
	}

	return 0;

out_master:
	if (ctl_drv->dmach[XGOLD_SPI_DMA_TX])
		dma_release_channel(ctl_drv->dmach[XGOLD_SPI_DMA_TX]);
	if (ctl_drv->dmach[XGOLD_SPI_DMA_RX])
		dma_release_channel(ctl_drv->dmach[XGOLD_SPI_DMA_TX]);
out_irqs:
	while (i > 0)
		free_irq(ctl_drv->spi_irq[--i], ctl_drv);
out_workqueue:
	destroy_workqueue(ctl_drv->workqueue);
out_disable:
#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = platform_device_pm_set_state_by_name(pdev,
			platdata->pm_platdata->pm_state_D3_name);
#else
	xgold_spi_set_power_state(pdev, XGOLD_SPI_D3);
#endif
	if (!IS_ERR(ctl_drv->platdata->clock_spi))
		clk_put(ctl_drv->platdata->clock_spi);
	if (!IS_ERR(ctl_drv->platdata->clock_ahb))
		clk_put(ctl_drv->platdata->clock_ahb);
out_release:
	iounmap(ctl_drv->base);
#ifdef CONFIG_OF
	kfree(platdata);
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(master);
	return ret;
}

#ifdef CONFIG_PM
static int xgold_spi_suspend(struct device *dev)
{
	struct spi_master *master = spi_master_get(dev_get_drvdata(dev));
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(master);
	struct xgold_spi_platdata *pdata = dev_get_platdata(dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ctl_drv->lock, flags);
	ctl_drv->state |= SUSPND;
	spin_unlock_irqrestore(&ctl_drv->lock, flags);

	while (ctl_drv->state & SPIBUSY)
		msleep(20);

	xgold_spi_set_pinctrl_state(dev, pdata->pins_sleep);
#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = device_state_pm_set_state_by_name(dev,
				pdata->pm_platdata->pm_state_D3_name);
#else
	ret = xgold_spi_set_power_state(dev, XGOLD_SPI_D0i3);
#endif
	ctl_drv->cur_speed = 0;

	return ret;
}

static int xgold_spi_resume(struct device *dev)
{
	struct spi_master *master = spi_master_get(dev_get_drvdata(dev));
	struct xgold_spi_ctl_drv *ctl_drv = spi_master_get_devdata(master);
	struct xgold_spi_platdata *pdata = dev_get_platdata(dev);
	unsigned long flags;
	int ret;

	xgold_spi_set_pinctrl_state(dev,  pdata->pins_default);
#ifdef CONFIG_PLATFORM_DEVICE_PM
	ret = device_state_pm_set_state_by_name(dev,
				pdata->pm_platdata->pm_state_D0_name);
#else
	ret = xgold_spi_set_power_state(dev, XGOLD_SPI_D0);
#endif
	if (ret)
		return ret;

	xgold_spi_hwinit(ctl_drv, dev);

	spin_lock_irqsave(&ctl_drv->lock, flags);
	ctl_drv->state &= ~SUSPND;
	spin_unlock_irqrestore(&ctl_drv->lock, flags);

	return 0;
}
#else
#define xgold_spi_suspend	NULL
#define xgold_spi_resume	NULL
#endif /* CONFIG_PM */

static int __exit xgold_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct xgold_spi_platdata *pdata = dev_get_platdata(&pdev->dev);
	struct xgold_spi_ctl_drv *ctl_drv;
	unsigned long flags;

	if (!master)
		return 0;

	ctl_drv = spi_master_get_devdata(master);

	spin_lock_irqsave(&ctl_drv->lock, flags);
	ctl_drv->state |= SUSPND;
	spin_unlock_irqrestore(&ctl_drv->lock, flags);

	while (ctl_drv->state & SPIBUSY)
		msleep(20);

	spi_unregister_master(master);

	destroy_workqueue(ctl_drv->workqueue);

	xgold_spi_set_pinctrl_state(&pdev->dev, pdata->pins_inactive);
	iounmap((void *)ctl_drv->base);
	spi_master_put(master);
	kfree(master);
	platform_set_drvdata(pdev, NULL);

	return 0;
}
static const struct dev_pm_ops xgold_spi_dev_pm_ops = {
	.suspend = xgold_spi_suspend,
	.resume = xgold_spi_resume,
};

static struct platform_driver xgold_spi_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = xgold_spi_of_match,
		   .pm = &xgold_spi_dev_pm_ops,
		   },
	.probe = xgold_spi_probe,
	.remove = __exit_p(xgold_spi_remove),
};


static int __init xgold_spi_init(void)
{
	int ret;

#ifndef CONFIG_PLATFORM_DEVICE_PM_VIRT
	ret = device_state_pm_add_class(&xgold_spi_pm_class);
	if (ret) {
		xgold_spi_debug(XGOLD_SPI_ERROR,
				"%s: %s: ERROR adding %s pm class\n",
				DRIVER_NAME, __func__,
				xgold_spi_pm_class.name);
		return ret;
	}
#endif

	ret = platform_driver_register(&xgold_spi_driver);
	if (ret) {
		xgold_spi_debug(XGOLD_SPI_ERROR, "%s: %s: ERROR\n",
				DRIVER_NAME, __func__);
		return -ENODEV;
	}
	return ret;
}

static void __exit xgold_spi_exit(void)
{
	platform_driver_unregister(&xgold_spi_driver);
}

subsys_initcall(xgold_spi_init);
module_exit(xgold_spi_exit);

MODULE_AUTHOR("MPE POSV CV");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XGold spi driver");
MODULE_DEVICE_TABLE(of, xgold_spi_of_match);
