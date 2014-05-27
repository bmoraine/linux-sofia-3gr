/*
 * Copyright (C) 2012-2014 Intel Mobile Communications GmbH
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/serial_core.h>
#include <linux/tty_flip.h>
#include <linux/of.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_ids.h>

#include <linux/wakelock.h>

#define IMC_IDI_ERROR_ENTER pr_debug("--> %s\n", __func__);
#define IMC_IDI_ERROR_EXIT pr_debug("<-- %s\n", __func__);

#define ERROR_BUFFER_SIZE		16

struct imc_idi_error {
	struct idi_peripheral_device *p_device;
	struct idi_channel_config rx_ch_config;
	struct work_struct work;
	struct workqueue_struct *queue;
	spinlock_t lock;
};

static void error_rx_transaction_complete(struct idi_transaction *trans)
{
	struct imc_idi_error *p_error =
		dev_get_drvdata(&trans->peripheral->device);
	struct idi_xfer *xfer = &trans->idi_xfer;

	IMC_IDI_ERROR_ENTER;
	dev_err(&p_error->p_device->device, "Fail address 0x%X\n",
			*xfer->cpu_base);

	queue_work(p_error->queue, &p_error->work);

	IMC_IDI_ERROR_EXIT;
}

static void imc_idi_error_prepare_rx(struct work_struct *work)
{
	struct imc_idi_error *p_error =
		container_of(work, struct imc_idi_error, work);
	struct idi_transaction *trans;
	int ret;

	IMC_IDI_ERROR_ENTER;

	trans = idi_alloc_transaction(GFP_KERNEL);
	if (!trans) {
		ret = -ENOMEM;
		goto err;
	}

	trans->idi_xfer.size = 1;
	trans->idi_xfer.channel_opts = IDI_PRIMARY_CHANNEL;
	trans->complete = error_rx_transaction_complete;
	ret = idi_async_read(p_error->p_device, trans);

	if (ret)
		goto err;

	IMC_IDI_ERROR_EXIT;
	return;

err:
	pr_err("%s: return error %d\n", __func__, ret);
}

static int __init imc_idi_error_probe(struct idi_peripheral_device *p_device,
			      const struct idi_device_id *id)
{
	struct imc_idi_error *p_error;
	struct idi_channel_config *rx_conf;
	int ret;

	IMC_IDI_ERROR_ENTER;

	p_error = devm_kzalloc(&p_device->device, sizeof(*p_error), GFP_KERNEL);
	if (!p_error)
		return -ENOMEM;

	dev_set_drvdata(&p_device->device, p_error);
	p_error->p_device = p_device;

	rx_conf	= &p_error->rx_ch_config;
	rx_conf->tx_or_rx = 0;
	rx_conf->priority = IDI_HIGH_PRIORITY;
	rx_conf->channel_opts = IDI_PRIMARY_CHANNEL;
	rx_conf->size = ERROR_BUFFER_SIZE;
	rx_conf->cpu_base = dma_alloc_coherent(&p_device->device, rx_conf->size,
			&rx_conf->base, GFP_KERNEL | GFP_DMA);

	if (!rx_conf->cpu_base) {
		dev_err(&p_device->device, "Unable to allocate RX coherent buffer\n");
		return -ENOMEM;
	}

	rx_conf->hw_fifo_size = ERROR_BUFFER_SIZE;

	ret = idi_set_channel_config(p_device, rx_conf);
	if (ret) {
		dev_err(&p_device->device, "Unable to set IDI read channel configuration\n");
		dma_free_coherent(&p_device->device, rx_conf->size,
				  rx_conf->cpu_base, rx_conf->base);
	}

	p_error->queue = create_singlethread_workqueue("idi_error_work_queue");
	if (!p_error->queue)
		return -1;

	INIT_WORK(&p_error->work, imc_idi_error_prepare_rx);
	queue_work(p_error->queue, &p_error->work);

	dev_info(&p_device->device, "%s: probe return %d\n", __func__, ret);
	IMC_IDI_ERROR_EXIT;

	return ret;
}

static int imc_idi_error_remove(struct idi_peripheral_device *p_device)
{
	int ret = 0;

	IMC_IDI_ERROR_ENTER;

	return ret;
}

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_ERROR,
	},
	{ /* end: all zeroes */},
};

static struct idi_peripheral_driver imc_idi_error_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "idi-error",
	},
	.p_type = IDI_ERROR,
	.id_table = idi_ids,
	.probe = imc_idi_error_probe,
	.remove = imc_idi_error_remove,
};

MODULE_DEVICE_TABLE(idi, idi_ids);

static int __init imc_idi_error_init(void)
{
	int ret = 0;
	pr_debug("IMC IDI client error driver initialization\n");

	IMC_IDI_ERROR_ENTER;

	ret = idi_register_peripheral_driver(&imc_idi_error_driver);

	IMC_IDI_ERROR_EXIT;
	return ret;
}

static void __exit imc_idi_error_exit(void)
{
	IMC_IDI_ERROR_ENTER;

	idi_unregister_peripheral_driver(&imc_idi_error_driver);

	IMC_IDI_ERROR_EXIT;
}

module_init(imc_idi_error_init);
module_exit(imc_idi_error_exit);

MODULE_LICENSE("GPL");
