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

#ifndef _IDI_TEST_SUITE_H
#define _IDI_TEST_SUITE_H

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_bus.h>

extern const unsigned char fmr_rxmain_data[];

struct idi_test_info {
	u32 event_flag;
	struct work_struct	work;
	struct work_struct	work_1;
	struct idi_peripheral_device *idi_dev;
	struct workqueue_struct *idi_test_wq;
	struct workqueue_struct *idi_test_wq_1;
	spinlock_t idi_test_sw_lock;
	wait_queue_head_t wait_q;
};

struct idi_peripheral {
	struct idi_peripheral_device *peripheral_device;

	void __iomem *ctrl_io;

	spinlock_t sw_lock;

	wait_queue_head_t wait_msg;

	int com_flag;
	struct completion rx_complete;
	struct completion tx_complete;
	struct task_struct *kthread;
};

enum msg_pending_e {
	NO_REQ = 0,
	MSG_PENDING
};

enum sdma_operation {
	SDMA_READ,
	SDMA_WRITE,
	SDMA_WRITE_READ,
};

struct sdma_data {
	u32	wr_size;
	u32	rd_size;
	u32 agold_mem_phy_addr;
	u32 *system_mem_addr;
	enum sdma_operation ops;
};

int idi_programmed_access(struct idi_peripheral_device *dev, u32 *command,
					u32 *reg_read, u32 command_num);
int idi_request_queue_status_test(struct idi_peripheral_device *dev,
				struct sdma_data *s_data, u32 *read_data);
int idi_software_defined_transfer(struct idi_peripheral_device *dev,
				struct sdma_data *s_data, u32 *read_data);
int test_idi_outstanding_read_sg_tx(struct idi_peripheral_device *dev,
					bool break_frame);
int idi_software_defined_transfer_buffer(struct idi_peripheral_device *dev,
				struct sdma_data *s_data, u32 *read_data);
#endif
