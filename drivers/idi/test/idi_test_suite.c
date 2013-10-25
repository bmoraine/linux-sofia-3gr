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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/scatterlist.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/kthread.h>
#include <asm/cacheflush.h>
#include <asm/dma.h>

#include "idi_test_suite.h"
#include "fmr_rxmain_hex.h"

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>

#define HELP_TEXT_SIZE 1000
#define MAX_LEN 10
#define OUTSTANDING_TRANSACTION 4
#define IDI_TEST_TOTAL 17
#define IDI_TEST_INVALID -1
/* SRC buffer may have any size.256 words has been taken here */
#define TX_DBB_BUFFER_RD_SIZE 1024
/* DST buffer may have any size. 256 words has been taken here */
#define TX_DBB_BUFFER_WR_SIZE 1024
#define BIT_31  (1 << 31)
#define TX_NOT_RX 1
#define RX_NOT_TX 0

#define ADDR_ABB_BASE_PHY 0x0600000
#define ADDR_ABB_BASE_VIRT 0xe6600000
#define ADDR_DBB_BASE_VIRT 0xe6100000
#define ADDR_IMC_IDI_RIS(_base) ((_base) + 0x80)
#define ADDR_IMC_IDI_IMSC(_base) ((_base) + 0x84)
#define ADDR_IMC_IDI_MIS(_base) ((_base) + 0x88)
#define ADDR_IMC_IDI_ISR(_base) ((_base) + 0x90)
#define ADDR_IMC_IDI_ICR(_base) ((_base) + 0x98)
#define ADDR_IMC_IDI_ISR1(_base) ((_base) + 0xB0)
#define ADDR_IMC_IDI_IMSC1(_base) ((_base) + 0xA4)
#define ADDR_IMC_IDI_SLMASK_CON(_base)((_base) + 0x168)
#define ADDR_IMC_IDI_RIS1(_base)((_base) + 0xA0)

#define IDI_PER_VIRQ_BASE 480
#define IDI_PER_VIRQ(v_pad) (v_pad + IDI_PER_VIRQ_BASE)


#endif

#ifdef CONFIG_DEBUG_FS
static u32 agold_irqs[] = {27, 28, 29, 30, 31};
static char *agold_irqs_name[] = {"fmr_int", "fmrx_ev_oneshot",
				"fmrx_ev_tune", "fmtx_ev_calib",
				"fmtx_ev_tune"};

static u32 phy_idi_reg[] = {
		0x060000C, 0x0600014, 0x0600018, 0x060001C
	};
static u32 virt_idi_reg[] = {
		0xE660000C, 0xE6600014, 0xE6600018, 0xE660001C
	};

static u32 virt_idi_reg_val[] = {
		0x3048204, 0x0000000C, 0x00001FF0, 0x00000001
	};

atomic_t test_flag;

static struct idi_test_info idi_info;

struct dentry *dirret, *fileret, *help_file;
static const char help_text[] = "0.	test outstanding read\n"
"1.	test outstanding read/write\n"
"2.	test software defined sdma transfer\n"
"3.	test scatter gather read/write\n"
"4.	test transparent read\n"
"5.	test outstanding + software defined\n"
"6.	test outstanding + scatter gather read\n"
"7.	test scatter gather read + software defined\n"
"8.	test transparent + software defined\n"
"9.	test transparent + oustanding read\n"
"10.	test transparent + oustanding SG read\n"
"11.	test forwarded interrupts\n"
"12.	test FMR FW download\n"
"13.	test request queue status\n"
"14.	test FMR FW download with buffer flush\n"
"15.	test scatter gather read/writ with break frame\n"
"16.	test error interrupts\n";

static struct debugfs_blob_wrapper help_blob;

static irqreturn_t test_fwd_isr(int irq, void *data)
{
	pr_info("[%s] : Irq No = %d\n", __func__, irq);
	atomic_dec(&test_flag);
	wake_up(&idi_info.wait_q);
	return IRQ_HANDLED;
}

void trigger_fw_irqs(u32 abb_slave_irq)
{
    /* setting the HSI_SET register */
	int isr_mask = 0, ris, ris1, slmask;
	void __iomem *base_addr  = (unsigned int *)ADDR_DBB_BASE_VIRT;
	isr_mask = (abb_slave_irq < 17 ?
	(1<<(abb_slave_irq+15)) : (1<<(abb_slave_irq-17)));
	if (abb_slave_irq < 17)
		iowrite32(isr_mask, ADDR_IMC_IDI_ISR(base_addr));
	else
		iowrite32(isr_mask, ADDR_IMC_IDI_ISR1(base_addr));
	ris = ioread32(ADDR_IMC_IDI_RIS(base_addr));
	ris1 = ioread32(ADDR_IMC_IDI_RIS1(base_addr));
	slmask = ioread32(ADDR_IMC_IDI_SLMASK_CON(base_addr));
	pr_debug(":%s slmask = %0x08x ris = %0x08x ris1 = %0x08x\n",
			__func__, slmask, ris, ris1);
}

void test_error_irq(void)
{
	/* setting the HSI_SET register */
	u32 abb_slave_irq = 4;
	int isr_mask = 0, ris, slmask;
	void __iomem *base_addr  = (unsigned int *)ADDR_DBB_BASE_VIRT;
	isr_mask = BIT(abb_slave_irq);
	iowrite32(isr_mask, ADDR_IMC_IDI_ISR(base_addr));
	ris = ioread32(ADDR_IMC_IDI_RIS(base_addr));
	slmask = ioread32(ADDR_IMC_IDI_SLMASK_CON(base_addr));
	pr_debug(":%s slmask = %0x08x ris = %0x08x\n",
			__func__, slmask, ris);
}

void test_fwd_irq(u32 index)
{
	int ret = 0;
	pr_info("Testing %s interrupt\n", agold_irqs_name[index]);
	ret = request_irq(IDI_PER_VIRQ(agold_irqs[index]),
			test_fwd_isr, 0, agold_irqs_name[index], NULL);
	if (ret)
		pr_err("request_irq ret = %d\n", ret);
	else
		trigger_fw_irqs(agold_irqs[index]);
}


static void fmr_fw_download_buffer(struct work_struct *data)
{
	u32 ret = 0;
	u32 *read_data;
	u32 bits_all_zero, index;

	struct sdma_data *s_data = kzalloc(sizeof(struct sdma_data),
							GFP_KERNEL);
	s_data->wr_size = fmr_rxmain_size;
	s_data->rd_size = fmr_rxmain_size;
	s_data->agold_mem_phy_addr = 0x1000000;	/*Fmr RAM area*/
	s_data->system_mem_addr = kzalloc(fmr_rxmain_size, GFP_KERNEL);
	memcpy(s_data->system_mem_addr, fmr_rxmain_data, s_data->wr_size);
	s_data->ops = SDMA_WRITE_READ;
	read_data = kzalloc(s_data->rd_size, GFP_KERNEL);
	ret = idi_software_defined_transfer_buffer(idi_info.idi_dev, s_data,
								read_data);
	if (ret)
		atomic_inc(&test_flag);

	for (index = 0; index < TX_DBB_BUFFER_RD_SIZE / 4; index++) {
		bits_all_zero = (read_data[index] ^
					s_data->system_mem_addr[index]);
		if (bits_all_zero) {
			pr_err("%s value %d index = %d\t bits_all_zero = %d\n",
			__func__, read_data[index], index, bits_all_zero);
			atomic_inc(&test_flag);
			break;
		}
	}

	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	kfree(read_data);
	kfree(s_data->system_mem_addr);
	kfree(s_data);
	wake_up(&idi_info.wait_q);
}

static void fmr_fw_download(struct work_struct *data)
{
	u32 ret = 0;
	u32 *read_data;
	u32 bits_all_zero, index;

	struct sdma_data *s_data = kzalloc(sizeof(struct sdma_data),
							GFP_KERNEL);
	s_data->wr_size = fmr_rxmain_size;
	s_data->rd_size = fmr_rxmain_size;
	s_data->agold_mem_phy_addr = 0x1000000;	/*Fmr RAM area*/
	s_data->system_mem_addr = kzalloc(fmr_rxmain_size, GFP_KERNEL);
	memcpy(s_data->system_mem_addr, fmr_rxmain_data, s_data->wr_size);
	s_data->ops = SDMA_WRITE_READ;
	read_data = kzalloc(s_data->rd_size, GFP_KERNEL);
	ret = idi_software_defined_transfer(idi_info.idi_dev, s_data,
								read_data);
	if (ret)
		atomic_inc(&test_flag);

	for (index = 0; index < TX_DBB_BUFFER_RD_SIZE / 4; index++) {
		bits_all_zero = (read_data[index] ^
					s_data->system_mem_addr[index]);
		if (bits_all_zero) {
			pr_err("%s value %d index = %d\t bits_all_zero = %d\n",
			__func__, read_data[index], index, bits_all_zero);
			atomic_inc(&test_flag);
			break;
		}
	}

	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	kfree(read_data);
	kfree(s_data->system_mem_addr);
	kfree(s_data);
	wake_up(&idi_info.wait_q);
}

static void idi_sdma_transfer(struct work_struct *data)
{
	u32 ret = 0;
	u32 *read_data;
	u32 bits_all_zero, index;

	struct sdma_data *s_data = kzalloc(sizeof(struct sdma_data),
							GFP_KERNEL);
	s_data->wr_size = TX_DBB_BUFFER_WR_SIZE;
	s_data->rd_size = TX_DBB_BUFFER_RD_SIZE;
	s_data->agold_mem_phy_addr = 0x1000200;	/*Fmr RAM area*/
	s_data->system_mem_addr = kzalloc(s_data->wr_size, GFP_KERNEL);
	memset(s_data->system_mem_addr, 0x1, s_data->wr_size);
	s_data->ops = SDMA_WRITE_READ;
	read_data = kzalloc(s_data->rd_size, GFP_KERNEL);
	ret = idi_software_defined_transfer(idi_info.idi_dev, s_data,
								read_data);
	if (ret)
		atomic_inc(&test_flag);

	for (index = 0; index < TX_DBB_BUFFER_RD_SIZE / 4; index++) {
		bits_all_zero = (read_data[index] ^
					s_data->system_mem_addr[index]);
		if (bits_all_zero) {
			pr_err("%s value %d index = %d\t bits_all_zero = %d\n",
			__func__, read_data[index], index, bits_all_zero);
			atomic_inc(&test_flag);
			break;
		}
	}

	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	kfree(read_data);
	kfree(s_data);
	wake_up(&idi_info.wait_q);
}

static void idi_sdma_transfer_with_queue_status(struct work_struct *data)
{
	u32 ret = 0;
	u32 *read_data;
	u32 bits_all_zero, index;

	struct sdma_data *s_data = kzalloc(sizeof(struct sdma_data),
							GFP_KERNEL);
	s_data->wr_size = TX_DBB_BUFFER_WR_SIZE;
	s_data->rd_size = TX_DBB_BUFFER_RD_SIZE;
	s_data->agold_mem_phy_addr = 0x1000200;	/*Fmr RAM area*/
	s_data->system_mem_addr = kzalloc(s_data->wr_size, GFP_KERNEL);
	memset(s_data->system_mem_addr, 0x1, s_data->wr_size);
	s_data->ops = SDMA_WRITE_READ;
	read_data = kzalloc(s_data->rd_size, GFP_KERNEL);
	ret = idi_request_queue_status_test(idi_info.idi_dev, s_data,
								read_data);
	if (ret)
		atomic_inc(&test_flag);

	for (index = 0; index < TX_DBB_BUFFER_RD_SIZE / 4; index++) {
		bits_all_zero = (read_data[index] ^
					s_data->system_mem_addr[index]);
		if (bits_all_zero) {
			pr_err("%s value %d index = %d\t bits_all_zero = %d\n",
			__func__, read_data[index], index, bits_all_zero);
			atomic_inc(&test_flag);
			break;
		}
	}

	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	kfree(read_data);
	kfree(s_data);
	wake_up(&idi_info.wait_q);
}

static void idi_abb_sg_read(struct work_struct *data)
{
	int ret  = 0;
	ret = test_idi_outstanding_read_sg_tx(idi_info.idi_dev, 0);
	if (ret)
		atomic_inc(&test_flag);
	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	wake_up(&idi_info.wait_q);
}

static void idi_abb_oustanding_read(struct work_struct *data)
{

	u32 command_num, index, ret = 0;
	u32 *command, *read_data;
	command_num = sizeof(phy_idi_reg) / sizeof(u32);
	pr_info("command_num = %d\n", command_num);
	command = kzalloc((sizeof(u32)*command_num),
						GFP_KERNEL);
	pr_info("command = %d\n", (unsigned int)command);
	for (index = 0; index < command_num; index++) {
		command[index] = READ_WORD(phy_idi_reg[index]);
		pr_info("Command %i : %x\n", index, command[index]);
	}
	read_data = kzalloc(sizeof(u32)*command_num,
						GFP_KERNEL);
	pr_info("read_data = %p\n", read_data);
	ret = idi_programmed_access(idi_info.idi_dev, command,
				read_data, command_num);
	if (ret)
		atomic_inc(&test_flag);

/* Reading the register values using Transparent read */
	for (index = 0; index < command_num; index++) {
		if (read_data[index] !=
			ioread32((void *)virt_idi_reg[index])) {
				atomic_inc(&test_flag);
				break;
		}
	}

	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	kfree(command);
	kfree(read_data);
	wake_up(&idi_info.wait_q);

}


static void idi_abb_oustanding_read_write(struct work_struct *data)
{

	u32 read_command_num, write_command_num, imsc_data, index = 0, ret = 0;
	u32 *command, *read_data;
	read_command_num = 1;
	write_command_num = 3;
	imsc_data = ioread32((void *)ADDR_IMC_IDI_IMSC(ADDR_ABB_BASE_VIRT));
	iowrite32(0, (void *)ADDR_IMC_IDI_IMSC(ADDR_ABB_BASE_VIRT));
	pr_info("read_command_num = %d, write_command_num = %d\n",
				read_command_num, write_command_num);
	command = kzalloc(sizeof(u32)*(write_command_num*2 + read_command_num),
								 GFP_KERNEL);
	pr_info("command = %d\n", (unsigned int)command);
	while (index < write_command_num*2) {
		command[index] =
			WRITE_WORD(ADDR_IMC_IDI_ISR(ADDR_ABB_BASE_PHY));
		index++;
		command[index] = (1 << (index)/2);
		index++;
	}
	command[index] = READ_WORD(ADDR_IMC_IDI_RIS(ADDR_ABB_BASE_PHY));
	read_data = kzalloc(sizeof(u32), GFP_KERNEL);
	ret = idi_programmed_access(idi_info.idi_dev, command,
			read_data, write_command_num + read_command_num);
	if (ret)
		atomic_inc(&test_flag);
	iowrite32(imsc_data, (void *)ADDR_IMC_IDI_IMSC(ADDR_ABB_BASE_VIRT));
/* Reading the register values using Transparent read */
	pr_info("read_data = %d\n", read_data[0]);

	spin_lock(&idi_info.idi_test_sw_lock);
	idi_info.event_flag = NO_REQ;
	spin_unlock(&idi_info.idi_test_sw_lock);
	kfree(command);
	kfree(read_data);
	wake_up(&idi_info.wait_q);
}

static void test_transparent_read(void)
{
	int index;
/* Reading the register values using Transparent read*/
	for (index = 0; index < sizeof(virt_idi_reg)/sizeof(u32); index++) {
		if (virt_idi_reg_val[index] !=
			ioread32((void *)virt_idi_reg[index])) {
			atomic_inc(&test_flag);
			break;
		}
	}

}
static ssize_t run_read_handler(struct file *fp, char __user *user_buffer,
						size_t count, loff_t *position)
{
	return 0;
}

static ssize_t run_write_handler(struct file *fp,
					const char __user *user_buffer,
						size_t count, loff_t *position)
{
	int choice, index = 0;

	char *kernel_buf;
	if (count > MAX_LEN)
		return -EINVAL;

	kernel_buf = kmalloc(count, GFP_KERNEL);
	count = simple_write_to_buffer(kernel_buf, count, position,
						user_buffer, count);
	choice = simple_strtoul(kernel_buf, NULL, 0);

	idi_set_power_state(idi_info.idi_dev, D0, true);
	if (choice > IDI_TEST_INVALID && choice < IDI_TEST_TOTAL) {
		switch (choice) {
		case 0:
			pr_info(" Running outstanding read\n");
			INIT_WORK(&idi_info.work, idi_abb_oustanding_read);
			queue_work(idi_info.idi_test_wq, &idi_info.work);
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 2:
			pr_info(" Running idi_sdma_transfer_dma\n");
			INIT_WORK(&idi_info.work, idi_sdma_transfer);
			queue_work(idi_info.idi_test_wq, &idi_info.work);
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 4:
			pr_info(" Running test transparent read\n");
			test_transparent_read();
			break;
		case 5:
			pr_info(" Running outstanding + software defined\n");
			INIT_WORK(&idi_info.work, idi_sdma_transfer);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			INIT_WORK(&idi_info.work_1, idi_abb_oustanding_read);
			queue_work(idi_info.idi_test_wq_1, &idi_info.work_1);
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			idi_info.event_flag = MSG_PENDING;
			spin_unlock(&idi_info.idi_test_sw_lock);

			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 1:
			pr_info(" Running outstanding read write\n");
			INIT_WORK(&idi_info.work,
					idi_abb_oustanding_read_write);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 3:
			pr_info(" Running scatter gather transfer\n");
			if (test_idi_outstanding_read_sg_tx(idi_info.idi_dev,
								0))
				atomic_inc(&test_flag);
			break;
		case 6:
			pr_info("Running scatter gather read + outstanding\n");
			INIT_WORK(&idi_info.work, idi_abb_sg_read);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			INIT_WORK(&idi_info.work_1, idi_abb_oustanding_read);
			queue_work(idi_info.idi_test_wq_1, &idi_info.work_1);

			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			idi_info.event_flag = MSG_PENDING;
			spin_unlock(&idi_info.idi_test_sw_lock);

			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 7:
			pr_info("Running SG read + software defined\n");
			INIT_WORK(&idi_info.work, idi_abb_sg_read);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			INIT_WORK(&idi_info.work_1, idi_sdma_transfer);
			queue_work(idi_info.idi_test_wq_1, &idi_info.work_1);

			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			idi_info.event_flag = MSG_PENDING;
			spin_unlock(&idi_info.idi_test_sw_lock);

			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 8:
			pr_info(" Running transparent + software defined\n");
			INIT_WORK(&idi_info.work, idi_sdma_transfer);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			test_transparent_read();
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 9:
			pr_info("Running transparent + oustanding read\n");
			INIT_WORK(&idi_info.work, idi_abb_oustanding_read);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			test_transparent_read();
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 10:
			pr_info("Running transparent + scatter gather read\n");
			INIT_WORK(&idi_info.work, idi_abb_sg_read);
			queue_work(idi_info.idi_test_wq, &idi_info.work);

			test_transparent_read();
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 11:
			pr_info("Testing Peripheral interrupts\n");
			atomic_set(&test_flag, 5);
			for (index = 0; index < 5; index++)
				test_fwd_irq(index);

			wait_event_interruptible_timeout(idi_info.wait_q,
						!atomic_read(&test_flag),
						msecs_to_jiffies(50000));

			for (index = 0; index < 5; index++)
				free_irq(IDI_PER_VIRQ(agold_irqs[index]),
									 NULL);
			break;
		case 12:

			pr_info("Running FMR FW download\n");
			INIT_WORK(&idi_info.work, fmr_fw_download);
			queue_work(idi_info.idi_test_wq, &idi_info.work);
			wait_event_interruptible_timeout(idi_info.wait_q,
				idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 13:
			pr_info(" Running idi_sdma_transfer_dma "
					"and requesting queue status\n");
			INIT_WORK(&idi_info.work,
					idi_sdma_transfer_with_queue_status);
			queue_work(idi_info.idi_test_wq, &idi_info.work);
			wait_event_interruptible_timeout(idi_info.wait_q,
					idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 14:
			pr_info("Running FMR FW download with buffer flush\n");
			INIT_WORK(&idi_info.work, fmr_fw_download_buffer);
			queue_work(idi_info.idi_test_wq, &idi_info.work);
			wait_event_interruptible_timeout(idi_info.wait_q,
				idi_info.event_flag != MSG_PENDING,
						msecs_to_jiffies(5000));
			spin_lock(&idi_info.idi_test_sw_lock);
			if (idi_info.event_flag == MSG_PENDING)
				atomic_inc(&test_flag);
			spin_unlock(&idi_info.idi_test_sw_lock);
			break;
		case 15:
			pr_info(" Running scatter gather transfer with break frame\n");
			if (test_idi_outstanding_read_sg_tx
					(idi_info.idi_dev, 1))
				atomic_inc(&test_flag);
			break;
		case 16:
			pr_info("Testing error interrupts\n");
			test_error_irq();
			break;
		default:
			break;
		}
	} else {
		pr_err("wrong choice entered %d\n", choice);
		return count;
	}

	idi_set_power_state(idi_info.idi_dev, D3, false);
	if (!atomic_read(&test_flag))
		pr_info(" TEST CASE PASSED\n");
	else
		pr_info(" TEST CASE FAILED\n");
	idi_info.event_flag = MSG_PENDING;
	atomic_set(&test_flag, 0);
	kfree(kernel_buf);
	return count;
}
static const struct file_operations fops_run = {
	.read = run_read_handler,
	.write = run_write_handler,
};

static int __init idi_add_tests_debugfs(void)
{
	dirret = debugfs_create_dir("idi-test", NULL);

	help_blob.data = (void *)help_text;
	help_blob.size = sizeof(help_text);

	help_file = debugfs_create_blob("help", S_IRUGO, dirret, &help_blob);

	fileret = debugfs_create_file("run", S_IWUSR | S_IWGRP, dirret,
							NULL, &fops_run);
	return 0;
}

static void __exit idi_remove_tests_debugfs(void)
{
	debugfs_remove_recursive(dirret);
}

#endif

static void write_complete(struct idi_transaction *trans)
{

	struct idi_peripheral *peripheral;
	peripheral = dev_get_drvdata(&trans->peripheral->device);
	peripheral->com_flag = NO_REQ;
	pr_info("IDI test %s\n", __func__);
	wake_up(&peripheral->wait_msg);
}

static void read_complete(struct idi_transaction *trans)
{

	struct idi_peripheral *peripheral;
	peripheral = dev_get_drvdata(&trans->peripheral->device);
	peripheral->com_flag = NO_REQ;
	pr_info("IDI test %s:\n", __func__);
	wake_up(&peripheral->wait_msg);
}

static void imc_idi_rx_complete(struct idi_transaction *trans)
{
	struct idi_peripheral *peripheral =
				dev_get_drvdata(&trans->peripheral->device);
	struct device *dev = &trans->peripheral->device;
	pr_info("IDI test: %s:\n", __func__);

	dev_info(dev, "IDI test: FMR RX transaction completion called\n");
	complete(&peripheral->rx_complete);
}

static void imc_idi_tx_complete(struct idi_transaction *trans)
{

	struct idi_peripheral *peripheral =
				dev_get_drvdata(&trans->peripheral->device);
	struct device *dev = &trans->peripheral->device;
	pr_info("IDI test: %s:\n", __func__);

	dev_info(dev, "IDI test: FMR TX transaction completion called\n");
	complete(&peripheral->tx_complete);
}

int idi_software_defined_transfer_buffer(struct idi_peripheral_device *dev,
				struct sdma_data *s_data, u32 *read_data)
{
	int err = 0, i, buffer_space, saved_channel_map = 0;
	struct idi_transaction *trans_tx = NULL, *trans_rx = NULL;
	u32 *readbuff = NULL, *writebuff = NULL;
	dma_addr_t read_dma_handle, write_dma_handle;

	struct idi_peripheral *peripheral;
	peripheral = dev_get_drvdata(&dev->device);

	if (s_data->ops == SDMA_READ || s_data->ops == SDMA_WRITE_READ) {
		trans_rx = idi_alloc_transaction(GFP_KERNEL);
		readbuff = dma_alloc_coherent(NULL, s_data->rd_size,
				      &read_dma_handle, GFP_KERNEL);
		if (readbuff == NULL) {
			pr_err("IDI test: couldn't allocate dma buffer\n");
			err = -ENOMEM;
			goto read_alloc_failed;
		}
		memset(readbuff, 0, s_data->rd_size);
		trans_rx->peripheral = dev;
		trans_rx->idi_xfer.cpu_base = readbuff;
		trans_rx->idi_xfer.base = read_dma_handle;
		trans_rx->idi_xfer.dst_addr = s_data->agold_mem_phy_addr;
		trans_rx->idi_xfer.size = s_data->rd_size;
		trans_rx->complete = read_complete;
	}

	if (s_data->ops == SDMA_WRITE || s_data->ops == SDMA_WRITE_READ) {
		trans_tx = idi_alloc_transaction(GFP_KERNEL);
		writebuff = dma_alloc_coherent(NULL, s_data->wr_size,
				       &write_dma_handle, GFP_KERNEL);
		if (writebuff == NULL) {
			pr_err("IDI test: couldn't allocate dma buffer\n");
			err = -ENOMEM;
			goto write_alloc_failed;
		}

		memcpy(writebuff, s_data->system_mem_addr, s_data->wr_size);
		trans_tx->peripheral = dev;
		trans_tx->idi_xfer.cpu_base = writebuff;
		trans_tx->idi_xfer.base = write_dma_handle;
		trans_tx->idi_xfer.dst_addr = s_data->agold_mem_phy_addr;
		trans_tx->idi_xfer.size = s_data->wr_size;
		trans_tx->complete = write_complete;
	}


	err = idi_open_software_channel(dev);
	if (err)
		goto software_channel_err;

	pr_info("IDI test: %s: channel_map = %d and sw_channel =%d\n",
		__func__, dev->channel_map, dev->sw_channel);

	saved_channel_map = dev->channel_map;
	dev->channel_map = 15;

	if (s_data->ops == SDMA_WRITE || s_data->ops == SDMA_WRITE_READ) {
		for (i = 0; i < 10; i++) {
			err = idi_software_write(dev, trans_tx);
			if (err)
				goto software_rw_err;
			buffer_space = idi_request_buffer_info(dev, TX_NOT_RX,
							IDI_PRIMARY_CHANNEL);
			if (buffer_space >= 0)
				pr_info("IDI test: Before peripheral flush"
					" TX Buffer space available = %d\n",
								buffer_space);
			else
				pr_info("IDI test: error in "
						"idi_request_buffer_info\n");
			pr_info("IDI test: Called idi_peripheral_flush\n");
			err = idi_peripheral_flush(dev, IDI_PRIMARY_CHANNEL);
			if (err)
				pr_info("IDI test: error in "
						"idi_peripheral_flush\n");

			buffer_space = idi_request_buffer_info(dev, TX_NOT_RX,
							IDI_PRIMARY_CHANNEL);
			if (buffer_space >= 0)
				pr_info("IDI test: After peripheral flush"
					" TX Buffer space available = %d\n",
								buffer_space);
			else
				pr_info("IDI test: error in "
						"idi_request_buffer_info\n");

		}
		wait_event(peripheral->wait_msg,
					peripheral->com_flag != MSG_PENDING);
	}

	if (s_data->ops == SDMA_WRITE_READ)
		peripheral->com_flag = MSG_PENDING;


	if (s_data->ops == SDMA_READ || s_data->ops == SDMA_WRITE_READ) {
		for (i = 0; i < 10; i++) {
			err = idi_software_read(dev, trans_rx);
			if (err)
				goto software_rw_err;
			buffer_space = idi_request_buffer_info(dev, RX_NOT_TX,
							IDI_PRIMARY_CHANNEL);
			if (buffer_space >= 0)
				pr_info("IDI test: RX Buffer space"
						" available = %d\n",
								buffer_space);
			else
				pr_info("IDI test: error in"
						" idi_request_buffer_info\n");
		}
		err = idi_request_buffer_flush(dev, IDI_PRIMARY_CHANNEL);
		if (err)
			pr_info("IDI test: error in idi_peripheral_flush\n");

		wait_event(peripheral->wait_msg,
					peripheral->com_flag != MSG_PENDING);
		if (read_data)
			memcpy(read_data, trans_rx->idi_xfer.cpu_base,
					trans_rx->idi_xfer.size);
	}

software_rw_err:
	dev->channel_map = saved_channel_map;
	idi_close_software_channel(dev);
software_channel_err:
write_alloc_failed:
	idi_free_transaction(trans_tx);
	if (writebuff != NULL)
		dma_free_coherent(NULL, s_data->wr_size, writebuff,
				  write_dma_handle);
read_alloc_failed:
	idi_free_transaction(trans_rx);
	if (readbuff != NULL)
		dma_free_coherent(NULL, s_data->rd_size, readbuff,
				  read_dma_handle);
	return err;
}

int idi_software_defined_transfer(struct idi_peripheral_device *dev ,
				struct sdma_data *s_data, u32 *read_data)
{
	int err = 0;
	struct idi_transaction *trans_tx = NULL, *trans_rx = NULL;
	u32 *readbuff = NULL, *writebuff = NULL;
	dma_addr_t read_dma_handle, write_dma_handle;

	struct idi_peripheral *peripheral;
	peripheral = dev_get_drvdata(&dev->device);

	if (s_data->ops == SDMA_READ || s_data->ops == SDMA_WRITE_READ) {
		trans_rx = idi_alloc_transaction(GFP_KERNEL);
		readbuff = dma_alloc_coherent(NULL, s_data->rd_size,
				      &read_dma_handle, GFP_KERNEL);
		if (readbuff == NULL) {
			pr_err("IDI test: couldn't allocate dma buffer\n");
			err = -ENOMEM;
			goto read_alloc_failed;
		}
		memset(readbuff, 0, s_data->rd_size);
		trans_rx->peripheral = dev;
		trans_rx->idi_xfer.cpu_base = readbuff;
		trans_rx->idi_xfer.base = read_dma_handle;
		trans_rx->idi_xfer.dst_addr = s_data->agold_mem_phy_addr;
		trans_rx->idi_xfer.size = s_data->rd_size;
		trans_rx->complete = read_complete;
	}

	if (s_data->ops == SDMA_WRITE || s_data->ops == SDMA_WRITE_READ) {
		trans_tx = idi_alloc_transaction(GFP_KERNEL);
		writebuff = dma_alloc_coherent(NULL, s_data->wr_size,
				       &write_dma_handle, GFP_KERNEL);
		if (writebuff == NULL) {
			pr_err("IDI test: couldn't allocate dma buffer\n");
			err = -ENOMEM;
			goto write_alloc_failed;
		}

		memcpy(writebuff, s_data->system_mem_addr, s_data->wr_size);
		trans_tx->peripheral = dev;
		trans_tx->idi_xfer.cpu_base = writebuff;
		trans_tx->idi_xfer.base = write_dma_handle;
		trans_tx->idi_xfer.dst_addr = s_data->agold_mem_phy_addr;
		trans_tx->idi_xfer.size = s_data->wr_size;
		trans_tx->complete = write_complete;
	}


	err = idi_open_software_channel(dev);
	if (err)
		goto software_channel_err;

	pr_info("IDI test: %s: channel_map = %d and sw_channel =%d\n",
		__func__, dev->channel_map, dev->sw_channel);

	if (s_data->ops == SDMA_WRITE || s_data->ops == SDMA_WRITE_READ) {
		err = idi_software_write(dev, trans_tx);
		if (err)
			goto software_rw_err;
		wait_event(peripheral->wait_msg,
					peripheral->com_flag != MSG_PENDING);
	}

	if (s_data->ops == SDMA_WRITE_READ)
		peripheral->com_flag = MSG_PENDING;

	if (s_data->ops == SDMA_READ || s_data->ops == SDMA_WRITE_READ) {
		err = idi_software_read(dev, trans_rx);
		if (err)
			goto software_rw_err;
		wait_event(peripheral->wait_msg,
					peripheral->com_flag != MSG_PENDING);
		if (read_data)
			memcpy(read_data, trans_rx->idi_xfer.cpu_base,
					trans_rx->idi_xfer.size);
	}

software_rw_err:
	idi_close_software_channel(dev);
software_channel_err:
write_alloc_failed:
	idi_free_transaction(trans_tx);
	if (writebuff != NULL)
		dma_free_coherent(NULL, s_data->wr_size, writebuff,
				  write_dma_handle);
read_alloc_failed:
	idi_free_transaction(trans_rx);
	if (readbuff != NULL)
		dma_free_coherent(NULL, s_data->rd_size, readbuff,
				  read_dma_handle);
	return err;
}

int idi_request_queue_status_test(struct idi_peripheral_device *dev ,
				struct sdma_data *s_data, u32 *read_data)
{
	int err = 0, i, saved_channel_map = 0;
	struct idi_transaction *trans_tx = NULL, *trans_rx = NULL;
	u32 *readbuff = NULL, *writebuff = NULL;
	dma_addr_t read_dma_handle, write_dma_handle;

	struct idi_peripheral *peripheral;
	peripheral = dev_get_drvdata(&dev->device);

	if (s_data->ops == SDMA_READ || s_data->ops == SDMA_WRITE_READ) {
		trans_rx = idi_alloc_transaction(GFP_KERNEL);
		readbuff = dma_alloc_coherent(NULL, s_data->rd_size,
				      &read_dma_handle, GFP_KERNEL);
		if (readbuff == NULL) {
			pr_err("IDI test: couldn't allocate dma buffer\n");
			err = -ENOMEM;
			goto read_alloc_failed;
		}
		memset(readbuff, 0, s_data->rd_size);
		trans_rx->peripheral = dev;
		trans_rx->idi_xfer.cpu_base = readbuff;
		trans_rx->idi_xfer.base = read_dma_handle;
		trans_rx->idi_xfer.dst_addr = s_data->agold_mem_phy_addr;
		trans_rx->idi_xfer.size = s_data->rd_size;
		trans_rx->complete = read_complete;
	}

	if (s_data->ops == SDMA_WRITE || s_data->ops == SDMA_WRITE_READ) {
		trans_tx = idi_alloc_transaction(GFP_KERNEL);
		writebuff = dma_alloc_coherent(NULL, s_data->wr_size,
				       &write_dma_handle, GFP_KERNEL);
		if (writebuff == NULL) {
			pr_err("IDI test: couldn't allocate dma buffer\n");
			err = -ENOMEM;
			goto write_alloc_failed;
		}

		memcpy(writebuff, s_data->system_mem_addr, s_data->wr_size);
		trans_tx->peripheral = dev;
		trans_tx->idi_xfer.cpu_base = writebuff;
		trans_tx->idi_xfer.base = write_dma_handle;
		trans_tx->idi_xfer.dst_addr = s_data->agold_mem_phy_addr;
		trans_tx->idi_xfer.size = s_data->wr_size;
		trans_tx->complete = write_complete;
	}


	err = idi_open_software_channel(dev);
	if (err)
		goto software_channel_err;

	pr_info("IDI test: %s: channel_map = %d and sw_channel =%d\n",
		__func__, dev->channel_map, dev->sw_channel);

	if (s_data->ops == SDMA_WRITE || s_data->ops == SDMA_WRITE_READ) {
		err = idi_software_write(dev, trans_tx);
		if (err)
			goto software_rw_err;
		wait_event(peripheral->wait_msg,
					peripheral->com_flag != MSG_PENDING);
	}

	if (s_data->ops == SDMA_WRITE_READ)
		peripheral->com_flag = MSG_PENDING;

	saved_channel_map = dev->channel_map;
	dev->channel_map = 15;

	if (s_data->ops == SDMA_READ || s_data->ops == SDMA_WRITE_READ) {
		for (i = 0; i < 10; i++) {
			err = idi_software_read(dev, trans_rx);
			if (err)
				goto software_rw_err;
			pr_info("Got queue status for read, count = %d\n",
				idi_request_queue_status(dev, IDI_TRANS_READ,
							IDI_PRIMARY_CHANNEL));
		}
		wait_event(peripheral->wait_msg,
					peripheral->com_flag != MSG_PENDING);
		if (read_data)
			memcpy(read_data, trans_rx->idi_xfer.cpu_base,
					trans_rx->idi_xfer.size);
	}

software_rw_err:
	dev->channel_map = saved_channel_map;
	idi_close_software_channel(dev);
software_channel_err:
write_alloc_failed:
	idi_free_transaction(trans_tx);
	if (writebuff != NULL)
		dma_free_coherent(NULL, s_data->wr_size, writebuff,
				  write_dma_handle);
read_alloc_failed:
	idi_free_transaction(trans_rx);
	if (readbuff != NULL)
		dma_free_coherent(NULL, s_data->rd_size, readbuff,
				  read_dma_handle);
	return err;
}

int  idi_programmed_access(struct idi_peripheral_device *dev, u32 *command,
						u32 *reg_read, u32 command_num)
{
	struct idi_peripheral *peripheral;
	int ret = 0;
	u32 write = 0, read = 0;
	u32 *command_ptr = command;
	struct idi_transaction *regs, *data;
	data = NULL;
	peripheral = dev_get_drvdata(&dev->device);
	pr_info(" peripheral = %p\n", peripheral);
	pr_info(" command_ptr = %p\n", command_ptr);
	while (command_num) {
		if (*command_ptr++ & BIT_31) {
			write++;
			command_ptr++;
		} else
			read++;
		command_num--;
	}
	pr_info(" read = %d write =%d\n", read, write);
	if (read) {
		data = idi_alloc_transaction(GFP_KERNEL);
		data->complete = imc_idi_rx_complete;
		init_completion(&peripheral->rx_complete);
		data->idi_xfer.size = read * sizeof(unsigned);
		pr_info(" data->idi_xfer.size =%d\n", data->idi_xfer.size);
		data->idi_xfer.cpu_base = dma_alloc_coherent(NULL,
			data->idi_xfer.size, &data->idi_xfer.base, GFP_KERNEL);
		memset(data->idi_xfer.cpu_base, 0,	data->idi_xfer.size);

		pr_info("IDI test:%s:RX Phy addr = 0x%08x virt addr= 0x%08x\n",
		 __func__, (unsigned int)data->idi_xfer.base,
				(unsigned int)data->idi_xfer.cpu_base);
	}

	regs = idi_alloc_transaction(GFP_KERNEL);
	regs->complete = imc_idi_tx_complete;
	init_completion(&peripheral->tx_complete);
	regs->idi_xfer.size = (read + write*2)*sizeof(unsigned);
	regs->idi_xfer.cpu_base = dma_alloc_coherent(NULL,
				regs->idi_xfer.size*2, &regs->idi_xfer.base,
								GFP_KERNEL);
	memset(regs->idi_xfer.cpu_base , 0, regs->idi_xfer.size*2);

	pr_info("%s:TX Phy add = 0x%08x virt addr = 0x%08x\n", __func__,
			(unsigned int)regs->idi_xfer.base,
				(unsigned int)regs->idi_xfer.cpu_base);

	memcpy(regs->idi_xfer.cpu_base, command, regs->idi_xfer.size);
	pr_info("regs->idi_xfer.size=%d\n", regs->idi_xfer.size);
	ret = idi_outstanding_read(dev, regs, data);

	if (ret)
		goto test_failed;

	wait_for_completion(&peripheral->tx_complete);
	if (read)
		wait_for_completion(&peripheral->rx_complete);

	if (reg_read)
		memcpy(reg_read, data->idi_xfer.cpu_base, data->idi_xfer.size);

test_failed:
	dma_free_coherent(NULL, regs->idi_xfer.size, regs->idi_xfer.cpu_base,
							regs->idi_xfer.base);
	dma_free_coherent(NULL, data->idi_xfer.size, data->idi_xfer.cpu_base,
							data->idi_xfer.base);

	idi_free_transaction(regs);
	idi_free_transaction(data);

	return ret;
}

int test_idi_outstanding_read_sg_tx(struct idi_peripheral_device *dev,
						bool break_frame)
{
	int err = 0;
	int index;
	unsigned int sg_trans_size = 16;
	unsigned int sg_recv_size = 32;
	unsigned int sg_size = 12;
	struct idi_transaction *trans_rx, *trans_tx;
	unsigned int *readbuff, *writebuff_1, *writebuff_2, *sg_buff_1,
								*sg_buff_2;
	dma_addr_t read_dma_handle, write_dma_handle_1, write_dma_handle_2,
					sg_dma_handle_1, sg_dma_handle_2;
	struct dma_pool *read_pool, *write_pool_1, *write_pool_2, *sg_pool_1,
								*sg_pool_2;
	struct idi_peripheral *peripheral;
	struct idi_sg_desc *sg1, *sg2;

	peripheral = dev_get_drvdata(&dev->device);
	trans_rx = idi_alloc_transaction(GFP_KERNEL);
	read_pool = dma_pool_create("idi_read_pool", NULL,
					sg_recv_size, 16, 0);
	if (!read_pool) {
		pr_err("%s(): cannot allocate idi_read_pool", __func__);
		err = -ENOMEM;
		return err;
	}
	readbuff = dma_pool_alloc(read_pool, GFP_KERNEL, &read_dma_handle);
	if (!readbuff) {
		pr_err("%s(): cannot allocate readbuff", __func__);
		err = -ENOMEM;
		return err;
	}
	pr_info("%s:RX Physical address = 0x%08x | virtual address = 0x%p\n",
					 __func__, read_dma_handle, readbuff);
	if (!IS_ALIGNED((unsigned)read_dma_handle, 16))
		return -EINVAL;

	memset(readbuff, 0, sg_recv_size);
	trans_tx = idi_alloc_transaction(GFP_KERNEL);
	write_pool_1 = dma_pool_create("idi_write_pool_1", NULL,
						sg_trans_size, 16, 0);
	if (!write_pool_1) {
		pr_err("%s(): cannot allocate idi_write_pool_1", __func__);
		err = -ENOMEM;
		return err;
	}
	writebuff_1 = dma_pool_alloc(write_pool_1, GFP_KERNEL,
							&write_dma_handle_1);
	if (!writebuff_1) {
		pr_err("%s(): cannot allocate writebuff_1", __func__);
		err = -ENOMEM;
		return err;
	}
	pr_info("%s:TX_1 Physical address = 0x%08x | virt address = 0x%p\n",
				__func__, write_dma_handle_1, writebuff_1);

	if (!IS_ALIGNED((unsigned)write_dma_handle_1, 16))
		return -EINVAL;
	memset(writebuff_1, 0, sg_trans_size);
	write_pool_2 = dma_pool_create("idi_write_pool_2", NULL,
							sg_trans_size, 16, 0);
	if (!write_pool_2) {
		pr_err("%s(): cannot allocate idi_write_pool_2",
							__func__);
		err = -ENOMEM;
		return err;
	}
	writebuff_2 = dma_pool_alloc(write_pool_2, GFP_KERNEL,
							&write_dma_handle_2);
	if (!writebuff_2) {
		pr_err("%s(): cannot allocate writebuff_2",
							__func__);
		err = -ENOMEM;
		return err;
	}
	pr_info("%s:TX_2 Physical address = 0x%08x | virt address = 0x%p\n",
			 __func__, write_dma_handle_2, writebuff_2);

	if (!IS_ALIGNED((unsigned)write_dma_handle_2, 16))
		return -EINVAL;
	memset(writebuff_2, 0, sg_trans_size);
	writebuff_1[0] = READ_WORD(0x060000C);
	writebuff_1[1] = READ_WORD(0x0600014);
	writebuff_1[2] = READ_WORD(0x0600018);
	writebuff_1[3] = READ_WORD(0x060001C);
	writebuff_2[0] = READ_WORD(0x060000C);
	writebuff_2[1] = READ_WORD(0x0600014);
	writebuff_2[2] = READ_WORD(0x0600018);
	writebuff_2[3] = READ_WORD(0x060001C);

	sg_pool_1 = dma_pool_create("idi_sg_pool_1", NULL, sg_size, 16, 0);
	if (!sg_pool_1) {
		pr_err("%s(): cannot allocate idi_sg_pool_1", __func__);
		err = -ENOMEM;
		return err;
	}
	sg_buff_1 = dma_pool_alloc(sg_pool_1, GFP_KERNEL, &sg_dma_handle_1);
	if (!sg_buff_1) {
		pr_err("%s(): cannot allocate sg_buff_1", __func__);
		err = -ENOMEM;
		return err;
	}
	pr_info("%s:TX_SG1 Physical address = 0x%08x | virt address = 0x%p\n",
				__func__, sg_dma_handle_1, sg_buff_1);
	if (!IS_ALIGNED((unsigned)sg_dma_handle_1, 16))
		return -EINVAL;
	memset(sg_buff_1, 0, sg_size);
	sg_pool_2 = dma_pool_create("idi_sg_pool_2", NULL, sg_size, 16, 0);
	if (!sg_pool_2) {
		pr_err("%s(): cannot allocate idi_sg_pool_2", __func__);
		err = -ENOMEM;
		return err;
	}
	sg_buff_2 = dma_pool_alloc(sg_pool_2, GFP_KERNEL, &sg_dma_handle_2);
	if (!sg_buff_2) {
		pr_err("%s(): cannot allocate sg_buff_2", __func__);
		err = -ENOMEM;
		return err;
	}
	pr_info("%s:TX_SG2 Physical address = 0x%08x | virt address = 0x%p\n",
					__func__, sg_dma_handle_2, sg_buff_2);

	if (!IS_ALIGNED((unsigned)sg_dma_handle_2, 16))
		return -EINVAL;
	memset(sg_buff_2, 0, sg_size);
	sg1 = (struct idi_sg_desc *)(sg_buff_1);
	sg2 = (struct idi_sg_desc *)(sg_buff_2);
	sg1->next = (unsigned int)sg_dma_handle_2;
	sg1->base = (unsigned int)write_dma_handle_1;
	sg1->size = sg_trans_size;
	pr_info("list-1 sg1->next = 0x%08x sg1->base = 0x%08x sg1->size=%d\n",
		sg1->next, sg1->base, sg1->size);
	sg2->next = 0x3;
	sg2->base = (unsigned int)write_dma_handle_2;
	sg2->size = sg_trans_size;
	pr_info("list-2 sg2->next = 0x%08x sg2->base = 0x%08x sg2->size=%d\n",
		sg2->next, sg2->base, sg2->size);
	trans_rx->peripheral = dev;
	trans_rx->idi_xfer.base = read_dma_handle;
	trans_rx->idi_xfer.size = sg_recv_size;
	trans_rx->complete = read_complete;
	trans_tx->peripheral = dev;
	trans_tx->idi_xfer.desc = (struct idi_sg_desc *)(sg_dma_handle_1);
	if (break_frame)
		trans_tx->break_frame = 1;  /*Enable the break frame*/
	trans_tx->complete = write_complete;
	peripheral->com_flag = MSG_PENDING;
	idi_outstanding_read(dev, trans_tx, trans_rx);
	wait_event_interruptible_timeout(peripheral->wait_msg,
				peripheral->com_flag != MSG_PENDING,
					msecs_to_jiffies(5000));
	if (peripheral->com_flag == MSG_PENDING) {
		err = -ETIME;
		goto read_failed;
	}

	for (index = 0; index < sg_recv_size/4; index++) {
		if (virt_idi_reg_val[index % 4] != readbuff[index]) {
			pr_err(" Read value = 0x%08x\n", readbuff[index]);
			atomic_set(&test_flag, 0);
			break;
		}
	}
read_failed:
	if (readbuff != NULL) {
		dma_pool_free(read_pool, readbuff, read_dma_handle);
		dma_pool_destroy(read_pool);
	}
	if (writebuff_1 != NULL) {
		dma_pool_free(write_pool_1, writebuff_1, write_dma_handle_1);
		dma_pool_destroy(write_pool_1);
	}
	if (writebuff_2 != NULL) {
		dma_pool_free(write_pool_2, writebuff_2, write_dma_handle_2);
		dma_pool_destroy(write_pool_2);
	}
	if (sg_buff_1 != NULL) {
		dma_pool_free(sg_pool_1, sg_buff_1, sg_dma_handle_1);
		dma_pool_destroy(sg_pool_1);
	}
	if (sg_buff_2 != NULL) {
		dma_pool_free(sg_pool_2, sg_buff_2, sg_dma_handle_2);
		dma_pool_destroy(sg_pool_2);
	}
	idi_free_transaction(trans_rx);
	idi_free_transaction(trans_tx);
	return err;
}

static int idi_peripheral_probe(struct idi_peripheral_device *dev,
			    const struct idi_device_id *id)
{
	void __iomem *peripheral_io;
	struct idi_peripheral *peripheral;
	struct resource *fmr_reg, *fmr_ram;
	struct idi_resource *idi_res;
	peripheral_io = NULL;
	idi_res = &dev->resources;
	pr_info("****%s****\n", __func__);
	/* NOTE: driver uses the static register mapping */
	fmr_ram = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "ram");
	if (!fmr_ram) {
		dev_err(&dev->device, "IDI test: no mem resource? (ram)\n");
		return -ENODEV;
	}
	fmr_reg = idi_get_resource_byname(idi_res, IORESOURCE_MEM, "register");
	if (!fmr_reg) {
		dev_err(&dev->device,
				"IDI test: no mem resource? (register)\n");
		return -ENODEV;
	}
	idi_set_power_state(dev, D0, true);
	pr_debug("%s : FMR RAM Addr = 0x%08x  FMR REG Addr = 0x%08x\n",
			__func__, fmr_ram->start, fmr_reg->start);
	idi_set_power_state(dev, D3, false);
	peripheral_io = (unsigned int *)fmr_reg->start;
	/*
	 * We matched, keep track of the io space and init peripheral data.
	 */
	peripheral = kzalloc(sizeof(*peripheral), GFP_KERNEL);
	if (peripheral == NULL)
		goto peripheral_failed;

	peripheral->ctrl_io = peripheral_io;
	peripheral->peripheral_device = dev;

	spin_lock_init(&idi_info.idi_test_sw_lock);

	atomic_set(&test_flag, 0);
	peripheral->com_flag = MSG_PENDING;
	init_waitqueue_head(&peripheral->wait_msg);
	idi_info.event_flag = MSG_PENDING;
	init_waitqueue_head(&idi_info.wait_q);

	idi_info.idi_test_wq =
		create_singlethread_workqueue("idi_test_work_queue");
	if (idi_info.idi_test_wq == NULL)
		return -1;

	idi_info.idi_test_wq_1 =
		create_singlethread_workqueue("idi_test_work_queue_1");
	if (idi_info.idi_test_wq_1 == NULL)
		return -1;

	dev_set_drvdata(&dev->device, peripheral);
	idi_info.idi_dev = dev;
	return 0;

peripheral_failed:
	/* no_match: */
	return -ENOMEM;

}				/* idi_peripheral_probe() */

/**
 * peripheral_remove - Clean up the peripheral device.
 * @dev: the device to work on.
 *
 * Do NOT call kfree on the p_device.  The release function in idi_bus.c
 * will take care of it/
 */
static int peripheral_remove(struct idi_peripheral_device *p_device)
{
	struct idi_peripheral *peripheral;

	peripheral = dev_get_drvdata(&p_device->device);
	dev_set_drvdata(&p_device->device, NULL);

	kzfree(peripheral);

	return 0;

}				/* peripheral_remove() */


struct idi_peripheral_driver idi_peripheral = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ag6x0_fmr",
		   },
	.p_type = IDI_FMR,
	.probe = idi_peripheral_probe,
	.remove = peripheral_remove,
};

/**
 * idi_test_init - module init function.
 */
static int __init idi_test_init(void)
{
	pr_debug(" %s :\n", __func__);
#ifdef CONFIG_DEBUG_FS
	idi_add_tests_debugfs();
#endif
	return idi_register_peripheral_driver(&idi_peripheral);

}				/* peripheral_register() */

module_init(idi_test_init);

/**
 * idi_test_exiit - Exit routine.
 *
 */
static void __exit idi_test_exit(void)
{
	pr_debug(" %s :\n", __func__);
#ifdef CONFIG_DEBUG_FS
	idi_remove_tests_debugfs();
#endif
	if (idi_info.idi_test_wq != NULL)
		destroy_workqueue(idi_info.idi_test_wq);
	if (idi_info.idi_test_wq_1 != NULL)
		destroy_workqueue(idi_info.idi_test_wq_1);

	idi_unregister_peripheral_driver(&idi_peripheral);
}

module_exit(idi_test_exit);

MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION("IDI Test driver");
MODULE_LICENSE("GPL");
