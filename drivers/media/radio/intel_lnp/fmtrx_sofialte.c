/*
 * Copyright (C) 2013 Intel Mobile Communications GmbH
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

/*
** =============================================================================
**
**				MODULE DESCRIPTION
**
** =============================================================================
*/
/* This file contains the HW abstraction interfaces for RX functionality */

/*
** =============================================================================
**
**				INCLUDE STATEMENTS
**
** =============================================================================
*/
#include "fmtrx_sys.h"		/* System related */
#include "fmtrx_common.h"
#include "fmtrx_v4l2.h"
#include "fmtrx_hw_cmds.h"

/*
** =============================================================================
**
**				DEFINES
**
** =============================================================================
*/
#define FMTRX_MODULE_VERSION "0.0.0"
#define FMTRX_MODULE_NAME "FMTRX_LNP_MOOR"
#define FMTRX_CARD_NAME "LNP"
#define FMTRX_MODULE_SELECT FMTRX_RX
#define FMTRX_TASK "FMTRX_MSG_TASK"
#define FMTRX_RX_FW_NAME "fmr_rxmain.bin"
#define FMTRX_TX_FW_NAME "fmr_txmain.bin"
#define FMTRX_RX_NVM_NAME "fmr_rxnvm.bin"
#define FMTRX_TX_NVM_NAME "fmr_txnvm.bin"

/* FM Radio RX Firmware binary name in user space */
#define FMRX_FW_NAME "fmr_rxmain.bin"

/* Timeouts */
#define FMR_HCI_CMD_TIMEOUT 1000
#define FMR_HCI_REG_COMPLETE_TIMEOUT 1000

/* Size of each element in HCI command */
#define FMR_HCI_H4_HDR_COMMAND_SIZE sizeof(u8)
#define FMR_HCI_HDR1_OPCODE_SIZE sizeof(u16)
#define FMR_HCI_HDR2_PARAM_LEN_SIZE sizeof(u8)
#define FMR_HCI_HDR2_ADDR_SIZE sizeof(u32)
#define FMR_HCI_HDR2_MODE_SIZE sizeof(u8)
#define FMR_HCI_HDR2_DATA_LEN_SIZE sizeof(u8)
#define FMR_HCI_HDR2_POWER_CMD_SIZE sizeof(u8)
#define FMR_HCI_HDR2_AUDIO_CMD_SIZE sizeof(u8)

#define FMR_HCI_WRITE16_DATA_LEN_SIZE sizeof(u16)
#define FMR_HCI_WRITE32_DATA_LEN_SIZE sizeof(u32)
#define FMR_HCI_WRITE_MAX_DATA_LEN_SIZE 249
#define FMR_HCI_WRITE_MAX_16BIT_DATA_LEN_SIZE 248
#define FMR_HCI_WRITE_MAX_32BIT_DATA_LEN_SIZE 248
#define FMR_HCI_READ16_DATA_LEN_SIZE sizeof(u16)
#define FMR_HCI_READ32_DATA_LEN_SIZE sizeof(u32)
#define FMR_HCI_READ_MAX_DATA_LEN_SIZE 247
#define FMR_HCI_READ_MAX_16BIT_DATA_LEN_SIZE 246
#define FMR_HCI_READ_MAX_32BIT_DATA_LEN_SIZE 244

/* HCI command byte positions */
#define FMR_HCI_CMD_PKT_TYPE_POS 0
#define FMR_HCI_CMD_HDR1_OP_POS (FMR_HCI_CMD_PKT_TYPE_POS + \
				FMR_HCI_H4_HDR_COMMAND_SIZE)
#define FMR_HCI_CMD_HDR2_TOT_LEN_POS (FMR_HCI_CMD_HDR1_OP_POS + \
				FMR_HCI_HDR1_OPCODE_SIZE)
#define FMR_HCI_CMD_HDR2_ADDR_POS (FMR_HCI_CMD_HDR2_TOT_LEN_POS + \
				FMR_HCI_HDR2_PARAM_LEN_SIZE)
#define FMR_HCI_CMD_HDR2_MODE_POS (FMR_HCI_CMD_HDR2_ADDR_POS + \
				FMR_HCI_HDR2_ADDR_SIZE)
#define FMR_HCI_CMD_HDR2_LEN_POS (FMR_HCI_CMD_HDR2_MODE_POS + \
				FMR_HCI_HDR2_MODE_SIZE)
#define FMR_HCI_CMD_DATA_POS (FMR_HCI_CMD_HDR2_LEN_POS + \
				FMR_HCI_HDR2_DATA_LEN_SIZE)

/* HCI command lengths */
#define FMR_HCI_CMD_LEN_EXCEPT_DATA (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE + \
		FMR_HCI_HDR2_ADDR_SIZE + FMR_HCI_HDR2_MODE_SIZE + \
		FMR_HCI_HDR2_DATA_LEN_SIZE)

#define FMR_HCI_WRITE16_CMD_LEN (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE + \
		FMR_HCI_HDR2_ADDR_SIZE + FMR_HCI_HDR2_MODE_SIZE + \
		FMR_HCI_HDR2_DATA_LEN_SIZE + FMR_HCI_WRITE16_DATA_LEN_SIZE)

#define FMR_HCI_WRITE32_CMD_LEN (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE + \
		FMR_HCI_HDR2_ADDR_SIZE + FMR_HCI_HDR2_MODE_SIZE + \
		FMR_HCI_HDR2_DATA_LEN_SIZE + FMR_HCI_WRITE32_DATA_LEN_SIZE)

#define FMR_HCI_READ_CMD_LEN (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE + \
		FMR_HCI_HDR2_ADDR_SIZE + FMR_HCI_HDR2_MODE_SIZE + \
		FMR_HCI_HDR2_DATA_LEN_SIZE)

#define FMR_HCI_AUDIO_CMD_LEN (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE + \
		FMR_HCI_HDR2_AUDIO_CMD_SIZE)

#define FMR_HCI_POWER_CMD_LEN (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE + \
		FMR_HCI_HDR2_POWER_CMD_SIZE)

#define FMR_HCI_CONFIRM_IRQ_CMD_LEN (FMR_HCI_H4_HDR_COMMAND_SIZE + \
		FMR_HCI_HDR1_OPCODE_SIZE + FMR_HCI_HDR2_PARAM_LEN_SIZE)

#define FMR_HCI_TOP_READ_CMD_LEN FMR_HCI_READ_CMD_LEN

/* HCI command header 2 total parameter length */
#define FMR_HCI_WRITE_HDR2_TOT_PARAM_LEN (FMR_HCI_HDR2_ADDR_SIZE + \
		FMR_HCI_HDR2_MODE_SIZE + \
/* Add data length to this for the total length */ \
		FMR_HCI_HDR2_DATA_LEN_SIZE)

#define FMR_HCI_READ_HDR2_TOT_PARAM_LEN (FMR_HCI_HDR2_ADDR_SIZE + \
		FMR_HCI_HDR2_MODE_SIZE + FMR_HCI_HDR2_DATA_LEN_SIZE)
#define FMR_HCI_POWER_HDR2_TOT_PARAM_LEN FMR_HCI_HDR2_POWER_CMD_SIZE
#define FMR_HCI_AUDIO_HDR2_TOT_PARAM_LEN FMR_HCI_HDR2_AUDIO_CMD_SIZE
#define FMR_HCI_TOP_WRITE_HDR2_TOT_PARAM_LEN FMR_HCI_WRITE_HDR2_TOT_PARAM_LEN
#define FMR_HCI_TOP_READ_HDR2_TOT_PARAM_LEN FMR_HCI_READ_HDR2_TOT_PARAM_LEN
#define FMR_HCI_IRQ_HDR2_TOT_PARAM_LEN 0

/* Size of each element in HCI event */
#define FMR_HCI_EVT_HDR1_EVENT_CODE sizeof(u8)
#define FMR_HCI_EVT_HDR2_PARAM_LEN_SIZE sizeof(u8)
#define FMR_HCI_EVT_HDR2_NUM_HCI_PKTS_SIZE sizeof(u8)
#define FMR_HCI_EVT_HDR2_OPCODE_SIZE sizeof(u16)
#define FMR_HCI_EVT_HDR2_STATUS_SIZE sizeof(u8)
#define FMR_HCI_EVT_HDR2_ADDR_SIZE sizeof(u32)

#define FMR_HCI_EVT_HDR2_TOT_PARAM_LEN (FMR_HCI_EVT_HDR2_NUM_HCI_PKTS_SIZE + \
		FMR_HCI_EVT_HDR2_OPCODE_SIZE + FMR_HCI_EVT_HDR2_STATUS_SIZE)

/* HCI H4 headers */
#define FMR_HCI_H4_HDR_COMMAND_PKT 0x01
#define FMR_HCI_H4_HDR_EVENT_PKT 0x04

/* HCI command header 1 - OGF/OCF value */
#define OPCODE(ogf, ocf) (ocf | ogf << 10)

/* Mode Value defines */
#define FMR_HCI_8BIT_ACCESS_MODE 0x0	/* 08 bit read access */
#define FMR_HCI_16BIT_ACCESS_MODE 0x1	/* 16 bit read access */
#define FMR_HCI_32BIT_ACCESS_MODE 0x2	/* 32 bit read access */

#define HCI_INTEL_FMR_WRITE 0xFC58	/* OGF: 0x3F, OCF: 0x0058 */
#define HCI_INTEL_FMR_READ 0xFC59	/* OGF: 0X3F, OCF: 0x0059 */
#define HCI_INTEL_FMR_POWER 0xFC5A	/* OGF: 0x3F, OCF: 0x005A */
#define HCI_INTEL_FMR_AUDIO 0xFC5B	/* OGF: 0X3F, OCF: 0x005B */
#define HCI_INTEL_FMR_IRQ_CONFIRM 0xFC5C	/* OGF: 0x3F, OCF: 0x005C */
#define HCI_INTEL_FMR_TOP_WRITE 0xFC5D	/* OGF: 0x3F, OCF: 0x005D */
#define HCI_INTEL_FMR_TOP_READ 0xFC5E	/* OGF: 0x3F, OCF: 0x005E */

#define HCI_INTEL_FMR_EVT_CMD_COMPLETE 0x0E
#define HCI_INTEL_FMR_EVT_INTERRUPT 0xFF

#define HCI_INTEL_FMR_EVT_INTERRUPT_EVTID 0x2B

#define HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS 0X00
#define HCI_INTEL_FMR_CMD_COMPLETE_FAILURE 0X01

#define FMR_DCDC_REG_OFFSET 0x8c04
#define FMR_DCDC_ACTIVE_HIGH_VOLTAGE 0x11E
#define FMR_DCDC_DEFAULT_VOLTAGE 0x0
#define FMR_DCDC_REGISTER_BIT 12
#define FMR_DCDC_REGISTER_WIDTH 9
/*
** =============================================================================
**
**				ENUM DECLARATIONS
**
** =============================================================================
*/
enum access_type {
	FMRIP,
	TOP
};

/*
** =============================================================================
**
**				STRUCT DECLARATIONS
**
** =============================================================================
*/
struct hci_intel_fmr_write {
	u32 addr_off;
	u8 mode;
	u8 length;
	u8 *data;
};

struct hci_intel_fmr_read {
	u32 addr_off;
	u8 mode;
	u8 length;
};

struct hci_intel_fmr_power {
	u8 pow_en;
};

struct hci_intel_fmr_audio {
	u8 aud_en;
};

union hci_intel_cmd_params {
	struct hci_intel_fmr_write fmr_hci_write_cmd;
	struct hci_intel_fmr_read fmr_hci_read_cmd;
	struct hci_intel_fmr_power fmr_hci_pow_cmd;
	struct hci_intel_fmr_audio fmr_hci_aud_cmd;
};

struct hci_intel_cmd_hdr1 {
	u16 opcode;		/* OCF (10 bit) and OGF (6 bit) */
};

struct hci_intel_cmd_hdr2 {
	u8 param_tot_len;
	union hci_intel_cmd_params params;
};

struct hci_intel_cmd_pkt {
	struct hci_intel_cmd_hdr1 cmd_hdr1;
	struct hci_intel_cmd_hdr2 cmd_hdr2;
};

struct hci_intel_read_evt {
	u32 addr_off;
	u8 data[FMR_HCI_READ_MAX_DATA_LEN_SIZE];
};

union hci_intel_evt_params {
	struct hci_intel_read_evt fmr_hci_read_evt;
};

struct hci_intel_evt_hdr1 {
	u8 evtcode;
};

struct hci_intel_evt_hdr2 {
	u8 param_tot_len;
	u8 num_hci_cmd_pkts;
	u16 opcode;
	u8 status;
	union hci_intel_evt_params params;
} __packed;

struct hci_intel_evt_pkt {
	struct hci_intel_evt_hdr1 evt_hdr1;
	struct hci_intel_evt_hdr2 evt_hdr2;
} __packed;

/*
** =============================================================================
**
**				LOCAL DATA DEFINITIONS
**
** =============================================================================
*/
static struct completion event_sem;
static struct completion read_cmd_sem;
static struct completion write_cmd_sem;
static struct completion power_cmd_sem;
static struct completion audio_cmd_sem;
static struct completion irq_cmd_sem;
static struct completion msg_process_sem;
static struct mutex cmd_sync_lock;
static struct hci_intel_evt_pkt g_hci_evt;
static enum interrupt_vector fmr_vector_type;
static const struct firmware *fw_entry;
static struct device *fmtrx_device;

#ifdef FMR_INTR_MODE
static struct task_struct *msg_process_task;
static bool msg_process_task_stop;
#endif

#ifdef LD_DRIVER
static struct fm_ld_drv_register ld_driver;
#endif

#ifdef LNP_EVB
static u16 int_lna_offsets[] = { 255, 250, 246, 241, 237,
		231, 226, 221, 215, 209, 202, 194, 187, 177, 165, 150
};

static u16 ext_lna_offsets[] = { 255, 250, 246, 241, 237,
		231, 226, 221, 215, 209, 202, 194, 187, 177, 165, 150
};

static u16 int_ppf_offsets[] = { 296, 288, 282, 273, 265,
		258, 251, 244, 236, 230, 224, 217, 212, 206, 201, 197
};

static u16 ext_ppf_offsets[] = { 296, 288, 282, 273, 265,
		258, 251, 244, 236, 230, 224, 217, 212, 206, 201, 197
};
#else
static u16 int_lna_offsets[] = { 252, 249, 244, 239, 235,
		230, 225, 218, 213, 207, 201, 193, 185, 176, 165, 150
};

static u16 ext_lna_offsets[] = { 252, 249, 244, 239, 235,
		230, 225, 218, 213, 207, 201, 193, 185, 176, 165, 150
};

static u16 int_ppf_offsets[] = { 320, 312, 306, 297, 289,
		282, 275, 268, 260, 254, 248, 241, 236, 230, 225, 221
};

static u16 ext_ppf_offsets[] = { 320, 312, 306, 297, 289,
		282, 275, 268, 260, 254, 248, 241, 236, 230, 225, 221
};
#endif

static u16 cp_init_offsets[] = { 950, 850, 800, 750, 700,
		650, 600, 550, 500, 450, 450, 450, 350, 350, 300, 250
};
static struct rssi_offsets int_rssi_offsets = { 108000,
		108000, 108000, 108000, 108000, 0, 0, 0, 0, 0, 0 };
static struct rssi_offsets ext_rssi_offsets = { 108000,
		108000, 108000, 108000, 108000, 0, 0, 0, 0, 0, 0 };
static struct rssi_offsets int_ext_lna_offsets = { 108000,
		108000, 108000, 108000, 108000, 0, 0, 0, 0, 0, 0 };
static struct rssi_offsets ext_ext_lna_offsets = { 108000,
		108000, 108000, 108000, 108000, 0, 0, 0, 0, 0, 0 };

/* IRQ activation flag */
static bool irq_active;

#ifdef FMR_DEBUG_MEAS
u32 g_total_recv = 0, g_total_sent = 0;
#endif

#ifdef FMR_DEBUG_LVL2
u32 g_recv_int_count = 0, g_req_int_count = 0;
#endif

/*
** =============================================================================
**
**				LOCAL	FUNCTION DECLARATIONS
**
** =============================================================================
*/
/* Driver probe function
 * @p_dev Pointer to device type
 */
static int fmtrx_driver_probe(
		struct platform_device *p_dev);

/* Driver remove function
 * @p_dev Pointer to device type
 */
static int fmtrx_driver_remove(
		struct platform_device *p_dev);

/* Wrapper to send command over UART
 * @data Command buffer
 * @size Size of the command buffer
 */
static int fmr_hci_send_cmd(
		u8 *data,
		u16 size);

/* Assembler for Write command
 * @fmr_hci_cmd Write command HCI structure
 * @data Command buffer
 * @size Size of the command buffer
 */
static int fmr_hci_write_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size);

/* Assembler for Read command
 * @fmr_hci_cmd Read command HCI structure
 * @data Command buffer
 * @size Size of the command buffer
 */
static int fmr_hci_read_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size);

/* Assembler for Power command
 * @fmr_hci_cmd Power command HCI structure
 * @data Command buffer
 * @size Size of the command buffer
 */
static int fmr_hci_power_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size);

/* Assembler for Audio command
 * @fmr_hci_cmd Audio command HCI structure
 * @data Command buffer
 * @size Size of the command buffer
 */
static int fmr_hci_audio_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size);

/* Assembler for IRQ confirm command
 * @fmr_hci_cmd IRQ confirm command HCI structure
 * @data Command buffer
 * @size Size of the command buffer
 */
static int fmr_hci_irq_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size);

/* Store the command header in the command buffer
 * @fmr_hci_cmd HCI command packet
 * @data Command buffer
 */
static int fmr_hci_store_cmdhdr1(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data);

/* Command Assembler
 * @fmr_hci_cmd HCI command packet
 * @data_pkt_size Size of the data packet
 */
static int fmr_hci_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd);

/* Convert 16-bit value to LE format
 * @value Value to convert
 * @data Pointer to the data where the converted value is written
 * @offset Offset in the data pointer from where converted value is written
 */
static int fmr_convert_to_le16(
		u16 value,
		u8 *data,
		u32 offset);

/* Convert 32-bit value to LE format
 * @value Value to convert
 * @data Pointer to the data where the converted value is written
 * @offset Offset in the data pointer from where converted value is written
 */
static int fmr_convert_to_le32(
		u32 value,
		u8 *data,
		u32 offset);

/* Read from register
 * @addr Register address
 * @data Read value
 * @size Number of bytes
 * @type Width - 16 or 32
 * @access FMRIP or TOP level
 */
static int fmr_generic_read(
		u32 addr_offs,
		u8 *data,
		u16 size,
		enum bit_type type,
		enum access_type access);

/* Write to register
 * @addr Register address
 * @data Write value
 * @size Number of bytes
 * @type Width - 16 or 32
 * @access FMRIP or TOP level
 */
static int fmr_generic_write(
		u32 addr_offs,
		const u8 *data,
		u16 size,
		enum bit_type type,
		enum access_type access);

/* Send IRQ confirm command
 */
static int fmr_irq_confirm(
		void);

/* Handler that receives the HCI event
 * @arg Pointer to private data
 * @skb Pointer to the HCI event data
 */
static long fmr_plat_hci_event_handler(
		void *arg,
		struct sk_buff *skb);

/* Callback invoked by LD driver to indicate registration completion
 * @arg Pointer to private data
 * @status Status of registration
 */
static void fmtrx_plat_reg_complete_cb(
		void *arg,
		char status);

/* Interrupt handler at platform level
 * @irq_status Value of the FMR IP IRQ status register
 * @data Pointer to current context
 */
static int fmtrx_plat_interrupt_handler(
		int irq_status,
		void *data);

/* Message/Interrupt processing task
 * @args Pointer to private data
 */
static int msg_process_task_run(
		void *args);

/* denotification to Frequency manager
 */
static void fmtrx_denotify_mitigation_interference(
		void);

/* Request for FMR dcdc 1.8v enable/disable
 */
static int fmtrx_sys_request_dcdc(
	bool enable);

/*
** =============================================================================
**
**				EXPORTED FUNCTION DEFINITIONS
**
** =============================================================================
*/
int fmtrx_sys_init(
		void)
{
	int err = 0;

	/* Semaphores */
	init_completion(&event_sem);
	init_completion(&read_cmd_sem);
	init_completion(&write_cmd_sem);
	init_completion(&power_cmd_sem);
	init_completion(&audio_cmd_sem);
	init_completion(&irq_cmd_sem);
	init_completion(&msg_process_sem);

	/* Mutexes */
	mutex_init(&cmd_sync_lock);

#ifdef FMR_INTR_MODE
	msg_process_task_stop = false;
	/* Threads */
	msg_process_task = kthread_run(msg_process_task_run, 0, FMTRX_TASK);
	if (0 == msg_process_task) {
		err = -EIO;
		fmtrx_sys_log
		   ("%s: %s %d, Starting thread failed! %d\n",
		   FILE, __func__,
		   __LINE__, err);
		goto fmtrx_sys_int_exit;
	}
#endif

#ifdef LD_DRIVER
	/* Register with LD driver */
	ld_driver.ld_drv_reg_complete_cb = &fmtrx_plat_reg_complete_cb;
	ld_driver.fm_cmd_handler = &fmr_plat_hci_event_handler;

	err = (int)register_fmdrv_to_ld_driv(&ld_driver);
	if ((0 != err) || (0 == ld_driver.fm_cmd_write)) {
		fmtrx_sys_log
		("%s: %s %d, Line discipline driver registration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_sys_int_exit1;
	}

	/* Block until registration completes */
	if (!wait_for_completion_timeout
	    (&event_sem, msecs_to_jiffies(FMR_HCI_REG_COMPLETE_TIMEOUT))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d, Wait for event completion timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_sys_int_exit1;
	}
#endif

	goto fmtrx_sys_int_exit;

 fmtrx_sys_int_exit1:
#ifdef FMR_INTR_MODE
	/* Kill the thread */
	if (false == msg_process_task_stop) {
		msg_process_task_stop = true;
		complete(&msg_process_sem);
	}
#endif
 fmtrx_sys_int_exit:
	return err;
}

int fmtrx_sys_deinit(
		void)
{
	int err = 0;

	/* Cleanup semaphores */
	complete(&event_sem);
	complete(&read_cmd_sem);
	complete(&write_cmd_sem);
	complete(&power_cmd_sem);
	complete(&audio_cmd_sem);
	complete(&irq_cmd_sem);

#ifdef FMR_INTR_MODE
	/* Kill the thread */
	if (false == msg_process_task_stop) {
		msg_process_task_stop = true;
		complete(&msg_process_sem);
	}
#endif

#ifdef LD_DRIVER
	/* Unregister from LD driver */
	if (0 != ld_driver.fm_cmd_write) {
		err = unregister_fmdrv_from_ld_driv(&ld_driver);
		if (err != 0) {
			fmtrx_sys_log
			("%s: %s %d,Line discipline driver unreg failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}
	}
#endif

	return err;
}

int
fmtrx_sys_mitigate_interference(enum component type, union component_data *data)
{
	int err = 0;

	switch (type) {
#ifdef CONFIG_IUI_FM_FMR
	struct iui_fm_fmr_info freq_mgr_fmrinfo;
	struct iui_fm_freq_notification freq_mgr_notify_data;
	/* fall through */
	case COMPONENT_FM:
		freq_mgr_fmrinfo.rx_freq =
			data->fm_cfg.frequency / 1000; /* frequency in kHz */
		freq_mgr_fmrinfo.inj_side =
			(INJECTION_SIDE_LSI == data->fm_cfg.side) ?
			IUI_FM_FMR_INJECTION_SIDE_LOW :
			IUI_FM_FMR_INJECTION_SIDE_HIGH;

		freq_mgr_notify_data.type = IUI_FM_FREQ_NOTIFICATION_TYPE_FMR;
		freq_mgr_notify_data.info.fmr_info = &freq_mgr_fmrinfo;
		fmtrx_sys_log
			("%s: %s %d,Notify FM with freq: %d and inj side: %d\n",
				FILE, __func__,
				__LINE__,
				freq_mgr_fmrinfo.rx_freq,
				freq_mgr_fmrinfo.inj_side);

		err = iui_fm_notify_frequency(IUI_FM_MACRO_ID_FMR,
					&freq_mgr_notify_data);
		fmtrx_sys_log
			("%s: %s %d,FM notification err code %d\n",
				FILE, __func__,
				__LINE__,
				err);
		break;
#endif /* CONFIG_IUI_FM_FMR */
	case COMPONENT_DCDC:
		break;
	default:
		break;
	}

	return err;
}

int fmtrx_sys_reg_read16(
		u32 addr_offs,
		u16 *data)
{
	int err = 0;

	err =
	    fmr_generic_read(addr_offs, (u8 *) data,
			     FMR_HCI_READ16_DATA_LEN_SIZE, WIDTH_16BIT, FMRIP);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d, Generic read 16-bit failed! %d\n",
		FILE, __func__,
		__LINE__, err);
	}

	return err;
}

int fmtrx_sys_reg_read32(
		u32 addr_offs,
		u32 *data)
{
	int err = 0;

	err =
	    fmr_generic_read(addr_offs, (u8 *) data,
			     FMR_HCI_READ32_DATA_LEN_SIZE, WIDTH_32BIT, FMRIP);
	if (0 != err) {
		fmtrx_sys_log
		 ("%s: %s %d, Generic read 32-bit failed! %d\n",
		 FILE, __func__,
		 __LINE__, err);
	}

	return err;
}

int fmtrx_sys_reg_write16(
		u32 addr_offs,
		u16 data)
{
	int err = 0;

	err =
	    fmr_generic_write(addr_offs, (u8 *) &data,
			      FMR_HCI_WRITE16_DATA_LEN_SIZE, WIDTH_16BIT,
			      FMRIP);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d, Generic write 16-bit failed! %d\n",
		FILE, __func__,
		__LINE__, err);
	}

	return err;
}

int fmtrx_sys_reg_write32(
		u32 addr_offs,
		u32 data)
{
	int err = 0;

	err =
	    fmr_generic_write(addr_offs, (u8 *) &data,
			      FMR_HCI_WRITE32_DATA_LEN_SIZE, WIDTH_32BIT,
			      FMRIP);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d, Generic write 32-bit failed! %d\n",
		FILE, __func__,
		__LINE__, err);
	}

	return err;
}

int fmtrx_sys_mem_write(
		u32 addr_offs,
		const u8 *image,
		u32 size)
{
	int err = 0;
	u32 rem_bytes = size, bytes_to_send = 0;
	const u8 *p_data = image;

	/* Validate input arguments */
	if ((0 == image) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmtrx_sys_mem_write_exit;
	}

	while (rem_bytes != 0) {
		if (0 == (rem_bytes % FMR_HCI_WRITE_MAX_16BIT_DATA_LEN_SIZE)) {
			bytes_to_send = FMR_HCI_WRITE_MAX_16BIT_DATA_LEN_SIZE;
		} else {
			bytes_to_send =
			    (rem_bytes % FMR_HCI_WRITE_MAX_16BIT_DATA_LEN_SIZE);
		}
		err =
		    fmr_generic_write(addr_offs + (size - rem_bytes), p_data,
				      bytes_to_send, WIDTH_16BIT, FMRIP);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Generic write 16-bit failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_sys_mem_write_exit;
		}
		/* Calculate remaining bytes */
		rem_bytes -= bytes_to_send;
		/* Update the source pointer */
		p_data += bytes_to_send;
	}

 fmtrx_sys_mem_write_exit:
	return err;
}

int fmtrx_sys_mem_read(
		u32 addr_offs,
		u8 *image,
		u32 size)
{
	int err = 0;
	u32 rem_bytes = size, bytes_to_send = 0;
	u8 *p_data = image;

	/* Validate input arguments */
	if ((0 == image) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmtrx_sys_mem_read_exit;
	}

	while (rem_bytes != 0) {
		if (0 == (rem_bytes % FMR_HCI_READ_MAX_16BIT_DATA_LEN_SIZE)) {
			bytes_to_send = FMR_HCI_READ_MAX_16BIT_DATA_LEN_SIZE;
		} else {
			bytes_to_send =
			    (rem_bytes % FMR_HCI_READ_MAX_16BIT_DATA_LEN_SIZE);
		}
		err =
		    fmr_generic_read(addr_offs + (size - rem_bytes), p_data,
				     bytes_to_send, WIDTH_16BIT, FMRIP);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Generic read 16-bit failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_sys_mem_read_exit;
		}
		/* Calculate remaining bytes */
		rem_bytes -= bytes_to_send;
		/* Update the source pointer */
		p_data += bytes_to_send;
	}

 fmtrx_sys_mem_read_exit:
	return err;
}

int fmtrx_sys_irq_enable(
		void)
{
	int err = 0;

	/* Activate IRQ lines */
	irq_active = true;
	fmtrx_sys_log
		("%s: %s %d, IRQ activated\n", FILE,
		__func__, __LINE__);

	return err;
}

int fmtrx_sys_irq_disable(
		void)
{
	int err = 0;

	/* De-activate IRQ lines */
	irq_active = false;
	fmtrx_sys_log
		("%s: %s %d, IRQ de-activated\n", FILE,
		__func__, __LINE__);

	if (0 == err) {
		/* Inform BT that IRQ is processed */
		err = fmr_irq_confirm();
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Send IRQ confirm command failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		}
	}

	return err;
}

int fmtrx_sys_clk_sel(
		enum clk_source src)
{
	int err = 0;

	/* Validate input arguments */
	if (CLK_SRC_INVALID <= src) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmtrx_sys_clk_sel_exit;
	}

 fmtrx_sys_clk_sel_exit:
	return err;
}

int fmtrx_sys_wait_for_event(
		int ms)
{
	int err = 0;

#ifdef FMR_INTR_MODE
#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d, Sys wait requested count - %d!\n",
			FILE, __func__,
			__LINE__, g_req_int_count++);
#endif
	if (!wait_for_completion_timeout(&event_sem, msecs_to_jiffies(ms))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d,Wait for command complete event timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
	}
#else
	fmtrx_sys_idle_wait(ms);
#endif

	return err;
}

int fmtrx_sys_wakeup_event(
		void)
{
	int err = 0;
#ifdef FMR_INTR_MODE
#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d, Sys wait released count - %d!\n",
		FILE, __func__,
		__LINE__, g_recv_int_count++);
#endif
	complete(&event_sem);
#else
#endif
	return err;
}

int fmtrx_sys_get_driver_ver_info(
		u8 *drv_name,
		u8 *card_name)
{
	int err = 0;

	if (0 != drv_name)
		strlcpy(drv_name, FMTRX_MODULE_NAME, sizeof(FMTRX_MODULE_NAME));

	if (0 != card_name)
		strlcpy(card_name, FMTRX_CARD_NAME, sizeof(FMTRX_CARD_NAME));

	return err;
}

int fmtrx_sys_power_enable(
		bool enable)
{
	int err = 0;
	struct hci_intel_cmd_pkt cmd_pkt;

	/* Request 1.8v dcdc voltage */
	if (enable) {
		err = fmtrx_sys_request_dcdc(enable);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,dcdc 1.8v enable req failed!%d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_sys_power_enable_exit;
		}
	}

	memset(&cmd_pkt, 0, sizeof(struct hci_intel_cmd_pkt));

	cmd_pkt.cmd_hdr1.opcode = OPCODE(0x3F, 0x5A);
	cmd_pkt.cmd_hdr2.param_tot_len = FMR_HCI_POWER_HDR2_TOT_PARAM_LEN;
	cmd_pkt.cmd_hdr2.params.fmr_hci_pow_cmd.pow_en = enable;

	/* Wait for lock to be acquired */
	mutex_lock(&cmd_sync_lock);

	/* Assemble HCI Power command and send */
	err = fmr_hci_cmd_assembly(&cmd_pkt);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, Send command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_sys_power_enable_exit;
	}
#ifdef FMR_HOST_TEST
	{
		/* Simulation */
		struct sk_buff skb;
		/*Allocate size of event packet + H4 header */
		u8 *evt_buff =
		      kzalloc(sizeof(struct hci_intel_evt_pkt) + 1,
				  GFP_KERNEL);
		struct hci_intel_evt_pkt *p_hci_evt =
		    (struct hci_intel_evt_pkt *)(evt_buff + 1);

		evt_buff[0] = FMR_HCI_H4_HDR_EVENT_PKT;
		p_hci_evt->evt_hdr1.evtcode = HCI_INTEL_FMR_EVT_CMD_COMPLETE;
		p_hci_evt->evt_hdr2.param_tot_len =
		    FMR_HCI_EVT_HDR2_TOT_PARAM_LEN;
		p_hci_evt->evt_hdr2.num_hci_cmd_pkts = 0;
		p_hci_evt->evt_hdr2.opcode = HCI_INTEL_FMR_POWER;
		p_hci_evt->evt_hdr2.status = HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS;

		skb.data = evt_buff;
		skb.len =
		    sizeof(struct hci_intel_evt_pkt) + 1 -
		    sizeof(struct hci_intel_read_evt);
		fmr_plat_hci_event_handler(0, &skb);
		kfree(evt_buff);
	}
#endif

	/* Wait for Power command complete */
	if (!wait_for_completion_timeout
	    (&power_cmd_sem, msecs_to_jiffies(FMR_HCI_CMD_TIMEOUT))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d,Wait for command complete event timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_sys_power_enable_exit;
	} else {
		if (HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS !=
		    g_hci_evt.evt_hdr2.status) {
			err = -EIO;
			fmtrx_sys_log
			("%s: %s %d,Command complete event failed! Status %d\n",
			 FILE, __func__,
			 __LINE__, g_hci_evt.evt_hdr2.status);
			goto fmtrx_sys_power_enable_exit;
		}
	}

#ifdef CONFIG_IUI_FM_FMR
	if (!enable)
		fmtrx_denotify_mitigation_interference();
#endif /* CONFIG_IUI_FM_FMR */

 fmtrx_sys_power_enable_exit:
	/* Release the lock */
	mutex_unlock(&cmd_sync_lock);
#ifdef FMR_DEBUG_MEAS
	if (enable)
		g_total_sent = g_total_recv = g_recv_int_count =
		    g_req_int_count = 0;
#endif

	/* Request default dcdc voltage */
	if (!enable) {
		err = fmtrx_sys_request_dcdc(enable);
		if (0 != err) {
			fmtrx_sys_log
				("%s: %s %d,dcdc 1.8v disable req failed!%d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmtrx_sys_power_enable_exit;
		}
	}

	return err;
}

int fmtrx_sys_audio_enable(
		bool enable)
{
	int err = 0;
	struct hci_intel_cmd_pkt cmd_pkt;

	memset(&cmd_pkt, 0, sizeof(struct hci_intel_cmd_pkt));

	cmd_pkt.cmd_hdr1.opcode = OPCODE(0x3F, 0x5B);
	cmd_pkt.cmd_hdr2.param_tot_len = FMR_HCI_AUDIO_HDR2_TOT_PARAM_LEN;
	cmd_pkt.cmd_hdr2.params.fmr_hci_aud_cmd.aud_en = enable;

	/* Wait for lock to be acquired */
	mutex_lock(&cmd_sync_lock);

	/* Assemble HCI Audio command */
	err = fmr_hci_cmd_assembly(&cmd_pkt);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, Send command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_sys_audio_enable_exit;
	}
#ifdef FMR_HOST_TEST
	{
		/* Simulation */
		struct sk_buff skb;
		/* Allocate size of event packet + H4 header */
		u8 *evt_buff =
		      kzalloc(sizeof(struct hci_intel_evt_pkt) + 1,
				  GFP_KERNEL);
		struct hci_intel_evt_pkt *p_hci_evt =
		    (struct hci_intel_evt_pkt *)(evt_buff + 1);

		evt_buff[0] = FMR_HCI_H4_HDR_EVENT_PKT;
		p_hci_evt->evt_hdr1.evtcode = HCI_INTEL_FMR_EVT_CMD_COMPLETE;
		p_hci_evt->evt_hdr2.param_tot_len =
		    FMR_HCI_EVT_HDR2_TOT_PARAM_LEN;
		p_hci_evt->evt_hdr2.num_hci_cmd_pkts = 0;
		p_hci_evt->evt_hdr2.opcode = HCI_INTEL_FMR_AUDIO;
		p_hci_evt->evt_hdr2.status = HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS;

		skb.data = evt_buff;
		skb.len =
		    sizeof(struct hci_intel_evt_pkt) + 1 -
		    sizeof(struct hci_intel_read_evt);
		fmr_plat_hci_event_handler(0, &skb);
		kfree(evt_buff);
	}
#endif

	if (!wait_for_completion_timeout
	    (&audio_cmd_sem, msecs_to_jiffies(FMR_HCI_CMD_TIMEOUT))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d,Wait for command complete event timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_sys_audio_enable_exit;
	} else {
		if (HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS !=
		    g_hci_evt.evt_hdr2.status) {
			err = -EIO;
			fmtrx_sys_log
			("%s: %s %d,Command complete event failed! Status %d\n",
				FILE, __func__,
				__LINE__, g_hci_evt.evt_hdr2.status);
			goto fmtrx_sys_audio_enable_exit;
		}
	}

 fmtrx_sys_audio_enable_exit:
	/* Release the lock */
	mutex_unlock(&cmd_sync_lock);
	return err;
}

int fmtrx_sys_fetch_fw(
		enum fmtrx_type type,
		const u8 **data,
		u32 *size)
{
	int err = 0;

	err = request_firmware(&fw_entry,
			       (type ==
				FMTRX_RX) ? FMTRX_RX_FW_NAME : FMTRX_TX_FW_NAME,
			       fmtrx_device);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d, Request firmware failed! %d\n",
		FILE, __func__,
		__LINE__, err);
		goto fmtrx_sys_fetch_fw_exit;
	}

	if ((0 != data) && (0 != size)) {
		*data = fw_entry->data;
		*size = fw_entry->size;
	}

 fmtrx_sys_fetch_fw_exit:
	return err;
}

int fmtrx_sys_release_fw(
		void)
{
	int err = 0;

	release_firmware(fw_entry);

	return err;
}

void fmtrx_sys_idle_wait(
		u32 ms)
{
	msleep(ms);
}

int fmtrx_sys_get_rx_default_config(
		struct fmrx_config **fmrx_cfg)
{
	int err = 0;
	struct fmrx_config *data = 0;

	/* Allocate the static configuration structure */
	*fmrx_cfg =
	    kzalloc(sizeof(struct fmrx_config),
					GFP_KERNEL);
	if (0 == *fmrx_cfg) {
		err = -ENOMEM;
		fmtrx_sys_log
		("%s: %s %d, FM RX internal config alloc failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		goto fmtrx_sys_get_rx_default_config_exit;
	}

	/* Load the FMR NVM data */
	err = request_firmware(&fw_entry, FMTRX_RX_NVM_NAME, fmtrx_device);
	if ((0 != err) || (0 >= fw_entry->size)
	    || (sizeof(struct fmrx_config) != fw_entry->size)) {
		fmtrx_sys_log
		("%s: %s %d, NVM data missing or corrupted! %d\n",
		 FILE, __func__,
		 __LINE__, err);
		fmtrx_sys_log
		("%s: %s %d, Switching to default values!\n",
		FILE, __func__,
		__LINE__);
		data = *fmrx_cfg;

		data->band_cfg.max = 108000;
		data->band_cfg.min = 76000;
		data->band_cfg.step = 50;
		data->band_cfg.deemp = DEEMP_50US;

		data->force_mono = false;
		data->antenna = ANTENNA_HS_SINGLEEND;
		data->routing = ROUTING_SRC;

		data->agc_cfg.enable = false;
		data->agc_cfg.idx = AGC_GAIN_INDEX_7;

		data->rssi_cfg.enable = false;
		data->rssi_cfg.lo_thr = 10;
		data->rssi_cfg.hi_thr = 40;

		data->mute = false;

		data->snc_cfg.enable = false;
		data->snc_cfg.lo_thr = 15;
		data->snc_cfg.hi_thr = 40;

		data->sm_cfg.enable = false;
		data->sm_cfg.thr = 15;
		data->sm_cfg.step = 1638;

		data->side = INJECTION_SIDE_AUTO;
		data->vol_cfg.left = data->vol_cfg.right = 88;
		data->rds_cfg.mode = RDS_ONMODE_ON;

		data->other_cfg.pn_thr = 800;
		data->other_cfg.lna_type = GAIN_0DB_REDUC;
		data->other_cfg.volume_ramp = 60;
		data->other_cfg.clk_switch_range_104 = 150;
		data->other_cfg.int_rssi_other_offset = 0;
		data->other_cfg.ext_rssi_other_offset = 0;
		data->other_cfg.seek_thr = 0;

		memcpy(data->int_lna_offsets, (u8 *) int_lna_offsets,
		       sizeof(int_lna_offsets));
		memcpy(data->ext_lna_offsets, (u8 *) ext_lna_offsets,
		       sizeof(ext_lna_offsets));
		memcpy(data->int_ppf_offsets, (u8 *) int_ppf_offsets,
		       sizeof(int_ppf_offsets));
		memcpy(data->ext_ppf_offsets, (u8 *) ext_ppf_offsets,
		       sizeof(ext_ppf_offsets));

		memcpy(data->cp_init_offsets, (u8 *) cp_init_offsets,
		       sizeof(cp_init_offsets));

		memcpy((u8 *) &(data->int_rssi_offsets),
		       (u8 *) &int_rssi_offsets, sizeof(struct rssi_offsets));
		memcpy((u8 *) &(data->ext_rssi_offsets),
		       (u8 *) &ext_rssi_offsets, sizeof(struct rssi_offsets));
		memcpy((u8 *) &(data->int_ext_lna_offsets),
		       (u8 *) &int_ext_lna_offsets,
		       sizeof(struct rssi_offsets));
		memcpy((u8 *) &(data->ext_ext_lna_offsets),
		       (u8 *) &ext_ext_lna_offsets,
		       sizeof(struct rssi_offsets));

		err = 0;
	} else {
		memcpy((u8 *) *fmrx_cfg, fw_entry->data, fw_entry->size);
	}
	release_firmware(fw_entry);

 fmtrx_sys_get_rx_default_config_exit:
	return err;
}

void fmtrx_sys_log_traffic(
		void)
{
#ifdef FMR_DEBUG_MEAS
	fmtrx_sys_log
	("%s: Sent %d bytes, Received %d bytes\n",
	FILE, g_total_sent, g_total_recv);
#endif
}

/*
** =============================================================================
**
**							INTERRUPT INTERFACES
**
** =============================================================================
*/
static int fmtrx_plat_interrupt_handler(
		int irq_status,
		void *data)
{
	int err = 0;
	enum interrupt_vector fmr_vector = FMTRX_VECTOR_INVALID;

	/* Find the exact interrupt */
	if (0 != (irq_status & IR_INTDED0)) {
		fmr_vector = FMTRX_VECTOR_DED0;
	} else if (0 != (irq_status & IR_INTDED1)) {
		fmr_vector = FMTRX_VECTOR_DED1;
	} else if (0 != (irq_status & IR_INTDED2)) {
		fmr_vector = FMTRX_VECTOR_DED2;
	} else if (0 != (irq_status & IR_INTDED3)) {
		fmr_vector = FMTRX_VECTOR_DED3;
	} else if (0 != (irq_status & IR_SWINT0)) {
		fmr_vector = FMTRX_VECTOR_SWINT0;
	} else if (0 != (irq_status & IR_SWINT1)) {
		fmr_vector = FMTRX_VECTOR_SWINT1;
	} else if (0 != (irq_status & IR_SWINT2)) {
		fmr_vector = FMTRX_VECTOR_SWINT2;
	} else if (0 != (irq_status & IR_SWINT3)) {
		fmr_vector = FMTRX_VECTOR_SWINT3;
	} else if (0 != (irq_status & IR_SWINT4)) {
		fmr_vector = FMTRX_VECTOR_SWINT4;
	} else {
		err = -EINVAL;
		fmtrx_sys_log
		 ("%s: %s %d, Invalid irq_status %d type!\n",
		 FILE, __func__,
		 __LINE__, irq_status);
		goto fmtrx_plat_interrupt_handler_exit;
	}

	/* Save it to be used in the IRQ task */
	fmr_vector_type = fmr_vector;

	/* Wake up the IRQ processing task */
	complete(&msg_process_sem);

 fmtrx_plat_interrupt_handler_exit:
	return err;
}

/*
** =============================================================================
**
**				HCI INTERFACES
**
** =============================================================================
*/
int fmr_convert_to_le16(
		u16 value,
		u8 *data,
		u32 offset)
{
	int err = 0;

	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_convert_to_le16_exit;
	}

	data[offset + 1] = (value & 0xff00) >> 8;
	data[offset] = value & 0x00ff;

 fmr_convert_to_le16_exit:
	return err;
}

int fmr_convert_to_le32(
		u32 value,
		u8 *data,
		u32 offset)
{
	int err = 0;

	if (0 == data) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_convert_to_le32_exit;
	}

	data[offset + 3] = (value & 0xff000000) >> 24;
	data[offset + 2] = (value & 0xff0000) >> 16;
	data[offset + 1] = (value & 0xff00) >> 8;
	data[offset] = value & 0x00ff;

 fmr_convert_to_le32_exit:
	return err;
}

int fmr_validate_access_mode(
		u8 mode,
		u8 length)
{
	int err = 0;

	switch (mode) {
	case FMR_HCI_8BIT_ACCESS_MODE:
		if (length % 1)
			err = -EINVAL;
		break;
	case FMR_HCI_16BIT_ACCESS_MODE:
		if (length % 2)
			err = -EINVAL;
		break;
	case FMR_HCI_32BIT_ACCESS_MODE:
		if (length % 4)
			err = -EINVAL;
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

/*------------------------------------------------------------------------------
 * Function...: fmr_irq_confirm
 * Returns....: None
 * Description: Sends IRQ confirm command
 * Created....: 18.10.2013 by chandras
 * Modified...: DD.MM.YYYY by NNN
------------------------------------------------------------------------------*/
static int fmr_irq_confirm(
		void)
{
	int err = 0;
	struct hci_intel_cmd_pkt cmd_pkt;

	memset(&cmd_pkt, 0, sizeof(struct hci_intel_cmd_pkt));

	cmd_pkt.cmd_hdr1.opcode = OPCODE(0x3F, 0x5C);
	cmd_pkt.cmd_hdr2.param_tot_len = FMR_HCI_IRQ_HDR2_TOT_PARAM_LEN;

	/* Wait for lock to be acquired */
	mutex_lock(&cmd_sync_lock);

	/* Assemble HCI irq command and send */
	err = fmr_hci_cmd_assembly(&cmd_pkt);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, Send command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_irq_confirm_exit;
	}
#ifdef FMR_HOST_TEST
	{
		/* Simulation */
		struct sk_buff skb;
		/* Allocate size of event packet + H4 header*/
		u8 *evt_buff =
		      kzalloc(sizeof(struct hci_intel_evt_pkt) + 1,
				  GFP_KERNEL);
		struct hci_intel_evt_pkt *p_hci_evt =
		    (struct hci_intel_evt_pkt *)(evt_buff + 1);

		evt_buff[0] = FMR_HCI_H4_HDR_EVENT_PKT;
		p_hci_evt->evt_hdr1.evtcode = HCI_INTEL_FMR_EVT_CMD_COMPLETE;
		p_hci_evt->evt_hdr2.param_tot_len =
		    FMR_HCI_EVT_HDR2_TOT_PARAM_LEN;
		p_hci_evt->evt_hdr2.num_hci_cmd_pkts = 0;
		p_hci_evt->evt_hdr2.opcode = HCI_INTEL_FMR_IRQ_CONFIRM;
		p_hci_evt->evt_hdr2.status = HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS;

		skb.data = evt_buff;
		skb.len =
		    sizeof(struct hci_intel_evt_pkt) + 1 -
		    sizeof(struct hci_intel_read_evt);
		fmr_plat_hci_event_handler(0, &skb);
		kfree(evt_buff);
	}
#endif

	/* Wait for Write command complete */
	if (!wait_for_completion_timeout
	    (&irq_cmd_sem, msecs_to_jiffies(FMR_HCI_CMD_TIMEOUT))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d,Wait for command complete event timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_irq_confirm_exit;
	} else {
		if (HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS !=
		    g_hci_evt.evt_hdr2.status) {
			err = -EIO;
			fmtrx_sys_log
			("%s: %s %d,Command complete event failed! Status %d\n",
				FILE, __func__,
				__LINE__, g_hci_evt.evt_hdr2.status);
			goto fmr_irq_confirm_exit;
		}
	}

 fmr_irq_confirm_exit:
	/* Release the lock */
	mutex_unlock(&cmd_sync_lock);
	return err;
}

static int fmr_generic_read(
		u32 addr_offs,
		 u8 *data,
		 u16 size,
		 enum bit_type type,
		 enum access_type access)
{
	int err = 0;
	struct hci_intel_cmd_pkt cmd_pkt;

	/* Validate input arguments */
	if ((0 == data) || (FMR_HCI_READ_MAX_DATA_LEN_SIZE < size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_generic_read_exit;
	}

	memset(&cmd_pkt, 0, sizeof(struct hci_intel_cmd_pkt));

	cmd_pkt.cmd_hdr1.opcode =
	    (FMRIP == access) ? OPCODE(0x3F, 0x59) : OPCODE(0x3F, 0x5E);
	cmd_pkt.cmd_hdr2.param_tot_len = FMR_HCI_READ_HDR2_TOT_PARAM_LEN;
	cmd_pkt.cmd_hdr2.params.fmr_hci_read_cmd.addr_off = addr_offs;
	cmd_pkt.cmd_hdr2.params.fmr_hci_read_cmd.mode =
	    (WIDTH_16BIT ==
	     type) ? FMR_HCI_16BIT_ACCESS_MODE : FMR_HCI_32BIT_ACCESS_MODE;
	cmd_pkt.cmd_hdr2.params.fmr_hci_read_cmd.length = size;

	/* Wait for lock to be acquired */
	mutex_lock(&cmd_sync_lock);

	/* Assemble HCI read command and send */
	err = fmr_hci_cmd_assembly(&cmd_pkt);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, Send command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_generic_read_exit1;
	}
#ifdef FMR_HOST_TEST
	{
		/* Simulation */
		struct sk_buff skb;
		/* Allocate size of event packet + H4 header */
		u8 *evt_buff =
		      kzalloc(sizeof(struct hci_intel_evt_pkt) + 1,
				  GFP_KERNEL);
		struct hci_intel_evt_pkt *p_hci_evt =
		    (struct hci_intel_evt_pkt *)(evt_buff + 1);

		evt_buff[0] = FMR_HCI_H4_HDR_EVENT_PKT;
		p_hci_evt->evt_hdr1.evtcode = HCI_INTEL_FMR_EVT_CMD_COMPLETE;
		p_hci_evt->evt_hdr2.param_tot_len =
		    FMR_HCI_EVT_HDR2_TOT_PARAM_LEN;
		p_hci_evt->evt_hdr2.num_hci_cmd_pkts = 0;
		p_hci_evt->evt_hdr2.opcode = HCI_INTEL_FMR_READ;
		p_hci_evt->evt_hdr2.status = HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS;

		p_hci_evt->evt_hdr2.params.fmr_hci_read_evt.addr_off =
		    addr_offs;
		*((u32 *) &p_hci_evt->evt_hdr2.params.fmr_hci_read_evt.data) =
		    0x0;

		skb.data = evt_buff;
		skb.len =
		    sizeof(struct hci_intel_evt_pkt) -
		    FMR_HCI_READ_MAX_DATA_LEN_SIZE + size;
		fmr_plat_hci_event_handler(0, &skb);
		kfree(evt_buff);
	}
#endif

	/* Wait for event completion */
	if (!wait_for_completion_timeout
	    (&read_cmd_sem, msecs_to_jiffies(FMR_HCI_CMD_TIMEOUT))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d,Wait for command complete event timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_generic_read_exit1;
	} else {
		if ((HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS ==
		     g_hci_evt.evt_hdr2.status)
		    && (addr_offs ==
			g_hci_evt.evt_hdr2.params.fmr_hci_read_evt.addr_off)) {
			memcpy(data,
			       g_hci_evt.evt_hdr2.params.fmr_hci_read_evt.data,
			       size);
		} else {
			err = -EIO;
			fmtrx_sys_log
			("%s: %s %d,Command complete event failed! Status %d\n",
			FILE, __func__,
			__LINE__, g_hci_evt.evt_hdr2.status);
			goto fmr_generic_read_exit1;
		}
	}

 fmr_generic_read_exit1:
	/* Release the lock */
	mutex_unlock(&cmd_sync_lock);
 fmr_generic_read_exit:
	return err;
}

static int fmr_generic_write(
		u32 addr_offs,
		const u8 *data,
		u16 size,
		enum bit_type type,
		enum access_type access)
{
	int err = 0, idx = 0;
	struct hci_intel_cmd_pkt cmd_pkt;
	u8 cmd_data[FMR_HCI_WRITE_MAX_DATA_LEN_SIZE] = { 0 };

	if ((0 == data) || (FMR_HCI_WRITE_MAX_DATA_LEN_SIZE < size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_generic_write_exit;
	}

	memset(&cmd_pkt, 0, sizeof(struct hci_intel_cmd_pkt));

	cmd_pkt.cmd_hdr1.opcode =
	    (FMRIP == access) ? OPCODE(0x3F, 0x58) : OPCODE(0x3F, 0x5D);
	cmd_pkt.cmd_hdr2.param_tot_len =
	    FMR_HCI_WRITE_HDR2_TOT_PARAM_LEN + size;
	cmd_pkt.cmd_hdr2.params.fmr_hci_write_cmd.addr_off = addr_offs;
	cmd_pkt.cmd_hdr2.params.fmr_hci_write_cmd.mode =
	    (WIDTH_16BIT ==
	     type) ? FMR_HCI_16BIT_ACCESS_MODE : FMR_HCI_32BIT_ACCESS_MODE;
	cmd_pkt.cmd_hdr2.params.fmr_hci_write_cmd.length = size;

	for (idx = 0; idx < size / sizeof(u16); idx++) {
		fmr_convert_to_le16(*((u16 *) (data + (idx * 2))), cmd_data,
				    idx * 2);
	}
	cmd_pkt.cmd_hdr2.params.fmr_hci_write_cmd.data = (u8 *) cmd_data;

	/* Wait for lock to be acquired */
	mutex_lock(&cmd_sync_lock);

	/* Assemble HCI write command and send */
	err = fmr_hci_cmd_assembly(&cmd_pkt);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, Send command failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_generic_write_exit1;
	}
#ifdef FMR_HOST_TEST
	{
		/* Simulation */
		struct sk_buff skb;
		/* Allocate size of event packet + H4 header */
		u8 *evt_buff =
		     kzalloc(sizeof(struct hci_intel_evt_pkt) + 1,
				 GFP_KERNEL);
		struct hci_intel_evt_pkt *p_hci_evt =
		    (struct hci_intel_evt_pkt *)(evt_buff + 1);

		evt_buff[0] = FMR_HCI_H4_HDR_EVENT_PKT;
		p_hci_evt->evt_hdr1.evtcode = HCI_INTEL_FMR_EVT_CMD_COMPLETE;
		p_hci_evt->evt_hdr2.param_tot_len =
		    FMR_HCI_EVT_HDR2_TOT_PARAM_LEN;
		p_hci_evt->evt_hdr2.num_hci_cmd_pkts = 0;
		p_hci_evt->evt_hdr2.opcode = HCI_INTEL_FMR_WRITE;
		p_hci_evt->evt_hdr2.status = HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS;

		skb.data = evt_buff;
		skb.len =
		    sizeof(struct hci_intel_evt_pkt) + 1 -
		    sizeof(struct hci_intel_read_evt);
		fmr_plat_hci_event_handler(0, &skb);
		kfree(evt_buff);
	}
#endif

	/* Wait for Write command complete */
	if (!wait_for_completion_timeout
	    (&write_cmd_sem, msecs_to_jiffies(FMR_HCI_CMD_TIMEOUT))) {
		err = -ETIMEDOUT;
		fmtrx_sys_log
		("%s: %s %d,Wait for command complete event timeout! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_generic_write_exit1;
	} else {
		if (HCI_INTEL_FMR_CMD_COMPLETE_SUCCESS !=
		    g_hci_evt.evt_hdr2.status) {
			err = -EIO;
			fmtrx_sys_log
			("%s: %s %d,Command complete event failed! Status %d\n",
				FILE, __func__,
				__LINE__, g_hci_evt.evt_hdr2.status);
			goto fmr_generic_write_exit1;
		}
	}

 fmr_generic_write_exit1:
	/* Release the lock */
	mutex_unlock(&cmd_sync_lock);
 fmr_generic_write_exit:
	return err;
}

static int fmr_hci_send_cmd(
		u8 *data,
		u16 size)
{
	int err = 0;

#if !defined(FMR_HOST_TEST) && defined(LD_DRIVER)
	if (0 != ld_driver.fm_cmd_write) {
		long no_of_bytes = 0;
		struct sk_buff *skb = alloc_skb(255, GFP_ATOMIC);

		/* Validate the allocation */
		if (0 == skb) {
			err = -ENOMEM;
			fmtrx_sys_log
			("%s: %s %d,SK buffer allocation failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmr_hci_send_cmd_exit;
		}

		/* Copy command buffer to sk_buff data pointer */
		memcpy(skb_put(skb, size), data, size);
		no_of_bytes = (int)ld_driver.fm_cmd_write(skb);
		if (0 > no_of_bytes) {
			fmtrx_sys_log
			("%s: %s %d,LD driver write failed! %ld\n",
				FILE, __func__,
				__LINE__, no_of_bytes);
		}
	}
#endif

#ifdef FMR_DEBUG_MEAS
	{
		int idx;
		char *ptr = kzalloc(1024, GFP_KERNEL);
		if (0 == ptr) {
			err = -ENOMEM;
			fmtrx_sys_log
			("%s: %s %d,Trace buffer alloc failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmr_hci_send_cmd_exit;
		}

		for (idx = 0; idx < size; idx++)
			snprintf((ptr + (idx * 3)), 10, "%02x ", data[idx]);
		fmtrx_sys_log("FMR <send>: %s\n", ptr);
		g_total_sent += size;
		kfree(ptr);
	}
#endif

 fmr_hci_send_cmd_exit:
	return err;
}

static int fmr_hci_write_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size)
{
	int err = 0;
	int index = 0;

	/* Validate input arguments */
	if ((0 == fmr_hci_cmd) || (0 == data) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
				FILE, __func__);
		goto fmr_hci_write_cmd_assembly_exit;
	}

	/* Store the command header 2, Parameter Total Len field */
	data[FMR_HCI_CMD_HDR2_TOT_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.param_tot_len;

	/* Store the command header 2, Address field */
	err =
	    fmr_convert_to_le32(fmr_hci_cmd->cmd_hdr2.params.fmr_hci_write_cmd.
				addr_off, data, FMR_HCI_CMD_HDR2_ADDR_POS);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,LE conversion failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		goto fmr_hci_write_cmd_assembly_exit;
	}

	err =
	    fmr_validate_access_mode(fmr_hci_cmd->cmd_hdr2.params.
				     fmr_hci_write_cmd.mode,
				     fmr_hci_cmd->cmd_hdr2.params.
				     fmr_hci_write_cmd.length);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,Access mode/length validation failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_hci_write_cmd_assembly_exit;
	}

	/* Store the command header 2, Mode field */
	data[FMR_HCI_CMD_HDR2_MODE_POS] =
	    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_write_cmd.mode;

	/* Store the command header 2, Length field */
	data[FMR_HCI_CMD_HDR2_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_write_cmd.length;

	for (index = 0;
	     index < fmr_hci_cmd->cmd_hdr2.params.fmr_hci_write_cmd.length;
	     index++) {
		data[FMR_HCI_CMD_DATA_POS + index] =
		    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_write_cmd.data[index];
	}

	/* Write the command */
	err = fmr_hci_send_cmd(data, size);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,HCI Send command failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		goto fmr_hci_write_cmd_assembly_exit;
	}

 fmr_hci_write_cmd_assembly_exit:
	return err;
}

static int fmr_hci_read_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size)
{
	int err = 0;

	/* Validate input arguments */
	if ((0 == fmr_hci_cmd) || (0 == data) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
				FILE, __func__);
		goto fmr_hci_read_cmd_assembly_exit;
	}

	/* Store the command header 2, Parameter Total Len field */
	data[FMR_HCI_CMD_HDR2_TOT_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.param_tot_len;

	/* Store the command header 2, Address field */
	err =
	    fmr_convert_to_le32(fmr_hci_cmd->cmd_hdr2.params.fmr_hci_read_cmd.
				addr_off, data, FMR_HCI_CMD_HDR2_ADDR_POS);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d,LE conversion failed! %d\n",
				FILE, __func__,
				__LINE__, err);
		goto fmr_hci_read_cmd_assembly_exit;
	}

	err =
	    fmr_validate_access_mode(fmr_hci_cmd->cmd_hdr2.
				     params.fmr_hci_read_cmd.mode,
				     fmr_hci_cmd->cmd_hdr2.
				     params.fmr_hci_read_cmd.length);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Access mode/length validation failed! %d\n",
		FILE, __func__,
		__LINE__, err);
		goto fmr_hci_read_cmd_assembly_exit;
	}

	/* Store the command header 2, Mode field */
	data[FMR_HCI_CMD_HDR2_MODE_POS] =
	    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_read_cmd.mode;

	/* Store the command header 2, Length field */
	data[FMR_HCI_CMD_HDR2_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_read_cmd.length;

	/* Write the command */
	err = fmr_hci_send_cmd(data, size);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,HCI Send command failed! %d\n",
		FILE, __func__,
		__LINE__, err);
		goto fmr_hci_read_cmd_assembly_exit;
	}

 fmr_hci_read_cmd_assembly_exit:
	return err;
}

static int fmr_hci_power_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size)
{
	int err = 0;

	/* Validate input arguments */
	if ((0 == fmr_hci_cmd) || (0 == data) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_hci_power_cmd_assembly_exit;
	}

	/* Store the command header 2, Parameter Total Len field */
	data[FMR_HCI_CMD_HDR2_TOT_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.param_tot_len;

	/* Store the command header 2, Pow enable/disable */
	data[FMR_HCI_CMD_HDR2_ADDR_POS] =
	    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_pow_cmd.pow_en;

	/* Write the command */
	err = fmr_hci_send_cmd(data, size);
	if (0 != err) {
		fmtrx_sys_log
		    ("%s: %s %d, HCI Send command failed! %d\n",
		     FILE, __func__,
		     __LINE__, err);
		goto fmr_hci_power_cmd_assembly_exit;
	}

 fmr_hci_power_cmd_assembly_exit:
	return err;
}

static int fmr_hci_audio_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size)
{
	int err = 0;

	/* Validate input arguments */
	if ((0 == fmr_hci_cmd) || (0 == data) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_hci_audio_cmd_assembly_exit;

	}

	/* Store the command header 2, Parameter Total Len field */
	data[FMR_HCI_CMD_HDR2_TOT_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.param_tot_len;

	/* Store the command header 2, Audio enable/disable */
	data[FMR_HCI_CMD_HDR2_ADDR_POS] =
	    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_aud_cmd.aud_en;

	/* Write the command */
	err = fmr_hci_send_cmd(data, size);
	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, HCI Send command failed! %d\n",
			FILE, __func__, __LINE__, err);
		goto fmr_hci_audio_cmd_assembly_exit;
	}

 fmr_hci_audio_cmd_assembly_exit:
	return err;
}

static int fmr_hci_irq_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data,
		u16 size)
{
	int err = 0;

	/* Validate input arguments */
	if ((0 == fmr_hci_cmd) || (0 == data) || (0 == size)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_hci_irq_cmd_assembly_exit;
	}

	/* Store the command header 2, Parameter Total Len field */
	data[FMR_HCI_CMD_HDR2_TOT_LEN_POS] =
	    fmr_hci_cmd->cmd_hdr2.param_tot_len;

	/* Write the command */
	err = fmr_hci_send_cmd(data, size);
	if (0 != err) {
		fmtrx_sys_log
		   ("%s: %s %d, HCI Send command failed! %d\n",
		    FILE, __func__, __LINE__, err);
		goto fmr_hci_irq_cmd_assembly_exit;
	}

 fmr_hci_irq_cmd_assembly_exit:
	return err;
}

static int fmr_hci_store_cmdhdr1(
		struct hci_intel_cmd_pkt *fmr_hci_cmd,
		u8 *data)
{
	int err = 0;

	/* Validate input arguments */
	if ((0 == fmr_hci_cmd) || (0 == data)) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			  FILE, __func__);
		goto fmr_hci_store_cmdhdr1_exit;
	}

	/* Store the packet type */
	data[FMR_HCI_CMD_PKT_TYPE_POS] = FMR_HCI_H4_HDR_COMMAND_PKT;

	/* Store the command header 1 */
	err =
	    fmr_convert_to_le16(fmr_hci_cmd->cmd_hdr1.opcode, data,
				FMR_HCI_CMD_HDR1_OP_POS);
	if (0 != err) {
		fmtrx_sys_log
		  ("%s: %s %d, LE conversion failed! %d\n",
		  FILE, __func__, __LINE__, err);
	}

 fmr_hci_store_cmdhdr1_exit:
	return err;
}

int fmr_hci_cmd_assembly(
		struct hci_intel_cmd_pkt *fmr_hci_cmd)
{
	int err = 0;
	u8 *data = 0;
	u16 total_cmd_size = 0;

	/* Validate input arguments */
	if (0 == fmr_hci_cmd) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			 FILE, __func__);
		goto fmr_hci_cmd_assembly_exit;
	}

	switch (fmr_hci_cmd->cmd_hdr1.opcode) {
	case HCI_INTEL_FMR_WRITE:
		/* Intended fallthrough */
	case HCI_INTEL_FMR_TOP_WRITE:
		total_cmd_size =
		    FMR_HCI_CMD_LEN_EXCEPT_DATA +
		    fmr_hci_cmd->cmd_hdr2.params.fmr_hci_write_cmd.length;
		break;
	case HCI_INTEL_FMR_READ:
		/* Intended fallthrough */
	case HCI_INTEL_FMR_TOP_READ:
		total_cmd_size = FMR_HCI_READ_CMD_LEN;
		break;
	case HCI_INTEL_FMR_POWER:
		total_cmd_size = FMR_HCI_POWER_CMD_LEN;
		break;
	case HCI_INTEL_FMR_AUDIO:
		total_cmd_size = FMR_HCI_AUDIO_CMD_LEN;
		break;
	case HCI_INTEL_FMR_IRQ_CONFIRM:
		total_cmd_size = FMR_HCI_CONFIRM_IRQ_CMD_LEN;
		break;
	default:
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s %d, LE conversion failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		break;
	}

	if (0 != err) {
		fmtrx_sys_log
			("%s: %s %d, Invalid opcode! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_hci_cmd_assembly_exit;
	}

	/* Allocate command buffer */
	data = kzalloc(total_cmd_size, GFP_KERNEL);
	if (0 == data) {
		err = -ENOMEM;
		fmtrx_sys_log
		("%s: %s %d,Command buffer allocation failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmr_hci_cmd_assembly_exit;
	}

	/* Fill the H4 header in the buffer */
	err = fmr_hci_store_cmdhdr1(fmr_hci_cmd, data);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Store command header failed! %d\n",
		FILE, __func__,
		__LINE__, err);
		goto fmr_hci_cmd_assembly_exit;
	}

	switch (fmr_hci_cmd->cmd_hdr1.opcode) {
	case HCI_INTEL_FMR_WRITE:
	case HCI_INTEL_FMR_TOP_WRITE:
		err =
		    fmr_hci_write_cmd_assembly(fmr_hci_cmd, data,
					       total_cmd_size);
		break;
	case HCI_INTEL_FMR_READ:
	case HCI_INTEL_FMR_TOP_READ:
		err =
		    fmr_hci_read_cmd_assembly(fmr_hci_cmd, data,
					      total_cmd_size);
		break;
	case HCI_INTEL_FMR_POWER:
		err =
		    fmr_hci_power_cmd_assembly(fmr_hci_cmd, data,
					       total_cmd_size);
		break;
	case HCI_INTEL_FMR_AUDIO:
		err =
		    fmr_hci_audio_cmd_assembly(fmr_hci_cmd, data,
					       total_cmd_size);
		break;
	case HCI_INTEL_FMR_IRQ_CONFIRM:
		err =
		    fmr_hci_irq_cmd_assembly(fmr_hci_cmd, data, total_cmd_size);
		break;
	default:
		break;
	}

	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Command assemby failed! %d\n",
		FILE, __func__,
		__LINE__, err);
		goto fmr_hci_cmd_assembly_exit;
	}

 fmr_hci_cmd_assembly_exit:
	if (0 != data)
		kfree(data);
	return err;
}

static long fmr_plat_hci_event_handler(
		void *arg,
		struct sk_buff *skb)
{
	int err = 0;
	struct hci_intel_evt_pkt *evt_params = 0;
	u8 *evt_pkt = 0;

	/* Validate input arguments */
	if (!((0 != skb) && (0 != skb->data) && (0 != skb->len))) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid arguments!\n",
			FILE, __func__);
		goto fmr_plat_hci_event_handler_exit;
	}
#ifdef FMR_DEBUG_MEAS
	{
		int idx;
		char *ptr = kzalloc(1024, GFP_KERNEL);
		if (0 == ptr) {
			err = -ENOMEM;
			fmtrx_sys_log
			("%s: %s %d,Trace buffer alloc failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmr_plat_hci_event_handler_exit;
		}

		for (idx = 0; idx < skb->len; idx++)
			snprintf((ptr + (idx * 3)), 10, "%02x ",
				skb->data[idx]);
		fmtrx_sys_log("FMR <recv>: %s\n", ptr);
		g_total_recv += skb->len;
		kfree(ptr);
	}
#endif

	/* Get the data packet */
	evt_pkt = skb->data;

	/* Validate H4 Header */
	if (FMR_HCI_H4_HDR_EVENT_PKT != *evt_pkt) {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid event packet!\n",
			FILE, __func__);
		goto fmr_plat_hci_event_handler_exit;
	}

	/* Strip the H4 header */
	evt_params = (struct hci_intel_evt_pkt *)(evt_pkt + 1);

	if (HCI_INTEL_FMR_EVT_CMD_COMPLETE == evt_params->evt_hdr1.evtcode) {
		/* Copy to the global event structure */
		memset((u8 *) &g_hci_evt, 0, sizeof(g_hci_evt));
		g_hci_evt.evt_hdr1.evtcode = evt_params->evt_hdr1.evtcode;
		g_hci_evt.evt_hdr2.param_tot_len =
		    evt_params->evt_hdr2.param_tot_len;
		g_hci_evt.evt_hdr2.num_hci_cmd_pkts =
		    evt_params->evt_hdr2.num_hci_cmd_pkts;
		g_hci_evt.evt_hdr2.opcode = evt_params->evt_hdr2.opcode;
		g_hci_evt.evt_hdr2.status = evt_params->evt_hdr2.status;

		switch (evt_params->evt_hdr2.opcode) {
		case HCI_INTEL_FMR_WRITE:
		case HCI_INTEL_FMR_TOP_WRITE:
			complete(&write_cmd_sem);
			break;
		case HCI_INTEL_FMR_READ:
		case HCI_INTEL_FMR_TOP_READ:
			g_hci_evt.evt_hdr2.params.fmr_hci_read_evt.addr_off =
			  evt_params->evt_hdr2.params.fmr_hci_read_evt.addr_off;
			memcpy(&g_hci_evt.evt_hdr2.params.fmr_hci_read_evt.data,
			   &evt_params->evt_hdr2.params.fmr_hci_read_evt.data,
			   (g_hci_evt.evt_hdr2.param_tot_len -
				FMR_HCI_EVT_HDR2_TOT_PARAM_LEN +
				FMR_HCI_EVT_HDR2_ADDR_SIZE));
			complete(&read_cmd_sem);
			break;
		case HCI_INTEL_FMR_POWER:
			complete(&power_cmd_sem);
			break;
		case HCI_INTEL_FMR_AUDIO:
			complete(&audio_cmd_sem);
			break;
		case HCI_INTEL_FMR_IRQ_CONFIRM:
			complete(&irq_cmd_sem);
			break;
		default:
			err = -EINVAL;
			fmtrx_sys_log
			    ("%s: %s, Invalid event opcode!\n",
			     FILE, __func__);
			break;
		}
	} else if ((HCI_INTEL_FMR_EVT_INTERRUPT ==
				evt_params->evt_hdr1.evtcode) &&
				(HCI_INTEL_FMR_EVT_INTERRUPT_EVTID ==
				evt_params->evt_hdr2.num_hci_cmd_pkts)) {
				/* Evt ID is stored in num_hci_cmd_pkts */
#ifdef FMR_INTR_MODE
		err =
		  fmtrx_plat_interrupt_handler(evt_params->evt_hdr2.opcode, 0);
		if (0 != err) {
			fmtrx_sys_log
			("%s: %s %d,Interrupt handler failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			goto fmr_plat_hci_event_handler_exit;
		}
#endif
	} else {
		err = -EINVAL;
		fmtrx_sys_log
			("%s: %s, Invalid event code!\n",
			  FILE, __func__);
	}

 fmr_plat_hci_event_handler_exit:
	return err;
}

static void fmtrx_plat_reg_complete_cb(
	void *arg,
	char data)
{
	complete(&event_sem);
}

#ifdef FMR_INTR_MODE
static int msg_process_task_run(
		void *args)
{
	int err = 0;
	bool irq_processed = false;

	while (!msg_process_task_stop) {
		/* Set irq processed flag to false */
		irq_processed = false;

		/* Wait for interrupt */
		if (!wait_for_completion_interruptible(&msg_process_sem)) {
#ifdef FMR_DEBUG_LVL2
			fmtrx_sys_log
				("%s: %s %d,Msg process thread interrupted!\n",
				FILE, __func__,
				__LINE__);
#endif
		}

		if (msg_process_task_stop == true) {
#ifdef FMR_DEBUG_LVL2
			fmtrx_sys_log
			 ("%s: %s %d, Msg processing thread exiting!\n",
			 FILE, __func__,
			 __LINE__);
#endif
			do_exit(0);
		}

		/* Skip spurious interrupts by checking the irq_active flag */
		if (irq_active) {
#ifdef FMR_DEBUG_LVL2
			fmtrx_sys_log
			 ("%s: %s %d, Interrupt type %d received!\n",
			  FILE, __func__,
			  __LINE__, fmr_vector_type);
#endif
			/* Process interrupts */
			switch (fmr_vector_type) {
			case FMTRX_VECTOR_DED0:
			case FMTRX_VECTOR_DED1:
			case FMTRX_VECTOR_SWINT1:
				fmtrx_sys_wakeup_event();
				irq_processed = true;
				break;
			default:
				break;
			}

		}

		if (!irq_processed) {
#ifdef FMR_DEBUG_LVL2
			fmtrx_sys_log
			 ("%s: %s %d,Spurious interrupt received!\n",
				FILE, __func__,
				__LINE__);
#endif
			/* Send IRQ confirm for the spurious
				or unintended interrupts */
			err = fmr_irq_confirm();
			if (0 != err) {
				fmtrx_sys_log
				("%s: %s %d, Send IRQ confirm cmd failed! %d\n",
				FILE, __func__,
				__LINE__, err);
			}
		}
	}

#ifdef FMR_DEBUG_LVL2
	fmtrx_sys_log
	("%s: %s %d,Msg processing thread exiting!\n",
		FILE, __func__,
		__LINE__);
#endif

	return err;
}
#endif

/*
** =============================================================================
**
**				LOCAL FUNCTION DEFINITIONS
**
** =============================================================================
*/
#ifdef CONFIG_IUI_FM_FMR
static void fmtrx_denotify_mitigation_interference(void)
{
	struct iui_fm_fmr_info freq_mgr_fmrinfo;
	struct iui_fm_freq_notification freq_mgr_notify_data;

	freq_mgr_fmrinfo.rx_freq = 0;
	freq_mgr_fmrinfo.inj_side = 0;

	freq_mgr_notify_data.type = IUI_FM_FREQ_NOTIFICATION_TYPE_FMR;
	freq_mgr_notify_data.info.fmr_info = &freq_mgr_fmrinfo;

	fmtrx_sys_log
		("%s: %s %d,FM denotification err code %d\n",
			FILE, __func__,
			__LINE__,
		iui_fm_notify_frequency(IUI_FM_MACRO_ID_FMR,
					&freq_mgr_notify_data));
}
#endif /* CONFIG_IUI_FM_FMR */

static int fmtrx_sys_request_dcdc(
	bool enable)
{
	int err = 0;
	u32 data = 0;

	err =
		fmr_generic_read(FMR_DCDC_REG_OFFSET, (u8 *) &data,
		FMR_HCI_READ32_DATA_LEN_SIZE, WIDTH_32BIT, TOP);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d, Generic read 16-bit failed! %d\n",
		FILE, __func__,
		__LINE__, err);
	}

	if (enable) {
		data = (data &
			~(((1 << FMR_DCDC_REGISTER_WIDTH) - 1)
			<< FMR_DCDC_REGISTER_BIT)) |
			(FMR_DCDC_ACTIVE_HIGH_VOLTAGE << FMR_DCDC_REGISTER_BIT);
	} else {
		data = (data &
			~(((1 << FMR_DCDC_REGISTER_WIDTH) - 1)
			<< FMR_DCDC_REGISTER_BIT)) |
			(FMR_DCDC_DEFAULT_VOLTAGE << FMR_DCDC_REGISTER_BIT);
	}

	err =
		fmr_generic_write(FMR_DCDC_REG_OFFSET, (u8 *) &data,
		FMR_HCI_READ32_DATA_LEN_SIZE, WIDTH_32BIT, TOP);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d, Generic write 32-bit failed! %d\n",
		FILE, __func__,
		__LINE__, err);
	}

	return err;
}

/*
** =============================================================================
**
**				MODULE INTERFACES
**
** =============================================================================
*/

static struct of_device_id intel_fmradio_of_match[] = {
	{.compatible = "intel,fm-radio",},
	{},
};

static struct platform_driver fmtrx_platform_driver = {
	.driver = {
		   .name = FMTRX_MODULE_NAME,
		   .of_match_table = intel_fmradio_of_match,
		   },
	.probe = fmtrx_driver_probe,
	.remove = fmtrx_driver_remove,
};

#ifdef CONFIG_IUI_FM_FMR
/* FM mitigation callback */
enum iui_fm_mitigation_status fmr_fm_mitigation_cb(
	const enum iui_fm_macro_id macro_id,
	const struct iui_fm_mitigation *mitigation, const uint32_t sequence)
{
	int err = 0;
	enum injection_side inj_side = INJECTION_SIDE_INVALID;
	enum iui_fm_mitigation_status mitigation_status =
		IUI_FM_MITIGATION_ERROR_INVALID_PARAM;

	if ((NULL == mitigation) || (IUI_FM_MACRO_ID_FMR != macro_id) ||
	    (IUI_FM_MITIGATION_TYPE_FMR != mitigation->type))
		return mitigation_status;

	inj_side = (IUI_FM_FMR_INJECTION_SIDE_LOW ==
			mitigation->info.fmr_inj_side) ?
			INJECTION_SIDE_LSI : INJECTION_SIDE_HSI;

	fmtrx_sys_log
		("%s: %s %d, FM mitigation cb - change inj side to %d\n",
			FILE, __func__,
			__LINE__,
			inj_side);

	/* set injection side */
	err = fmrx_set_sideband(inj_side, false);
	if (err != 0) {
		fmtrx_sys_log
			("%s: %s %d, failed to set inj side : %d, err: %d\n",
				FILE, __func__,
				__LINE__,
				inj_side, err);
		mitigation_status = IUI_FM_MITIGATION_ERROR;
	} else {
		fmtrx_sys_log
			("%s: %s %d, inj side: %u, changed successfully\n",
				FILE, __func__,
				__LINE__,
				inj_side);
		mitigation_status = IUI_FM_MITIGATION_COMPLETE_OK;
	}

	return mitigation_status;
}
#endif

static int fmtrx_driver_probe(
		struct platform_device *p_dev)
{
	int err = 0;

	/* Initialize V4L2 core driver */
	err = fmtrx_v4l2_init(&p_dev->dev, FMTRX_MODULE_SELECT);
	if (0 != err) {
		err = -EINVAL;
		fmtrx_sys_log
		("%s: %s %d, V4L2 driver registration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		goto fmtrx_driver_probe_exit;
	}

#ifdef CONFIG_IUI_FM_FMR
	/* Register with Frequency manager */
	err = iui_fm_register_mitigation_callback(IUI_FM_MACRO_ID_FMR,
			fmr_fm_mitigation_cb);
	if (err != 0) {
		fmtrx_sys_log
			("%s: %s %d, IUI FM registration failed! %d\n",
			FILE, __func__,
			__LINE__, err);
		err = -EIO;
		goto fmtrx_driver_probe_exit;
	}
#endif

	fmtrx_device = &p_dev->dev;
	fmtrx_sys_log
		("%s: Intel's FM Radio Driver Version %s loaded successfully\n",
		FILE, FMTRX_MODULE_VERSION);

 fmtrx_driver_probe_exit:
	return err;
}

static int fmtrx_driver_remove(
		struct platform_device *p_dev)
{
	int err = 0;

	/* Unregister from V4L2 subsystem */
	err = fmtrx_v4l2_deinit(&p_dev->dev, FMTRX_MODULE_SELECT);
	if (0 != err) {
		err = -EINVAL;
		fmtrx_sys_log
		("%s: %s %d,V4L2 driver unregistration failed! %d\n",
		  FILE, __func__,
		  __LINE__, err);
		goto fmtrx_driver_remove_exit;
	}
	/* de-register from Frequency manager */
#ifdef CONFIG_IUI_FM_FMR
	err = iui_fm_register_mitigation_callback(
				IUI_FM_MACRO_ID_FMR,
				NULL);
	fmtrx_sys_log
	("%s: %s %d,Frequency manager notification err code %d\n",
	  FILE, __func__,
	  __LINE__, err);
#endif

	fmtrx_device = 0;
	fmtrx_sys_log
	("%s: Intel's FM Radio Driver Version %s un-loaded successfully\n",
	FILE, FMTRX_MODULE_VERSION);

 fmtrx_driver_remove_exit:
	return err;
}

static int __init fmtrx_module_init(
		void)
{
	int err = 0;

	/* Register FM Radio driver */
	err = platform_driver_register(&fmtrx_platform_driver);
	if (0 != err) {
		fmtrx_sys_log
		("%s: %s %d,Platform driver registration failed! %d\n",
		   FILE, __func__,
		   __LINE__, err);
		goto fmtrx_module_init_exit;
	}

 fmtrx_module_init_exit:
	return err;
}

static void __exit fmtrx_module_exit(
		void)
{
	platform_driver_unregister(&fmtrx_platform_driver);
}

module_init(fmtrx_module_init);
module_exit(fmtrx_module_exit);

/* Module Info */
MODULE_AUTHOR("Intel Mobile Communications");
MODULE_DESCRIPTION
	("V4L2 driver to control FM Radio IP of Intel's LnP chip");
MODULE_VERSION(FMTRX_MODULE_VERSION);
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(of, intel_fmradio_of_match);
/* end of file */
