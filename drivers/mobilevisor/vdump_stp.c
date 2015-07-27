/*
* Copyright (C) 2015 Intel Mobile Communications GmbH
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
*
*/
#include <linux/module.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>


#include <stddef.h>
#include <stdarg.h>
/* #include <stdint.h> */

#include <sofia/pal_sys_exception_types.h>
#include <sofia/pal_coredump_types.h>

#include "vdump_stp.h"
#include "vdump_istp.h"
#include "vdump_file.h"


/*============================================================================
DEFINES AND MACROS
============================================================================*/
#define vd_debug_printf printk

#define VD_ISTP_TRACE_MESSAGE_SIZE  8192

#define VD_TRACE_PORT_USIF          0
#define VD_TRACE_PORT_MIPI          1
#define VD_TRACE_PORT_USB           2

#define VD_STP_TS_NOT_REQUIRED     0
#define VD_STP_TS_REQUIRED         1

#define VD_COREDUMP_PROTOCOL_STP   2
#define VD_COREDUMP_PROTOCOL_ISTP  3

#define VD_STP_BBTP_MIN_HEADER_SIZE 6

#define TADO_MSG_TYPE_CORE_DUMP 0x05
#define TADO_MSG_TYPE_TRAP      0x03
#define TADO_MSG_TYPE_LEGACY_SIO		0x0f

#define SW_TRAP_VECTOR 0xDDDD

enum MT_TRAP {
	/**< TRAP_MSG with trap_exception_type structure (string "TREG") */
	MT_TRAP_EXCEPTION = 0x54524547,
	/**< TRAP_MSG with arm_regs_t structure (string "TXCP")*/
	MT_TRAP_REG       = 0x54584350,
} MT_TRAP;

struct cd_oct_buffer {
	int in_use;
	void *start_ptr;
	void *start_ptr_phy;
	void *readpos_ptr;
	void *writepos_ptr;
	unsigned int size;
	unsigned int bytesInBuf;
};

/* 11.10.2010 AJH: The following defines are for DTM for
	ISTP/STP coredump and are compatible with DTMv2. */
#define VD_DTM_VERSION 2

/* For coredump, it doesn't really matter which of the Frame
	IDs for ASCII text messages we send. => Selecting Dual Mode. */
#define VD_DTM_ASCII_FRAME_ID     0x01
#define VD_DTM_ASCII_LEADIN_SIZE  17

/* 11.10.2010 AJH: The following enum is Generic for DTMv0,1,2. */
/* Definition of the byte order of the start sequence of a DTM frame */
enum {
	enDtmHeaderStart = 0,		/* Start indication     => fix 0xF9  */
	enDtmHeaderVersion,			/* Protocol version     */
	enDtmHeaderFrameId,			/* Extended Frame id    */
	enDtmHeaderCounter,			/* Sequence counter     */
	enDtmHeaderHighLen,			/* High byte of length  */
	enDtmHeaderLowLen,			/* Low byte of length   */
	enDtmHeaderFcs,				/* FCS calculation      */
	enDtmHeaderNum
};

/* DTM Frame */
#define MA_INTRO_CHAR_DTM   0xF9

/* 07.10 declarations.
 */
static const unsigned char fcs_tab[256] = {
	0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C,
	0x09, 0x98, 0xEA, 0x7B, 0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,
	0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A,
	0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
	0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58,
	0x2D, 0xBC, 0xCE, 0x5F, 0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05,
	0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E,
	0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
	0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34,
	0x41, 0xD0, 0xA2, 0x33, 0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21,
	0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 0xE0, 0x71, 0x03, 0x92,
	0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
	0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80,
	0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD,
	0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3, 0xC4, 0x55, 0x27, 0xB6,
	0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
	0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC,
	0x99, 0x08, 0x7A, 0xEB, 0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,
	0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8, 0x39, 0x4B, 0xDA,
	0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
	0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8,
	0xBD, 0x2C, 0x5E, 0xCF
};
struct cd_oct_buffer cd_oct_info;

/*============================================================================
LOCAL FUNCTION PROTOTYPES
============================================================================*/

static void VD_Stp_deinit(void);
static void VD_Stp_send_coredump_header(unsigned int wNumFrames);
static void VD_Stp_send_coredump_terminator(unsigned char bFrameNum);
static int VD_Stp_format_register(
	char *pBuffer,
	int BfrLength,
	struct x86_cpu_regs *regs_ptr);
static void VD_Stp_dump_register(
	char *pBuffer,
	int BfrLength,
	struct x86_cpu_regs *regs_ptr,
	unsigned char bFrameCount);
static void VD_Stp_dump_vm(
	char *pBuffer,
	int BfrLength,
	struct sys_vm_dump *vm_dump_ptr,
	unsigned char bFrameCount);
static void VD_Stp_dump_frame(
	unsigned char bFrameType,
	unsigned char *pFramePhyPtr,
	unsigned char *pFramePtr,
	unsigned int wFrameSize,
	unsigned char bFrameCount);
static void VD_Stp_send_segment(
	unsigned int wMagicNumber,
	unsigned char bMsgType,
	unsigned int wSegmentNumber,
	unsigned char bFramenumber,
	unsigned char *pbContents,
	unsigned int wContentsSize,
	unsigned char *pInfoBlock,
	unsigned int  wInfoSize);
static void VD_Stp_send_header(
	unsigned int  wMagicNumber,
	unsigned char bMsgType,
	unsigned int  wSegmentNumber,
	unsigned char bFramenumber,
	unsigned int  wSize,
	unsigned int  wLast);
static void VD_Stp_tx_data(
	const unsigned char *start,
	unsigned int length,
	unsigned int wLast);
static void VD_Stp_dump_info(
	char *pBuffer,
	int  BfrLength,
	unsigned char bFrameCount,
	unsigned int  wNumFrames,
	unsigned int  nof_areas,
	struct cd_ram *area_ptr,
	struct x86_cpu_regs *regs_ptr,
	struct sys_trap *trap_ptr,
	struct sys_vm_dump *vm_dump_ptr,
	struct cd_oct_buffer *oct_ptr);
/*
static void VD_Stp_send_dtm_frame(
	int frameId,
	const char *pStart,
	int length, int nullHeadLen,
	unsigned char counter);
*/
/* static void VD_Stp_set_CID(unsigned int ccid); */
/* static void VD_Stp_MA_text_info(const char *pStr, int reset_counter); */
static unsigned int VD_Stp_build_cd_header(
	unsigned char *pBuffer,
	unsigned int wBuffersize,
	unsigned char *pFramePtr,
	unsigned int wFrameSize);
static void VD_Stp_send_bbtp_header(int MessageType);
static void VD_Stp_send_trap_frame(
	enum MT_TRAP frameId,
	unsigned char *pData,
	unsigned int length);

static void VD_Istp_dump_pre_mortem_buffer(struct cd_oct_buffer *pOct_buff);
static unsigned int VD_Istp_check_oct_pointers
	(struct cd_oct_buffer *pOct_buff);

/*============================================================================
EXTERNAL VARIABLES
============================================================================*/

/* FCS lookup table as calculated by genfcstab (same as for PPP) */
/* extern const unsigned short lwip_fcstab[256]; */


/*============================================================================
EXTERNAL FUNCTIONS
============================================================================*/
extern int oct_get_buffer_info(
	void **start_address,
	void **start_address_phy,
	unsigned int *size,
	void **read_ptr,
	void **write_ptr);


/*============================================================================
GLOBALS
============================================================================*/
static char TracePort = VD_TRACE_PORT_USIF;
static int Protocol = VD_COREDUMP_PROTOCOL_STP;
/* static void *TadoHandle = NULL; */
static int MsgTypeBBTP = TADO_MSG_TYPE_CORE_DUMP;
char vd_stp_buffer[8096];

/*============================================================================
FUNCTIONS
============================================================================*/

/*****************************************************************************
 * Function:     VD_stp_start
 *-----------------------------------------------------------------------------
 * Description: Called from coredump handler
 *
 * Inputs:  None
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
void VD_stp_start(struct cd_sharemem *cd_info)
{
	/* This function is called only from the trap handler,
	 * interrupts should be disabled at this point.
	 */
	unsigned int tNumFrames = 1;
	unsigned int tFrameCount = 0;
	int					BfrPos = 0;
	struct cd_ram	 *area_ptr = cd_info->memory_range;
	struct sys_trap *trap_ptr = &cd_info->trap_ptr;
	struct x86_cpu_regs *regs_ptr = NULL;
	struct sys_vm_dump *vm_dump_ptr = &cd_info->vm_dump;
#if 1
	struct cd_oct_buffer cd_oct_info_dbg;
#endif

	vdump_open_coredump();

	TracePort = VD_TRACE_PORT_USIF;
	Protocol = VD_COREDUMP_PROTOCOL_ISTP;
	VD_Istp_init(VD_STP_CHID_CORE_DUMP);

	/*	TRAP frames have to be sent before
			dumping the OCT buffer, because:
	 *	 - the OCT buffer may contain broken
			messages (trace tool may show decoding error)
	 *	 - the trap frames signalize that a crash happened
	 */
	if (trap_ptr) {
		VD_Stp_send_trap_frame(MT_TRAP_EXCEPTION,
			(unsigned char *)trap_ptr, sizeof(*trap_ptr));
		regs_ptr = &trap_ptr->regs;
		VD_Stp_send_trap_frame(MT_TRAP_REG,
			(unsigned char *)regs_ptr, sizeof(*regs_ptr));

	}

	/* dump out the ISTP pre-mortem traces */
	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP) {

		cd_oct_info.in_use =
			oct_get_buffer_info(
				&cd_oct_info.start_ptr,
				&cd_oct_info.start_ptr_phy,
				&cd_oct_info.size,
				&cd_oct_info.readpos_ptr,
				&cd_oct_info.writepos_ptr);
		if (cd_oct_info.in_use) {
			VD_Istp_dump_pre_mortem_buffer(&cd_oct_info);
		} else {
			vd_debug_printf("VD: %s oct buffer: %s", __func__,
			(cd_oct_info.in_use) ?
			"no data in buffer" :
			"not in use");
		}
	}

	vd_debug_printf("VD: COREDUMP - Start\n");

	/* count available frames for coredump header frame */
	if (area_ptr) {
		/* 2 as we always send an info message
			with each memory dump region */
		tNumFrames += (cd_info->number_of_ranges * 2);
		if (cd_oct_info.in_use)
			tNumFrames += 2;
	}

	if (trap_ptr)
		tNumFrames++;

	if (regs_ptr)
		tNumFrames++;

	if (vm_dump_ptr)
		tNumFrames++;


	/* Send core dump header frame */
	VD_Stp_send_coredump_header(++tNumFrames);
	tFrameCount++;

	/* Send a plain text info message. In particular;
		 Number of frame we are about to send.
		 Type of frames to be sent.
		 Size of frames to send.
	*/
	VD_Stp_dump_info(vd_stp_buffer, sizeof(vd_stp_buffer),
		tFrameCount++, tNumFrames,
		cd_info->number_of_ranges, area_ptr,
		regs_ptr, trap_ptr, vm_dump_ptr, &cd_oct_info);
	if (trap_ptr) {
		/* dump trap info */
		BfrPos = 0;

		BfrPos += snprintf(&vd_stp_buffer[BfrPos],
			sizeof(vd_stp_buffer) - BfrPos,
			"Trap info\n");
		if (SYS_EXCEPTION_MEX == trap_ptr->exception_type)
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap in MEX\n");
		else if (SYS_EXCEPTION_LINUX == trap_ptr->exception_type)
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap in LINUX\n");
		else if (SYS_EXCEPTION_VMM == trap_ptr->exception_type)
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap in Mobilevisor\n");
		else if (SYS_EXCEPTION_SECURITY == trap_ptr->exception_type)
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap in Security\n");

		BfrPos += snprintf(&vd_stp_buffer[BfrPos],
			sizeof(vd_stp_buffer) - BfrPos,
			"Trap vector: 0x%04x\n", trap_ptr->trap_vector);
		BfrPos += snprintf(&vd_stp_buffer[BfrPos],
			sizeof(vd_stp_buffer) - BfrPos,
			"Trap date: %s\n", (char *)(trap_ptr->date));
		BfrPos += snprintf(&vd_stp_buffer[BfrPos],
			sizeof(vd_stp_buffer) - BfrPos,
			"Trap time: %s\n", (char *)(trap_ptr->time));
		if (SW_TRAP_VECTOR == trap_ptr->trap_vector) {
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap filename: %s\n",
				(char *)(trap_ptr->filename));
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap line number: %d\n", trap_ptr->line);
		} else {
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap filename:\n");
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"Trap line number:\n");
		}
		BfrPos += snprintf(&vd_stp_buffer[BfrPos],
			sizeof(vd_stp_buffer)-BfrPos,
			"Trap log:\n%s", (char *)(trap_ptr->log_data));
		VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_INFO, 0,
			(unsigned char *)vd_stp_buffer,
			BfrPos + sizeof(vd_stp_buffer[0]),
			tFrameCount++);
		BfrPos = 0;
	}

	if (regs_ptr)
		VD_Stp_dump_register(vd_stp_buffer,
			sizeof(vd_stp_buffer), regs_ptr,
			tFrameCount++);

	if (vm_dump_ptr)
		VD_Stp_dump_vm(vd_stp_buffer,
			sizeof(vd_stp_buffer), vm_dump_ptr,
			tFrameCount++);

	if (area_ptr) {
		unsigned int i;
		struct cd_ram *a_ptr = area_ptr;
		BfrPos = 0;

		/* normal memory dump */
		for (i = 0; i < cd_info->number_of_ranges; i++) {
			/* dump memory info */
			BfrPos += snprintf(&vd_stp_buffer[BfrPos],
				sizeof(vd_stp_buffer) - BfrPos,
				"VD Info, frame %d for data message frame %d, containing Memory region starting at address 0x%08x, logical address 0x%08x, length 0x%08x\n",
				tFrameCount, tFrameCount + 1,
				(unsigned int)a_ptr->physical_start,
				(unsigned int)a_ptr->physical_start,
				a_ptr->length);

			VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_INFO, 0,
				(unsigned char *)vd_stp_buffer,
				BfrPos + sizeof(vd_stp_buffer[0]),
				tFrameCount++);
			BfrPos = 0;

			a_ptr->logical_start =
				(uint32_t)ioremap_cache(
				a_ptr->physical_start,
				a_ptr->length);

			vd_debug_printf(
				"VD: Dumping Memory Region 0x%x physical_start=0x%x logical_start=0x%x Size=0x%x\n",
				i, a_ptr->physical_start,
				a_ptr->logical_start, a_ptr->length);

			/* dump memory area */
			VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_DATA,
				(unsigned char *)a_ptr->physical_start,
				(unsigned char *)a_ptr->logical_start,
				a_ptr->length, tFrameCount++);

			iounmap((void *)a_ptr->logical_start);
			a_ptr->logical_start = 0;

			a_ptr++;
		}
	}

	if (cd_oct_info.in_use) {
		struct cd_ram *a_ptr;
		struct cd_ram oct_buffer;

		BfrPos = 0;

		oct_buffer.physical_start = (uint32_t)cd_oct_info.start_ptr_phy;
		oct_buffer.logical_start = (uint32_t)cd_oct_info.start_ptr;
		oct_buffer.length = cd_oct_info.size;
		a_ptr = &oct_buffer;

		BfrPos += snprintf(&vd_stp_buffer[BfrPos],
			sizeof(vd_stp_buffer) - BfrPos,
			"VD Info, frame %d for data message frame %d, containing Memory region starting at address 0x%08x, logical address 0x%08x, length 0x%08x\n",
			tFrameCount, tFrameCount + 1,
			(unsigned int)a_ptr->physical_start,
			(unsigned int)a_ptr->physical_start,
			a_ptr->length);

		VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_INFO, 0,
			(unsigned char *)vd_stp_buffer,
			BfrPos + sizeof(vd_stp_buffer[0]),
			tFrameCount++);

		dma_sync_single_for_cpu(
			NULL, (dma_addr_t)a_ptr->physical_start,
			a_ptr->length, DMA_FROM_DEVICE);

		vd_debug_printf(
			"VD: Dumping OCT Region physical_start=0x%x logical_start=0x%x Size=0x%x\n",
			a_ptr->physical_start,
			a_ptr->logical_start,
			a_ptr->length);
		/* check for current OCT pointer again */
		oct_get_buffer_info(
			&cd_oct_info_dbg.start_ptr,
			&cd_oct_info_dbg.start_ptr_phy,
			&cd_oct_info_dbg.size,
			&cd_oct_info_dbg.readpos_ptr,
			&cd_oct_info_dbg.writepos_ptr);

		VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_DATA,
			(unsigned char *)a_ptr->physical_start,
			(unsigned char *)a_ptr->logical_start,
			a_ptr->length, tFrameCount++);

	}

	VD_Stp_send_coredump_terminator(tFrameCount);

	VD_Stp_deinit();

	/* Close the file */
	vdump_close_coredump();

	vd_debug_printf("VD: COREDUMP - End\n");
}

/*****************************************************************************
 * Function:     VD_Stp_dump_info
 *-----------------------------------------------------------------------------
 * Description:   Send a plain text info message
					containing nNumber of frame we
					are about to send,
 *                Type of frames to be sent, size of frames to send.
 *
 * Inputs:  Pointer to Buffer for message creation
 *          Bufferlength
 *          Current frame count.
 *          Number of frames that will be sent.
 *          Number of memory areas.
 *          Pointer to array containing memory area pointers and sizes.
 *          Pointer to registers save.
 *          Pointer to Trap information.
 *          Number of ctd areas.
 *          Pointer to array containing ctd memory area pointers and sizes.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/

static void VD_Stp_dump_info(
	char *pBuffer,
	int	BfrLength,
	unsigned char bFrameCount,
	unsigned int	wNumFrames,
	unsigned int	nof_areas,
	struct cd_ram		 *area_ptr,
	struct x86_cpu_regs *regs_ptr,
	struct sys_trap		*trap_ptr,
	struct sys_vm_dump *vm_dump_ptr,
	struct cd_oct_buffer *oct_ptr)
{
	int		BfrPos = 0;
	unsigned int tFrameCount = 1;
	unsigned int i;


	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"VD info, Core dump with %d frames.\n", wNumFrames);
	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"Frame 0, type 0 - Core dump header\nFrame %d, type Info\n",
		tFrameCount++);

	if (trap_ptr)
		BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
			"Frame %d, type Info - trap data\n", tFrameCount++);

	if (regs_ptr)
		BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
			"Frame %d, type info - register dump\n", tFrameCount++);

	if (vm_dump_ptr)
		BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
			"Frame %d, type info - VM dump\n", tFrameCount++);


	if (area_ptr) {
		struct cd_ram *a_ptr = area_ptr;

		for (i = 0; i < nof_areas; i++) {
			BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
				"Frame %d, data type - info, text info on following data frame.\n",
				tFrameCount++);

			BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
				"Frame %d, data type - Data, physical location 0x%08x, logical location 0x%08x, length %d, num segments = %d\n",
				tFrameCount++,
				(unsigned int)a_ptr->physical_start,
				(unsigned int)a_ptr->physical_start,
				a_ptr->length,
				((a_ptr->length - 1)/
				VD_ISTP_TRACE_MESSAGE_SIZE) + 1);
			a_ptr++;
		}
	}

	if (oct_ptr) {
		struct cd_ram *a_ptr;
		struct cd_ram oct_buffer;

		oct_buffer.physical_start = (uint32_t)oct_ptr->start_ptr;
		oct_buffer.logical_start = (uint32_t)oct_ptr->start_ptr;
		oct_buffer.length = oct_ptr->size;
		a_ptr = &oct_buffer;

		BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
			"Frame %d, data type - info, text info on following data frame.\n",
			tFrameCount++);

		BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
			"Frame %d, data type - Data, physical location 0x%08x, logical location 0x%08x, length %d, num segments = %d\n",
			tFrameCount++,
			(unsigned int)a_ptr->physical_start,
			(unsigned int)a_ptr->physical_start,
			a_ptr->length,
			((a_ptr->length - 1)/
			VD_ISTP_TRACE_MESSAGE_SIZE) + 1);

	}

	BfrPos += snprintf(pBuffer + BfrPos,
		BfrLength - BfrPos,
		"Frame %d, terminator type\n", tFrameCount);

	VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_INFO, 0,
		(unsigned char *) pBuffer,
		BfrPos + sizeof(char), bFrameCount);
}


/*****************************************************************************
 * Function:     VD_Stp_send_coredump_header
 *-----------------------------------------------------------------------------
 * Description:   Sends a core dump header message,
					contains information which can be
					used by the received to check
 *                whether a core dump has been received correctly
 *
 * Inputs:  Number of following frames, not inc terminator.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_send_coredump_header(unsigned int wNumFrames)
{
	unsigned int tHeadercontents = wNumFrames;

	vd_debug_printf(
		"VD: Send Start Frame|Seq=0x0|0x0, Size=0x4, 0x%x frames to follow\n",
		wNumFrames);

	VD_Stp_send_segment(VD_STP_HDR_MAGIC_NUMBER,
		VD_STP_HDR_MSG_TYPE_HEADER,
		0x000000,
		0x00,
		(unsigned char *)(&tHeadercontents),
		4,
		NULL,
		0);

}

/*****************************************************************************
 * Function:     VD_Stp_send_coredump_terminator
 *-----------------------------------------------------------------------------
 * Description:   Sends a core dump terminator message,
 *
 * Inputs:  Frame number
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_send_coredump_terminator(unsigned char bFrameNum)
{

	vd_debug_printf(
		"VD: Send End Frame|Seq=0x%x|0x0, Size=0x0\n",
		bFrameNum);

	VD_Stp_send_segment(VD_STP_HDR_MAGIC_NUMBER,
		VD_STP_HDR_MSG_TYPE_TERMINATOR,
		0x000000,
		bFrameNum,
		NULL,
		0,
		NULL,
		0);

}

/*****************************************************************************
 * Function:     VD_Stp_format_register
 *-----------------------------------------------------------------------------
 * Description: Format the registers context to the input buffer.
 *
 * Inputs:  Pointer to Buffer for message creation
 *          Bufferlength
 *          Pointer to saved register contents
 *
 * Outputs: None
 *
 * Return:  Formated length of buffer
 *
 *****************************************************************************/
static int VD_Stp_format_register(
	char *pBuffer, int BfrLength,
	struct x86_cpu_regs *regs_ptr)
{
	int	BfrPos = 0;


	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA gp regs:\nEAX 0x%08x\nEBX 0x%08x\nECX 0x%08x\nEDX 0x%08x\nESI 0x%08x\nEDI 0x%08x\nEBP 0x%08x\nESP 0x%08x\n\n",
		regs_ptr->gp_regs.eax,
		regs_ptr->gp_regs.ebx,
		regs_ptr->gp_regs.ecx,
		regs_ptr->gp_regs.edx,
		regs_ptr->gp_regs.esi,
		regs_ptr->gp_regs.edi,
		regs_ptr->gp_regs.ebp,
		regs_ptr->gp_regs.esp);

	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA segment regs:\nCS 0x%04x\nDS 0x%04x\nSS 0x%04x\nES 0x%04x\nFS 0x%04x\nGS 0x%04x\n\n",
		regs_ptr->segment_regs.cs,
		regs_ptr->segment_regs.ds,
		regs_ptr->segment_regs.ss,
		regs_ptr->segment_regs.es,
		regs_ptr->segment_regs.fs,
		regs_ptr->segment_regs.gs);

	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA flag reg:\nEFLAG 0x%08x\n\n",
		regs_ptr->eflags_reg.eflags);

	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA ip reg:\nEIP 0x%08x\n\n",
		regs_ptr->eip_reg.eip);

	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA control regs:\nCR0 0x%08x\nCR2 0x%08x\nCR3 0x%08x\nCR4 0x%08x\n\n",
		regs_ptr->ctrl_regs.cr0,
		regs_ptr->ctrl_regs.cr2,
		regs_ptr->ctrl_regs.cr3,
		regs_ptr->ctrl_regs.cr4);

	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA mm regs:\nGDTB 0x%08x\nIDTB 0x%08x\nLDTR 0x%08x\nTSSB 0x%08x\nGDTL 0x%04x\nIDTL 0x%04x\nLDTL 0x%04x\nTSSL 0x%04x\nTR 0x%04x\nLDTR 0x%04x\n\n",
		regs_ptr->mm_regs.gdtb,
		regs_ptr->mm_regs.idtb,
		regs_ptr->mm_regs.ldtb,
		regs_ptr->mm_regs.tssb,
		regs_ptr->mm_regs.gdtl,
		regs_ptr->mm_regs.idtl,
		regs_ptr->mm_regs.ldtl,
		regs_ptr->mm_regs.tssl,
		regs_ptr->mm_regs.tr,
		regs_ptr->mm_regs.ldtr);


	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos,
		"IA debug regs:\nDR0 0x%08x\nDR1 0x%08x\nDR2 0x%08x\nDR3 0x%08x\nDR4 0x%08x\nDR5 0x%08x\nDR6 0x%08x\nDR7 0x%08x\n\n",
		regs_ptr->debug_regs.dr0,
		regs_ptr->debug_regs.dr1,
		regs_ptr->debug_regs.dr2,
		regs_ptr->debug_regs.dr3,
		regs_ptr->debug_regs.dr4,
		regs_ptr->debug_regs.dr5,
		regs_ptr->debug_regs.dr6,
		regs_ptr->debug_regs.dr7);


	return BfrPos;
}

/*****************************************************************************
 * Function:     VD_Stp_dump_register
 *-----------------------------------------------------------------------------
 * Description: Sends a text message containing the saved register data.
 *
 * Inputs:  Pointer to Buffer for message creation
 *          Bufferlength
 *          Pointer to saved register contents
 *          Current frame count
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_dump_register(
	char *pBuffer, int BfrLength,
	struct x86_cpu_regs *regs_ptr,
	unsigned char bFrameCount)
{
	int	BfrPos = 0;


	BfrPos += snprintf(pBuffer + BfrPos,
		BfrLength - BfrPos, "Register dump\n");

	BfrPos += VD_Stp_format_register(pBuffer + BfrPos,
		BfrLength - BfrPos, regs_ptr);

	VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_INFO, 0,
		(unsigned char *)pBuffer, BfrPos + sizeof(char),
		bFrameCount);
}


/*****************************************************************************
 * Function:     VD_Stp_dump_vm
 *-----------------------------------------------------------------------------
 * Description: Sends a text message
				containing the registers
				of all vcpu in the guest vm.
 *
 * Inputs:  Pointer to Buffer for message creation
 *          Bufferlength
 *          Pointer to saved vm
 *          Current frame count
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_dump_vm(
	char *pBuffer, int BfrLength,
	struct sys_vm_dump *vm_dump_ptr,
	unsigned char bFrameCount)
{
	int	BfrPos = 0;
	int vm_count;
	int vcpu_count;

	BfrPos += snprintf(pBuffer + BfrPos, BfrLength - BfrPos, "VM dump\n\n");

	for (vm_count = 0; vm_count < vm_dump_ptr->no_of_vm; vm_count++) {
		BfrPos += snprintf(pBuffer + BfrPos,
					BfrLength - BfrPos,
					"OS ID = %d\n\n",
					vm_dump_ptr->vm[vm_count].os_id);
		for (vcpu_count = 0;
			vcpu_count < vm_dump_ptr->vm[vm_count].no_of_vcpu;
			vcpu_count++) {
			BfrPos += snprintf(pBuffer + BfrPos,
						BfrLength - BfrPos,
						"VCPU = %d\n\n",
						vcpu_count);
			BfrPos += VD_Stp_format_register(pBuffer + BfrPos,
						BfrLength - BfrPos,
						&vm_dump_ptr->vm[vm_count]
							.vcpu_reg[vcpu_count]);
		}
	}

	VD_Stp_dump_frame(VD_STP_HDR_MSG_TYPE_INFO, 0,
		(unsigned char *)pBuffer,
		BfrPos + sizeof(char), bFrameCount);
}



/*****************************************************************************
 * Function:     VD_Stp_dump_frame
 *-----------------------------------------------------------------------------
 * Description: Dumps a complete frame.
				Automatically splits the frame
				into the required number of segments.
 *
 * Inputs:  Frame type applied to the segment headers
 *          Pointer to the data to be sent
 *          Total size of the data to be sent in bytes
 *          The maximum segment size
 *          Current frame count number to be applied to the segment headers
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/

static void VD_Stp_dump_frame(
	unsigned char bFrameType, unsigned char *pFramePhyPtr,
	unsigned char *pFramePtr, unsigned int wFrameSize,
	unsigned char bFrameCount)
{
	unsigned char tInfoBlock[VD_STP_DUMP_REGION_HEADER_SIZE];
	unsigned int tCurrentLength;
	unsigned int tLengthRemaining = wFrameSize;
	unsigned int tSegmentCount = 0;
	unsigned char *tpDataPtr = pFramePtr;
	unsigned int tInfoSize = 0;

	static unsigned char last_bFrameCount;
	static unsigned int last_tSegmentCount;


	while (tLengthRemaining > 0) {
		if ((VD_STP_HDR_MSG_TYPE_DATA == bFrameType) &&
			(tSegmentCount == 0))
			tInfoSize = VD_Stp_build_cd_header(
				tInfoBlock,
				sizeof(tInfoBlock),
				pFramePhyPtr, wFrameSize);
		else
			tInfoSize = 0;

		if (tLengthRemaining > VD_ISTP_TRACE_MESSAGE_SIZE) {
			tCurrentLength		= VD_ISTP_TRACE_MESSAGE_SIZE;
			tLengthRemaining -= VD_ISTP_TRACE_MESSAGE_SIZE;
		} else {
			tCurrentLength	 = tLengthRemaining;
			tLengthRemaining = 0;
		}

		if ((last_bFrameCount == bFrameCount) &&
			(tSegmentCount == last_tSegmentCount + 1)) {

			/* cd_debug_putc("VD: ",
			(tCurrentLength + tInfoSize) <
			VD_ISTP_TRACE_MESSAGE_SIZE ? '*' : '#'); */
		} else {

		/* vd_debug_printf(
		"VD: Starting %s frame transmission
		at SN=0x%x (#>=%dBytes, *<%dBytes)",
		bFrameType == VD_STP_HDR_MSG_TYPE_INFO ?
		"Info" : "Data",
		bFrameCount,
		VD_ISTP_TRACE_MESSAGE_SIZE,
		VD_ISTP_TRACE_MESSAGE_SIZE);
		*/

			/* cd_debug_putc("VD: ",
				(tCurrentLength + tInfoSize)
				< VD_ISTP_TRACE_MESSAGE_SIZE ?
				'*' : '#'); */

			last_bFrameCount = bFrameCount;

		}

		last_tSegmentCount = tSegmentCount;



		VD_Stp_send_segment(VD_STP_HDR_MAGIC_NUMBER, bFrameType,
			tSegmentCount, bFrameCount,
			tpDataPtr, tCurrentLength,
			tInfoBlock, tInfoSize);

		if (tLengthRemaining) { /* For lint */
			/*lint !e662 */
			/* Pointer may go beyond end,
				but will never be used in that condition */
			tpDataPtr += tCurrentLength;
		}

		tSegmentCount++;
	}
}


/*****************************************************************************
 * Function:     VD_Stp_send_segment
 *-----------------------------------------------------------------------------
 * Description:   Sends a single stp format segment,
					including header. Note if an info
					block is to be sent
					then the size of the segment sent
					will be the info size plus contents size
 *
 *
 * Inputs:  Header magic number,
 *          Message type,
 *          Segment number,
 *          frame number,
 *          Contents start pointer,
 *          Contents size
 *          Pointer to Info block
 *          Info block size
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_send_segment(
	unsigned int wMagicNumber, unsigned char bMsgType,
	unsigned int wSegmentNumber, unsigned char bFramenumber,
	unsigned char *pbContents, unsigned int	wContentsSize,
	unsigned char *pInfoBlock, unsigned int	wInfoSize)
{
	unsigned int tSize = wContentsSize + wInfoSize;

	/* if there are contents, then there
		will always be a header+Infoblock+Contents
		in the segment*/
	/* Therefore only need to test for valid contents here. */
	if (pbContents && (wContentsSize > 0)) {
		VD_Stp_send_header(wMagicNumber, bMsgType,
			wSegmentNumber, bFramenumber, tSize,
			VD_STP_TS_NOT_REQUIRED);

		if (pInfoBlock && (wInfoSize > 0))
			VD_Stp_tx_data(pInfoBlock,
				wInfoSize, VD_STP_TS_NOT_REQUIRED);

		/* this is the last bit of data - send TS */
		VD_Stp_tx_data(pbContents, wContentsSize, VD_STP_TS_REQUIRED);
	} else if (pInfoBlock && (wInfoSize > 0)) {
		VD_Stp_send_header(wMagicNumber, bMsgType,
			wSegmentNumber, bFramenumber, tSize,
			VD_STP_TS_NOT_REQUIRED);

		VD_Stp_tx_data(pInfoBlock, wInfoSize, VD_STP_TS_REQUIRED);
	} else {
		/* this is the last bit of data - send TS */
		VD_Stp_send_header(wMagicNumber, bMsgType,
			wSegmentNumber, bFramenumber, tSize,
			VD_STP_TS_REQUIRED);
	}
}

/*****************************************************************************
 * Function:     VD_Stp_send_header
 *-----------------------------------------------------------------------------
 * Description:   Sends a single core dump stp format header.
 *
 * Inputs:  Header magic number,
 *          Message type,
 *          Segment number,
 *          frame number,
 *          Contents size
 *          wLast - Indication if this is the
			last block of data and therefore
			a End of Msg TimeStamp is required.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void	VD_Stp_send_header(
	unsigned int	wMagicNumber, unsigned char bMsgType,
	unsigned int	wSegmentNumber, unsigned char bFramenumber,
	unsigned int	wSize, unsigned int	wLast)
{
	unsigned char tHeader[VD_STP_HEADER_SIZE];

	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP)
		VD_Stp_send_bbtp_header(TADO_MSG_TYPE_CORE_DUMP);

	/* prepare header */
	tHeader[VD_STP_HDR_MAG_NUM_BPOSN] = (wMagicNumber >> 8)	& 0xff;
	tHeader[VD_STP_HDR_MAG_NUM_BPOSN + 1] = (wMagicNumber >> 16) & 0xff;
	tHeader[VD_STP_HDR_MAG_NUM_BPOSN + 2] = (wMagicNumber >> 24) & 0xff;

	tHeader[VD_STP_HDR_TYPE_BPOSN]				= bMsgType;

	tHeader[VD_STP_HDR_SEG_SEQ_BPOSN] =	wSegmentNumber & 0xff;
	tHeader[VD_STP_HDR_SEG_SEQ_BPOSN + 1] = (wSegmentNumber >> 8)	& 0xff;
	tHeader[VD_STP_HDR_SEG_SEQ_BPOSN + 2] = (wSegmentNumber >> 16) & 0xff;

	tHeader[VD_STP_HDR_FRAME_NUM_BPOSN]	 = bFramenumber;

	tHeader[VD_STP_HDR_FRAME_SIZE_BPOSN]		 = (wSize & 0xff);
	tHeader[VD_STP_HDR_FRAME_SIZE_BPOSN + 1] = (wSize >> 8)	& 0xff;
	tHeader[VD_STP_HDR_FRAME_SIZE_BPOSN + 2] = (wSize >> 16) & 0xff;
	tHeader[VD_STP_HDR_FRAME_SIZE_BPOSN + 3] = (wSize >> 24) & 0xff;

	VD_Stp_tx_data(tHeader, sizeof(tHeader), wLast);
}


/*****************************************************************************
 * Function:     VD_Stp_deinit
 *-----------------------------------------------------------------------------
 * Description:   Send an STP ST message to ensure
					termination and deinitializes
					the TADO interface.
 *
 * Inputs:  None
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_deinit(void)
{
	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP)
		VD_Istp_deinit();

	if (TracePort == VD_TRACE_PORT_MIPI) {
		/* if(TadoHandle) */
			/* tado_close(TadoHandle); */
		/* Turn off other trace masters to ensure
			we don't get stuck waiting for
			the hardware buffers to empty */
		/* UtaTrcCtrlSwitchOffTrcSources( UTATRC_AT_PORT_FLAG_MAP ); */
		/* while (0 !=
			UtaTrcCtrlGetHwFillLevel( UTATRC_AT_SRC_bb_sw ) ); */
	}
}

/*****************************************************************************
 * Function:     VD_Stp_tx_data
 *-----------------------------------------------------------------------------
 * Description:   Sends raw data. using MIPI.
					For debug purposes can also
					send simultaneously via USIF.
 *
 * Inputs:  Pointer to data
 *          length of data
 *          wLast - Indication if this is the last block
			of data and therefore a End of
			Msg TimeStamp is required.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_tx_data(const unsigned char *start,
	unsigned int length, unsigned int wLast)
{
	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP) {
		VD_Istp_SendData((unsigned char *)start, length);

		if (wLast == VD_STP_TS_REQUIRED)
			VD_Istp_SendDataEnd();
	} else if (Protocol == VD_COREDUMP_PROTOCOL_STP) {
		/* if(TadoHandle == NULL)
			TadoHandle = tado_open(MsgTypeBBTP); */

		/* SMS02848675 CD transmission-speed to be reduced
			 delay to mipi transfer with
			maximum 8 KByte data transfer delay per 1000 usec
		*/
		/* tado_wait(1000); */

		/* tado_write(TadoHandle, (void*)start, length, wLast); */

		/* if(wLast == VD_STP_TS_REQUIRED)
			TadoHandle = NULL; */
	}
}

/*****************************************************************************
 * Function:     VD_Stp_MA_text_info
 *-----------------------------------------------------------------------------
 * Description: Outputs a DTM formatted string for displaying info text in MA
 *
 *
 * Inputs:     pointer to string to be output.
 *             reset_counter -
				if TRUE, the sequence number
				counter will be reset to 0
				if FALSE then the sequence number
				counter increments normally.
 *
 *
 * Outputs:    None
 *
 * Return:     None
 *
 *****************************************************************************/
#if 0
static void VD_Stp_MA_text_info(const char *pStr, int reset_counter)
{
	static unsigned char counter;

	if (reset_counter)
		counter = 0;

	/* change channel id to non-CD so that MA sees the data. */
	VD_Stp_set_CID(VD_STP_CHID_CORE_DUMP_INFO);

	/* send text indication to MA */
	VD_Stp_send_dtm_frame(VD_DTM_ASCII_FRAME_ID,
		pStr, strlen(pStr), VD_DTM_ASCII_LEADIN_SIZE, counter);
	counter++;

	/* change channel back to CD */
	VD_Stp_set_CID(VD_STP_CHID_CORE_DUMP);
}
#endif

/*****************************************************************************
 * Function:     VD_Stp_set_CID
 *-----------------------------------------------------------------------------
 * Description: Sets the ISTP channel id
 *
 *
 * Inputs:     channel id
 *
 * Outputs:    None
 *
 * Return:     None
 *
 *****************************************************************************/
#if 0
static void VD_Stp_set_CID(unsigned int ccid)
{
	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP)
		VD_Istp_setCID(ccid);
}
#endif


/*****************************************************************************
 * Function:     VD_Stp_send_dtm_frame
 *-----------------------------------------------------------------------------
 * Description: Outputs a DTM frame as ISTP
 *
 *
 * Inputs:  frameId - type of DTM frame,
 *          pStart - pointer to data start,
 *          lenght - length of data,
 *          nullHeadLen - num of nulls to add at end of data
 *          counter - sequence number counter.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
#if 0
static void VD_Stp_send_dtm_frame(int frameId,
	const char *pStart, int length, int nullHeadLen, unsigned char counter)
{
	unsigned char header[enDtmHeaderNum];
	int uLp;
	char zero = '\0';
	int len = length + nullHeadLen;
	/* initialise to avoid stupid lint warning w644 */
	int TmpMsgTypeBBTP = TADO_MSG_TYPE_CORE_DUMP;

	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP)
		VD_Stp_send_bbtp_header(TADO_MSG_TYPE_LEGACY_SIO);
	else if (Protocol == VD_COREDUMP_PROTOCOL_STP) {
		TmpMsgTypeBBTP = MsgTypeBBTP;
		MsgTypeBBTP = TADO_MSG_TYPE_LEGACY_SIO;
	}

	/* Header settings	*/
	header[enDtmHeaderStart]	 = (unsigned char)MA_INTRO_CHAR_DTM;
	header[enDtmHeaderVersion] = (unsigned char)VD_DTM_VERSION;
	header[enDtmHeaderFrameId] = (unsigned char)frameId;
	header[enDtmHeaderCounter] = counter;
	header[enDtmHeaderHighLen] = (unsigned char)(len >> 8);
	header[enDtmHeaderLowLen]	= (unsigned char)(len >> 0);

	/* Calculation of the FCS	 */
	header[enDtmHeaderFcs] = 0xFF;

	for (uLp = enDtmHeaderVersion; uLp < enDtmHeaderFcs; uLp++)
		header[enDtmHeaderFcs] =
		fcs_tab[(header[enDtmHeaderFcs] ^ header[uLp]) & 0xFF];

	header[enDtmHeaderFcs] = 0xFF - header[enDtmHeaderFcs];

	VD_Stp_tx_data((unsigned char *)(&header),
		enDtmHeaderNum, VD_STP_TS_NOT_REQUIRED);

	/* send the leadin data -
		this is just null apart from
		the last byte which is the sequence number */
	while (nullHeadLen-- > 1)
		VD_Stp_tx_data((void *)&zero, 1, 0);

	/* last byte of leadin aka sequence number */
	VD_Stp_tx_data((unsigned char *)(&header[enDtmHeaderCounter]),
		1, VD_STP_TS_NOT_REQUIRED);

	/* send the payload */
	VD_Stp_tx_data((unsigned char *)pStart, length, VD_STP_TS_NOT_REQUIRED);

	/* last byte to send */
	VD_Stp_tx_data((unsigned char *)(&header[enDtmHeaderFcs]),
		1, VD_STP_TS_REQUIRED);

	if (Protocol == VD_COREDUMP_PROTOCOL_STP)
		MsgTypeBBTP = TmpMsgTypeBBTP;
}
#endif

/*****************************************************************************
 * Function:     VD_Stp_send_trap_frame
 *-----------------------------------------------------------------------------
 * Description: Outputs a TRAP frame as ISTP
 *
 *
 * Inputs:  frameId - type of trap frame,
 *          pData - pointer to data start,
 *          lenght - length of data
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Stp_send_trap_frame(enum MT_TRAP frameId,
	unsigned char *pData, unsigned int length)
{
	/* initialise to avoid stupid lint warning w644 */
	int TmpMsgTypeBBTP = TADO_MSG_TYPE_CORE_DUMP;

	if (Protocol == VD_COREDUMP_PROTOCOL_ISTP)
		VD_Stp_send_bbtp_header(TADO_MSG_TYPE_TRAP);
	else if (Protocol == VD_COREDUMP_PROTOCOL_STP) {
		TmpMsgTypeBBTP = MsgTypeBBTP;
		MsgTypeBBTP		= TADO_MSG_TYPE_TRAP;
	}

	VD_Stp_tx_data((unsigned char *)(&frameId),
		sizeof(frameId), VD_STP_TS_NOT_REQUIRED);
	VD_Stp_tx_data((unsigned char *)pData,
		length, VD_STP_TS_REQUIRED);

	if (Protocol == VD_COREDUMP_PROTOCOL_STP)
		MsgTypeBBTP = TmpMsgTypeBBTP;
}

/*****************************************************************************
 * Function:     VD_Stp_build_cd_header
 *-----------------------------------------------------------------------------
 * Description: function to build coredump header
 *
 *
 * Inputs:  pBuffer     - pointer to buffer,
 *          wBuffersize - available buffersize,
 *          pFramePtr   - coredump address,
 *          wFrameSize  - size of coredump data.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static unsigned int VD_Stp_build_cd_header
	(unsigned char *pBuffer, unsigned int wBuffersize,
	unsigned char *pFramePtr, unsigned int wFrameSize)
{
	unsigned int ret = 0;

	if (pBuffer && (wBuffersize >= VD_STP_DUMP_REGION_HEADER_SIZE)) {
		pBuffer[0] = (unsigned int)pFramePtr & 0xff;
		pBuffer[1] = ((unsigned int)pFramePtr >> 8) & 0xff;
		pBuffer[2] = ((unsigned int)pFramePtr >> 16) & 0xff;
		pBuffer[3] = ((unsigned int)pFramePtr >> 24) & 0xff;

		pBuffer[4] = wFrameSize & 0xff;
		pBuffer[5] = (wFrameSize >> 8) & 0xff;
		pBuffer[6] = (wFrameSize >> 16) & 0xff;
		pBuffer[7] = (wFrameSize >> 24) & 0xff;

		ret = VD_STP_DUMP_REGION_HEADER_SIZE;
	}

	return ret;
}

/*****************************************************************************
 * Function:     VD_Stp_send_bbtp_header
 *-----------------------------------------------------------------------------
 * Description: function to build and send baseband trace protocol header
 *
 *
 * Inputs:  MessageType - bbtp message type
 *
 * Outputs: None
 *
 * Return:  used buffersize
 *
 *****************************************************************************/
static void VD_Stp_send_bbtp_header(int MessageType)
{
	static unsigned int gts_temp = 1;
	unsigned char Buffer[VD_STP_BBTP_MIN_HEADER_SIZE];
	unsigned int *pTs = (unsigned int *)Buffer;

	gts_temp++;
	*pTs = gts_temp; /* tado_get_gts_32bit(); */

	Buffer[4] = MessageType;
	Buffer[5] = 0; /* don't use flexible timestamp now */

	VD_Stp_tx_data(Buffer, sizeof(Buffer), VD_STP_TS_NOT_REQUIRED);
}

/*****************************************************************************
 * Function:      VD_Istp_check_oct_pointers
 *-----------------------------------------------------------------------------
 * Description:   Validity checks on the OCT pointers
 *
 * Inputs:        pOct_buff - pointer to the OCT buffer parameters
 *
 * Outputs:       None
 *
 * Return:        1 if error, 0 if checks are ok
 *
 *****************************************************************************/
static unsigned int VD_Istp_check_oct_pointers(struct cd_oct_buffer *pOct_buff)
{
	unsigned int nStart	 = (unsigned int)pOct_buff->start_ptr;
	unsigned int nRead		= (unsigned int)pOct_buff->readpos_ptr;
	unsigned int nWrite	 = (unsigned int)pOct_buff->writepos_ptr;
	unsigned int nBufSize = (unsigned int)pOct_buff->size;


	/* check if any pointers are invalid */
	if (!nStart || !nRead || !nWrite) {
		vd_debug_printf(
			"VD: %s Failed. NULL pointer",
			__func__);
		return 1;
	}

	if (((nStart & 0x3) != 0) ||
		((nRead & 0x3) != 0) || ((nWrite & 0x3) != 0)) {
		vd_debug_printf(
			"VD: %s Failed. Misaligned pointer.",
			__func__);
		return 1;
	}

	if ((nRead < nStart) || (nRead > (nStart + nBufSize))) {
		vd_debug_printf(
			"VD: %s Failed. Read Pointer out of bounds.",
			__func__);
		return 1;
	}

	if ((nWrite < nStart) || (nWrite > (nStart + nBufSize))) {
		vd_debug_printf(
			"VD: %s Failed. Write pointer out of bounds.",
			__func__);
		return 1;
	}

	/* pWrite	= pRead. This shouldn't happen,
		since we have been told that the
		buffer isn't empty. => sth wrong. */
	/*
	if (nWrite == nRead) {
		vd_debug_printf("VD: %s Failed. Trace buffer empty.", __func__);
		return 1;
	}
	*/

	return 0; /* valid pointers */
}

/*****************************************************************************
 * Function:      VD_Istp_dump_pre_mortem_buffer
 *-----------------------------------------------------------------------------
 * Description:   Writes the ISTP data from the trace buffer over VD device
 *
 * Inputs:        pOct_buff - pointer to the OCT buffer parameters
 *
 * Outputs:       None
 *
 * Return:        None
 *
 *****************************************************************************/
static void VD_Istp_dump_pre_mortem_buffer(struct cd_oct_buffer *pOct_buff)
{
	int	bytes	 = 0;
	char *pStart = (char *)pOct_buff->start_ptr;
	char *pStart_phy = (char *)pOct_buff->start_ptr_phy;
	char *pRead	= (char *)pOct_buff->readpos_ptr;
	char *pWrite = (char *)pOct_buff->writepos_ptr;
	int	BufSize = pOct_buff->size;

	vd_debug_printf(
		"VD: %s pStart=0x%x,pRead=0x%x,pWrite=0x%x,size=%d.\n",
		__func__,
		(unsigned int)pStart,
		(unsigned int)pRead,
		(unsigned int)pWrite,
		(unsigned int)BufSize);

	if (VD_Istp_check_oct_pointers(pOct_buff)) {
		vd_debug_printf(
			"VD: %s Skipping dump of pre-mortem buffer.",
			__func__);
		return;
	}

	dma_sync_single_for_cpu(
		NULL, (dma_addr_t)pStart_phy, BufSize, DMA_FROM_DEVICE);

	/* wrap case */
	bytes = (pStart + BufSize) - pWrite;
	vd_debug_printf(
		"VD: oct dump: address:0x%x, size=0x%x\n",
		(unsigned int)pWrite,
		(unsigned int)bytes);
	vdump_save_coredump(pWrite, bytes);
	bytes = pWrite - pStart; /* bytes can be 0 if pWrite = pStart */
	if (bytes) {
		vd_debug_printf
			("VD: oct dump: address:0x%x, size=0x%x\n",
			(unsigned int)pStart,
			(unsigned int)bytes);
		vdump_save_coredump(pStart, bytes);
	}
}

/*****************************************************************************
 * Function:      VD_Stp_dump_log_buffer
 *-----------------------------------------------------------------------------
 * Description:   Send debug log buffer as info trace(s)
 *
 * Inputs:        None
 *
 * Outputs:       None
 *
 * Return:        1 if error, 0 if checks are ok
 *
 *****************************************************************************/
unsigned int VD_Stp_dump_log_buffer(void)
{
	/* Not implemented yet... */
	return 1;
}

