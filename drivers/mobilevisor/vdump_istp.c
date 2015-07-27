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
#include <stdarg.h>
#include <linux/io.h>
#include "vdump_istp.h"
#include "vdump_file.h"

/*============================================================================
LOCAL FUNCTION PROTOTYPES
============================================================================*/

static void VD_Istp_start_frame(void);
static void VD_Istp_add_data_to_frame(unsigned char *data, int length);
static void VD_Istp_end_frame(void);
static void VD_Istp_put_byte(unsigned char data);
static void VD_Istp_flush_buffer(void);


/*============================================================================
DEFINES AND MACROS
============================================================================*/

/* FCS lookup table */

static const unsigned int fcstab_32[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba,
	0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
/* 0x10 */
	0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de,
	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
	0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
/* 0x20 */
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
	0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940,
	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
/* 0x30 */
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116,
	0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
/* 0x40 */
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
	0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a,
	0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818,
/* 0x50 */
	0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
	0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c,
/* 0x60 */
	0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2,
	0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
	0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086,
	0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4,
	0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
	0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
	0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe,
	0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
	0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252,
	0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60,
	0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
	0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04,
	0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a,
	0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
	0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e,
	0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c,
	0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
	0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0,
	0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6,
	0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

#define PPPINITFCS32 0xffffffff;   /* initial FCS value */

#define INVALID_CID       0xffff /* CID, bigger than possible (255) */
#define INVALID_MID       0xffff /* MID, bigger than possible (255) */

#define MAX_ISTP_SIZE (1024)

struct _ISTP_ENCODER_DATA {
	/* 256 MSN counters, one per MID */
	unsigned char  msn;
	/* current MID (=invalid) */
	unsigned short  cmid;
	/* current CID (=invalid) */
	unsigned short  ccid;
	/* current FCS */
	unsigned int fcs;
	/* ISTP frame buffer - x 256 for MA */
	unsigned char  fbuf[MAX_ISTP_SIZE];
	/* current ISTP frame buffer write position x 256 for MA */
	unsigned int fbuf_pos;
} ISTP_ENCODER_DATA, *PISTP_ENCODER_DATA;


/*============================================================================
EXTERNAL VARIABLES
============================================================================*/


/*============================================================================
EXTERNAL FUNCTIONS
============================================================================*/


/*============================================================================
GLOBALS
============================================================================*/

static int TSEnd = 1;
static struct _ISTP_ENCODER_DATA Istp_enc_data;


/*============================================================================
FUNCTIONS
============================================================================*/

/*****************************************************************************
 * Function:     VD_Istp_start_frame
 *-----------------------------------------------------------------------------
 * Description:
 *
 * Inputs:  None
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Istp_start_frame(void)
{
	struct _ISTP_ENCODER_DATA *pLocalData = &Istp_enc_data;
	unsigned char cMid = (unsigned char)pLocalData->cmid;
	unsigned char cCid = (unsigned char)pLocalData->ccid;


	pLocalData->fcs = PPPINITFCS32; /* Init FCS */

	VD_Istp_put_byte(0x7E);	 /* Start flag */
	VD_Istp_add_data_to_frame(&cMid, 1);
	VD_Istp_add_data_to_frame(&(pLocalData->msn), 1);
	/* Since msn is unsigned char,
		it should wrap around without masking!!! */
	pLocalData->msn++;
	VD_Istp_add_data_to_frame(&cCid, 1);
}

static void VD_Istp_add_data_to_frame(unsigned char *data, int length)
{
	struct _ISTP_ENCODER_DATA *pLocalData = &Istp_enc_data;
	int tCnt;


	for (tCnt = 0; tCnt < length; tCnt++, data++) {
		unsigned char cTmp = ioread8(data);

		/* update FCS */
		pLocalData->fcs =
			(pLocalData->fcs >> 8)
			^fcstab_32[(pLocalData->fcs ^ cTmp) & 0xff];

		if (cTmp == 0x7E) {
			/* byte stuffing */
			VD_Istp_put_byte(0x7D);
			VD_Istp_put_byte(0x5E);
		} else if (cTmp == 0x7D) {
			/* byte stuffing */
			VD_Istp_put_byte(0x7D);
			VD_Istp_put_byte(0x5D);
		} else {
			VD_Istp_put_byte(cTmp);
		}
	}
}

/*****************************************************************************
 * Function:     VD_Istp_end_frame
 *-----------------------------------------------------------------------------
 * Description:
 *
 * Inputs:  None
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Istp_end_frame(void)
{
	struct _ISTP_ENCODER_DATA *pLocalData = &Istp_enc_data;
	unsigned char Data[4];
	unsigned int tFcs;

	/* our code is divided into segments that will be
	the first and last of the data so the flag will always be 0xC0 */
	Data[0] = 0xC0; /* F=1, L=1, O=0 */
	VD_Istp_add_data_to_frame(&Data[0], 1);

	/* Add FCS */
	/* complement and save -
		add_data_to_frame also updates the fcs */
	tFcs = ~pLocalData->fcs;

	Data[0] = (unsigned char)(tFcs & 0xff);
	Data[1] = (unsigned char)((tFcs >> 8) & 0xff);
	Data[2] = (unsigned char)((tFcs >> 16) & 0xff);
	Data[3] = (unsigned char)(tFcs >> 24);

	VD_Istp_add_data_to_frame(Data, sizeof(Data));

	VD_Istp_put_byte(0x7E);

	VD_Istp_flush_buffer();
}

/*****************************************************************************
 * Function:     VD_Istp_put_byte
 *-----------------------------------------------------------------------------
 * Description:
 *
 * Inputs:  data - data byte
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Istp_put_byte(unsigned char data)
{
	struct _ISTP_ENCODER_DATA *pLocalData = &Istp_enc_data;

	/* Check that we will not overrun the buffer */
	if (pLocalData->fbuf_pos == MAX_ISTP_SIZE)
		VD_Istp_flush_buffer();

	pLocalData->fbuf[pLocalData->fbuf_pos++] = data;
}

/*****************************************************************************
 * Function:     VD_Istp_flush_buffer
 *-----------------------------------------------------------------------------
 * Description:
 *
 * Inputs:  None
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
static void VD_Istp_flush_buffer(void)
{
	struct _ISTP_ENCODER_DATA *pLocalData = &Istp_enc_data;

	/* cd_stream_write
		(CD_STREAM_COREDUMP, pLocalData->fbuf, pLocalData->fbuf_pos); */
	vdump_save_coredump((void *)pLocalData->fbuf,
		(int)pLocalData->fbuf_pos);
	pLocalData->fbuf_pos = 0;
}

/*****************************************************************************
 * Function:     VD_Istp_init
 *-----------------------------------------------------------------------------
 * Description:
 *
 *
 * Inputs:     None
 *
 * Outputs: None
 *
 * Return:     1 if successful or 0 if error
 *
 *****************************************************************************/
char VD_Istp_init(unsigned short ccid)
{
	/* set MID and CID */
	Istp_enc_data.msn  = 0;
	Istp_enc_data.cmid = 0;
	Istp_enc_data.ccid = ccid;

	return 1;
}

/*****************************************************************************
 * Function:     VD_Istp_deinit
 *-----------------------------------------------------------------------------
 * Description:
 *
 *
 * Inputs:     None
 *
 * Outputs: None
 *
 * Return:     None
 *
 *****************************************************************************/
void VD_Istp_deinit(void)
{
	/* set to invalid handles */
	Istp_enc_data.cmid = INVALID_MID;
	Istp_enc_data.ccid = INVALID_CID;
}

/*****************************************************************************
 * Function:     VD_Istp_setCID
 *-----------------------------------------------------------------------------
 * Description:
 *
 *
 * Inputs:     channel id
 *
 * Outputs: None
 *
 * Return:     1 if successful or 0 if error
 *
 *****************************************************************************/
void VD_Istp_setCID(unsigned short ccid)
{
	/* set channel id */
	Istp_enc_data.ccid = ccid;
}


/*****************************************************************************
 * Function:     VD_Istp_SendData
 *-----------------------------------------------------------------------------
 * Description: sends data with ISTP protocol.
 *              Sends the frame start bytes,
				if the data is the first segment in a frame.
 *
 * Inputs:  Pointer to data to be dumped
 *          Length of data.
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
void VD_Istp_SendData(unsigned char *pData, unsigned int length)
{
	/* check if we need to start a new segment */
	if (TSEnd) {
		VD_Istp_start_frame();
		TSEnd = 0;
	}

	VD_Istp_add_data_to_frame(pData, length);
}

/*****************************************************************************
 * Function:     VD_Istp_SendDataEnd
 *-----------------------------------------------------------------------------
 * Description: Ends an ISTP frame
 *
 * Inputs:     None
 *
 * Outputs: None
 *
 * Return:  None
 *
 *****************************************************************************/
void VD_Istp_SendDataEnd(void)
{
	VD_Istp_end_frame();
	TSEnd = 1;
}




