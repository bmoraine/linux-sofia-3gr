/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#ifndef __MEAS_HW_REG_H__
#define __MEAS_HW_REG_H__


/*****************************************************************************/

#ifdef HWREG_U32
	#undef HWREG_U32
#endif

#ifdef HWREG_U16
	#undef HWREG_U16
#endif

#ifdef HWREG_U8
	#undef HWREG_U8
#endif

#define HWREG_U32 unsigned int
#define HWREG_U16 unsigned short
#define HWREG_U8 unsigned char

#define AG620_MEAS_CLC(_base)		((_base) + 0x0)
#define AG620_MEAS_ID(_base)		((_base) + 0x8)
#define AG620_MEAS_RUN_CTRL(_base)	((_base) + 0x10)
#define AG620_MEAS_IDINT(_base)		((_base) + 0x14)
#define AG620_MEAS_CONF(_base)		((_base) + 0x18)
#define AG620_MEAS_CTRL_Ax(_base, _x)	((_base) + 0x1C + (_x * 4))
#define AG620_MEAS_CTRL_Bx(_base, _x)	((_base) + 0x3C + (_x * 4))
#define AG620_MEAS_CTRL_Cx(_base, _x)	((_base) + 0x5C + (_x * 4))
#define AG620_MEAS_CLK(_base)		((_base) + 0x6C)
#define AG620_MEAS_CALI(_base)		((_base) + 0xE8)
#define AG620_MEAS_TIMER_CTRL(_base)	((_base) + 0xEC)
#define AG620_MEAS_PEAK(_base)		((_base) + 0xF0)
#define AG620_MEAS_TEMP_ALERT(_base)	((_base) + 0xF4)

#define AG620_MEAS_SWSTAT0(_base)	((_base) + 0x70)
#define AG620_MEAS_SWSTAT1ORIG(_base)	((_base) + 0x74)
#define AG620_MEAS_SWSTAT1(_base)	((_base) + 0x78)
#define AG620_MEAS_STAT(_base)		((_base) + 0x7C)

#define AG620_MEAS_DATA_Ax(_base, _x)	((_base) + 0x80 + (_x * 4))
#define AG620_MEAS_DATA_Bx(_base, _x)	((_base) + 0xA0 + (_x * 4))
#define AG620_MEAS_DATA_Cx(_base, _x)	((_base) + 0xC0 + (_x * 4))

#define AG620_MEAS_RIS(_base)		((_base) + 0xD0)
#define AG620_MEAS_IMSC(_base)		((_base) + 0xD4)
#define AG620_MEAS_MIS(_base)		((_base) + 0xD8)
#define AG620_MEAS_ICR(_base)		((_base) + 0xDC)
#define AG620_MEAS_ISR(_base)		((_base) + 0xE0)


/** @brief MEAS_CLC register description at address offset 0x0
  *
  *  MEAS Clock Control Register
  */
struct S_MEAS_CLCSTRUCTURE {
	HWREG_U32 DISR : 1; /**<  */
	const HWREG_U32 DISS : 1; /**<  */
	HWREG_U32 SPEN : 1; /**<  */
	HWREG_U32 EDIS : 1; /**<  */
	HWREG_U32 SBWE : 1; /**<  */
	HWREG_U32 FSOE : 1; /**<  */
	const HWREG_U32 RESV1 : 2; /**< Unused bitfield */
	HWREG_U32 RMC : 8; /**<  */
	HWREG_U32 ORMC : 8; /**<  */
	const HWREG_U32 RESV2 : 8; /**< Unused bitfield */
};


/** @brief MEAS_CLC Bitfield widths
*/
#define LENMEAS_CLC_DISR 1
#define LENMEAS_CLC_DISS 1
#define LENMEAS_CLC_SPEN 1
#define LENMEAS_CLC_EDIS 1
#define LENMEAS_CLC_SBWE 1
#define LENMEAS_CLC_FSOE 1
#define LENMEAS_CLC_RMC 8
#define LENMEAS_CLC_ORMC 8

enum E_MEAS_CLCLSB {
	MEAS_CLC_LSB_DISR = 0,
	MEAS_CLC_LSB_DISS = 1,
	MEAS_CLC_LSB_SPEN = 2,
	MEAS_CLC_LSB_EDIS = 3,
	MEAS_CLC_LSB_SBWE = 4,
	MEAS_CLC_LSB_FSOE = 5,
	MEAS_CLC_LSB_RMC = 8,
	MEAS_CLC_LSB_ORMC = 16
};

union U_MEAS_CLC {
	HWREG_U32 val;
	struct S_MEAS_CLCSTRUCTURE MEAS_CLC_STRUCTURE;
};

enum E_MEAS_CLC_DISR {
	MEAS_CLC_DISR_NREQ = 0,
	MEAS_CLC_DISR_REQ = 1
};

enum E_MEAS_CLC_DISS {
	MEAS_CLC_DISS_EN = 0,
	MEAS_CLC_DISS_DIS = 1
};

enum E_MEAS_CLC_SPEN {
	MEAS_CLC_SPEN_DIS = 0,
	MEAS_CLC_SPEN_EN = 1
};

enum E_MEAS_CLC_EDIS {
	MEAS_CLC_EDIS_EN = 0,
	MEAS_CLC_EDIS_DIS = 1
};

enum E_MEAS_CLC_SBWE {
	MEAS_CLC_SBWE_WRTPROT = 0,
	MEAS_CLC_SBWE_OVERWRT = 1
};

enum E_MEAS_CLC_FSOE {
	MEAS_CLC_FSOE_FCLKSWON = 0,
	MEAS_CLC_FSOE_FCLKSWOFF = 1
};

/** @brief MEAS_ID register description at address offset 0x8
  *
  *  MEAS Configuration Register
  */
struct meas_id_s {
	const HWREG_U32 REVN:8; /**<  */
	const HWREG_U32 TYPE:8; /**<  */
	const HWREG_U32 MODN:16; /**<  */
};


/** @brief MEAS_ID Bitfield widths
*/
#define LENMEAS_ID_REVN 8
#define LENMEAS_ID_TYPE 8
#define LENMEAS_ID_MODN 16

enum E_MEAS_IDLSB {
	MEAS_ID_LSB_REVN = 0,
	MEAS_ID_LSB_TYPE = 8,
	MEAS_ID_LSB_MODN = 16
};

union U_MEAS_ID {
	HWREG_U32 val;
	struct meas_id_s s;
};

/** @brief MEAS_RUN_CTRL register description at address offset 0x10
  *
  *  MEAS Run Control Register
  */
struct S_MEAS_RUN_CTRLSTRUCTURE {
	HWREG_U32 RUN:1; /**<  */
	const HWREG_U32 Res0:31; /**< Reserved bitfield */
};


/** @brief MEAS_RUN_CTRL Bitfield widths
*/
#define LENMEAS_RUN_CTRL_RUN 1

enum E_MEAS_RUN_CTRLLSB {
	MEAS_RUN_CTRL_LSB_RUN = 0
};

union U_MEAS_RUN_CTRL {
	HWREG_U32 val;
	struct S_MEAS_RUN_CTRLSTRUCTURE MEAS_RUN_CTRL_STRUCTURE;
};

enum E_MEAS_RUN_CTRL_RUN {
	MEAS_RUN_CTRL_RUN_CONFIG = 0,
	MEAS_RUN_CTRL_RUN_ACTIVE = 1
};

/** @brief MEAS_IDINT register description at address offset 0x14
  *
  *  Internal MEAS Identification Register
  */
struct S_MEAS_IDINTSTRUCTURE {
	const HWREG_U32 ANALOG_ID:4; /**<  */
	const HWREG_U32:28; /**< Unused bitfield */
};


/** @brief MEAS_IDINT Bitfield widths
*/
#define LENMEAS_IDINT_ANALOG_ID 4

enum E_MEAS_IDINTLSB {
	MEAS_IDINT_LSB_ANALOG_ID = 0
};

union U_MEAS_IDINT {
	HWREG_U32 MEAS_IDINT_CONTENT;
	struct S_MEAS_IDINTSTRUCTURE MEAS_IDINT_STRUCTURE;
};

/** @brief MEAS_CONF register description at address offset 0x18
  *
  *  MEAS Configuration Register
  */
struct s_meas_conf {
	HWREG_U32 FREQ:4; /**<  */
	HWREG_U32 BUFSIZE:3; /**<  */
	HWREG_U32 BLOCKSIZE:3; /**<  */
	HWREG_U32 ENSTEPA:1; /**<  */
	HWREG_U32 DISA:1; /**<  */
	HWREG_U32 ENTRIGA:2; /**<  */
	HWREG_U32 SWTRIGA:1; /**<  */
	HWREG_U32 STARTA:1; /**<  */
	HWREG_U32 ADCON:1; /**<  */
	HWREG_U32 TSON:1; /**<  */
	HWREG_U32 ENPENINT:1; /**<  */
	HWREG_U32 ENIRQA:1; /**<  */
	HWREG_U32 ENIRQB:1; /**<  */
	HWREG_U32 ENIRQC:1; /**<  */
	HWREG_U32 ENBLOCKC:1; /**<  */
	HWREG_U32 ENTRIGC:1; /**<  */
	HWREG_U32 SELDIS:1; /**<  */
	HWREG_U32 RES_WPTRB:1; /**<  */
	HWREG_U32 ENSTOP:1; /**<  */
	HWREG_U32 DISB:1; /**<  */
	HWREG_U32 ENTRIGB:2; /**<  */
	HWREG_U32 SWTRIGB:1; /**<  */
	HWREG_U32 STARTB:1; /**<  */
};


/** @brief MEAS_CONF Bitfield widths
*/
#define LENMEAS_CONF_FREQ 4
#define LENMEAS_CONF_BUFSIZE 3
#define LENMEAS_CONF_BLOCKSIZE 3
#define LENMEAS_CONF_ENSTEPA 1
#define LENMEAS_CONF_DISA 1
#define LENMEAS_CONF_ENTRIGA 2
#define LENMEAS_CONF_SWTRIGA 1
#define LENMEAS_CONF_STARTA 1
#define LENMEAS_CONF_ADCON 1
#define LENMEAS_CONF_TSON 1
#define LENMEAS_CONF_ENPENINT 1
#define LENMEAS_CONF_ENIRQA 1
#define LENMEAS_CONF_ENIRQB 1
#define LENMEAS_CONF_ENIRQC 1
#define LENMEAS_CONF_ENBLOCKC 1
#define LENMEAS_CONF_ENTRIGC 1
#define LENMEAS_CONF_SELDIS 1
#define LENMEAS_CONF_RES_WPTRB 1
#define LENMEAS_CONF_ENSTOP 1
#define LENMEAS_CONF_DISB 1
#define LENMEAS_CONF_ENTRIGB 2
#define LENMEAS_CONF_SWTRIGB 1
#define LENMEAS_CONF_STARTB 1

enum E_MEAS_CONFLSB {
	MEAS_CONF_LSB_FREQ = 0,
	MEAS_CONF_LSB_BUFSIZE = 4,
	MEAS_CONF_LSB_BLOCKSIZE = 7,
	MEAS_CONF_LSB_ENSTEPA = 10,
	MEAS_CONF_LSB_DISA = 11,
	MEAS_CONF_LSB_ENTRIGA = 12,
	MEAS_CONF_LSB_SWTRIGA = 14,
	MEAS_CONF_LSB_STARTA = 15,
	MEAS_CONF_LSB_ADCON = 16,
	MEAS_CONF_LSB_TSON = 17,
	MEAS_CONF_LSB_ENPENINT = 18,
	MEAS_CONF_LSB_ENIRQA = 19,
	MEAS_CONF_LSB_ENIRQB = 20,
	MEAS_CONF_LSB_ENIRQC = 21,
	MEAS_CONF_LSB_ENBLOCKC = 22,
	MEAS_CONF_LSB_ENTRIGC = 23,
	MEAS_CONF_LSB_SELDIS = 24,
	MEAS_CONF_LSB_RES_WPTRB = 25,
	MEAS_CONF_LSB_ENSTOP = 26,
	MEAS_CONF_LSB_DISB = 27,
	MEAS_CONF_LSB_ENTRIGB = 28,
	MEAS_CONF_LSB_SWTRIGB = 30,
	MEAS_CONF_LSB_STARTB = 31
};

union meas_conf {
	HWREG_U32 val;
	struct s_meas_conf s;
};

enum E_MEAS_CONF_FREQ {
	MEAS_CONF_FREQ_0 = 0,
	MEAS_CONF_FREQ_1 = 1,
	MEAS_CONF_FREQ_2 = 2,
	MEAS_CONF_FREQ_3 = 3,
	MEAS_CONF_FREQ_4 = 4,
	MEAS_CONF_FREQ_5 = 5,
	MEAS_CONF_FREQ_6 = 6,
	MEAS_CONF_FREQ_7 = 7,
	MEAS_CONF_FREQ_8 = 8,
	MEAS_CONF_FREQ_9 = 9,
	MEAS_CONF_FREQ_10 = 10,
	MEAS_CONF_FREQ_11 = 11,
	MEAS_CONF_FREQ_12 = 12,
	MEAS_CONF_FREQ_13 = 13,
	MEAS_CONF_FREQ_14 = 14,
	MEAS_CONF_FREQ_15 = 15
};

enum E_MEAS_CONF_BUFSIZE {
	MEAS_CONF_BUFSIZE_A0 = 0,
	MEAS_CONF_BUFSIZE_A0A1 = 1,
	MEAS_CONF_BUFSIZE_A0A2 = 2,
	MEAS_CONF_BUFSIZE_A0A3 = 3,
	MEAS_CONF_BUFSIZE_A0A4 = 4,
	MEAS_CONF_BUFSIZE_A0A5 = 5,
	MEAS_CONF_BUFSIZE_A0A6 = 6,
	MEAS_CONF_BUFSIZE_A0A7 = 7
};

enum E_MEAS_CONF_BLOCKSIZE {
	MEAS_CONF_BLOCKSIZE_A0 = 0,
	MEAS_CONF_BLOCKSIZE_A0A1 = 1,
	MEAS_CONF_BLOCKSIZE_A0A2 = 2,
	MEAS_CONF_BLOCKSIZE_A0A3 = 3,
	MEAS_CONF_BLOCKSIZE_A0A4 = 4,
	MEAS_CONF_BLOCKSIZE_A0A5 = 5,
	MEAS_CONF_BLOCKSIZE_A0A6 = 6,
	MEAS_CONF_BLOCKSIZE_A0A7 = 7
};

enum E_MEAS_CONF_ENSTEPA {
	MEAS_CONF_ENSTEPA_BLOCKMEAS = 0,
	MEAS_CONF_ENSTEPA_SEQMEAS = 1
};

enum E_MEAS_CONF_ENTRIGA {
	MEAS_CONF_ENTRIGA_NOTRIG = 0,
	MEAS_CONF_ENTRIGA_ADCTRIG = 1,
	MEAS_CONF_ENTRIGA_EXTTRIG = 2,
	MEAS_CONF_ENTRIGA_SWTRIG = 3
};

enum E_MEAS_CONF_ADCON {
	MEAS_CONF_ADCON_0 = 0,
	MEAS_CONF_ADCON_1 = 1
};

enum E_MEAS_CONF_ENPENINT {
	MEAS_CONF_ENPENINT_0 = 0,
	MEAS_CONF_ENPENINT_1 = 1
};

enum E_MEAS_CONF_ENIRQA {
	MEAS_CONF_ENIRQA_0 = 0,
	MEAS_CONF_ENIRQA_1 = 1
};

enum E_MEAS_CONF_ENIRQB {
	MEAS_CONF_ENIRQB_0 = 0,
	MEAS_CONF_ENIRQB_1 = 1
};

enum E_MEAS_CONF_ENIRQC {
	MEAS_CONF_ENIRQC_0 = 0,
	MEAS_CONF_ENIRQC_1 = 1
};

enum E_MEAS_CONF_ENBLOCKC {
	MEAS_CONF_ENBLOCKC_0 = 0,
	MEAS_CONF_ENBLOCKC_1 = 1
};

enum E_MEAS_CONF_SELDIS {
	MEAS_CONF_SELDIS_0 = 0,
	MEAS_CONF_SELDIS_1 = 1
};

enum E_MEAS_CONF_ENSTOP {
	MEAS_CONF_ENSTOP_0 = 0,
	MEAS_CONF_ENSTOP_1 = 1
};

enum E_MEAS_CONF_ENTRIGB {
	MEAS_CONF_ENTRIGB_NOTRIG = 0,
	MEAS_CONF_ENTRIGB_ADCTRIG = 1,
	MEAS_CONF_ENTRIGB_EXTTRIG = 2,
	MEAS_CONF_ENTRIGB_SWTRIG = 3
};

/** @brief MEAS_CTRL_A0 register description at address offset 0x1c
  *
  *  MEAS Control Register A0
  */
struct S_MEAS_CTRL_ASTRUCTURE {
	HWREG_U32 MXAX:5; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
	HWREG_U32 GAX:1; /**<  */
	const HWREG_U32:2; /**< Unused bitfield */
	HWREG_U32 TCAX:3; /**<  */
	const HWREG_U32:1; /**< Unused bitfield */
	HWREG_U32 SETTLINGAX:2; /**<  */
	const HWREG_U32:5; /**< Unused bitfield */
	HWREG_U32 CONTAX:1; /**<  */
	HWREG_U32 BYPAX:1; /**<  */
	HWREG_U32 MAVGAX:2; /**<  */
	HWREG_U32 AVGAX:1; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
};


/** @brief MEAS_CTRL_A Bitfield widths
*/
#define LENMEAS_CTRL_A_MXAX 5
#define LENMEAS_CTRL_A_GAX 1
#define LENMEAS_CTRL_A_TCAX 3
#define LENMEAS_CTRL_A_SETTLINGAX 2
#define LENMEAS_CTRL_A_CONTAX 1
#define LENMEAS_CTRL_A_BYPAX 1
#define LENMEAS_CTRL_A_MAVGAX 2
#define LENMEAS_CTRL_A_AVGAX 1

enum E_MEAS_CTRL_ALSB {
	MEAS_CTRL_A_LSB_MXAX = 0,
	MEAS_CTRL_A_LSB_GAX = 9,
	MEAS_CTRL_A_LSB_TCAX = 12,
	MEAS_CTRL_A_LSB_SETTLINGAX = 16,
	MEAS_CTRL_A_LSB_CONTAX = 23,
	MEAS_CTRL_A_LSB_BYPAX = 24,
	MEAS_CTRL_A_LSB_MAVGAX = 25,
	MEAS_CTRL_A_LSB_AVGAX = 27
};

union meas_ctrl_ax {
	HWREG_U32 MEAS_CTRL_A_CONTENT;
	struct S_MEAS_CTRL_ASTRUCTURE MEAS_CTRL_A_STRUCTURE;
};

enum E_MEAS_CTRL_A_MXAX {
	MEAS_CTRL_A_MXAX_OFF = 0,
	MEAS_CTRL_A_MXAX_M0 = 1,
	MEAS_CTRL_A_MXAX_M1 = 2,
	MEAS_CTRL_A_MXAX_M2 = 3,
	MEAS_CTRL_A_MXAX_M3 = 4,
	MEAS_CTRL_A_MXAX_M4 = 5,
	MEAS_CTRL_A_MXAX_M5 = 6,
	MEAS_CTRL_A_MXAX_M6 = 7,
	MEAS_CTRL_A_MXAX_M7 = 8,
	MEAS_CTRL_A_MXAX_M8 = 9,
	MEAS_CTRL_A_MXAX_M9 = 10,
	MEAS_CTRL_A_MXAX_M10 = 11,
	MEAS_CTRL_A_MXAX_M11 = 12,
	MEAS_CTRL_A_MXAX_M12 = 13,
	MEAS_CTRL_A_MXAX_M13 = 14,
	MEAS_CTRL_A_MXAX_M15 = 15,
	MEAS_CTRL_A_MXAX_TEMPCALIB = 16,
	MEAS_CTRL_A_MXAX_TEMPDEBUG = 17,
	MEAS_CTRL_A_MXAX_OT1 = 18,
	MEAS_CTRL_A_MXAX_OT2 = 19,
	MEAS_CTRL_A_MXAX_OT3 = 20,
	MEAS_CTRL_A_MXAX_NA2 = 21,
	MEAS_CTRL_A_MXAX_NA3 = 22,
	MEAS_CTRL_A_MXAX_NA4 = 23,
	MEAS_CTRL_A_MXAX_NA5 = 24
};

enum E_MEAS_CTRL_A_GAX {
	MEAS_CTRL_A_GAX_0 = 0,
	MEAS_CTRL_A_GAX_1 = 1
};

enum E_MEAS_CTRL_A_TCAX {
	MEAS_CTRL_A_TCAX_0 = 0,
	MEAS_CTRL_A_TCAX_1 = 1,
	MEAS_CTRL_A_TCAX_2 = 2,
	MEAS_CTRL_A_TCAX_3 = 3,
	MEAS_CTRL_A_TCAX_4 = 4,
	MEAS_CTRL_A_TCAX_5 = 5,
	MEAS_CTRL_A_TCAX_6 = 6,
	MEAS_CTRL_A_TCAX_7 = 7
};

enum E_MEAS_CTRL_A_SETTLINGAX {
	MEAS_CTRL_A_SETTLINGAX_0 = 0,
	MEAS_CTRL_A_SETTLINGAX_1 = 1,
	MEAS_CTRL_A_SETTLINGAX_2 = 2,
	MEAS_CTRL_A_SETTLINGAX_3 = 3
};

enum E_MEAS_CTRL_A_CONTAX {
	MEAS_CTRL_A_CONTAX_0 = 0,
	MEAS_CTRL_A_CONTAX_1 = 1
};

enum E_MEAS_CTRL_A_BYPAX {
	MEAS_CTRL_A_BYPAX_0 = 0,
	MEAS_CTRL_A_BYPAX_1 = 1
};

enum E_MEAS_CTRL_A_MAVGAX {
	MEAS_CTRL_A_MAVGAX_0 = 0,
	MEAS_CTRL_A_MAVGAX_1 = 1,
	MEAS_CTRL_A_MAVGAX_2 = 2,
	MEAS_CTRL_A_MAVGAX_3 = 3
};

enum E_MEAS_CTRL_A_AVGAX {
	MEAS_CTRL_A_AVGAX_0 = 0,
	MEAS_CTRL_A_AVGAX_1 = 1
};

/** @brief MEAS_CTRL_B0 register description at address offset 0x3c
  *
  *  MEAS Control Register B0
  */
struct S_MEAS_CTRL_BSTRUCTURE {
	HWREG_U32 MXBY:5; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
	HWREG_U32 GBY:1; /**<  */
	const HWREG_U32:2; /**< Unused bitfield */
	HWREG_U32 TCBY:3; /**<  */
	const HWREG_U32:1; /**< Unused bitfield */
	HWREG_U32 SETTLINGBY:2; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
	HWREG_U32 SYNCBY:1; /**<  */
	HWREG_U32 CONTBY:1; /**<  */
	HWREG_U32 BYPBY:1; /**<  */
	HWREG_U32 MAVGBY:2; /**<  */
	HWREG_U32 AVGBY:1; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
};


/** @brief MEAS_CTRL_B Bitfield widths
*/
#define LENMEAS_CTRL_B_MXBY 5
#define LENMEAS_CTRL_B_GBY 1
#define LENMEAS_CTRL_B_TCBY 3
#define LENMEAS_CTRL_B_SETTLINGBY 2
#define LENMEAS_CTRL_B_SYNCBY 1
#define LENMEAS_CTRL_B_CONTBY 1
#define LENMEAS_CTRL_B_BYPBY 1
#define LENMEAS_CTRL_B_MAVGBY 2
#define LENMEAS_CTRL_B_AVGBY 1

enum E_MEAS_CTRL_BLSB {
	MEAS_CTRL_B_LSB_MXBY = 0,
	MEAS_CTRL_B_LSB_GBY = 9,
	MEAS_CTRL_B_LSB_TCBY = 12,
	MEAS_CTRL_B_LSB_SETTLINGBY = 16,
	MEAS_CTRL_B_LSB_SYNCBY = 22,
	MEAS_CTRL_B_LSB_CONTBY = 23,
	MEAS_CTRL_B_LSB_BYPBY = 24,
	MEAS_CTRL_B_LSB_MAVGBY = 25,
	MEAS_CTRL_B_LSB_AVGBY = 27
};

union meas_ctrl_bx {
	HWREG_U32 val;
	struct S_MEAS_CTRL_BSTRUCTURE s;
};

enum E_MEAS_CTRL_B_MXBY {
	MEAS_CTRL_B_MXBY_OFF = 0,
	MEAS_CTRL_B_MXBY_M0 = 1,
	MEAS_CTRL_B_MXBY_M1 = 2,
	MEAS_CTRL_B_MXBY_M2 = 3,
	MEAS_CTRL_B_MXBY_M3 = 4,
	MEAS_CTRL_B_MXBY_M4 = 5,
	MEAS_CTRL_B_MXBY_M5 = 6,
	MEAS_CTRL_B_MXBY_M6 = 7,
	MEAS_CTRL_B_MXBY_M7 = 8,
	MEAS_CTRL_B_MXBY_M8 = 9,
	MEAS_CTRL_B_MXBY_M9 = 10,
	MEAS_CTRL_B_MXBY_M10 = 11,
	MEAS_CTRL_B_MXBY_M11 = 12,
	MEAS_CTRL_B_MXBY_M12 = 13,
	MEAS_CTRL_B_MXBY_M13 = 14,
	MEAS_CTRL_B_MXBY_M15 = 15,
	MEAS_CTRL_B_MXBY_TEMPCALIB = 16,
	MEAS_CTRL_B_MXBY_TEMPDEBUG = 17,
	MEAS_CTRL_B_MXBY_OT1 = 18,
	MEAS_CTRL_B_MXBY_OT2 = 19,
	MEAS_CTRL_B_MXBY_OT3 = 20,
	MEAS_CTRL_B_MXBY_NA2 = 21,
	MEAS_CTRL_B_MXBY_NA4 = 23,
	MEAS_CTRL_B_MXBY_NA5 = 24
};

enum E_MEAS_CTRL_B_GBY {
	MEAS_CTRL_B_GBY_0 = 0,
	MEAS_CTRL_B_GBY_1 = 1
};

enum E_MEAS_CTRL_B_TCBY {
	MEAS_CTRL_B_TCBY_0 = 0,
	MEAS_CTRL_B_TCBY_1 = 1,
	MEAS_CTRL_B_TCBY_2 = 2,
	MEAS_CTRL_B_TCBY_3 = 3,
	MEAS_CTRL_B_TCBY_4 = 4,
	MEAS_CTRL_B_TCBY_5 = 5,
	MEAS_CTRL_B_TCBY_6 = 6,
	MEAS_CTRL_B_TCBY_7 = 7
};

enum E_MEAS_CTRL_B_SETTLINGBY {
	MEAS_CTRL_B_SETTLINGBY_0 = 0,
	MEAS_CTRL_B_SETTLINGBY_1 = 1,
	MEAS_CTRL_B_SETTLINGBY_2 = 2,
	MEAS_CTRL_B_SETTLINGBY_3 = 3
};

enum E_MEAS_CTRL_B_SYNCBY {
	MEAS_CTRL_B_SYNCBY_0 = 0,
	MEAS_CTRL_B_SYNCBY_1 = 1
};

enum E_MEAS_CTRL_B_CONTBY {
	MEAS_CTRL_B_CONTBY_0 = 0,
	MEAS_CTRL_B_CONTBY_1 = 1
};

enum E_MEAS_CTRL_B_BYPBY {
	MEAS_CTRL_B_BYPBY_0 = 0,
	MEAS_CTRL_B_BYPBY_1 = 1
};

enum E_MEAS_CTRL_B_MAVGBY {
	MEAS_CTRL_B_MAVGBY_0 = 0,
	MEAS_CTRL_B_MAVGBY_1 = 1,
	MEAS_CTRL_B_MAVGBY_2 = 2,
	MEAS_CTRL_B_MAVGBY_3 = 3
};

enum E_MEAS_CTRL_B_AVGBY {
	MEAS_CTRL_B_AVGBY_0 = 0,
	MEAS_CTRL_B_AVGBY_1 = 1
};

/** @brief MEAS_CTRL_C0 register description at address offset 0x5c
  *
  *  MEAS Control Register C0
  */
struct S_MEAS_CTRL_CSTRUCTURE {
	HWREG_U32 MXCZ:5; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
	HWREG_U32 GCZ:1; /**<  */
	const HWREG_U32:2; /**< Unused bitfield */
	HWREG_U32 TCCZ:3; /**<  */
	const HWREG_U32:1; /**< Unused bitfield */
	HWREG_U32 SETTLINGCZ:2; /**<  */
	const HWREG_U32:6; /**< Unused bitfield */
	HWREG_U32 BYPCZ:1; /**<  */
	HWREG_U32 MAVGCZ:2; /**<  */
	HWREG_U32 AVGCZ:1; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
};


/** @brief MEAS_CTRL_C Bitfield widths
*/
#define LENMEAS_CTRL_C_MXCZ 5
#define LENMEAS_CTRL_C_GCZ 1
#define LENMEAS_CTRL_C_TCCZ 3
#define LENMEAS_CTRL_C_SETTLINGCZ 2
#define LENMEAS_CTRL_C_BYPCZ 1
#define LENMEAS_CTRL_C_MAVGCZ 2
#define LENMEAS_CTRL_C_AVGCZ 1

enum E_MEAS_CTRL_CLSB {
	MEAS_CTRL_C_LSB_MXCZ = 0,
	MEAS_CTRL_C_LSB_GCZ = 9,
	MEAS_CTRL_C_LSB_TCCZ = 12,
	MEAS_CTRL_C_LSB_SETTLINGCZ = 16,
	MEAS_CTRL_C_LSB_BYPCZ = 24,
	MEAS_CTRL_C_LSB_MAVGCZ = 25,
	MEAS_CTRL_C_LSB_AVGCZ = 27
};

union meas_ctrl_cx {
	HWREG_U32 MEAS_CTRL_C_CONTENT;
	struct S_MEAS_CTRL_CSTRUCTURE MEAS_CTRL_C_STRUCTURE;
};

enum E_MEAS_CTRL_C_MXCZ {
	MEAS_CTRL_C_MXCZ_OFF = 0,
	MEAS_CTRL_C_MXCZ_M0 = 1,
	MEAS_CTRL_C_MXCZ_M1 = 2,
	MEAS_CTRL_C_MXCZ_M2 = 3,
	MEAS_CTRL_C_MXCZ_M3 = 4,
	MEAS_CTRL_C_MXCZ_M4 = 5,
	MEAS_CTRL_C_MXCZ_M5 = 6,
	MEAS_CTRL_C_MXCZ_M6 = 7,
	MEAS_CTRL_C_MXCZ_M7 = 8,
	MEAS_CTRL_C_MXCZ_M8 = 9,
	MEAS_CTRL_C_MXCZ_M9 = 10,
	MEAS_CTRL_C_MXCZ_M10 = 11,
	MEAS_CTRL_C_MXCZ_M11 = 12,
	MEAS_CTRL_C_MXCZ_M12 = 13,
	MEAS_CTRL_C_MXCZ_M13 = 14,
	MEAS_CTRL_C_MXCZ_M15 = 15,
	MEAS_CTRL_C_MXCZ_TEMPCALIB = 16,
	MEAS_CTRL_C_MXCZ_TEMPDEBUG = 17,
	MEAS_CTRL_C_MXCZ_OT1 = 18,
	MEAS_CTRL_C_MXCZ_OT2 = 19,
	MEAS_CTRL_C_MXCZ_OT3 = 20,
	MEAS_CTRL_C_MXCZ_NA2 = 21,
	MEAS_CTRL_C_MXCZ_NA4 = 23,
	MEAS_CTRL_C_MXCZ_NA5 = 24
};

enum E_MEAS_CTRL_C_GCZ {
	MEAS_CTRL_C_GCZ_0 = 0,
	MEAS_CTRL_C_GCZ_1 = 1
};

enum E_MEAS_CTRL_C_TCCZ {
	MEAS_CTRL_C_TCCZ_0 = 0,
	MEAS_CTRL_C_TCCZ_1 = 1,
	MEAS_CTRL_C_TCCZ_2 = 2,
	MEAS_CTRL_C_TCCZ_3 = 3,
	MEAS_CTRL_C_TCCZ_4 = 4,
	MEAS_CTRL_C_TCCZ_5 = 5,
	MEAS_CTRL_C_TCCZ_6 = 6,
	MEAS_CTRL_C_TCCZ_7 = 7
};

enum E_MEAS_CTRL_C_SETTLINGCZ {
	MEAS_CTRL_C_SETTLINGCZ_0 = 0,
	MEAS_CTRL_C_SETTLINGCZ_1 = 1,
	MEAS_CTRL_C_SETTLINGCZ_2 = 2
};

enum E_MEAS_CTRL_C_BYPCZ {
	MEAS_CTRL_C_BYPCZ_0 = 0,
	MEAS_CTRL_C_BYPCZ_1 = 1
};

enum E_MEAS_CTRL_C_MAVGCZ {
	MEAS_CTRL_C_MAVGCZ_0 = 0,
	MEAS_CTRL_C_MAVGCZ_1 = 1,
	MEAS_CTRL_C_MAVGCZ_2 = 2,
	MEAS_CTRL_C_MAVGCZ_3 = 3
};

enum E_MEAS_CTRL_C_AVGCZ {
	MEAS_CTRL_C_AVGCZ_0 = 0,
	MEAS_CTRL_C_AVGCZ_1 = 1
};

/** @brief MEAS_CLK register description at address offset 0x6c
  *
  *  MEAS Peripheral Clock Control Register
  */
struct S_MEAS_CLKSTRUCTURE {
	HWREG_U32 K:5; /**<  */
	const HWREG_U32:3; /**< Unused bitfield */
	HWREG_U32 L:7; /**<  */
	const HWREG_U32:17; /**< Unused bitfield */
};


/** @brief MEAS_CLK Bitfield widths
*/
#define LENMEAS_CLK_K 5
#define LENMEAS_CLK_L 7

enum E_MEAS_CLKLSB {
	MEAS_CLK_LSB_K = 0,
	MEAS_CLK_LSB_L = 8
};

union U_MEAS_CLK {
	HWREG_U32 val;
	struct S_MEAS_CLKSTRUCTURE MEAS_CLK_STRUCTURE;
};

/** @brief MEAS_SWSTAT0 register description at address offset 0x70
  *
  *  MEAS Software Status Register 0
  */
struct S_MEAS_SWSTAT0STRUCTURE {
	HWREG_U32 SWSTAT0:8; /**<  */
	const HWREG_U32:24; /**< Unused bitfield */
};

/** @brief MEAS_SWSTAT0 Bitfield widths
*/
#define LENMEAS_SWSTAT0_SWSTAT0 8

enum E_MEAS_SWSTAT0LSB {
	MEAS_SWSTAT0_LSB_SWSTAT0 = 0
};

union U_MEAS_SWSTAT0 {
	HWREG_U32 MEAS_SWSTAT0_CONTENT;
	struct S_MEAS_SWSTAT0STRUCTURE MEAS_SWSTAT0_STRUCTURE;
};

/** @brief MEAS_SWSTAT1ORIG register description at address offset 0x74
  *
  *  MEAS Software Status Register 1 Original
  */
struct S_MEAS_SWSTAT1ORIGSTRUCTURE {
	HWREG_U32 SWSTAT1ORIG:12; /**<  */
	const HWREG_U32:20; /**< Unused bitfield */
};


/** @brief MEAS_SWSTAT1ORIG Bitfield widths
*/
#define LENMEAS_SWSTAT1ORIG_SWSTAT1ORIG 12

enum E_MEAS_SWSTAT1ORIGLSB {
	MEAS_SWSTAT1ORIG_LSB_SWSTAT1ORIG = 0
};

union U_MEAS_SWSTAT1ORIG {
	HWREG_U32 MEAS_SWSTAT1ORIG_CONTENT;
	struct S_MEAS_SWSTAT1ORIGSTRUCTURE MEAS_SWSTAT1ORIG_STRUCTURE;
};

/** @brief MEAS_SWSTAT1 register description at address offset 0x78
  *
  *  MEAS Software Status Register 1
  */
struct S_MEAS_SWSTAT1STRUCTURE {
	const HWREG_U32 SWSTAT1:12; /**<  */
	const HWREG_U32:20; /**< Unused bitfield */
};


/** @brief MEAS_SWSTAT1 Bitfield widths
*/
#define LENMEAS_SWSTAT1_SWSTAT1 12

enum E_MEAS_SWSTAT1LSB {
	MEAS_SWSTAT1_LSB_SWSTAT1 = 0
};

union U_MEAS_SWSTAT1 {
	HWREG_U32 MEAS_SWSTAT1_CONTENT;
	struct S_MEAS_SWSTAT1STRUCTURE MEAS_SWSTAT1_STRUCTURE;
};

/** @brief MEAS_STAT register description at address offset 0x7c
  *
  *  MEAS Status Register
  */
struct S_MEAS_STATSTRUCTURE {
	const HWREG_U32 WPTRA:3; /**<  */
	const HWREG_U32 READYA:1; /**<  */
	const HWREG_U32 WPTRB:3; /**<  */
	const HWREG_U32 READYB:1; /**<  */
	const HWREG_U32 WPTRC:2; /**<  */
	const HWREG_U32 RESV1:1; /**< Unused bitfield */
	const HWREG_U32 READYC:1; /**<  */
	const HWREG_U32 RESX:3; /**<  */
	const HWREG_U32 RESV2:9; /**< Unused bitfield */
	const HWREG_U32 STATE:4; /**<  */
	const HWREG_U32 ERROR:1; /**<  */
	const HWREG_U32 PENINT:1; /**<  */
	const HWREG_U32 BUSYA:1; /**<  */
	const HWREG_U32 BUSYADC:1; /**<  */
};


/** @brief MEAS_STAT Bitfield widths
*/
#define LENMEAS_STAT_WPTRA 3
#define LENMEAS_STAT_READYA 1
#define LENMEAS_STAT_WPTRB 3
#define LENMEAS_STAT_READYB 1
#define LENMEAS_STAT_WPTRC 2
#define LENMEAS_STAT_READYC 1
#define LENMEAS_STAT_RESX 3
#define LENMEAS_STAT_STATE 4
#define LENMEAS_STAT_ERROR 1
#define LENMEAS_STAT_PENINT 1
#define LENMEAS_STAT_BUSYA 1
#define LENMEAS_STAT_BUSYADC 1

enum E_MEAS_STATLSB {
	MEAS_STAT_LSB_WPTRA = 0,
	MEAS_STAT_LSB_READYA = 3,
	MEAS_STAT_LSB_WPTRB = 4,
	MEAS_STAT_LSB_READYB = 7,
	MEAS_STAT_LSB_WPTRC = 8,
	MEAS_STAT_LSB_READYC = 11,
	MEAS_STAT_LSB_RESX = 12,
	MEAS_STAT_LSB_STATE = 24,
	MEAS_STAT_LSB_ERROR = 28,
	MEAS_STAT_LSB_PENINT = 29,
	MEAS_STAT_LSB_BUSYA = 30,
	MEAS_STAT_LSB_BUSYADC = 31
};

union meas_stat {
	HWREG_U32 val;
	struct S_MEAS_STATSTRUCTURE s;
};

enum E_MEAS_STAT_WPTRA {
	MEAS_STAT_WPTRA_A0 = 0,
	MEAS_STAT_WPTRA_A0A1 = 1,
	MEAS_STAT_WPTRA_A0A2 = 2,
	MEAS_STAT_WPTRA_A0A3 = 3,
	MEAS_STAT_WPTRA_A0A4 = 4,
	MEAS_STAT_WPTRA_A0A5 = 5,
	MEAS_STAT_WPTRA_A0A6 = 6,
	MEAS_STAT_WPTRA_A0A7 = 7
};

enum E_MEAS_STAT_WPTRB {
	MEAS_STAT_WPTRB_B0 = 0,
	MEAS_STAT_WPTRB_B0B1 = 1,
	MEAS_STAT_WPTRB_B0B2 = 2,
	MEAS_STAT_WPTRB_B0B3 = 3,
	MEAS_STAT_WPTRB_B0B4 = 4,
	MEAS_STAT_WPTRB_B0B5 = 5,
	MEAS_STAT_WPTRB_B0B6 = 6,
	MEAS_STAT_WPTRB_B0B7 = 7
};

enum E_MEAS_STAT_WPTRC {
	MEAS_STAT_WPTRC_C0 = 0,
	MEAS_STAT_WPTRC_C1 = 1,
	MEAS_STAT_WPTRC_C2 = 2,
	MEAS_STAT_WPTRC_C3 = 3
};

enum E_MEAS_STAT_STATE {
	MEAS_STAT_STATE_0 = 0,
	MEAS_STAT_STATE_1 = 1,
	MEAS_STAT_STATE_2 = 2,
	MEAS_STAT_STATE_3 = 3,
	MEAS_STAT_STATE_4 = 4,
	MEAS_STAT_STATE_5 = 5,
	MEAS_STAT_STATE_6 = 6,
	MEAS_STAT_STATE_7 = 7,
	MEAS_STAT_STATE_8 = 8,
	MEAS_STAT_STATE_9 = 9,
	MEAS_STAT_STATE_10 = 10,
	MEAS_STAT_STATE_11 = 11,
	MEAS_STAT_STATE_12 = 12,
	MEAS_STAT_STATE_13 = 13,
	MEAS_STAT_STATE_14 = 14,
	MEAS_STAT_STATE_15 = 15
};

enum E_MEAS_STAT_BUSYA {
	MEAS_STAT_BUSYA_0 = 0,
	MEAS_STAT_BUSYA_1 = 1
};

/** @brief MEAS_DATA_A0 register description at address offset 0x80
  *
  *  MEAS Data Register A0
  */
struct S_MEAS_DATA_ASTRUCTURE {
	const HWREG_U32 DATA_AX:12; /**<  */
	const HWREG_U32 SWSTAT0_AX:8; /**<  */
	const HWREG_U32 SWSTAT1_AX:12; /**<  */
};


/** @brief MEAS_DATA_A Bitfield widths
*/
#define LENMEAS_DATA_A_DATA_AX 12
#define LENMEAS_DATA_A_SWSTAT0_AX 8
#define LENMEAS_DATA_A_SWSTAT1_AX 12

enum E_MEAS_DATA_ALSB {
	MEAS_DATA_A_LSB_DATA_AX = 0,
	MEAS_DATA_A_LSB_SWSTAT0_AX = 12,
	MEAS_DATA_A_LSB_SWSTAT1_AX = 20
};

union meas_data_a {
	HWREG_U32 MEAS_DATA_A_CONTENT;
	struct S_MEAS_DATA_ASTRUCTURE MEAS_DATA_A_STRUCTURE;
};

/** @brief MEAS_DATA_B0 register description at address offset 0xa0
  *
  *  MEAS Data Register B0
  */
struct S_MEAS_DATA_BSTRUCTURE {
	const HWREG_U32 DATA_BY:12; /**<  */
	const HWREG_U32 SWSTAT0_BY:8; /**<  */
	const HWREG_U32 SWSTAT1_BY:12; /**<  */
};


/** @brief MEAS_DATA_B Bitfield widths
*/
#define LENMEAS_DATA_B_DATA_BY 12
#define LENMEAS_DATA_B_SWSTAT0_BY 8
#define LENMEAS_DATA_B_SWSTAT1_BY 12

enum E_MEAS_DATA_BLSB {
	MEAS_DATA_B_LSB_DATA_BY = 0,
	MEAS_DATA_B_LSB_SWSTAT0_BY = 12,
	MEAS_DATA_B_LSB_SWSTAT1_BY = 20
};

union meas_data_b {
	HWREG_U32 val;
	struct S_MEAS_DATA_BSTRUCTURE s;
};

/** @brief MEAS_DATA_C0 register description at address offset 0xc0
  *
  *  MEAS Data Register C0
  */
struct S_MEAS_DATA_CSTRUCTURE {
	const HWREG_U32 DATA_CZ:12; /**<  */
	const HWREG_U32 SWSTAT0_CZ:8; /**<  */
	const HWREG_U32 SWSTAT1_CZ:12; /**<  */
};


/** @brief MEAS_DATA_C Bitfield widths
*/
#define LENMEAS_DATA_C_DATA_CZ 12
#define LENMEAS_DATA_C_SWSTAT0_CZ 8
#define LENMEAS_DATA_C_SWSTAT1_CZ 12

enum E_MEAS_DATA_CLSB {
	MEAS_DATA_C_LSB_DATA_CZ = 0,
	MEAS_DATA_C_LSB_SWSTAT0_CZ = 12,
	MEAS_DATA_C_LSB_SWSTAT1_CZ = 20
};

union meas_data_c {
	HWREG_U32 MEAS_DATA_C_CONTENT;
	struct S_MEAS_DATA_CSTRUCTURE MEAS_DATA_C_STRUCTURE;
};

/** @brief MEAS_RIS register description at address offset 0xd0
  *
  *  MEAS RIS Register
  */
struct S_MEAS_RISSTRUCTURE {
	const HWREG_U32:8; /**< Unused bitfield */
	const HWREG_U32 MEAS_RDY:1; /**<  */
	const HWREG_U32:23; /**< Unused bitfield */
};


/** @brief MEAS_RIS Bitfield widths
*/
#define LENMEAS_RIS_MEAS_RDY 1

enum E_MEAS_RISLSB {
	MEAS_RIS_LSB_MEAS_RDY = 8
};

union U_MEAS_RIS {
	HWREG_U32 MEAS_RIS_CONTENT;
	struct S_MEAS_RISSTRUCTURE MEAS_RIS_STRUCTURE;
};

enum E_MEAS_RIS_MEAS_RDY {
	MEAS_RIS_MEAS_RDY_NOINT = 0,
	MEAS_RIS_MEAS_RDY_PENDING = 1
};

/** @brief MEAS_IMSC register description at address offset 0xd4
  *
  *  MEAS IMSC Register
  */
struct S_MEAS_IMSCSTRUCTURE {
	const HWREG_U32:8; /**< Unused bitfield */
	HWREG_U32 MEAS_RDY:1; /**<  */
	const HWREG_U32:23; /**< Unused bitfield */
};

/** @brief MEAS_IMSC Bitfield widths
*/
#define LENMEAS_IMSC_MEAS_RDY 1

enum E_MEAS_IMSCLSB {
	MEAS_IMSC_LSB_MEAS_RDY = 8
};

union meas_imsc {
	HWREG_U32 val;
	struct S_MEAS_IMSCSTRUCTURE s;
};

enum E_MEAS_IMSC_MEAS_RDY {
	MEAS_IMSC_MEAS_RDY_DIS = 0,
	MEAS_IMSC_MEAS_RDY_EN = 1
};

/** @brief MEAS_MIS register description at address offset 0xd8
  *
  *  MEAS MIS Register
  */
struct S_MEAS_MISSTRUCTURE {
	const HWREG_U32:8; /**< Unused bitfield */
	const HWREG_U32 MEAS_RDY:1; /**<  */
	const HWREG_U32:23; /**< Unused bitfield */
};


/** @brief MEAS_MIS Bitfield widths
*/
#define LENMEAS_MIS_MEAS_RDY 1

enum E_MEAS_MISLSB {
	MEAS_MIS_LSB_MEAS_RDY = 8
};

union U_MEAS_MIS {
	HWREG_U32 MEAS_MIS_CONTENT;
	struct S_MEAS_MISSTRUCTURE MEAS_MIS_STRUCTURE;
};

enum E_MEAS_MIS_MEAS_RDY {
	MEAS_MIS_MEAS_RDY_DISABLED = 0,
	MEAS_MIS_MEAS_RDY_ENABLED = 1
};

/** @brief MEAS_ICR register description at address offset 0xdc
  *
  *  MEAS ISR Register
  */
struct S_MEAS_ICRSTRUCTURE {
	const HWREG_U32:8; /**< Unused bitfield */
	HWREG_U32 MEAS_RDY:1; /**<  */
	const HWREG_U32:23; /**< Unused bitfield */
};


/** @brief MEAS_ICR Bitfield widths
*/
#define LENMEAS_ICR_MEAS_RDY 1

enum E_MEAS_ICRLSB {
	MEAS_ICR_LSB_MEAS_RDY = 8
};

union meas_icr {
	HWREG_U32 val;
	struct S_MEAS_ICRSTRUCTURE s;
};

enum E_MEAS_ICR_MEAS_RDY {
	MEAS_ICR_MEAS_RDY_NO_CHANGE = 0,
	MEAS_ICR_MEAS_RDY_CLEAR = 1
};

/** @brief MEAS_ISR register description at address offset 0xe0
  *
  *  MEAS ISR Register
  */
struct S_MEAS_ISRSTRUCTURE {
	const HWREG_U32:8; /**< Unused bitfield */
	HWREG_U32 MEAS_RDY:1; /**<  */
	const HWREG_U32:23; /**< Unused bitfield */
};


/** @brief MEAS_ISR Bitfield widths
*/
#define LENMEAS_ISR_MEAS_RDY 1

enum E_MEAS_ISRLSB {
	MEAS_ISR_LSB_MEAS_RDY = 8
};

union U_MEAS_ISR {
	HWREG_U32 MEAS_ISR_CONTENT;
	struct S_MEAS_ISRSTRUCTURE MEAS_ISR_STRUCTURE;
};

enum E_MEAS_ISR_MEAS_RDY {
	MEAS_ISR_MEAS_RDY_NO_CHANGE = 0,
	MEAS_ISR_MEAS_RDY_SET = 1
};

/** @brief MEAS_CALI register description at address offset 0xe8
  *
  *  MEAS Calibration Register
  */
struct S_MEAS_CALISTRUCTURE {
	HWREG_U32 Y0:12; /**<  */
	const HWREG_U32:4; /**< Unused bitfield */
	HWREG_U32 Y1:12; /**<  */
	const HWREG_U32:1; /**< Unused bitfield */
	HWREG_U32 AVP:1; /**<  */
	HWREG_U32 XBON:1; /**<  */
	HWREG_U32 BYP:1; /**<  */
};


/** @brief MEAS_CALI Bitfield widths
*/
#define LENMEAS_CALI_Y0 12
#define LENMEAS_CALI_Y1 12
#define LENMEAS_CALI_AVP 1
#define LENMEAS_CALI_XBON 1
#define LENMEAS_CALI_BYP 1

enum E_MEAS_CALILSB {
	MEAS_CALI_LSB_Y0 = 0,
	MEAS_CALI_LSB_Y1 = 16,
	MEAS_CALI_LSB_AVP = 29,
	MEAS_CALI_LSB_XBON = 30,
	MEAS_CALI_LSB_BYP = 31
};

union meas_cali {
	HWREG_U32 val;
	struct S_MEAS_CALISTRUCTURE s;
};

enum E_MEAS_CALI_AVP {
	MEAS_CALI_AVP_0 = 0,
	MEAS_CALI_AVP_1 = 1
};

enum E_MEAS_CALI_XBON {
	MEAS_CALI_XBON_0 = 0,
	MEAS_CALI_XBON_1 = 1
};

enum E_MEAS_CALI_BYP {
	MEAS_CALI_BYP_0 = 0,
	MEAS_CALI_BYP_1 = 1
};

/** @brief MEAS_TIMER_CTRL register description at address offset 0xec
  *
  *  MEAS TX Timer Register
  */
struct S_MEAS_TIMER_CTRLSTRUCTURE {
	HWREG_U32 Delay_2G:16; /**<  */
	HWREG_U32 Delay_3G:16; /**<  */
};


/** @brief MEAS_TIMER_CTRL Bitfield widths
*/
#define LENMEAS_TIMER_CTRL_DELAY_2G 16
#define LENMEAS_TIMER_CTRL_DELAY_3G 16

enum E_MEAS_TIMER_CTRLLSB {
	MEAS_TIMER_CTRL_LSB_DELAY_2G = 0,
	MEAS_TIMER_CTRL_LSB_DELAY_3G = 16
};

union U_MEAS_TIMER_CTRL {
	HWREG_U32 MEAS_TIMER_CTRL_CONTENT;
	struct S_MEAS_TIMER_CTRLSTRUCTURE MEAS_TIMER_CTRL_STRUCTURE;
};

/** @brief MEAS_PEAK register description at address offset 0xf0
  *
  *  MEAS Peak Detection Register
  */
struct s_meas_peak {
	HWREG_U32 PEAKON:1; /**<  */
	HWREG_U32 PEAKMODE:2; /**<  */
	HWREG_U32 PEAKRESET:1; /**<  */
	const HWREG_U32:28; /**< Unused bitfield */
};

/** @brief MEAS_PEAK Bitfield widths
*/
#define LENMEAS_PEAK_PEAKON 1
#define LENMEAS_PEAK_PEAKMODE 2
#define LENMEAS_PEAK_PEAKRESET 1

enum E_MEAS_PEAKLSB {
	MEAS_PEAK_LSB_PEAKON = 0,
	MEAS_PEAK_LSB_PEAKMODE = 1,
	MEAS_PEAK_LSB_PEAKRESET = 3
};

union meas_peak {
	HWREG_U32 val;
	struct s_meas_peak s;
};

enum E_MEAS_PEAK_PEAKON {
	MEAS_PEAK_PEAKON_OFF = 0,
	MEAS_PEAK_PEAKON_ON = 1
};

enum E_MEAS_PEAK_PEAKMODE {
	MEAS_PEAK_PEAKMODE_MAXDETECT = 0,
	MEAS_PEAK_PEAKMODE_MINDETECT = 1,
	MEAS_PEAK_PEAKMODE_READOUT = 2
};

enum E_MEAS_PEAK_PEAKRESET {
	MEAS_PEAK_PEAKRESET_NORESET = 0,
	MEAS_PEAK_PEAKRESET_RESET = 1
};


/** @brief MEAS_TEMP_ALERT register description at address offset 0xf4
  *
  *  MEAS Temperature Alert Register
  */
struct S_MEAS_TEMP_ALERTSTRUCTURE {
	HWREG_U32 ON:1; /**<  */
	HWREG_U32 Int105:1; /**<  */
	HWREG_U32 Int115:1; /**<  */
	HWREG_U32 Stop105:1; /**<  */
	HWREG_U32 Stop115:1; /**<  */
	const HWREG_U32:27; /**< Unused bitfield */
};


/** @brief MEAS_TEMP_ALERT Bitfield widths
*/
#define LENMEAS_TEMP_ALERT_ON 1
#define LENMEAS_TEMP_ALERT_INT105 1
#define LENMEAS_TEMP_ALERT_INT115 1
#define LENMEAS_TEMP_ALERT_STOP105 1
#define LENMEAS_TEMP_ALERT_STOP115 1

enum E_MEAS_TEMP_ALERTLSB {
	MEAS_TEMP_ALERT_LSB_ON = 0,
	MEAS_TEMP_ALERT_LSB_INT105 = 1,
	MEAS_TEMP_ALERT_LSB_INT115 = 2,
	MEAS_TEMP_ALERT_LSB_STOP105 = 3,
	MEAS_TEMP_ALERT_LSB_STOP115 = 4
};

union U_MEAS_TEMP_ALERT {
	HWREG_U32 MEAS_TEMP_ALERT_CONTENT;
	struct S_MEAS_TEMP_ALERTSTRUCTURE MEAS_TEMP_ALERT_STRUCTURE;
};

enum E_MEAS_TEMP_ALERT_ON {
	MEAS_TEMP_ALERT_ON_OFF = 0,
	MEAS_TEMP_ALERT_ON_ON = 1
};

enum E_MEAS_TEMP_ALERT_INT105 {
	MEAS_TEMP_ALERT_INT105_OFF = 0,
	MEAS_TEMP_ALERT_INT105_ON = 1
};

enum E_MEAS_TEMP_ALERT_INT115 {
	MEAS_TEMP_ALERT_INT115_OFF = 0,
	MEAS_TEMP_ALERT_INT115_ON = 1
};

enum E_MEAS_TEMP_ALERT_STOP105 {
	MEAS_TEMP_ALERT_STOP105_OFF = 0,
	MEAS_TEMP_ALERT_STOP105_ON = 1
};

enum E_MEAS_TEMP_ALERT_STOP115 {
	MEAS_TEMP_ALERT_STOP115_OFF = 0,
	MEAS_TEMP_ALERT_STOP115_ON = 1
};

enum emeasresetvalues {
	MEAS_CLC_RESET_VALUE = (int)0x3,
	MEAS_ID_RESET_VALUE = (int)0xf024c011L,
	MEAS_RUN_CTRL_RESET_VALUE = (int)0x0,
	MEAS_IDINT_RESET_VALUE = (int)0x1,
	MEAS_CONF_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A0_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A1_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A2_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A3_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A4_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A5_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A6_RESET_VALUE = (int)0x0,
	MEAS_CTRL_A7_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B0_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B1_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B2_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B3_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B4_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B5_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B6_RESET_VALUE = (int)0x0,
	MEAS_CTRL_B7_RESET_VALUE = (int)0x0,
	MEAS_CTRL_C0_RESET_VALUE = (int)0x0,
	MEAS_CTRL_C1_RESET_VALUE = (int)0x0,
	MEAS_CTRL_C2_RESET_VALUE = (int)0x0,
	MEAS_CTRL_C3_RESET_VALUE = (int)0x0,
	MEAS_CLK_RESET_VALUE = (int)0x100,
	MEAS_SWSTAT0_RESET_VALUE = (int)0x0,
	MEAS_SWSTAT1ORIG_RESET_VALUE = (int)0x0,
	MEAS_SWSTAT1_RESET_VALUE = (int)0x0,
	MEAS_STAT_RESET_VALUE = (int)0x80000000L,
	MEAS_DATA_A0_RESET_VALUE = (int)0x0,
	MEAS_DATA_A1_RESET_VALUE = (int)0x0,
	MEAS_DATA_A2_RESET_VALUE = (int)0x0,
	MEAS_DATA_A3_RESET_VALUE = (int)0x0,
	MEAS_DATA_A4_RESET_VALUE = (int)0x0,
	MEAS_DATA_A5_RESET_VALUE = (int)0x0,
	MEAS_DATA_A6_RESET_VALUE = (int)0x0,
	MEAS_DATA_A7_RESET_VALUE = (int)0x0,
	MEAS_DATA_B0_RESET_VALUE = (int)0x0,
	MEAS_DATA_B1_RESET_VALUE = (int)0x0,
	MEAS_DATA_B2_RESET_VALUE = (int)0x0,
	MEAS_DATA_B3_RESET_VALUE = (int)0x0,
	MEAS_DATA_B4_RESET_VALUE = (int)0x0,
	MEAS_DATA_B5_RESET_VALUE = (int)0x0,
	MEAS_DATA_B6_RESET_VALUE = (int)0x0,
	MEAS_DATA_B7_RESET_VALUE = (int)0x0,
	MEAS_DATA_C0_RESET_VALUE = (int)0x0,
	MEAS_DATA_C1_RESET_VALUE = (int)0x0,
	MEAS_DATA_C2_RESET_VALUE = (int)0x0,
	MEAS_DATA_C3_RESET_VALUE = (int)0x0,
	MEAS_RIS_RESET_VALUE = (int)0x0,
	MEAS_IMSC_RESET_VALUE = (int)0x3,
	MEAS_MIS_RESET_VALUE = (int)0x0,
	MEAS_ICR_RESET_VALUE = (int)0x0,
	MEAS_ISR_RESET_VALUE = (int)0x0,
	MEAS_CALI_RESET_VALUE = (int)0xd500800,
	MEAS_TIMER_CTRL_RESET_VALUE = (int)0x0,
	MEAS_PEAK_RESET_VALUE = (int)0x0,
	MEAS_TEMP_ALERT_RESET_VALUE = (int)0x12
};


/* Re-definition of the ADC physical channels to abstract HW version specific
information */
enum intel_adc_phy_channel {
	ADC_PHY_OFF = MEAS_CTRL_B_MXBY_OFF,
	ADC_PHY_M0 = MEAS_CTRL_B_MXBY_M0,
	ADC_PHY_M1 = MEAS_CTRL_B_MXBY_M1,
	ADC_PHY_M2 = MEAS_CTRL_B_MXBY_M2,
	ADC_PHY_M3 = MEAS_CTRL_B_MXBY_M3,
	ADC_PHY_M4 = MEAS_CTRL_B_MXBY_M4,
	ADC_PHY_M5 = MEAS_CTRL_B_MXBY_M5,
	ADC_PHY_M6 = MEAS_CTRL_B_MXBY_M6,
	ADC_PHY_M7 = MEAS_CTRL_B_MXBY_M7,
	ADC_PHY_M8 = MEAS_CTRL_B_MXBY_M8,
	ADC_PHY_M9 = MEAS_CTRL_B_MXBY_M9,
	ADC_PHY_M10 = MEAS_CTRL_B_MXBY_M10,
	ADC_PHY_M11 = MEAS_CTRL_B_MXBY_M11,
	ADC_PHY_M12 = MEAS_CTRL_B_MXBY_M12,
	ADC_PHY_M13 = MEAS_CTRL_B_MXBY_M13,
	ADC_PHY_M15 = MEAS_CTRL_B_MXBY_M15
	};

#endif

