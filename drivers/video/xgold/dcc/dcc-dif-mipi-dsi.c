
/*
 ****************************************************************
 *
 *  Component: DCC driver
 *
 *  Copyright (C) 2011, Intel Mobile Communications GmbH.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#include <linux/delay.h>

#include "dcc-core.h"
#include "dcc-display.h"
#include "dcc-hwregs.h"
#include "dcc-gra.h"

/* DSI specification */
/* Procssor-sourced Packet type: DSI_M (Master) */
enum {
	DSI_M_V_SYNC_START                           = 0x01,
	DSI_M_V_SYNC_END                             = 0x11,
	DSI_M_H_SYNC_START                           = 0x21,
	DSI_M_H_SYNC_END                             = 0x31,

	DSI_M_COLOR_MODE_OFF                         = 0x02,
	DSI_M_COLOR_MODE_ON                          = 0x12,
	DSI_M_SHUTDOWN_PERIPHERAL                    = 0x22,
	DSI_M_TURN_ON_PERIPHERAL                     = 0x32,

	DSI_M_GENERIC_SHORT_WRITE_0_PARAM            = 0x03,
	DSI_M_GENERIC_SHORT_WRITE_1_PARAM            = 0x13,
	DSI_M_GENERIC_SHORT_WRITE_2_PARAM            = 0x23,

	DSI_M_GENERIC_READ_0_PARAM                   = 0x04,
	DSI_M_GENERIC_READ_1_PARAM                   = 0x14,
	DSI_M_GENERIC_READ_2_PARAM                   = 0x24,

	DSI_M_DCS_SHORT_WRITE_0_PARAM                = 0x05,
	DSI_M_DCS_SHORT_WRITE_1_PARAM                = 0x15,

	DSI_M_DCS_READ_0_PARAM                       = 0x06,

	DSI_M_SET_MAX_RETURN_PACKET_SIZE             = 0x37,

	DSI_M_EOT_PACKET                             = 0x08,

	DSI_M_NULL_PACKET                            = 0x09,
	DSI_M_BLANKING_PACKET                        = 0x19,
	DSI_M_GENERIC_LONG_WRITE                     = 0x29,
	DSI_M_DCS_LONG_WRITE                         = 0x39,

	DSI_M_PIX_STREAM_RGB565P                     = 0x0E,
	DSI_M_PIX_STREAM_RGB666P                     = 0x1E,
	DSI_M_PIX_STREAM_RGB666LP                    = 0x2E,
	DSI_M_PIX_STREAM_RGB888P                     = 0x3E,
};

/* Peripheral-sourced Packet type: DSI_S (Slave) */
enum {
	DSI_S_ACK_AND_ERR_REPORT              = 0x02,
	DSI_S_EOT_PACKET                      = 0x08,
	DSI_S_GENERIC_SHORT_READ_RESP_1BYTE   = 0x11,
	DSI_S_GENERIC_SHORT_READ_RESP_2BYTE   = 0x12,
	DSI_S_GENERIC_LONG_READ_RESP          = 0x1A,
	DSI_S_DCS_LONG_READ_RESP              = 0x1C,
	DSI_S_DCS_SHORT_READ_RESP_1BYTE       = 0x21,
	DSI_S_DCS_SHORT_READ_RESP_2BYTE       = 0x22,
};

/* #define USE_DSI_ACKNOWLEDGE */

#define DSICFG_DATA_PIX	0
#define DSICFG_DATA_DAT	1

#define DSICFG_COMMON(_en_) (\
	BITFLDS(INR_DIF_DSICFG_CRC,	0) |\
	BITFLDS(INR_DIF_DSICFG_ECC,	0) |\
	BITFLDS(INR_DIF_DSICFG_GATE,	0) |\
	BITFLDS(INR_DIF_DSICFG_EOT,	1) |\
	BITFLDS(INR_DIF_DSICFG_TURN,	0) |\
	BITFLDS(INR_DIF_DSICFG_VALID,	0) |\
	BITFLDS(INR_DIF_DSICFG_STP,	0) |\
	BITFLDS(INR_DIF_DSICFG_ULPS,	0) |\
	BITFLDS(INR_DIF_DSICFG_ID,	0) |\
	BITFLDS(INR_DIF_DSICFG_EN,	_en_) \
	)

#define DSI_CFG_OFF(_mode_) (\
		DSICFG_COMMON(0) |\
		BITFLDS(INR_DIF_DSICFG_LANES, 0) |\
		BITFLDS(INR_DIF_DSICFG_LP, 0) |\
		BITFLDS(INR_DIF_DSICFG_MODE, _mode_)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_INIT(_nlanes_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, (_nlanes_-1)) |\
		BITFLDS(INR_DIF_DSICFG_LP, 0) |\
		BITFLDS(INR_DIF_DSICFG_MODE, DSI_CMD)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_RX_LP_DATA(_nlanes_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, (_nlanes_-1)) |\
		BITFLDS(INR_DIF_DSICFG_TX, 1) |\
		BITFLDS(INR_DIF_DSICFG_LP, 1) |\
		BITFLDS(INR_DIF_DSICFG_TURN, 1) |\
		BITFLDS(INR_DIF_DSICFG_MODE, DSI_CMD)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_RX_LP_STP(_stp_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, 0) |\
		BITFLDS(INR_DIF_DSICFG_TX, 0) |\
		BITFLDS(INR_DIF_DSICFG_LP, 1) |\
		BITFLDS(INR_DIF_DSICFG_STP, _stp_) |\
		BITFLDS(INR_DIF_DSICFG_MODE, DSI_CMD)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_TX_LP_DATA(_nlanes_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, (0)) |\
		BITFLDS(INR_DIF_DSICFG_TX, 1) |\
		BITFLDS(INR_DIF_DSICFG_LP, 1) |\
		BITFLDS(INR_DIF_DSICFG_MODE, DSI_CMD)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_TX_HS_DATA_ACK(_nlanes_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, (_nlanes_-1)) |\
		BITFLDS(INR_DIF_DSICFG_TX, 1) |\
		BITFLDS(INR_DIF_DSICFG_LP, 0) |\
		BITFLDS(INR_DIF_DSICFG_TURN, 1) |\
		BITFLDS(INR_DIF_DSICFG_MODE, DSI_CMD)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_TX_HS_DATA(_nlanes_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, (_nlanes_-1)) |\
		BITFLDS(INR_DIF_DSICFG_TX, 1) |\
		BITFLDS(INR_DIF_DSICFG_LP, 0) |\
		BITFLDS(INR_DIF_DSICFG_MODE, DSI_CMD)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_DAT))

#define DSI_CFG_TX_HS_PIXEL(_nlanes_, _mode_) (\
		DSICFG_COMMON(1) |\
		BITFLDS(INR_DIF_DSICFG_LANES, (_nlanes_-1)) |\
		BITFLDS(INR_DIF_DSICFG_TX, 1) |\
		BITFLDS(INR_DIF_DSICFG_LP, 0) |\
		BITFLDS(INR_DIF_DSICFG_MODE, _mode_)|\
		BITFLDS(INR_DIF_DSICFG_DATA, DSICFG_DATA_PIX))


static void dcc_wr32tofifo(struct dcc_drvdata *pdata,
		unsigned int data)
{
	/* Write data to the DIF FIFO */
	gra_write_field(pdata, EXR_DIF_TXD, data);
}


int gra_waitfor_dsidir(struct dcc_drvdata *pdata,
		unsigned int value)
{
	unsigned int reg = 0xFF;
	int ret = 0;

	while (reg != value)
		gra_read_field(pdata, EXR_DIF_STAT_DSIDIR, &reg);

	return ret;
}

/**
 * Common TX functions
 */
static void dcc_mipidsi_send_short_packet(struct dcc_display *lcd,
		struct display_msg *msg, unsigned int dsicfg)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	unsigned int dsihead =
		BITFLDS(INR_DIF_DSIHEAD_WCNT, msg->header) |
		BITFLDS(INR_DIF_DSIHEAD_HEADER, msg->type);
	unsigned int difcsreg =
		BITFLDS(EXR_DIF_CSREG_GRACMD, 1);

	if (msg->length) {
		unsigned char *data_msg = msg->datas;
		dsihead |= data_msg[0]<<16;
	}

	DCC_DBG3("dsi short pkt: (head:0x%08x cfg:0x%08x)\n",
			dsihead, dsicfg);

	gra_write_field(pdata, EXR_DIF_CSREG, difcsreg);

	gra_write_field(pdata, INR_DIF_DSIVID6,
		BITFLDS(INR_DIF_DSIVID6_LASTPIXEL, msg->length));

	gra_write_field(pdata, INR_DIF_DSIHEAD, dsihead);
	gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
}

/* #define XG632_ES2_FIX */
static void dcc_mipidsi_send_long_packet_dma(struct dcc_display *lcd,
		struct display_msg *msg, unsigned int dsicfg)
{
	int i = 0;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	unsigned char *data_msg = msg->datas;
	unsigned int length = msg->length;
	unsigned int final_data_length = length;
	unsigned int dsihead =
		BITFLDS(INR_DIF_DSIHEAD_WCNT, msg->length+1) |
		BITFLDS(INR_DIF_DSIHEAD_HEADER, msg->type);

	/* the msg->header for Generic Long Write packet shouldnt be put
	 * in CMDBYTE of DSI_HEAD. This is only needed for DCS Long packet.
	 */
	if (DSI_M_DCS_LONG_WRITE == msg->type)
		dsihead |= BITFLDS(INR_DIF_DSIHEAD_CMD, msg->header);
	else
		final_data_length++;

	DCC_DBG4(" packet length = %d\n", final_data_length);
#ifdef XG632_ES2_FIX
	/* the msg->header for Generic Long Write packet shouldnt be put
	 * in CMDBYTE of DSI_HEAD. This is only needed for DCS Long packet.
	 */
	if (DSI_M_DCS_LONG_WRITE != msg->type) {
		int j = 0;
		unsigned int reg = msg->header;
		for (j = 1; j < 4 && length; j++) {
			length--;
			reg |= ((uint8_t) *data_msg++)<<(j*8);
		}
		iowrite32(reg, pdata->mem.vbase+(i*4));
		DCC_DBG4(" ---- @0x%p = 0x%08x\n",
				pdata->mem.vbase+(i*4), reg);
		i++;
	}
	while (length > 0) {
		int j = 0;
		unsigned int reg = 0;

		for (j = 0; j < 4 && length; j++) {
			length--;
			reg |= ((uint8_t) *data_msg++)<<(j*8);
		}

		iowrite32(reg, pdata->mem.vbase+(i*4));
		DCC_DBG4(" ---- @0x%p = 0x%08x\n",
				pdata->mem.vbase+(i*4), reg);
		i++;
	}
	gra_write_field(pdata, EXR_DIF_CSREG,
		BITFLDS(EXR_DIF_CSREG_GRACMD, 1) | 0x3);
#else
	/* the msg->header for Generic Long Write packet shouldnt be put
	 * in CMDBYTE of DSI_HEAD. This is only needed for DCS Long packet.
	 */
	if (DSI_M_DCS_LONG_WRITE != msg->type) {
		unsigned int reg = msg->header;
		iowrite32(reg, pdata->mem.vbase+(i*4));
		DCC_DBG4(" ---- @0x%p = 0x%08x\n",
				pdata->mem.vbase+(i*4), reg);
		i++;
	}
	while (length > 0) {
		unsigned int reg = 0;

		length--;
		reg = (uint8_t) *data_msg++;

		iowrite32(reg, pdata->mem.vbase+(i*4));
		DCC_DBG4(" ---- @0x%p = 0x%08x\n",
				pdata->mem.vbase+(i*4), reg);
		i++;
	}
	/* Drain the write buffer */
	wmb();
	gra_write_field(pdata, EXR_DIF_CSREG,
		BITFLDS(EXR_DIF_CSREG_GRACMD, 1));
#endif

	gra_write_field(pdata, INR_DIF_VIDEOBASE, pdata->mem.pbase);
	gra_write_field(pdata, INR_DIF_VIDEOSIZE, i);

	DCC_DBG3(
		"dsi long dma pkt: wcnt:0x%04x (head:0x%08x cfg:0x%08x)\n",
		msg->length+1, dsihead, dsicfg);
	gra_write_field(pdata, INR_DIF_DSIVID1,
			BITFLDS(INR_DIF_DSIVID1_PIXEL, 0x3) |
			BITFLDS(INR_DIF_DSIVID1_TOFILL, 0x3FF));
	gra_write_field(pdata, INR_DIF_DSIVID3,
		BITFLDS(INR_DIF_DSIVID3_PIXELBYTES, final_data_length)|
		BITFLDS(INR_DIF_DSIVID3_PIXELPACKETS, 1));
	gra_write_field(pdata, INR_DIF_DSIVID6,
		BITFLDS(INR_DIF_DSIVID6_LASTPIXEL, final_data_length));
	gra_write_field(pdata, INR_DIF_DSIHEAD, dsihead);

	gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
	dcc_wr32tofifo(pdata, 0x00000090);
	dcc_wr32tofifo(pdata, 0x00000000);
	dcc_wr32tofifo(pdata, (i-1)<<8);
}

static void dcc_mipidsi_send_writecmd(struct dcc_display *lcd,
		unsigned int dsicfg)
{
	int ret = 0;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	unsigned int dsihead =
		BITFLDS(INR_DIF_DSIHEAD_WCNT, 0x29) |
		BITFLDS(INR_DIF_DSIHEAD_HEADER, 0x05);
	unsigned int difcsreg =
		BITFLDS(EXR_DIF_CSREG_GRACMD, 1);

	DCC_DBG2("dsi-tx short pkt: (head:0x%08x cfg:0x%08x)\n",
			dsihead, dsicfg);

	gra_write_field(pdata, EXR_DIF_CSREG, difcsreg);
	gra_write_field(pdata, INR_DIF_DSIHEAD, dsihead);
	gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
	DCC_DBG4("wait for eoc\n");
	ret = dcc_completion_timeout_ms(&pdata->sync.dsifin,
			pdata->sync.dsifin_to);
	if (!ret) {
		dcc_warn("dsifin interrupt timedout %dms\n",
				pdata->sync.dsifin_to);
	}
	gra_waitfor_dsidir(pdata, 0);
	DCC_DBG4("eoc received\n");
}


/**
 * Note: beware that reading short packet implies sending a DSI_CMD_MRPS
 * with 1 or 2 as param instead of the default value
 */
static void dcc_mipidsi_readid(struct dcc_display *lcd,
				unsigned int dsicfg)
{
	int ret = 0;
	unsigned int i = 0, nwords, nbytes;
	unsigned int data[4];
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	unsigned int dsihead =
		BITFLDS(INR_DIF_DSIHEAD_WCNT, 0x04) |
		BITFLDS(INR_DIF_DSIHEAD_HEADER, 0x06);
	unsigned int difcsreg =
		BITFLDS(EXR_DIF_CSREG_GRACMD, 1) | 0x3;

	/* DCC_DBG2("dsi read pkt: (head:0x%08x cfg:0x%08x)\n",
			dsihead, dsicfg); */

	gra_write_field(pdata, EXR_DIF_CSREG, difcsreg);
	gra_write_field(pdata, INR_DIF_DSIHEAD, dsihead);
	gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);

	DCC_DBG3("wait for eoc\n");
	ret = dcc_completion_timeout_ms(&pdata->sync.dsifin,
			pdata->sync.dsifin_to);
	if (!ret) {
		dcc_warn("dsifin interrupt timedout %dms\n",
				pdata->sync.dsifin_to);
	}
	DCC_DBG3("eoc received\n");
	gra_waitfor_dsidir(pdata, 0);

	gra_read_field(pdata, EXR_DIF_RXFFS_STAT, &nwords);
	DCC_DBG3("EXR_DIF_RXFFS_STAT = %#x\n", nwords);

	if (nwords) {
		unsigned int id = 0;
		unsigned char *cdat = (unsigned char *)data;
		gra_read_field(pdata, EXR_DIF_RPS_STAT, &nbytes);
		DCC_DBG3("EXR_DIF_RPS_STAT = %#x\n", nbytes);

		while (nwords) {
			gra_read_field(pdata, EXR_DIF_RXD, &data[i]);
			DCC_DBG3("EXR_DIF_RXD = %#x\n", data[i]);
			nwords--;
			i++;
		}
		id = cdat[4] | cdat[5] << 8 | cdat[6] << 16;
		DCC_DBG2("Vendor ID = %#x\n", id);
	}
	gra_waitfor_dsidir(pdata, 0);
}

static int dcc_mipidsi_force_ownership(struct dcc_display *lcd)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	gra_write_field(pdata, INR_DIF_DSICFG, DSI_CFG_RX_LP_STP(1));
	gra_write_field(pdata, INR_DIF_DSICFG, DSI_CFG_RX_LP_STP(0));
	return 0;
}

static int dcc_mipidsi_bta(struct dcc_display *lcd)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	unsigned int dsicfg = DSI_CFG_RX_LP_DATA(1);

	DCC_DBG3("dsi send bta: (cfg:0x%08x)\n", dsicfg);

	/* send single BTA */
	gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
	return 0;
}

static int dcc_mipidsi_ack_wait(struct dcc_display *lcd)
{
	unsigned int nwords, nbytes;
	unsigned int data = 0;
	struct dcc_drvdata *p = m_to_dccdata(lcd, display);

	gra_waitfor_dsidir(p, 0);

	gra_read_field(p, EXR_DIF_RXFFS_STAT, &nwords);
	DCC_DBG3("EXR_DIF_RXFFS_STAT = %#x\n", nwords);

	if (nwords) {
		gra_read_field(p, EXR_DIF_RPS_STAT, &nbytes);
		DCC_DBG3("EXR_DIF_RPS_STAT = %#x\n", nbytes);
		while (nwords) {
			gra_read_field(p, EXR_DIF_RXD, &data);
			DCC_DBG3("EXR_DIF_RXD = %#x\n", data);
			nwords--;
		}
		DCC_DBG2("error returned (0x%x)\n", data);
	}

	return data;
}
/**
 * VSYNC functions
 */
static void dcc_mipidsi_reqvsync(struct dcc_display *lcd,
		int force_bus_back)
{
	int ret = 0;
	int error = 0;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);

	ret = dcc_mipidsi_bta(lcd);
	udelay(100);
	error = dcc_mipidsi_ack_wait(lcd);
	if (error)
		ret = dcc_mipidsi_bta(lcd);

	DCC_DBG3("wait for vsync\n");
	ret = dcc_completion_timeout_ms(&pdata->sync.dsitr1,
			pdata->sync.dsitr1_to);
	if (!ret) {
		dcc_warn("dsitr1 interrupt timedout %dms\n",
				pdata->sync.dsitr1_to);
		dcc_mipidsi_force_ownership(lcd);
	} else {
		DCC_DBG4("vsync received\n");
	}
	gra_waitfor_dsidir(pdata, 0);
	if (force_bus_back && error)
		dcc_mipidsi_force_ownership(lcd);
}

static void dcc_mipidsi_test(struct dcc_display *lcd)
{
	int i, nloop = 3;

	DCC_DBGT("--> MIPI-DSI vsync test started\n");
	DCC_DBGT("--> testing mipidsi vsync standalone\n");
	for (i = 0; i < nloop; i++) {
		DCC_DBGT("    loop #%d / %d\n", i, nloop);
		dcc_mipidsi_reqvsync(lcd, 0);
	}
	DCC_DBGT("<-- testing mipidsi vsync standalone\n");

	DCC_DBGT("--> testing mipidsi vsync + LP msg\n");
	for (i = 0; i < nloop; i++) {
		DCC_DBGT("    loop #%d / %d\n", i, nloop);
		dcc_mipidsi_send_writecmd(lcd, DSI_CFG_TX_LP_DATA(1));
		dcc_mipidsi_reqvsync(lcd, 0);
	}
	DCC_DBGT("<-- testing mipidsi vsync + LP msg\n");

	DCC_DBGT("--> testing mipidsi vsync + HS msg\n");
	for (i = 0; i < nloop; i++) {
		DCC_DBGT("    loop #%d / %d\n", i, nloop);
		dcc_mipidsi_send_writecmd(lcd, DSI_CFG_TX_HS_DATA(2));
		dcc_mipidsi_reqvsync(lcd, 0);
	}
	DCC_DBGT("--> testing mipidsi vsync + LP read ID msg\n");
	for (i = 0; i < nloop; i++) {
		DCC_DBGT("    loop #%d / %d\n", i, nloop);
		dcc_mipidsi_readid(lcd, DSI_CFG_RX_LP_DATA(1));
	}
	DCC_DBGT("<-- testing mipidsi vsync + LP read ID msg\n");
	DCC_DBGT("<-- MIPI-DSI vsync test stopped\n");
}


void dcc_dsi_send_cmd(struct dcc_display *lcd,
				    struct display_msg *msg)
{
	int ret = 0;
	unsigned int dsicfg;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);

	if (msg->flags & LCD_MSG_LP)
		dsicfg = DSI_CFG_TX_LP_DATA(1);
	else
		dsicfg = DSI_CFG_TX_HS_DATA(lcd->dif.u.dsi.nblanes);

	if (msg->length <= 1)
		dcc_mipidsi_send_short_packet(lcd, msg, dsicfg);
	else
		dcc_mipidsi_send_long_packet_dma(lcd, msg, dsicfg);

	DCC_DBG3("wait for eoc\n");
	ret = dcc_completion_timeout_ms(&pdata->sync.dsifin,
			pdata->sync.dsifin_to);
	if (!ret) {
		dcc_warn("dsifin interrupt timedout %dms\n",
				pdata->sync.dsifin_to);
	} else {
		DCC_DBG3("eoc received\n");
#ifdef USE_DSI_ACKNOWLEDGE
		ret = dcc_mipidsi_ack_wait(lcd);
		dcc_mipidsi_force_ownership(lcd);
#endif
	}
	mdelay(msg->delay);
}

static int dcc_dsi_get_bllp(struct dcc_display *lcd,
		int nlines, int bytes, int clk,
		int fps, int bitrate, int nlanes,
		int *bllp_time, int *line_time)
{
	/* bits per frames */
	unsigned int bitpframe = bytes * nlines * 8;
	/* maximum framerate */
	unsigned int maxfrate = bitrate * nlanes / bitpframe;
	/* shortest line time */
	unsigned int slt = NSEC_PER_SEC / maxfrate / nlines;
	/* target line time */
	unsigned int tlt = NSEC_PER_SEC / (fps + 1) / nlines;
	/* clock cycle duration in ps */
	unsigned int clk_time = 1000000 / (clk/1000000);

	*bllp_time = ((tlt - slt)*1000) / clk_time;
	*line_time = tlt*1000 / clk_time;

	DCC_DBG2("%d bytes / %d lines\n", bytes, nlines);
	DCC_DBG2("bits / frame = %d bits\n", bitpframe);
	DCC_DBG2("fps target %d fps (max=%d)\n", fps, maxfrate);
	DCC_DBG2("active time  = %d ns\n", slt);
	DCC_DBG2("target time  = %d ns\n", tlt);
	DCC_DBG2("clock cycle  = %d\n", clk_time);
	DCC_DBG2("bllp_time 0x%08x(%d)\n", *bllp_time, *bllp_time);
	DCC_DBG2("line_time 0x%08x(%d)\n", *line_time, *line_time);

	if (fps >= maxfrate) {
		dcc_err("target framerate(%d) cannot be reached\n", fps);
		*bllp_time = 0;
		*line_time = 0;
		return 0;
	}

	return 0;
}

static int dcc_dsi_configure_video_mode(struct dcc_display *lcd,
					int stride, int nlines)
{
	unsigned int dsicfg;
	unsigned int vid0, vid1, vid2, vid3, vid4, vid5, vid6;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	struct dcc_display_if_mipi_dsi *dif =
					&lcd->dif.u.dsi;
	if (lcd->dif.u.dsi.mode != DSI_VIDEO) {
		dcc_err("not video mode\n");
		return -EINVAL;
	}
	dsicfg = DSI_CFG_TX_HS_PIXEL(dif->nblanes, dif->mode);

	dcc_dsi_get_bllp(lcd,
			nlines + dif->vfp + dif->vbp + dif->vsa,
			stride + dif->hfp + dif->hbp,
			pdata->clk_rate,
			lcd->fps,
			lcd->get_rate(lcd),
			dif->nblanes, &dif->bllp_time, &dif->line_time);

	vid0 =	BITFLDS(INR_DIF_DSIVID0_HFP, (!!lcd->dif.u.dsi.hfp))|
		BITFLDS(INR_DIF_DSIVID0_HBP, (!!lcd->dif.u.dsi.hbp))|
		BITFLDS(INR_DIF_DSIVID0_HSA, (!!lcd->dif.u.dsi.hsa))|
		BITFLDS(INR_DIF_DSIVID0_HFPLP, dif->hfp_lp)|
		BITFLDS(INR_DIF_DSIVID0_HBPLP, dif->hbp_lp)|
		BITFLDS(INR_DIF_DSIVID0_HSALP, dif->hsa_lp)|
		BITFLDS(INR_DIF_DSIVID0_HFPBYTES, dif->hfp)|
		BITFLDS(INR_DIF_DSIVID0_HBPBYTES, dif->hbp)|
		BITFLDS(INR_DIF_DSIVID0_HSABYTES, dif->hsa);

	vid1 =	BITFLDS(INR_DIF_DSIVID1_VACT, nlines)|
		BITFLDS(INR_DIF_DSIVID1_MODE, dif->video_mode)|
		BITFLDS(INR_DIF_DSIVID1_ID, dif->id)|
		BITFLDS(INR_DIF_DSIVID1_PIXEL, dif->video_pixel)|
		BITFLDS(INR_DIF_DSIVID1_TOFILL, 0xFF);

	vid2 =	BITFLDS(INR_DIF_DSIVID2_VFP, dif->vfp)|
		BITFLDS(INR_DIF_DSIVID2_VBP, dif->vbp)|
		BITFLDS(INR_DIF_DSIVID2_VSA, dif->vsa);

	vid3 =	BITFLDS(INR_DIF_DSIVID3_PIXELBYTES, stride)|
		BITFLDS(INR_DIF_DSIVID3_PIXELPACKETS, 1);

	vid4 =	BITFLDS(INR_DIF_DSIVID4_BLANKBYTES, stride)|
		BITFLDS(INR_DIF_DSIVID4_BLANKPACKETS, 0);

	vid5 =	BITFLDS(INR_DIF_DSIVID5_LINE, dif->line_time)|
		BITFLDS(INR_DIF_DSIVID5_BLLP, dif->bllp_time);

	vid6 =	BITFLDS(INR_DIF_DSIVID6_LASTBLANK, stride)|
		BITFLDS(INR_DIF_DSIVID6_LASTPIXEL, stride);

	dcc_boot_info(
		"MIPI-DSI video:0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x)\n",
		vid0, vid1, vid2, vid3, vid4, vid5, vid6);

	gra_write_field(pdata, INR_DIF_DSIVID0, vid0);
	gra_write_field(pdata, INR_DIF_DSIVID1, vid1);
	gra_write_field(pdata, INR_DIF_DSIVID2, vid2);
	gra_write_field(pdata, INR_DIF_DSIVID3, vid3);
	gra_write_field(pdata, INR_DIF_DSIVID4, vid4);
	gra_write_field(pdata, INR_DIF_DSIVID5, vid5);
	gra_write_field(pdata, INR_DIF_DSIVID6, vid6);

	/*to be fixed in HW SF3GES2.1:
	 * DSICFG must be set after update command is sent
	 * gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
	 */

	return 0;
}

static int SMS05120496_once;
void dcc_dsi_start_video(struct dcc_display *lcd)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	struct dcc_display_if_mipi_dsi *dif =
					&lcd->dif.u.dsi;
	if (lcd->dif.u.dsi.mode != DSI_VIDEO) {
		dcc_err("not video mode\n");
		return;
	}
	if (SMS05120496_once) {/* TEMP PATCH SMS05120496 */
		unsigned int dsicfg =
			DSI_CFG_TX_HS_PIXEL(dif->nblanes, dif->mode);
		gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
		SMS05120496_once--;
	}
}

static void dcc_dsi_frame_prepare(struct dcc_display *lcd,
					int stride, int nlines)
{
	struct display_msg *msg;
	unsigned int dsicfg;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);

	msg = list_first_entry_or_null(&lcd->msgs_update->list,
				struct display_msg, list);

	if (!msg) {
		dcc_err("no msg for update command\n");
		return;
	}
	dsicfg = DSI_CFG_TX_HS_PIXEL(lcd->dif.u.dsi.nblanes,
					lcd->dif.u.dsi.mode);

	if (pdata->test.mipidsi_vsync)
		dcc_mipidsi_test(lcd);

	if (lcd->dif.u.dsi.mode == DSI_VIDEO) {
		return;
	} else {
	unsigned int dsihead;
	dsihead = BITFLDS(INR_DIF_DSIHEAD_CMD, msg->header) |
		BITFLDS(INR_DIF_DSIHEAD_WCNT, 1+stride) |
		BITFLDS(INR_DIF_DSIHEAD_HEADER, msg->type);

		DCC_DBG2("frame cmd: wcnt:0x%04x (head:0x%08x cfg:0x%08x)\n",
			stride, dsihead, dsicfg);
		gra_write_field(pdata, INR_DIF_DSIVID1,
			BITFLDS(INR_DIF_DSIVID1_PIXEL,
				DSI_PIX_BIT24P) |
			BITFLDS(INR_DIF_DSIVID1_TOFILL, 0x3FF));
		gra_write_field(pdata, INR_DIF_DSIVID3,
			BITFLDS(INR_DIF_DSIVID3_PIXELBYTES, stride)|
			BITFLDS(INR_DIF_DSIVID3_PIXELPACKETS, nlines));
		gra_write_field(pdata, INR_DIF_DSIVID6,
			BITFLDS(INR_DIF_DSIVID6_LASTPIXEL, stride));
		gra_write_field(pdata, INR_DIF_DSIHEAD, dsihead);
		gra_write_field(pdata, INR_DIF_DSICFG, dsicfg);
	}

}

static int dcc_dsi_frame_wfe(struct dcc_display *lcd)
{
	int ret = 0;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	DCC_DBG3("wait for end of update\n");
	ret = dcc_completion_timeout_ms(&pdata->sync.eof,
				pdata->sync.eof_to);
	if (!ret) {
		DCC_DBG2("eof interrupt timedout %dms (err=%d)\n",
				pdata->sync.eof_to, ret);
	} else {
		DCC_DBG3("received end of update\n");
	}

	return ret;
}



/**
 * Callbacks
 */
#define MAX_PHY_N	0xFF
#define MAX_PHY_M	0xF
#define DSI_BASE_CLK (26000000)
#define DSI_RATE(_n_, _m_)	((((DSI_BASE_CLK/1000)*(_n_+1))/(_m_+1))*1000)

int dcc_dsi_get_rate(struct dcc_display *lcd)
{
	return DSI_RATE(lcd->dif.u.dsi.n, lcd->dif.u.dsi.m);
}

int dcc_dsi_set_phy(struct dcc_display *lcd)
{
	unsigned int phy0 = 0, phy1 = 0, phy2 = 0, phy3 = 0, stat = 0, pllstat;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);

	phy0 =	BITFLDS(INR_DIF_DSIPHY0_SHARE, 0x0) |
		BITFLDS(INR_DIF_DSIPHY0_M, lcd->dif.u.dsi.m) |
		BITFLDS(INR_DIF_DSIPHY0_N, lcd->dif.u.dsi.n) |
		BITFLDS(INR_DIF_DSIPHY0_PWUP, lcd->dif.u.dsi.pwup) |
		BITFLDS(INR_DIF_DSIPHY0_CALIB, lcd->dif.u.dsi.calib) |
		BITFLDS(INR_DIF_DSIPHY0_TOREQ,
				lcd->dif.u.dsi.to_lp_hs_req);

	phy1 =	BITFLDS(INR_DIF_DSIPHY1_TOLPHSDIS,
				lcd->dif.u.dsi.to_lp_hs_dis) |
		BITFLDS(INR_DIF_DSIPHY1_TOLPHSEOT,
				lcd->dif.u.dsi.to_lp_hs_eot) |
		BITFLDS(INR_DIF_DSIPHY1_TOHSZERO,
				lcd->dif.u.dsi.to_hs_zero) |
		BITFLDS(INR_DIF_DSIPHY1_TOHSFLIP,
				lcd->dif.u.dsi.to_hs_flip) |
		BITFLDS(INR_DIF_DSIPHY1_LPCLKDIV,
				lcd->dif.u.dsi.lp_clk_div);

	phy2 =	BITFLDS(INR_DIF_DSIPHY2_HSCLKPRE,
				lcd->dif.u.dsi.to_hs_clk_pre) |
		BITFLDS(INR_DIF_DSIPHY2_HSCLKPOST,
				lcd->dif.u.dsi.to_hs_clk_post) |
		BITFLDS(INR_DIF_DSIPHY2_DATDELAY,
				lcd->dif.u.dsi.data_delay) |
		BITFLDS(INR_DIF_DSIPHY2_CLKDELAY,
				lcd->dif.u.dsi.clock_delay) |
		BITFLDS(INR_DIF_DSIPHY2_LPTX_TFALL,
				lcd->dif.u.dsi.lp_tx_tfall);

	phy3 =	BITFLDS(INR_DIF_DSIPHY3_EN, 0x1) |
		BITFLDS(INR_DIF_DSIPHY3_LPTX_TRISE,
				lcd->dif.u.dsi.lp_tx_trise) |
		BITFLDS(INR_DIF_DSIPHY3_LPTX_VREF,
				lcd->dif.u.dsi.lp_tx_vref);

	dcc_boot_info(
		"MIPI-DSI @%d bps (%d,%d): 0x%08x 0x%08x 0x%08x 0x%08x)\n",
		dcc_dsi_get_rate(lcd),
		lcd->dif.u.dsi.n, lcd->dif.u.dsi.m,
		phy0, phy1, phy2, phy3);

	gra_write_field(pdata, INR_DIF_DSIPHY0, phy0);
	gra_write_field(pdata, INR_DIF_DSIPHY1, phy1);
	gra_write_field(pdata, INR_DIF_DSIPHY2, phy2);
	gra_write_field(pdata, INR_DIF_DSIPHY3, phy3);

	if (!((lcd->dif.u.dsi.n == 0xFF) && (lcd->dif.u.dsi.m == 0))) {
		int wait_delay_ms = 5;
		int wait_loop_n = 2000 / wait_delay_ms; /* 2sec timeout*/
		/* wait for PLL lock */
		pllstat = BITFLDS(EXR_DIF_STAT_DSILOCK, 1);
		gra_read_field(pdata, EXR_DIF_STAT, &stat);
		DCC_DBG2("wait dsi pll lock 0x%08x (0x%08x)\n", stat, pllstat);

		while ((stat & pllstat) != pllstat) {
			mdelay(wait_delay_ms);
			wait_loop_n--;
			if (!wait_loop_n) {
				dcc_err("dsi powerup loop timedout!\n");
				return -EBUSY;
			}
			gra_read_field(pdata, EXR_DIF_STAT, &stat);
			DCC_DBG2("wait dsi pll lock 0x%08x (0x%08x)\n",
					stat, pllstat);
		}
	}
	return 0;
}

int dcc_dsi_set_phy_lock(struct dcc_display *lcd)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	int ret;
	if (down_interruptible(&pdata->sem))
		return -ERESTARTSYS;
	ret = dcc_dsi_set_phy(lcd);

	up(&pdata->sem);
	return ret;
}

int dcc_dsi_set_rate(struct dcc_display *lcd, int rate)
{
	int n = 0, m = 0, ret = 0;

	if (rate == 0) {/* power off pll */
		n = 0xFF;
		m = 0;
		goto found;
	}

	if ((rate < lcd->dif.u.dsi.brmin) || (rate > lcd->dif.u.dsi.brmax))
		goto notfound;

	while (m <= MAX_PHY_M) {
		for (n = 0; n <= MAX_PHY_N; n++) {
			DCC_DBG4("(%02d, %d) = %d bps (%d bps)\n",
					n, m, DSI_RATE(n, m), rate);
			if (rate == DSI_RATE(n, m))
				goto found;
			if (rate == ((DSI_RATE(n, m)/1000000)*1000000))
				goto found;
			if (DSI_RATE(n, m) > rate)
				break;
		}
		m++;
	}
notfound:
	dcc_err("phy (n,m) not found for %d [%d,%d]\n",
			rate, lcd->dif.u.dsi.brmin, lcd->dif.u.dsi.brmax);
	return -EINVAL;

found:
	if ((lcd->dif.u.dsi.n != n) || (lcd->dif.u.dsi.m != m)) {
		lcd->dif.u.dsi.n = n;
		lcd->dif.u.dsi.m = m;
		ret = dcc_dsi_set_phy(lcd);
	}
	return ret;

}

static void dcc_dsi_send_msglist(struct dcc_display *lcd,
			    struct display_msg *msgs)
{
	struct display_msg *msg;

	list_for_each_entry(msg, &msgs->list, list) {
		mdelay(1);
		DCC_DBG3("Sending command 0x%02x 0x%02x of length %d\n",
		 msg->header, msg->type, msg->length);
		dcc_dsi_send_cmd(lcd, msg);
	}
}

static int dcc_panel_init(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_init;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		dcc_dsi_send_msglist(lcd, msgs);

	return 0;
}

static int dcc_dsi_stop(struct dcc_display *lcd)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	DCC_DBG2("%s\n", __func__);
	/* swicth off PLL */
	lcd->set_rate(lcd, 0);
	/* switch off phy */
	gra_write_field(pdata, INR_DIF_DSIPHY3,
			BITFLDS(INR_DIF_DSIPHY3_EN, 0x0));
	/* switch off DSI block */
	gra_write_field(pdata, INR_DIF_DSICFG,	DSI_CFG_OFF(DSI_CMD));

	dcc_wait_status(pdata, EXR_DIF_STAT,
			BITFLDS(EXR_DIF_STAT_BSY, 0), 2000);
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	/* disable mipi dsi */
	gra_write_field(pdata, EXR_DIF_PERREG,
			BITFLDS(EXR_DIF_PERREG_MIPIEN, 0));
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_RUN);

	return 0;
}

static int dcc_dsi_init(struct dcc_display *lcd)
{
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);
	DCC_DBG2("%s\n", __func__);

	dcc_wait_status(pdata, EXR_DIF_STAT,
			BITFLDS(EXR_DIF_STAT_BSY, 0), 2000);
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	gra_write_field(pdata, EXR_DIF_PERREG,
			BITFLDS(EXR_DIF_PERREG_MIPIEN, 1));
	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
	gra_write_field(pdata, EXR_DIF_CSREG,
			BITFLDS(EXR_DIF_CSREG_GRACMD, 1)); /* gra */

	gra_write_field(pdata, INR_DIF_DSICLK, 0x000F000F);
	gra_write_field(pdata, INR_DIF_DSITO0, 0);
	gra_write_field(pdata, INR_DIF_DSITO1, 0);
	gra_write_field(pdata, INR_DIF_DSICFG, DSI_CFG_RX_LP_STP(1));
	SMS05120496_once = 1;
	return 0;
}

static int dcc_panel_power_on(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_power_on;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		dcc_dsi_send_msglist(lcd, msgs);

	return 0;
}

static int dcc_panel_power_off(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_power_off;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		dcc_dsi_send_msglist(lcd, msgs);

	return 0;
}

static int dcc_panel_sleep_in(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_sleep_in;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		dcc_dsi_send_msglist(lcd, msgs);

	return 0;
}

static int dcc_panel_sleep_out(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_sleep_out;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		dcc_dsi_send_msglist(lcd, msgs);

	return 0;
}

static int dcc_dsi_config(struct dcc_display *lcd, int type)
{
	int ret = 0;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);

	DCC_DBG2("%s\n", __func__);
	if (type == DIF_TX_PIXELS) {
		if (lcd->dif.u.dsi.mode == DSI_VIDEO) {
			gra_write_field(pdata, INR_DIF_DSICFG,
					DSI_CFG_OFF(DSI_VIDEO));
			dcc_dsi_configure_video_mode(lcd,
					(dcc_get_display_w(pdata) * 3),
					dcc_get_display_h(pdata));
		}

		lcd->set_rate(lcd, lcd->dif.u.dsi.brdef);
	} else {

		gra_write_field(pdata, INR_DIF_DSICFG,	DSI_CFG_OFF(DSI_CMD));

		lcd->set_rate(lcd, lcd->dif.u.dsi.brdef);

		gra_write_field(pdata, INR_DIF_DSICFG,
				DSI_CFG_INIT(lcd->dif.u.dsi.nblanes));
	}

	return ret;
}

int dcc_dsi_probe(struct dcc_display *lcd)
{
	lcd->dif_config = dcc_dsi_config;
	lcd->dif_stop = dcc_dsi_stop;
	lcd->dif_init = dcc_dsi_init;
	lcd->panel_init = dcc_panel_init;
	lcd->power_on = dcc_panel_power_on;
	lcd->power_off = dcc_panel_power_off;
	lcd->sleep_in = dcc_panel_sleep_in;
	lcd->sleep_out = dcc_panel_sleep_out;
	lcd->send_cmd = dcc_dsi_send_cmd;
	lcd->set_rate = dcc_dsi_set_rate;
	lcd->get_rate = dcc_dsi_get_rate;
	lcd->frame_prepare = dcc_dsi_frame_prepare;
	lcd->frame_wfe = dcc_dsi_frame_wfe;

	return 0;
}
