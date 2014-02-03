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
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include "dcc-core.h"
#include "dcc-display.h"
#include "dcc-hwregs.h"


#define DIF_FORMAT_CMD(X) (X)


static void dcc_wr16tofifo(struct dcc_drvdata *p,
		uint16_t data)
{
	/* Write data to the DIF FIFO */
	gra_write_field(p, EXR_DIF_TXD, data);
	gra_waitfor_external(p, EXR_DIF_STAT_BSY, 0x0);
}


static uint8_t dcclcd_getsplit(struct dcc_drvdata *p)
{
	unsigned int data;
	gra_read_field(p, EXR_DIF_CSREG_BSCONF, &data);
	return data;
}

static void dcclcd_setsplit(struct dcc_drvdata *p, uint8_t count)
{
	gra_write_field(p, EXR_DIF_CSREG_BSCONF, count);
}

void dcclcd_set_cs(struct dcc_drvdata *p, uint8_t cs)
{
	if (cs == 0)
		gra_write_field(p, EXR_DIF_CSREG_CS1, 1);
	else if (cs == 1)
		gra_write_field(p, EXR_DIF_CSREG_CS2, 1);
	else if (cs == 2)
		gra_write_field(p, EXR_DIF_CSREG_CS3, 1);
}


static void dcc_pcl_for_data(struct dcc_drvdata *p)
{
	gpio_set_value(p->gpio_cd, 1);
}

static void dcc_pcl_for_cmd(struct dcc_drvdata *p)
{
	gpio_set_value(p->gpio_cd, 0);
}
int dcc_dbi_init_gpio(struct dcc_drvdata *pdata)
{
	if (gpio_request(pdata->gpio_cd, "dif_cd")) {
		dcc_err("failed to request gpio set %d\n", pdata->gpio_cd);
		return -EINVAL;
	}
	pinctrl_gpio_direction_output(pdata->gpio_cd);
	gpio_direction_output(pdata->gpio_cd, 1);
	return 0;
}


static void dcc_dbi_send_cmd_data(struct dcc_drvdata *p,
		const unsigned char *data_msg,
		unsigned length)
{
	while (length--) {
		unsigned short data = DIF_FORMAT_CMD((uint8_t) *data_msg++);
#ifdef CONFIG_XGOLD_DCC_DEBUG
		pr_info(" 0x%02x", data);
#endif
		dcc_wr16tofifo(p, data);
	}
}

static void dcc_dbi_send_cmd_head(struct dcc_drvdata *p, unsigned char cmd)
{
	unsigned short data = DIF_FORMAT_CMD(cmd);
#ifdef CONFIG_XGOLD_DCC_DEBUG
	pr_info("dcc: display cmd : 0x%02x", data);
#endif
	dcc_wr16tofifo(p, data);
}


void dcc_dbi_send_cmd(struct dcc_display *lcd,
		struct display_msg *msg)
{
	unsigned char bsconf;
	struct dcc_drvdata *p = m_to_dccdata(lcd, display);

	gra_write_field(p, EXR_DIF_CSREG_GRACMD, 0); /* to display */
	gra_write_field(p, EXR_DIF_CSREG_CD, 1); /* Set Command mode */
	bsconf = dcclcd_getsplit(p); /* save bsconf configuration */
	dcclcd_setsplit(p, 0);

	dcc_pcl_for_cmd(p);
	dcc_dbi_send_cmd_head(p, msg->header);
	dcc_pcl_for_data(p);

	if (!msg->length)
		goto end_of_cmd;

	dcc_dbi_send_cmd_data(p, msg->datas, msg->length);

end_of_cmd:
#ifdef CONFIG_XGOLD_DCC_DEBUG
	pr_info("\n");
#endif
	gra_write_field(p, EXR_DIF_CSREG_CD, 0); /* Set Data mode */
	dcclcd_setsplit(p, bsconf); /* Restore bsconf configuration */
	gra_write_field(p, EXR_DIF_CSREG_GRACMD, 1);
}

#define NBMUX_PER_BMREG	(6)
#define NBBIT_PER_MUX	(5)
static int dcc_set_mux(unsigned int *bmreg, int old_offset, int new_offset)
{
	int ireg = 0, shift = 0;

	if (old_offset < 0 || old_offset > 31)
		dcc_err
		    ("%25s: Error setting multiplexer parameters \for %d bit\n",
		     __func__, old_offset);

	ireg = old_offset / NBMUX_PER_BMREG;

	if ((old_offset % NBMUX_PER_BMREG) <= 2)
		shift = (old_offset % NBMUX_PER_BMREG) * NBBIT_PER_MUX;
	else
		shift = ((old_offset % NBMUX_PER_BMREG) * NBBIT_PER_MUX) + 1;

	bmreg[ireg] |= new_offset << shift;

	return 0;
}

int dcc_dbi_set_bitmux(struct dcc_drvdata *pdata)
{
	struct dcc_display *display = &pdata->display;
	unsigned int bmreg[6] = { 0, 0, 0, 0, 0, 0 };
	int i;
	int retval = 0;
	int bcsel;

	if (!display) {
		dcc_err("%s, display is a NULL pointer\n", __func__);
		goto exit;
	}
	/* Setup multiplexer */
	gra_write_field(pdata, EXR_DIF_RUNCTRL, 0); /* enter config mode */

	for (i = 0; i < 32; i++) {
		char bout = display->dif.u.dbi.mux_params[i];
		dcc_set_mux(bmreg, i, (bout & 0x1F));
		bcsel = (bout >> 6) & 0x3;
		if (bcsel) {
			if (i < 16) {
				unsigned int reg;
				gra_read_field(pdata, EXR_DIF_BCSEL0, &reg);
				reg = (reg & ~(3 << (2 * i)))
					| bcsel << (2 * i);
				gra_write_field(pdata, EXR_DIF_BCSEL0, reg);
				dcc_info("DIF[%02d]: BCSEL0 = 0x%08x\n",
						i, reg);
			} else {
				unsigned int reg;
				gra_read_field(pdata, EXR_DIF_BCSEL1, &reg);
				reg = (reg & ~(3 << (2 * (i - 16))))
					| bcsel << (2 * (i - 16));
				gra_write_field(pdata, EXR_DIF_BCSEL1,
						bcsel << (2 *
							(i - 16)));
				dcc_info("DIF[%02d]: BCSEL1 = 0x%08x\n",
						i, reg);
			}
			if (bout & (1 << 5))
				gra_write_field(pdata, EXR_DIF_BCREG,
						(1 << i));
		}
#ifdef DCC_DEBUG_DISPLAY
		DCC_DBG2("FB[%02d] = LCD[%02d]\n",
				i, (display->dif.u.dbi.mux_params)[i]);
#endif
	}

#ifdef DCC_DEBUG_DISPLAY
	for (i = 0; i < 5; i++)
		DCC_DBG2("BMREG[%d] = 0x%08x\n", i, bmreg[i]);
#endif

	/* set color bit multiplexer registers (converting 32 bit
	 * color value to value capable for hardware, according to
	 * COLOR.length and COLOR.offset)
	 */
	gra_write_field(pdata, EXR_DIF_BMREG0, bmreg[0]);
	gra_write_field(pdata, EXR_DIF_BMREG1, bmreg[1]);
	gra_write_field(pdata, EXR_DIF_BMREG2, bmreg[2]);
	gra_write_field(pdata, EXR_DIF_BMREG3, bmreg[3]);
	gra_write_field(pdata, EXR_DIF_BMREG4, bmreg[4]);
	gra_write_field(pdata, EXR_DIF_BMREG5, bmreg[5]);

	return 0;
exit:
	dcc_err("%25s: Error while setting gracr parameters (Errorcode: %d)\n",
		__func__, retval);
	return retval;
}


static unsigned char dcc_lcd_ns2cycles(unsigned int freq, unsigned char ns)
{
	unsigned int ns_per_cycle = 0;
	unsigned char cycles = 0;

	freq /= (1000*1000); /* to MHz */
	ns_per_cycle = (1000 / freq) + ((ns_per_cycle*freq) != 1000);
	/* dcc_boot_dbg("%s, %dMHz, %d ns/cycle\n",
				__func__, freq , ns_per_cycle); */

	cycles = (ns/ns_per_cycle) + ((ns%ns_per_cycle) != 0);
	return cycles;
}

void dcc_dbi_timings(struct dcc_drvdata *pdata)
{
	struct dcc_display *lcd = &pdata->display;

	if (DISPLAY_IS_MIPI_DBI_IF(lcd->dif.type)) {
		unsigned split = 0;
		struct dcc_display_if_mipi_dbi *tim = &lcd->dif.u.dbi;

		dcc_boot_dbg("MIPI-BDI timing in ns\n");
		dcc_boot_dbg("dd:%d ac:%d ad:%d wr:%d->%d cs:%d->%d\n",
			tim->data_delay_ns, tim->access_cycle_ns,
			tim->addr_delay_ns, tim->wr_rd_act_ns,
			tim->wr_rd_deact_ns, tim->cs_act_ns, tim->cs_deact_ns);
		dcc_boot_dbg("MIPI-BDI timing in cycomp\n");
		dcc_boot_dbg("dd:%d ac:%d ad:%d wr:%d->%d cs:%d->%d\n",
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->data_delay_ns),
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->access_cycle_ns),
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->addr_delay_ns),
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->wr_rd_act_ns),
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->wr_rd_deact_ns),
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->cs_act_ns),
			dcc_lcd_ns2cycles(pdata->clk_rate,
				tim->cs_deact_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM1_DATADELAY,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->data_delay_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM1_ACCESSCYCLE,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->access_cycle_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM1_ADDRDELAY,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->addr_delay_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM2_WRRDDEACT,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->wr_rd_deact_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM2_WRRDACT,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->wr_rd_act_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM2_CSDEACT,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->cs_deact_ns));
		gra_write_field(pdata, EXR_DIF_LCDTIM2_CSACT,
			dcc_lcd_ns2cycles(pdata->clk_rate,
					tim->cs_act_ns));

		dcclcd_set_cs(pdata, lcd->cs);
		if (lcd->dif.u.dbi.bits_per_segment == 8)
			split = 0x7 &
				(((lcd->dif.u.dbi.segments_per_pix - 1)<<1)
					| 0x1);
		else if (lcd->dif.u.dbi.bits_per_segment == 9)
			split = 0x7 &
				(lcd->dif.u.dbi.segments_per_pix<<1);
		else
			dcc_err("Unknowm bus format\n");

		dcclcd_setsplit(pdata, split);
		gra_write_field(pdata, EXR_DIF_PERREG_DIFPERMODE, 1);

	} else {
		/* Should not go here */
		dcc_err("Unknowm interface\n");
	}

}


int dcc_dbi_polarity(struct dcc_display *lcd)
{
	struct dcc_drvdata *p = m_to_dccdata(lcd, display);
	struct xgold_lcd_periph_parameters *periph_params =
						&lcd->dif.u.dbi.periph_params;
	int err = 0;

	if (lcd->cs == 0) {
		gra_write_field(p, EXR_DIF_PERREG_CS1POL,
				periph_params->cs_polarity);
	} else if (lcd->cs == 1) {
		gra_write_field(p, EXR_DIF_PERREG_CS2POL,
				periph_params->cs_polarity);
	} else if (lcd->cs == 2) {
		gra_write_field(p, EXR_DIF_PERREG_CS3POL,
				periph_params->cs_polarity);
	}
	gra_write_field(p, EXR_DIF_PERREG_CDPOL,
			periph_params->cd_polarity);
	gra_write_field(p, EXR_DIF_PERREG_WRPOL,
			periph_params->wr_polarity);
	gra_write_field(p, EXR_DIF_PERREG_RDPOL,
			periph_params->rd_polarity);

	return err;
}


int dcc_dbi_sync(struct dcc_display *lcd)
{
	unsigned int data = 0;
	int err = 0;
	struct dcc_drvdata *p = m_to_dccdata(lcd, display);
	struct xgold_lcd_periph_parameters *periph_params =
						&lcd->dif.u.dbi.periph_params;

	gra_write_field(p, EXR_DIF_SYNCCOUNT_NUMROWS, lcd->yres);
	gra_write_field(p, EXR_DIF_SYNCCOUNT_NUMBYTES, lcd->xres * 2);
	gra_write_field(p, EXR_DIF_SYNCCOUNT_HDSTART, 0x3FF);
	gra_read_field(p, EXR_DIF_SYNCCOUNT, &data);
	dcc_boot_dbg("DIF_SYNC_COUNT set to %#x\n", data);

	if (lcd->cs == 0) {
		gra_write_field(p, EXR_DIF_SYNCCONFIG_SYNCCS1,
				periph_params->cs_polarity);
	} else if (lcd->cs == 1) {
		gra_write_field(p, EXR_DIF_SYNCCONFIG_SYNCCS2,
				periph_params->cs_polarity);
	} else if (lcd->cs == 2) {
		gra_write_field(p, EXR_DIF_SYNCCONFIG_SYNCCS3,
				periph_params->cs_polarity);
	}
	gra_write_field(p, EXR_DIF_SYNCCONFIG_VDPOL,
			periph_params->vd_polarity);
	gra_write_field(p, EXR_DIF_SYNCCONFIG_SYNCEN, 1);

	gra_read_field(p, EXR_DIF_SYNCCONFIG, &data);
	dcc_boot_dbg("DIF_SYNC_CONFIG set to %#x\n", data);

	return err;
}

static int dcc_panel_init(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_init;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		lcd->send_cmd(lcd, msgs);

	return 0;
}

static int dcc_panel_power_on(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_power_on;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		lcd->send_cmd(lcd, msgs);

	return 0;
}

static int dcc_panel_power_off(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_power_off;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		lcd->send_cmd(lcd, msgs);

	return 0;
}

static int dcc_panel_sleep_in(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_sleep_in;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		lcd->send_cmd(lcd, msgs);

	return 0;
}

static int dcc_panel_sleep_out(struct dcc_display *lcd)
{
	struct display_msg *msgs = lcd->msgs_sleep_out;
	DCC_DBG2("%s\n", __func__);
	if (msgs != NULL)
		lcd->send_cmd(lcd, msgs);

	return 0;
}

int dcc_dbi_config(struct dcc_display *lcd, int type)
{
	int err = 0;
	struct dcc_drvdata *pdata = m_to_dccdata(lcd, display);

	gra_write_field(pdata, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	err = dcc_dbi_init_gpio(pdata);
	if (err)
		goto error;
	dcc_dbi_polarity(lcd);
	dcc_dbi_timings(pdata);


	dcc_boot_dbg("Display probed!\n");
	return 0;
error:
	dcc_err("Display setup failed\n");
	return -EINVAL;
}


int dcc_dbi_probe(struct dcc_display *lcd)
{
	lcd->dif_config = dcc_dbi_config;
	lcd->send_cmd = dcc_dbi_send_cmd;
	lcd->panel_init = dcc_panel_init;
	lcd->power_on = dcc_panel_power_on;
	lcd->power_off = dcc_panel_power_off;
	lcd->sleep_in = dcc_panel_sleep_in;
	lcd->sleep_out = dcc_panel_sleep_out;

	return 0;
}

