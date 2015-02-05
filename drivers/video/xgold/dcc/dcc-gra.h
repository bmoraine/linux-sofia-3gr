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

#include <video/xgold-dcc.h>

#ifndef __XGOLD_GRA_H__
#define __XGOLD_GRA_H__

/* WARNING: These values comes from DIF_IMAGE_SRC rotation field */
enum {
	DCC_ROTATE_0 = 0,
	DCC_ROTATE_90 = 1,
	DCC_ROTATE_180 = 2,
	DCC_ROTATE_270 = 3,
};

#define DCC_VIDEOSIZE_MAX 1023

/* RQ Processing */
void dcc_config_reset(struct dcc_drvdata *p);
int dcc_rq_convert(struct dcc_drvdata *p, struct dcc_rq_t *rq);
int dcc_rq_update(struct dcc_drvdata *p, struct dcc_rect_t *r,
		unsigned int pbase_yuv);
int dcc_rq_acquire_and_compose(struct dcc_drvdata *p,
		struct dcc_update_layers *updt, int updt_pt);
int dcc_rq_fillrectangle(struct dcc_drvdata *p, struct dcc_rect_t *r);
int dcc_rq_setpixel(struct dcc_drvdata *p, struct dcc_point_t *pu);
int dcc_rq_drawline(struct dcc_drvdata *p, struct dcc_rect_t *ru);
int dcc_rq_drawlinerel(struct dcc_drvdata *p, struct dcc_rect_t *ru);
int dcc_rq_blit(struct dcc_drvdata *p, struct dcc_rq_t *rq);
int dcc_rq_scrollmove(struct dcc_drvdata *p, struct dcc_rq_t *rq);
int dcc_rq_resize(struct dcc_drvdata *p, struct dcc_rq_resize_t *rq);
int dcc_rq_rotate(struct dcc_drvdata *p, struct dcc_rq_t *rq);

/* GRA configuration*/
int dcc_setbufferformat(struct dcc_drvdata *p, int format);
int dcc_overlay_off(struct dcc_drvdata *p);
void dcc_format_set_convertmatrix(struct dcc_drvdata *p,
		int sfmt, int dfmt, int force);
void dcc_hwreset(struct dcc_drvdata *p);
void dcc_bootscreen(struct dcc_drvdata *p);
void dcc_clearscreen(struct dcc_drvdata *p);
void dcc_bootlogo(struct dcc_drvdata *p);
void dcc_clearvideomem(struct dcc_drvdata *p);

int dcc_wait_status(struct dcc_drvdata *pdata,
		unsigned int reg,
		unsigned int pattern,
		int to);
#endif
