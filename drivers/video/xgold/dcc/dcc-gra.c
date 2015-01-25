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
#include <linux/types.h>
#include <linux/io.h>
#include <linux/delay.h>

#include "dcc-core.h"
#include "dcc-gra.h"
#include "dcc-hwregs.h"

#define DCC_CHECK(_test_, fmt, arg...) { \
	if ((_test_) == 0) { \
		pr_info(": "fmt, ##arg); \
		return -1; \
	} }

static int dcc_set_framebuffer(struct dcc_drvdata *p,
		unsigned int base, unsigned int size)
{
	gra_write_field(p, INR_DIF_VIDEOBASE, base);
	gra_write_field(p, INR_DIF_VIDEOSIZE, size);
	return 0;
}

enum {
	ALPHA_NONE,
	ALPHA_PLANE,
	ALPHA_PIXEL,
	ALPHA_COLORKEY,
};

enum {
	DCC_DEST_DRAW2MEM = 1,
	DCC_DEST_DRAW2DISP,
};

/*only on older than xg631*/
#define DIF_CONF_OVERLAY	BITFLDS(INR_DIF_CONF_OVERLAY, 0x1)

#define DIF_CONF_SPR		BITFLDS(INR_DIF_CONF_SPR, 0x1)
#define DIF_CONF_BACK		BITFLDS(INR_DIF_CONF_BACK, 0x1)
#define DIF_CONF_TRANS		BITFLDS(INR_DIF_CONF_TRANS, 0x1)
#define DIF_CONF_BLEND		BITFLDS(INR_DIF_CONF_BLEND, 0x1)
#define DIF_CONF_ALPHA		BITFLDS(INR_DIF_CONF_ALPHA, 0x3)
#define DIF_CONF_GLOBAL		BITFLDS(INR_DIF_CONF_GLOBAL, 0x1)
#define DIF_CONF_STENCIL_MASK	BITFLDS(INR_DIF_CONF_STENCIL, 0x3)
#define DIF_CONF_STENCIL_INTER	BITFLDS(INR_DIF_CONF_STENCIL, 0x0)
#define DIF_CONF_STENCIL_BUFFER	BITFLDS(INR_DIF_CONF_STENCIL, 0x1)
#define DIF_CONF_STENCIL_CHROMA0 BITFLDS(INR_DIF_CONF_STENCIL, 0x2)
#define DIF_CONF_STENCIL_CHROMA1 BITFLDS(INR_DIF_CONF_STENCIL, 0x3)

#define DRAWMODE_SET	0x0	/* mem = newvalue */
#define DRAWMODE_AND	0x1	/* mem = mem & newvalue */
#define DRAWMODE_OR	0x2	/* mem = mem | newvalue */
#define DRAWMODE_XOR	0x3	/* mem = mem xor newvalue */
static int dcc_setdrawmode(struct dcc_drvdata *p, unsigned int dm)
{
	gra_sendcmd(p, GRACMD_SET_DRAWMODE, 0, &dm, 1);
	return 0;
}

#define NO_RESCALING	0x1000

#define HW_YCBCR422PACKED	(0x1)
#define HW_RGB16bit		(0x1)
#define HW_YCBCR420PLANAR	(0x2)
#define HW_YCBCR422PLANAR	(0x3)
#define HW_Y_POS0   0x0
#define HW_Y_POS1   0x1
#define HW_Y_POS2   0x2
#define HW_Y_POS3   0x3

static int
dcc_setsrcformat(struct dcc_drvdata *p,
		int angle,
		int srcfmt,
		int dstfmt, unsigned int cr_off, unsigned int cb_off)
{
	unsigned int yuv2rgb = 0;
	unsigned int rotation = 0;
	unsigned int mirror = 0;
	unsigned int format = 0;

	/* Set conversion flag */
	if (srcfmt != dstfmt)
		yuv2rgb = 1;

	/* Set Rotation flag */
	if (angle == DCC_ROTATE90)
		rotation = 0x1;
	else if (angle == DCC_ROTATE180)
		rotation = 0x2;
	else if (angle == DCC_ROTATE270)
		rotation = 0x3;
	else
		rotation = 0;

	/* get mirror flag */
	mirror = DCC_GET_MIRROR(angle);

	/* Set format parameters */
	if ((srcfmt == DCC_FMT_YUV420PLANAR)
	    || (srcfmt == DCC_FMT_YVU420PLANAR)) {
		format =
		    BITFLDS(INR_DIF_IMG_SRC_FORMAT, HW_YCBCR420PLANAR);
		gra_write_field(p, INR_DIF_IMG_UOFFSET, cb_off);
		gra_write_field(p, INR_DIF_IMG_VOFFSET, cr_off);

	} else if (srcfmt == DCC_FMT_YUV420SP) {
		format = BITFLDS(INR_DIF_IMG_SRC_SEMI, 0x1) |
		    BITFLDS(INR_DIF_IMG_SRC_FORMAT, HW_YCBCR420PLANAR);
		gra_write_field(p, INR_DIF_IMG_UOFFSET, cb_off);
		gra_write_field(p, INR_DIF_IMG_VOFFSET, cr_off);

	} else if (srcfmt == DCC_FMT_YUV422PLANAR) {
		format =
		    BITFLDS(INR_DIF_IMG_SRC_FORMAT, HW_YCBCR422PLANAR);
		gra_write_field(p, INR_DIF_IMG_UOFFSET, cb_off);
		gra_write_field(p, INR_DIF_IMG_VOFFSET, cr_off);

	} else if (srcfmt == DCC_FMT_YUV422PACKED) {
		format =
		    BITFLDS(INR_DIF_IMG_SRC_U_POS, cb_off) |
		    BITFLDS(INR_DIF_IMG_SRC_V_POS, cr_off) |
		    BITFLDS(INR_DIF_IMG_SRC_Y2_POS, HW_Y_POS2) |
		    BITFLDS(INR_DIF_IMG_SRC_Y1_POS, HW_Y_POS0) |
		    BITFLDS(INR_DIF_IMG_SRC_FORMAT, HW_YCBCR422PACKED);

	} else if (srcfmt == DCC_FMT_YUV444PACKED) {
		format = 0;
		yuv2rgb = 1;

	} else if ((srcfmt == DCC_FMT_RGB888) || (srcfmt == DCC_FMT_BGR888)) {
		yuv2rgb = 0;

	} else if ((srcfmt == DCC_FMT_ARGB8888)
		|| (srcfmt == DCC_FMT_ABGR8888)) {
		yuv2rgb = 0;

	} else if ((srcfmt == DCC_FMT_RGB565)
		   || (srcfmt == DCC_FMT_RGB4444)
		   || (srcfmt == DCC_FMT_RGB1555)) {
		/* TODO check fb format == dstfmt */
		yuv2rgb = 0;
		format = BITFLDS(INR_DIF_IMG_SRC_FORMAT, HW_RGB16bit);

	} else {
		dcc_err("%s l.%d: unsupported pixel format 0x%x",
			__func__, __LINE__, srcfmt);
	}

	gra_write_field(p, INR_DIF_IMG_SRC,
			BITFLDS(INR_DIF_IMG_SRC_MIRROR, mirror) |
			BITFLDS(INR_DIF_IMG_SRC_ROT, rotation) |
			format | yuv2rgb);

	return 0;
}

static int dcc_setsrcimage(struct dcc_drvdata *p,
		struct x_area_t *sarea, struct x_rect_t *win,
		uint32_t scalex, uint32_t scaley)
{
	if (win != NULL) {

		gra_write_field(p, INR_DIF_IMG_SIZE,
				BITFLDS(INR_DIF_IMG_SIZE_YRES,
						 sarea->
						 h) |
				BITFLDS(INR_DIF_IMG_SIZE_XRES,
						 sarea->w));
		gra_write_field(p, INR_DIF_IMG_SCALE,
				BITFLDS(INR_DIF_IMG_SCALE_SCALEY,
						 scaley) |
				BITFLDS(INR_DIF_IMG_SCALE_SCALEX,
						 scalex));
		gra_write_field(p, INR_DIF_IMG_WINTL,
				BITFLDS(INR_DIF_IMG_WINTL_Y1,
						 win->
						 y) |
				BITFLDS(INR_DIF_IMG_WINTL_X1, win->x));
		gra_write_field(p, INR_DIF_IMG_WINBR,
				BITFLDS(INR_DIF_IMG_WINBR_Y2,
						 Y2(win)) |
				BITFLDS(INR_DIF_IMG_WINBR_X2,
						 X2(win)));
	} else {
		gra_write_field(p, INR_DIF_IMG_SIZE,
				BITFLDS(INR_DIF_IMG_SIZE_YRES,
						 sarea->
						 h) |
				BITFLDS(INR_DIF_IMG_SIZE_XRES,
						 sarea->w));
		gra_write_field(p, INR_DIF_IMG_SCALE,
				BITFLDS(INR_DIF_IMG_SCALE_SCALEY,
						 scaley) |
				BITFLDS(INR_DIF_IMG_SCALE_SCALEX,
						 scalex));
		gra_write_field(p, INR_DIF_IMG_WINTL, 0);
		gra_write_field(p, INR_DIF_IMG_WINBR,
				BITFLDS(INR_DIF_IMG_WINBR_Y2,
						 (sarea->h -
						  1)) |
				BITFLDS(INR_DIF_IMG_WINBR_X2,
						 sarea->w));
	}

	return 0;
}

static void dcc_getcbcr_offsets(int *cb_off, int *cr_off, int fmt, int w, int h)
{
	if (fmt == DCC_FMT_YUV420PLANAR) {	/* YUV 420 Planar or semi */
		*cb_off = (w * h);
		*cr_off = (w * h) + ((w * h) >> 2);
	} else if (fmt == DCC_FMT_YUV420SP) {	/* YVU 420 SemiPlanar */
		*cb_off = (w * h);
		*cr_off = (w * h) + (2 << 4);
	} else if (fmt == DCC_FMT_YVU420PLANAR) {	/* YVU 420 Planar */
		*cb_off = (w * h) + ((w * h) >> 2);
		*cr_off = (w * h);
	} else if (fmt == DCC_FMT_YUV422PLANAR) {	/* YUV 422 Planar */
		*cb_off = (w * h);
		*cr_off = (w * h) + ((w * h) >> 1);
	} else if (fmt == DCC_FMT_YUV422PACKED) {	/* YUV 422 Packed */
		*cb_off = 1;
		*cr_off = 3;
	} else if (fmt == DCC_FMT_YUV444PACKED) {	/* YUV 444 Packed */
		*cb_off = 2;
		*cr_off = 1;
	} else if ((fmt == DCC_FMT_RGB565) ||
		   (fmt == DCC_FMT_RGB888) ||
		   (fmt == DCC_FMT_BGR888) ||
		   (fmt == DCC_FMT_ARGB8888) ||
		   (fmt == DCC_FMT_ABGR8888) ||
		   (fmt == DCC_FMT_RGB4444) || (fmt == DCC_FMT_RGB1555)) {
		*cb_off = 0;
		*cr_off = 0;
	} else {
		dcc_err("%s, l.%d: unsupported pixel format 0x%x", __func__,
			__LINE__, fmt);
	}
}

/**----------------------------------------------------------------------------
 * SPRITE MANAGEMENT FUNCTIONS
 *---------------------------------------------------------------------------*/
#define DIF_SPRITE_CONF_SET(_a_, _t_, _x_, _y_, _b_) (\
	BITFLDS(INR_DIF_SPRITE_CONF0_TRI, 0) | \
	BITFLDS(INR_DIF_SPRITE_CONF0_BGR, _b_) | \
	BITFLDS(INR_DIF_SPRITE_CONF0_ACT, _a_) | \
	BITFLDS(INR_DIF_SPRITE_CONF0_TYP, _t_) | \
	BITFLDS(INR_DIF_SPRITE_CONF0_XPOS, _x_) | \
	BITFLDS(INR_DIF_SPRITE_CONF0_YPOS, _y_))

#define DIF_SPRITE_SIZE_SET(_w_, _h_, _a_, _g_) (\
	BITFLDS(INR_DIF_SPRITE_SIZE0_WIDTH, _w_) | \
	BITFLDS(INR_DIF_SPRITE_SIZE0_HEIGHT, _h_) | \
	BITFLDS(INR_DIF_SPRITE_SIZE0_WIDTHMSB, (_w_>>10)) | \
	BITFLDS(INR_DIF_SPRITE_SIZE0_ALPHA, _a_) | \
	BITFLDS(INR_DIF_SPRITE_SIZE0_GLOBAL, _g_))

#define DIF_OVERLAY_CONF_SET(_a_, _t_, _b_, _c_, _r_) (\
	BITFLDS(INR_DIF_SPRITE_CONF2_ACT, _a_) | \
	BITFLDS(INR_DIF_SPRITE_CONF2_BGR, _r_) | \
	BITFLDS(INR_DIF_SPRITE_CONF2_TYP, _t_) | \
	BITFLDS(INR_DIF_SPRITE_CONF2_BLEND, _b_) | \
	BITFLDS(INR_DIF_SPRITE_CONF2_CHROMA, _c_))

/**
 * Globally enable/disable sprites
 */
static int dcc_sprite_global(struct dcc_drvdata *p, int en)
{
	unsigned int sprite_confx = 0, tmp = 0;

	if (en) {
		gra_read_field(p, INR_DIF_CONF, &sprite_confx);
		if ((sprite_confx & 0x2) == 0) {
			sprite_confx |= (DIF_CONF_SPR | DIF_CONF_BLEND);
			gra_write_field(p, INR_DIF_CONF, sprite_confx);
			DCC_DBG2("sprite global status %s\n",
				en ? "ON " : "OFF");
		}
	} else {
		gra_read_field(p, INR_DIF_SPRITE_CONF0, &sprite_confx);
		gra_read_field(p, INR_DIF_SPRITE_CONF1, &tmp);
		sprite_confx |= tmp;
		gra_read_field(p, INR_DIF_SPRITE_CONF2, &tmp);
		sprite_confx |= tmp;
		gra_read_field(p, INR_DIF_SPRITE_CONF3, &tmp);
		sprite_confx |= tmp;

		if ((sprite_confx &
			BITFLDS(INR_DIF_SPRITE_CONF0_ACT, 1)) == 0) {
			gra_read_field(p, INR_DIF_CONF, &sprite_confx);
			sprite_confx &= ~(DIF_CONF_SPR | DIF_CONF_BLEND);
			gra_write_field(p, INR_DIF_CONF, sprite_confx);
			DCC_DBG2("sprite global status %s\n",
				en ? "ON " : "OFF");
		}
	}
	return 0;
}


static inline int dcc_sprite_fmt2type(unsigned int fmt)
{
	unsigned int type;

	switch (fmt) {
	case DCC_FMT_ARGB8888:
	case DCC_FMT_RGB888:
		type = 0x0;
		break;
	case DCC_FMT_RGB565:
		type = 0x1;
		break;
	case DCC_FMT_RGB1555:
		type = 0x2;
		break;
	case DCC_FMT_RGB4444:
		type = 0x3;
		break;
	case DCC_FMT_YUV422PACKED:
	case DCC_FMT_YUV420PLANAR:
	case DCC_FMT_YVU420PLANAR:
	case DCC_FMT_YUV422PLANAR:
	case DCC_FMT_YUV444PACKED:
	case DCC_FMT_YUV444PLANAR:
	case DCC_FMT_YUV444SP:
	case DCC_FMT_YUV422SP:
	case DCC_FMT_YUV420SP:
		type = 0x4;
		break;
	default:
		type = 0xF;
		dcc_warn("%s wrong sprite format %d\n", __func__, fmt);
		break;
	}

	return type;
}

static inline int dcc_sprite_fmt2bpp(unsigned int fmt)
{
	/*
	These are the formats currently used by hwcomposer!
	Needs to be enhanced for all supported RGB formats.
	YUV formats always return zero.
	*/

	switch (fmt) {
	case DCC_FMT_ARGB8888:
	case DCC_FMT_ABGR8888:
		return 4;

	case DCC_FMT_RGB565:
		return 2;

	default:
		return 0;
	}
}

static int dcc_sprite_needs_rgb_bgr(struct dcc_sprite_t *spr)
{
	unsigned int spr_is_bgr = 0;
	/* check if RGB to BGR conversion is needed */
	switch (spr->fmt) {
	case DCC_FMT_ABGR8888:
	case DCC_FMT_BGR888:
	case DCC_FMT_RGB565:
		spr_is_bgr = 1;
		break;
	}
	return spr_is_bgr;
}
/**
 * Configure a sprite
 */
static int dcc_sprite_conf(struct dcc_drvdata *p, struct dcc_sprite_t *spr)
{
	int err = 0;
	unsigned int type;
	unsigned int base_reg;
	unsigned int size_reg = 0, size_val;
	unsigned int conf_reg, conf_val;
	static unsigned int last_conf_vals[DCC_OVERLAY_NUM] = {0, 0, 0, 0};

	/* Check sprite id number */
	switch (spr->id) {
	case 0:
		conf_reg = INR_DIF_SPRITE_CONF0;
		size_reg = INR_DIF_SPRITE_SIZE0;
		base_reg = INR_DIF_SPRITE_BASEx0;
		break;
	case 1:
		conf_reg = INR_DIF_SPRITE_CONF1;
		size_reg = INR_DIF_SPRITE_SIZE1;
		base_reg = INR_DIF_SPRITE_BASEx1;
		break;
	case 2:
		conf_reg = INR_DIF_SPRITE_CONF2;
		size_reg = INR_DIF_SPRITE_SIZE2;
		base_reg = INR_DIF_SPRITE_BASEx2;
		break;
	case 3:
		conf_reg = INR_DIF_SPRITE_CONF3;
		size_reg = INR_DIF_SPRITE_SIZE3;
		base_reg = INR_DIF_SPRITE_BASEx3;
		break;
	default:
		err = -1;
		dcc_err("wrong sprite id %d\n", spr->id);
		goto out;
		break;
	}

	if (spr->phys == 0) {
		conf_val = last_conf_vals[spr->id];
		/* deactivate sprite and exit, do not modify any
		 * other field of the sprite as it may be on screen */
		switch (spr->id) {
		case 0:
			conf_val &= ~(BITFLDS(INR_DIF_SPRITE_CONF0_ACT, 1));
			break;
		case 1:
			conf_val &= ~(BITFLDS(INR_DIF_SPRITE_CONF1_ACT, 1));
			break;
		case 2:
			conf_val &= ~(BITFLDS(INR_DIF_SPRITE_CONF2_ACT, 1));
			break;
		case 3:
			conf_val &= ~(BITFLDS(INR_DIF_SPRITE_CONF3_ACT, 1));
			break;
		default:
			err = -1;
			dcc_err("wrong sprite id %d\n", spr->id);
			goto out;
			break;
		}
		gra_write_field(p, conf_reg, conf_val);
		last_conf_vals[spr->id] = conf_val;
		goto out;
	}

	DCC_DBG3("   ovl[%d] %s @0x%08x %dx%d(%d,%d) %s a:0x%x glb:%d\n",
		spr->id, spr->phys ? "ON " : "OFF", spr->phys, spr->w, spr->h,
		spr->x, spr->y, dcc_format_name(spr->fmt), spr->alpha,
		spr->global);

	/**
	 * Set DIF_SPRITE_CONFx value
	 */
	type = dcc_sprite_fmt2type(spr->fmt);
	if (type == 0xF) {
		dcc_warn("Wrong sprite format(%d) for layer(%d) !\n", spr->fmt,
			 spr->id);
		err = -1;
		goto out;
	}

	if (spr->id == 2 && !p->dcc_sprite2_unified) {
		unsigned int spr_is_bgr = dcc_sprite_needs_rgb_bgr(spr);
		if (type == 0x4) {
			dcc_warn("Wrong sprite format(%d) for layer(%d) !\n",
				 spr->fmt, spr->id);
			err = -1;
		}
		if ((spr->x != 0) || (spr->y != 0)) {
			dcc_warn("wrong position for sprite %d %dx%d\n",
				 spr->id, spr->x, spr->y);
			err = -1;
		}
		if ((spr->w != dcc_get_display_w(p))
		    || (spr->h != dcc_get_display_h(p))) {
			dcc_warn("wrong resolution for sprite %d %dx%d\n",
				 spr->id, spr->w, spr->h);
			err = -1;
		}
		if (err)
			goto out;

		conf_val =
		    DIF_OVERLAY_CONF_SET((!!spr->phys), type, spr->global,
					 spr->chromakey, spr_is_bgr);
	} else {
		unsigned int spr_is_bgr = dcc_sprite_needs_rgb_bgr(spr);
		conf_val = DIF_SPRITE_CONF_SET((!!spr->phys), type,
			spr->x, spr->y,	spr_is_bgr);
	}
	gra_write_field(p, conf_reg, conf_val);
	last_conf_vals[spr->id] = conf_val;

	/**
	 * Set DIF_SPRITE_SIZEx value
	 */
	if (spr->id != 2 || p->dcc_sprite2_unified) {
		size_val =
		    DIF_SPRITE_SIZE_SET(spr->w, spr->h, spr->alpha,
					spr->global);
		gra_write_field(p, size_reg, size_val);
	}

	/**
	 * Set DIF_SPRITE_BASEx value
	 */
	gra_write_field(p, base_reg, spr->phys);

out:
	return err;
}

/**
 * Set global alpha
 */
static int dcc_setalpha(struct dcc_drvdata *p, unsigned int alpha)
{
	static int alpha_global = -1;

	if (alpha_global != alpha) {
		gra_sendcmd(p, GRACMD_SET_ALPHA, 0, &alpha, 1);
		alpha_global = alpha;
	}
	return 0;
}

static int dcc_dif_conf_set(struct dcc_drvdata *p, int backen)
{
	int ret = 0;
	unsigned int difconf = 0;

	gra_read_field(p, INR_DIF_CONF, &difconf);
	difconf &= ~BITFLDS(INR_DIF_CONF_BACK, 1);
	difconf |= BITFLDS(INR_DIF_CONF_BACK, !!backen);

	/* SMS06203470 (not mandatory from SoFIA LTE Es2.0) */
	if (p->display_autorefresh)
		difconf |= BITFLDS(INR_DIF_CONF_DEST, 1);
	/* SMS06203470 - end */
	ret = gra_write_field(p, INR_DIF_CONF, difconf);
	return ret;
}

static int dcc_transparency(struct dcc_drvdata *p, int cmd, int value)
{
	unsigned int difconf;

	gra_read_field(p, INR_DIF_CONF, &difconf);

	if ((cmd == ALPHA_PLANE) && (value == 0xFF))
		cmd = ALPHA_NONE;

	switch (cmd) {
	case ALPHA_NONE:
		difconf = difconf & ~DIF_CONF_ALPHA;
		difconf = difconf & ~DIF_CONF_TRANS;
		difconf = difconf & ~DIF_CONF_GLOBAL;
		break;
	case ALPHA_PLANE:
		difconf = difconf & ~DIF_CONF_ALPHA;
		difconf = difconf | DIF_CONF_TRANS;
		difconf = difconf | DIF_CONF_GLOBAL;
		break;
	case ALPHA_PIXEL:
		difconf = difconf | DIF_CONF_TRANS;
		difconf = difconf & ~DIF_CONF_GLOBAL;
		difconf = difconf | DIF_CONF_ALPHA;
		difconf = difconf & ~DIF_CONF_STENCIL_MASK;
		break;
	case ALPHA_COLORKEY:
		difconf = difconf | DIF_CONF_TRANS;
		difconf = difconf & ~DIF_CONF_GLOBAL;
		difconf = difconf | DIF_CONF_ALPHA;
		difconf = difconf & ~DIF_CONF_STENCIL_MASK;
		difconf = difconf | DIF_CONF_STENCIL_CHROMA0;
		break;
	default:
		dcc_err("%s: unknown command (%d)\n", __func__, cmd);
	}

	gra_write_field(p, INR_DIF_CONF, difconf);
	if ((cmd == ALPHA_PLANE) || (cmd == ALPHA_COLORKEY))
		dcc_setalpha(p, value);

	return 0;
}

static int dcc_setbgcolor(struct dcc_drvdata *p, unsigned int color)
{
	static unsigned int shdow_bgcolor;
	uint32_t data[3];

	if (shdow_bgcolor != color) {
		shdow_bgcolor = color;
		data[0] = color & 0xFF;	/* B */
		data[1] = (color >> 8) & 0xFF;	/* G */
		data[2] = (color >> 16) & 0xFF;	/* R */
		gra_sendcmd(p, GRACMD_SET_BK_COLOR, 0, data, 3);
	}

	return 0;
}

int
dcc_flag_to_transparency(struct dcc_drvdata *p,
		unsigned int flag, int alpha, unsigned int colorkey)
{
	int err = 0;

	if (!flag) {
		err = dcc_transparency(p, ALPHA_NONE, 0);

	} else if (flag & DCC_FLAG_COLORKEY) {
		dcc_setbgcolor(p, colorkey);
		dcc_transparency(p, ALPHA_COLORKEY, 0xFF);

	} else if (flag & DCC_BLEND_ALPHA_PIXEL) {
		err = dcc_transparency(p, ALPHA_PIXEL, 0);

	} else if (flag & DCC_BLEND_ALPHA_PLANE) {
		err = dcc_transparency(p, ALPHA_PLANE, alpha);

	}

	return err;
}

static int dcc_setdrawimagedest(struct dcc_drvdata *p,
		int d)
{
#define DCC_DEST_MEMORY		0
#define DCC_DEST_DISPLAY	1
	static int destdraw = -1;
	int err = 0;

	if (destdraw != d) {
		if (d == DCC_DEST_DRAW2MEM) {
			gra_write_field(p, INR_DIF_CONF_DEST, DCC_DEST_MEMORY);
		} else if (d == DCC_DEST_DRAW2DISP) {
			gra_write_field(p, INR_DIF_CONF_DEST, DCC_DEST_DISPLAY);
		} else {
			dcc_err("unknown destination");
			err = -1;
		}
		destdraw = d;
	}
	return err;
}

/**
 * This function configure the video buffer format
 */
int dcc_setbufferformat(struct dcc_drvdata *p, int format)
{
	if (format == DCC_FMT_RGB888) {
		DCC_DBG2("Video buffer format is RGB888\n");
		gra_write_field(p, INR_DIF_CONF_PIXEL, 4);
	} else if (format == DCC_FMT_RGB565) {
		DCC_DBG2("Video buffer format is RGB565\n");
		gra_write_field(p, INR_DIF_CONF_PIXEL, 2);
	} else {
		dcc_err("Video buffer format is unknown ! ");
	}

	return 0;
}

/**
 * Reset DCC ip by toggling RUN bit.
 * Whatever the previous state, After this function, DCC is in CONF mode
 */
void dcc_hwreset(struct dcc_drvdata *p)
{
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
}

void dcc_config_reset(struct dcc_drvdata *p)
{
	/* set drawimage destination to memory */
	dcc_setdrawimagedest(p, DCC_DEST_DRAW2MEM);
	gra_write_field(p, EXR_DIF_ICR, 0xfff);
	/* Reset sprite configuration */
	gra_write_field(p, INR_DIF_SPRITE_CONF0, 0);
	gra_write_field(p, INR_DIF_SPRITE_CONF1, 0);
	gra_write_field(p, INR_DIF_SPRITE_CONF2, 0);
	gra_write_field(p, INR_DIF_SPRITE_CONF3, 0);
	gra_write_field(p, INR_DIF_CONF_SPR, 0);	/* disable sprites */
	dcc_setdrawmode(p, DRAWMODE_SET);
	dcc_transparency(p, ALPHA_NONE, 0);
}

static int dcc_setcolor(struct dcc_drvdata *p, unsigned int color)
{
	uint32_t data[3];

	data[0] = color & 0xFF;	/* B */
	data[1] = (color >> 8) & 0xFF;	/* G */
	data[2] = (color >> 16) & 0xFF;	/* R */
	gra_sendcmd(p, GRACMD_SET_COLOR, 0, data, 3);

	return 0;
}

static int dcc_fillrectangle(struct dcc_drvdata *p,
		unsigned int color, struct x_rect_t *r)
{
	int err = 0;
	uint32_t data[4];
	int alpha = (color >> 24) & 0xFF;

	if ((r->fmt == DCC_FMT_ARGB8888) || (r->fmt == DCC_FMT_ABGR8888)) {
		if (alpha == 0xFF)
			dcc_flag_to_transparency(p, DCC_FLAG_NONE, 0, 0);
		else if (alpha == 0)
			goto exit;
		else
			dcc_flag_to_transparency(p, DCC_BLEND_ALPHA_PLANE,
					alpha, 0);
	} else {
		dcc_flag_to_transparency(p, DCC_FLAG_NONE, 0, 0);
	}

	err = dcc_setcolor(p, color);
	if (err)
		goto exit;

	/* send command */
	data[0] = r->x;		/* x1 */
	data[1] = r->y;		/* y1 */
	data[2] = X2(r);	/* x2 */
	data[3] = Y2(r);	/* y2 */
	gra_sendcmd(p, GRACMD_RECT, 0, data, 4);

exit:
	return err;
}

static int dcc_drawline(struct dcc_drvdata *p,
		struct x_rect_t *r, unsigned int color)
{
	uint32_t data[2];
	int err = 0;

	data[0] = r->x;		/* x1 */
	data[1] = r->y;		/* y1 */
	gra_sendcmd(p, GRACMD_MOVE_TO, 0, data, 2);

	dcc_transparency(p, ALPHA_PLANE, (color >> 24) & 0xFF);
	err = dcc_setcolor(p, color);
	if (err)
		goto exit;

	data[0] = X2(r);	/* x2 */
	data[1] = Y2(r);	/* y2 */
	gra_sendcmd(p, GRACMD_LINE_TO, 0, data, 2);

exit:
	return err;
}

static int dcc_drawlinerel(struct dcc_drvdata *p,
		struct x_rect_t *r, unsigned int color)
{
	uint32_t data[2];
	int err = 0;

	/* Move cursor to the buffer base coordinates */
	data[0] = 0;
	data[1] = 0;
	gra_sendcmd(p, GRACMD_MOVE_TO, 0, data, 2);

	/* set color + transparency */
	dcc_transparency(p, ALPHA_PLANE, (color >> 24) & 0xFF);
	err = dcc_setcolor(p, color);
	if (err)
		goto exit;

	/* move cursor relatively from current position */
	data[0] = r->x;		/* x1 */
	data[1] = r->y;		/* y1 */
	gra_sendcmd(p, GRACMD_MOVE_REL, 0, data, 2);

	/* draw line relatively from current position */
	data[0] = X2(r);	/* x2 */
	data[1] = Y2(r);	/* y2 */
	gra_sendcmd(p, GRACMD_LINE_REL, 0, data, 2);

exit:
	return err;
}

static void dcc_update(struct dcc_drvdata *p, struct x_rect_t *r,
			unsigned int alpha, unsigned int pbase_yuv)
{
	/* prepare update */
	dcc_dif_conf_set(p, DCC_UPDATE_NOBG_GET(r->flags));

	if (p->display_autorefresh)
		DCC_UPDATE_MODE_SET(r->flags, DCC_UPDATE_CONTINOUS);

	if (DCC_UPDATE_MODE_GET(r->flags) == DCC_UPDATE_ONESHOT_SYNC) {
		uint32_t paramssc[7] = {
				alpha, 0x5, r->x, r->y, X2(r), Y2(r), 0 };

		if (p->display.frame_prepare)
			p->display.frame_prepare(&p->display,
					(dcc_get_display_w(p) * 3) / 2,
					2 * dcc_get_display_h(p));

		gra_sendcmd(p, GRACMD_SCHEDULE_UPDATE, 0, paramssc, 7);
		if (p->display.frame_wfe)
			p->display.frame_wfe(&p->display);

	} else if (DCC_UPDATE_MODE_GET(r->flags) == DCC_UPDATE_ONESHOT_ASYNC) {
		uint32_t paramssc[7] = {
				alpha, 0x5, r->x, r->y, X2(r), Y2(r), 0 };

		if (p->display.frame_prepare)
			p->display.frame_prepare(&p->display,
					(dcc_get_display_w(p) * 3) / 2,
					2 * dcc_get_display_h(p));

		gra_sendcmd(p, GRACMD_SCHEDULE_UPDATE, 0, paramssc, 7);

	} else if (DCC_UPDATE_MODE_GET(r->flags) == DCC_UPDATE_CONTINOUS) {
		uint32_t paramssc[7] = {
			alpha, 0x6, r->x, r->y, X2(r), Y2(r), pbase_yuv };

		gra_sendcmd(p, GRACMD_SCHEDULE_UPDATE, 0, paramssc, 7);
		dcc_dsi_start_video(&p->display);/* TEMP PATCH SMS05120496 */
	}
}

void dcc_bootscreen(struct dcc_drvdata *p)
{
	struct x_rect_t r;
	RECT_INIT(r, 0, 0, dcc_get_display_w(p), dcc_get_display_h(p),
		  dcc_get_fb_fmt(p), DCC_UPDATE_ONESHOT_SYNC);

	DCC_DBGT("--> Bootscreen test started\n");
	dcc_set_framebuffer(p, p->mem.pbase, dcc_get_display_w(p));

	dcc_fillrectangle(p, 0xFF0000FF, &r);
	dcc_update(p, &r, 0, 0);

	mdelay(p->test.bootscreen_msdelay);
	dcc_fillrectangle(p, 0xFF00FF00, &r);
	dcc_update(p, &r, 0, 0);

	mdelay(p->test.bootscreen_msdelay);
	dcc_fillrectangle(p, 0xFFFF0000, &r);
	dcc_update(p, &r, 0, 0);

	mdelay(p->test.bootscreen_msdelay);
	DCC_DBGT("<-- Bootscreen test passed\n");
}

void dcc_clearscreen(struct dcc_drvdata *p)
{
	struct x_rect_t r;
	RECT_INIT(r, 0, 0, dcc_get_display_w(p), dcc_get_display_h(p),
		  dcc_get_fb_fmt(p), DCC_UPDATE_ONESHOT_SYNC);

	dcc_set_framebuffer(p, p->mem.pbase, dcc_get_display_w(p));

	if (p->use_fbapi)
		dcc_fillrectangle(p, 0xFF000000, &r); /* black screen */
	else
		DCC_UPDATE_NOBG_SET(r.flags, 1); /* black screen */

	dcc_update(p, &r, 0xFF, 0);
}

void dcc_clearvideomem(struct dcc_drvdata *p)
{
	unsigned int nbpixels = p->mem.size / dcc_get_fb_bpp(p);
	unsigned int physbase = p->mem.pbase;

	while (nbpixels >= 1024) {
		int w = 1024;
		int h = (nbpixels / 1024 >= 1024) ? 1024 : nbpixels / 1024;
		struct x_rect_t rblack = { 0, 0, w, h };
		RECT_INIT(rblack, physbase, 0, w, h, dcc_get_fb_fmt(p),
				DCC_UPDATE_ONESHOT_SYNC);

		DCC_DBG2("%s clear 0x%08x, (%dx%d) remaining pixels %d\n",
			__func__, physbase, w, h, nbpixels);
		dcc_set_framebuffer(p, physbase, rblack.w);
		dcc_fillrectangle(p, 0xFF000000, &rblack);
		physbase += w * h * dcc_get_fb_bpp(p);
		nbpixels -= w * h;
	}
	DCC_DBG2("%s done\n", __func__);
}

#ifdef DCC_CLIPPING	/* currently unneedded */
int setclipping(struct dcc_drvdata *p, struct x_rect_t *win)
{
	if (win == NULL) {
		gra_write_field(p, INR_DIF_CONF_CLIP, 0);
	} else {
		gra_write_field(p, INR_DIF_CLIPPING_TL,
				(win->y << 16) | win->x);
		gra_write_field(p, INR_DIF_CLIPPING_BR,
				(Y2(win) << 16) | X2(win));
		gra_write_field(p, INR_DIF_CONF_CLIP, 1);
		DCC_DBG2("Set clipping area (%d,%d)%dx%d\n", win->x, win->y,
			win->w, win->h);
	}

	return 0;
}
#endif

int dcc_wait_status(struct dcc_drvdata *pdata,
		unsigned int reg,
		unsigned int pattern,
		int to)
{
	int wait_delay_ms = 5;
	int wait_loop_n = to / wait_delay_ms;

	gra_read_field(pdata, reg, &reg);
	DCC_DBG2("wait idle 0x%08x (0x%08x)\n", reg, pattern);

	while ((reg & pattern) != pattern) {
		mdelay(wait_delay_ms);
		wait_loop_n--;
		if (!wait_loop_n) {
			dcc_err("idle loop timedout!\n");
			return -EBUSY;
		}
		gra_read_field(pdata, reg, &reg);
	}
	return 0;
}

void dcc_setBGR2RGBcoeff(struct dcc_drvdata *p)
{
	unsigned int reg1 = 0, reg2 = 0, reg3 = 0, regoff = 0;

	static int coeff[4][3] = {
		{0, 0, 128},
		{0, 128, 0},
		{128, 0, 0},
		{0, 0, 0}
	};

	reg1 =
	    ((coeff[0][2] & 0x3FF) << 20) | ((coeff[0][1] & 0x3FF) << 10) |
	    (coeff[0][0] & 0x3FF);
	reg2 =
	    ((coeff[1][2] & 0x3FF) << 20) | ((coeff[1][1] & 0x3FF) << 10) |
	    (coeff[1][0] & 0x3FF);
	reg3 =
	    ((coeff[2][2] & 0x3FF) << 20) | ((coeff[2][1] & 0x3FF) << 10) |
	    (coeff[2][0] & 0x3FF);
	regoff =
	    ((coeff[3][2] & 0x3FF) << 20) | ((coeff[3][1] & 0x3FF) << 10) |
	    (coeff[3][0] & 0x3FF);

	DCC_DBG2("BGR 2 RGB coeff(0x%08x,0x%08x,0x%08x) offset(0x%08x)\n",
		reg1, reg2, reg3, regoff);

	dcc_wait_status(p, EXR_DIF_STAT, BITFLDS(EXR_DIF_STAT_BSY, 0), 2000);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	gra_write_field(p, EXR_DIF_COEFF_REG1, reg1);
	gra_write_field(p, EXR_DIF_COEFF_REG2, reg2);
	gra_write_field(p, EXR_DIF_COEFF_REG3, reg3);
	gra_write_field(p, EXR_DIF_OFFSET_REG, regoff);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
}

void dcc_setYUV2RGBcoeff(struct dcc_drvdata *p)
{
	unsigned int reg1 = 0, reg2 = 0, reg3 = 0, regoff = 0;

	static int coeff[4][3] = {
		{149, 0, 258},
		{149, -104, -50},
		{149, 204, 0},
		{16, 128, 128}
	};

	reg1 =
	    ((coeff[0][2] & 0x3FF) << 20) | ((coeff[0][1] & 0x3FF) << 10) |
	    (coeff[0][0] & 0x3FF);
	reg2 =
	    ((coeff[1][2] & 0x3FF) << 20) | ((coeff[1][1] & 0x3FF) << 10) |
	    (coeff[1][0] & 0x3FF);
	reg3 =
	    ((coeff[2][2] & 0x3FF) << 20) | ((coeff[2][1] & 0x3FF) << 10) |
	    (coeff[2][0] & 0x3FF);
	regoff =
	    ((coeff[3][2] & 0x3FF) << 20) | ((coeff[3][1] & 0x3FF) << 10) |
	    (coeff[3][0] & 0x3FF);

	DCC_DBG2("YUV 2 RGB coeff(0x%08x,0x%08x,0x%08x) offset(0x%08x)\n",
		reg1, reg2, reg3, regoff);

	dcc_wait_status(p, EXR_DIF_STAT, BITFLDS(EXR_DIF_STAT_BSY, 0), 2000);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_CONF);
	gra_write_field(p, EXR_DIF_COEFF_REG1, reg1);
	gra_write_field(p, EXR_DIF_COEFF_REG2, reg2);
	gra_write_field(p, EXR_DIF_COEFF_REG3, reg3);
	gra_write_field(p, EXR_DIF_OFFSET_REG, regoff);
	gra_write_field(p, EXR_DIF_RUNCTRL, DCC_MODE_RUN);
}

void dcc_format_set_convertmatrix(struct dcc_drvdata *p,
		int sfmt, int dfmt, int force)
{
	static int yuv = -1;

	if (((sfmt == DCC_FMT_ABGR8888) && (dfmt == DCC_FMT_RGB565)) ||
	    ((sfmt == DCC_FMT_BGR888) && (dfmt == DCC_FMT_RGB565))) {
		if ((force) || (yuv != 0)) {
			dcc_setBGR2RGBcoeff(p);
			yuv = 0;
		}
	} else if (((sfmt == DCC_FMT_YUV422PLANAR) ||
		    (sfmt == DCC_FMT_YUV422PACKED) ||
		    (sfmt == DCC_FMT_YUV444PACKED) ||
		    (sfmt == DCC_FMT_YVU420PLANAR) ||
		    (sfmt == DCC_FMT_YUV420SP) ||
		    (sfmt == DCC_FMT_YUV420PLANAR)) &&
		   ((dfmt == DCC_FMT_RGB565) ||
		    (dfmt == DCC_FMT_RGB888) ||
		    (dfmt == DCC_FMT_ABGR8888) || (dfmt == DCC_FMT_ARGB8888))) {
		if ((force) || (yuv != 1)) {
			dcc_setYUV2RGBcoeff(p);
			yuv = 1;
		}
	}
}

#define NO_RESCALING	0x1000

unsigned int scaling_factor_int(unsigned int src_size, unsigned int dst_size)
{
	unsigned int scalingfactor = 0;

	/* upscaling limit is 1:4 */
	if (dst_size > (4 * src_size)) {
		dcc_err("cannot handle scaling factor dst(%d) > 4*src(%d)",
			dst_size, src_size);
		return NO_RESCALING;
	}
	/* downscaling limit is 16:1 */
	if ((16 * dst_size) < src_size) {
		dcc_err("cannot handle scaling factor dst(%d) > 4*src(%d)",
			dst_size, src_size);
		return NO_RESCALING;
	}

	if (src_size % dst_size == 0) {
		scalingfactor = (unsigned short)((src_size << 12) / dst_size);
	} else {
		/*fast method to calculate the scaling factor
		 * (for details see the specification...) */
		scalingfactor =
		    ((unsigned short)(((src_size - 1) << 12) / (dst_size)) + 1);
	}

	return scalingfactor;
}

static int dcc_drawimage(struct dcc_drvdata *p,
		struct x_area_t *sarea,
		struct x_rect_t *swin,
		struct x_rect_t *drect,
		unsigned int srcaddr, int flag, int alpha)
{
	uint32_t data[6];
	uint32_t scalex = NO_RESCALING;
	uint32_t scaley = NO_RESCALING;
	unsigned int cr_off = 0, cb_off = 0;

	int convRGB2BGR = (sarea->fmt == DCC_FMT_BGR888)
	    || (sarea->fmt == DCC_FMT_ABGR8888);

	dcc_format_set_convertmatrix(p, sarea->fmt, drect->fmt, 0);
	if (convRGB2BGR) {
		sarea->fmt = DCC_FMT_YUV444PACKED;
		swin->fmt = DCC_FMT_YUV444PACKED;
	}

	/* Compute scale ratio on X and Y axis */
	if ((swin == NULL)
	    && ((sarea->w != drect->w) || (sarea->h != drect->h))) {
		scalex = scaling_factor_int(sarea->w, drect->w);
		scaley = scaling_factor_int(sarea->h, drect->h);
	} else if (swin) {
		if ((swin->w != drect->w) || (swin->h != drect->h)) {
			scalex = scaling_factor_int(swin->w, drect->w);
			scaley = scaling_factor_int(swin->h, drect->h);
		}
	} else {
		dcc_err("swin is null\n");
	}

	/* check constraints */
	DCC_CHECK((srcaddr & 0x3) == 0,
		  "src adress(0x%08x) is not dividable by 4 !!", srcaddr);
	DCC_CHECK(scalex <= 0xFFFF, "scaleX factor(0x%08x) overflow !!",
		  scalex);
	DCC_CHECK(scaley <= 0xFFFF, "scaleY factor(0x%08x) overflow !!",
		  scaley);

	/* Compute Cr and Cb offsets if needed */
	dcc_getcbcr_offsets(&cb_off, &cr_off, sarea->fmt, sarea->w, sarea->h);
	dcc_setsrcformat(p, 0, sarea->fmt, drect->fmt, cr_off, cb_off);
	dcc_setsrcimage(p, sarea, swin, scalex, scaley);
	if (flag & DCC_FLAG_DRAW2DISP) {
		struct x_rect_t r;
		struct x_rect_t *pr = &r;
		RECT_INIT(r, 0, 0, dcc_get_display_w(p), dcc_get_display_h(p),
			  dcc_get_fb_fmt(p), DCC_UPDATE_ONESHOT_SYNC);
		data[0] = 0xFF;	/* alpha */
		data[1] = 0;	/* x1 */
		data[2] = 0;	/* Y1 */
		data[3] = X2(pr);	/* x2 */
		data[4] = Y2(pr);	/* y2 */
		data[5] = srcaddr;	/* physical src address */

		if (p->display.frame_prepare)
			p->display.frame_prepare(&p->display,
						dcc_get_display_w(p) * 3,
						dcc_get_display_h(p));
	} else {

		data[0] = 0xFF;		/* alpha */
		data[1] = drect->x;	/* x1 */
		data[2] = drect->y;	/* Y1 */
		data[3] = X2(drect);	/* x2 */
		data[4] = Y2(drect);	/* y2 */
		data[5] = srcaddr;	/* physical src address */
	}
	gra_sendcmd(p, GRACMD_DRAW_IMAGE, 0, data, 6);
	if (flag & DCC_FLAG_DRAW2DISP) {
		if (p->display.frame_wfe) {
			int ret = 0;
			ret = p->display.frame_wfe(&p->display);
		}
	}
	return 0;
}

static int dcc_cfg_video(struct dcc_drvdata *p,
		struct x_area_t *sarea,
		struct x_rect_t *swin,
		struct x_rect_t *drect,
		unsigned int srcaddr, int flag, int alpha)
{
	uint32_t scalex = NO_RESCALING;
	uint32_t scaley = NO_RESCALING;
	unsigned int cr_off = 0, cb_off = 0;

	dcc_format_set_convertmatrix(p, sarea->fmt, drect->fmt, 0);

	/* Compute scale ratio on X and Y axis */
	if ((swin == NULL)
	    && ((sarea->w != drect->w) || (sarea->h != drect->h))) {
		scalex = scaling_factor_int(sarea->w, drect->w);
		scaley = scaling_factor_int(sarea->h, drect->h);
	} else if (swin) {
		if ((swin->w != drect->w) || (swin->h != drect->h)) {
			scalex = scaling_factor_int(swin->w, drect->w);
			scaley = scaling_factor_int(swin->h, drect->h);
		}
	} else {
		dcc_err("swin is null\n");
	}

	/* check constraints */
	DCC_CHECK((srcaddr & 0x3) == 0,
		  "src adress(0x%08x) is not dividable by 4 !!", srcaddr);
	DCC_CHECK(scalex <= 0xFFFF, "scaleX factor(0x%08x) overflow !!",
		  scalex);
	DCC_CHECK(scaley <= 0xFFFF, "scaleY factor(0x%08x) overflow !!",
		  scaley);

	/* Compute Cr and Cb offsets if needed */
	dcc_getcbcr_offsets(&cb_off, &cr_off, sarea->fmt, sarea->w, sarea->h);
	dcc_setsrcformat(p, 0, sarea->fmt, drect->fmt, cr_off, cb_off);
	dcc_setsrcimage(p, sarea, swin, scalex, scaley);
	return 0;
}

int dcc_bitblit(struct dcc_drvdata *p,
		struct x_rect_t *r, unsigned int srcaddr)
{
	uint32_t data[6];

	data[0] = r->x;		/* x1 */
	data[1] = r->y;		/* y1 */
	data[2] = X2(r);	/* x2 */
	data[3] = Y2(r);	/* y2 */
	data[4] = 0;		/* color expansion mode */
	data[5] = srcaddr;	/* src */
	gra_sendcmd(p, GRACMD_BITBLT, 0, data, 6);

	return 0;
}

int dcc_blitimage(struct dcc_drvdata *p,
		unsigned int srcaddr, int srcw, struct x_rect_t *drect)
{
	uint32_t data[9];
	int lineincr = (srcw - drect->w);

	data[0] = drect->x;	/* x1 */
	data[1] = drect->y;	/* y1 */
	data[2] = X2(drect);	/* x2 */
	data[3] = Y2(drect);	/* y2 */
	data[4] = 0;		/* synchronization with update */
	data[5] = 0;		/* linked list source */
	data[6] = 0;		/* convert 16/32bit */
	data[7] = lineincr;	/*  */
	data[8] = srcaddr;	/* src */
	gra_sendcmd(p, GRACMD_BITBLT2, 0, data, 9);

	return 0;
}

int
dcc_rotateimage(struct dcc_drvdata *p,
		struct x_area_t *sarea, unsigned int srcaddr,
		unsigned int dstaddr, int angle)
{
	uint32_t data[2];
	unsigned int cr_off, cb_off;

	/* Compute Cr and Cb offsets if needed */
	dcc_getcbcr_offsets(&cb_off, &cr_off, sarea->fmt, sarea->w, sarea->h);

	dcc_setsrcformat(p, angle, sarea->fmt, sarea->fmt, cr_off, cb_off);
	dcc_setsrcimage(p, sarea, NULL, NO_RESCALING, NO_RESCALING);

	data[0] = srcaddr;	/* physical src address */
	data[1] = dstaddr;	/* physical dst address */
	gra_sendcmd(p, GRACMD_ROTATE_IMAGE, 0, data, 2);

	return 0;
}

static int dcc_scrollmove(struct dcc_drvdata *p,
		struct x_rect_t *sr, struct x_rect_t *dr)
{
	uint32_t data[6];

	data[0] = dr->x;	/* x1 */
	data[1] = dr->y;	/* y1 */
	data[2] = X2(dr);	/* x2 */
	data[3] = Y2(dr);	/* y2 */
	data[4] = sr->x;	/* x1 */
	data[5] = sr->y;	/* y1 */
	gra_sendcmd(p, GRACMD_SCROLL_MOVE, 0, data, 6);

	return 0;
}

#define DCC_RQ_CONVERT_DBG \
	"convert %dx%d @ 0x%x fmt 0x%x -> %dx%d @ 0x%x (%d,%d) fmt 0x%x\n"
int dcc_rq_convert(struct dcc_drvdata *p,
		struct dcc_rq_t *rq)
{
	struct x_area_t sarea;
	struct x_rect_t drect;

	AREA_INIT(sarea, rq->sw, rq->sh, rq->sfmt);
	RECT_INIT(drect, rq->dx, rq->dy, rq->dw, rq->dh, rq->dfmt, rq->flags);
	DCC_DBG2(DCC_RQ_CONVERT_DBG,
	     rq->sw, rq->sh, rq->sphys, rq->sfmt, rq->dw, rq->dh, rq->dphys,
	     rq->dx, rq->dy, rq->dfmt);

	dcc_set_framebuffer(p, rq->dphys, rq->fbwidth);
	dcc_drawimage(p, &sarea, NULL, &drect, rq->sphys, 0, rq->alpha);
	return 0;
}

int dcc_rq_update(struct dcc_drvdata *p, struct dcc_rect_t *ru,
		unsigned int pbase_yuv)
{
	struct x_rect_t rk;

	RECT_INIT(rk, 0, 0, ru->w, ru->h, dcc_get_fb_fmt(p), ru->flags);

	if ((ru->fmt == dcc_get_fb_fmt(p)) ||
	    ((ru->fmt == DCC_FMT_ARGB8888)
	     && (dcc_get_fb_fmt(p) == DCC_FMT_RGB888))
	    || ((ru->fmt == DCC_FMT_RGB888)
		&& (dcc_get_fb_fmt(p) == DCC_FMT_ARGB8888))) {
		DCC_DBG2("update 0x%x --> (%d,%d) %dx%d %s f:0x%x\n",
			ru->phys, rk.x, rk.y, rk.w, rk.h,
			dcc_format_name(ru->fmt), rk.flags);
		dcc_set_framebuffer(p, ru->phys, ru->fbwidth);
		dcc_update(p, &rk, (ru->color >> 24) & 0xFF,
				pbase_yuv);
	} else {
		struct x_area_t sarea;
		AREA_INIT(sarea, ru->w, ru->h, ru->fmt);

		DCC_DBG2
		    ("update conversion %dx%d @ 0x%x %s -> (%d,%d) %dx%d %s\n",
		     ru->w, ru->h, ru->phys, dcc_format_name(ru->fmt), rk.x,
		     rk.y, rk.w, rk.h, dcc_format_name(rk.fmt));
		dcc_set_framebuffer(p, ru->phys, ru->fbwidth);
		dcc_setdrawimagedest(p, DCC_DEST_DRAW2DISP);
		dcc_drawimage(p, &sarea, NULL, &rk, ru->phys, 0, 0xFF);
		dcc_setdrawimagedest(p, DCC_DEST_DRAW2MEM);
	}
	p->debug.frame_update_number++;
	return 0;
}

/*
Check if a sprite can be used for the current update, rules are:

 - max 2 overlays enabled
 - do not use the same ones that were used in previous update
 - two sets are allowed: (0,1) or (2,3)

note: the first overlay must be full screen (limitation from sprite 2).
*/
static int dcc_ovl_en(int spr_id, int ovl_upd_id)
{
	static const int ovl_enabled[2][DCC_OVERLAY_NUM] = {
		{ 1, 1, 0, 0 },
		{ 0, 0, 1, 1 }
	};
	if (spr_id < DCC_OVERLAY_NUM)
		return ovl_enabled[ovl_upd_id & 0x01][spr_id];
	else
		return 0;
}

static void dcc_set_overlay(struct dcc_sprite_t *spr, struct dcc_layer_ovl *l,
		int ovl_id, int l_id)
{

	DCC_DBG3("  spr[%d]<-ov[%d], @0x%x\n", ovl_id, l_id, l->phys);

	/* YUV formats offset not supported! */
	if (!l->phys)
		BUG();

	l->phys += dcc_sprite_fmt2bpp(l->fmt) *
		(l->src.y * l->src.w + l->src.x);

	DCC_SPRITE_INIT((*spr),
		1,
		ovl_id,
		l->phys,
		l->dst.x, l->dst.y, l->dst.w, l->dst.h,
		l->alpha, l->global, l->fmt, l->chromakey);
}

static void dcc_clr_overlay(struct dcc_sprite_t *spr, int spr_id)
{
	DCC_SPRITE_INIT((*spr), 0, spr_id, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

static int dcc_rq_compose(struct dcc_drvdata *p,
		struct dcc_update_layers *updt, int update_pt)
{
	struct dcc_rect_t ru;
	struct dcc_sprite_t spr;
	unsigned int pbase_yuv = 0;
	int ret = 0, global_ovl_status = 0, l_id = 0, ovl_id;
	struct dcc_layer_ovl *lyuv = NULL;
	struct dcc_layer_ovl *l = &updt->ovls[l_id];

	DCC_DBG2("compose --> BACK @0x%x %s, ov_up_cnt=%d updt_pt=%d\n",
			updt->back.phys, dcc_format_name(updt->back.fmt),
			p->overlay_updt_cnt, update_pt);
	for (ovl_id = 0; ovl_id < DCC_OVERLAY_NUM; ovl_id++) {
		/* skip update layers with a null pointer */
		while (!l->phys && l_id < DCC_OVERLAY_NUM) {
			l = &updt->ovls[l_id];
			l_id++;
		}

		if (!dcc_ovl_en(ovl_id, p->overlay_updt_cnt) ||
			l_id >= DCC_OVERLAY_NUM) {
			/* this overlay cannot be used this time, skip it and
			 * disable for next update */
			dcc_clr_overlay(&spr, ovl_id);
		} else {
			/* use this overlay for current layer
			 * and go to next one */
		DCC_DBG2("ovl[%d] %s @0x%x %dx%d(%d,%d) --> %dx%d(%d,%d) %s\n",
				ovl_id, l->phys ? "ON " : "OFF", l->phys,
				l->src.w, l->src.h, l->src.x, l->src.y,
				l->dst.w, l->dst.h, l->dst.x, l->dst.y,
				dcc_format_name(l->fmt));
			dcc_set_overlay(&spr, l, ovl_id, l_id);
			/* register yuv layer */
			if (IS_DCC_FMT_YUV(l->fmt))
				lyuv = l;
			l_id++;
			l = &updt->ovls[l_id];
			global_ovl_status = 1;
		}
		dcc_sprite_conf(p, &spr);
	}
	dcc_sprite_global(p, (!!global_ovl_status));

	/* if any overlay is enabled toggle configuration for next time */
	if (global_ovl_status)
		p->overlay_updt_cnt++;

	/* check if all update layers could be rendered */
	while (l_id < DCC_OVERLAY_NUM) {
		l = &updt->ovls[l_id];
		if (l->phys)
			dcc_warn("too many overlays: ov[%d] @0x%x not rendered",
				l_id, l->phys);
		l_id++;
	}

	if (lyuv) {
		/* configure video related registers */
		struct x_rect_t dr, swin;
		struct x_area_t sarea;

		AREA_INIT(sarea, lyuv->src.w, lyuv->src.h,
				lyuv->fmt);
		RECT_INIT(swin, lyuv->src.x, lyuv->src.y,
				lyuv->src.w, lyuv->src.h,
				l->fmt, 0);
		RECT_INIT(dr, lyuv->dst.x, lyuv->dst.y,
				lyuv->dst.w, lyuv->dst.h,
				dcc_get_fb_fmt(p), 0);

		pbase_yuv = lyuv->phys;

		dcc_cfg_video(p, &sarea, &swin, &dr, updt->back.phys, 0, 0xFF);
	}

	/* set up FB */
	DCC_INIT_RECT(ru, updt->back.phys, updt->back.stride,
			updt->back.src.x, updt->back.src.y,
			updt->back.src.w, updt->back.src.h,
			updt->back.fmt,	0, updt->flags);

	ret = dcc_rq_update(p, &ru, pbase_yuv);

	down(&p->update_sem);
	p->update_pt_curr = update_pt;
	up(&p->update_sem);

	return ret;
}

static void acq_fence_wq(struct work_struct *ws)
{
	struct dcc_acq_fence_work *w;
	w = container_of(ws, struct dcc_acq_fence_work, work);

	DCC_DBG3("acq start, updt_pt=%d\n", w->update_pt);
#if defined(CONFIG_SYNC)
	if (w->drv->use_fences) {
		int i;
		/* Wait for acquire fence to signal if we got one */
		for (i = 0 ; i < DCC_OVERLAY_NUM + 2; i++) {
			struct sync_fence *fence;
			fence = w->acquire_fence[i];
			if (fence != NULL) {
				if(sync_fence_wait(fence, 1000))
					dcc_err("fence timedout\n");
				sync_fence_put(fence);
			}
		}
	}
#endif
	dcc_rq_compose(w->drv, &w->update, w->update_pt);
	kfree(ws);
}

#ifdef CONFIG_SW_SYNC_USER
static struct sync_fence *dcc_update_queue_fence_create(
		struct dcc_drvdata *pdata, unsigned int timeline_value)
{
	struct sync_pt *point = NULL;
	struct sync_fence *fence = 0;

	if (!pdata->updt_done_tl)
		goto leave_err;

	/* Create sync point */
	point = sw_sync_pt_create(pdata->updt_done_tl,
			timeline_value);
	if (point == NULL)
		goto leave_err;

	/* Create fence */
	fence = sync_fence_create("dcc-update-fence", point);
	if (fence == NULL)
		goto leave_err;
	return fence;

leave_err:
	if (point != NULL)
		sync_pt_free(point);
	return fence;
}
#endif


int dcc_rq_acquire_and_compose(struct dcc_drvdata *p,
		struct dcc_update_layers *updt, int updt_pt)
{
	struct dcc_acq_fence_work *work;
	unsigned int i;
#if defined(CONFIG_SYNC)
	int fence;
#endif

	DCC_DBG3("rq updt_pt=%d\n", updt_pt);
	work = kzalloc(sizeof(*work), GFP_KERNEL);
	if (!work) {
		dcc_err("allocation of fence acquire item failed\n");
		return -ENOMEM;
	}
	INIT_WORK(&work->work, acq_fence_wq);
	work->drv = p;
	work->update = *updt;
	work->update_pt = updt_pt;
#if defined(CONFIG_SYNC)
	if (p->use_fences) {
		fence = updt->back.fence_acquire;
		if (fence >= 0)
			work->acquire_fence[0] = sync_fence_fdget(fence);
		for (i = 0; i < DCC_OVERLAY_NUM ; i++) {
			fence = updt->ovls[i].fence_acquire;
			if (fence >= 0)
				work->acquire_fence[i + 1] =
					sync_fence_fdget(fence);
		}
	}
#ifdef CONFIG_SW_SYNC_USER
	if (updt_pt > 0) {
		struct sync_fence *f;
		/* add a fence on the previous update */
		f = dcc_update_queue_fence_create(p, updt_pt - 1);
		work->acquire_fence[DCC_OVERLAY_NUM + 1] = f;
		if (!f)
			dcc_err("prev_update fence creation failed\n");
	}
#endif
#endif
	queue_work(p->acq_wq, &work->work);
	return 0;
}

int dcc_rq_fillrectangle(struct dcc_drvdata *p, struct dcc_rect_t *ru)
{
	struct x_rect_t rk;
	RECT_INIT(rk, ru->x, ru->y, ru->w, ru->h, ru->fmt, ru->flags);

	DCC_DBG2("fill rectangle 0x%x --> (%d,%d) %dx%d with color 0x%x, %s\n",
		ru->phys, rk.x, rk.y, rk.w, rk.h, ru->color,
		dcc_format_name(ru->fmt));
	dcc_set_framebuffer(p, ru->phys, ru->fbwidth);
	return dcc_fillrectangle(p, ru->color, &rk);
}

int dcc_rq_setpixel(struct dcc_drvdata *pdata, struct dcc_point_t *p)
{
	uint32_t data[2];
	int err = 0;

	DCC_DBG2("set pixel 0x%x + (%d,%d) with color 0x%x\n",
		p->phys, p->x, p->y, p->color);

	/* set color */
	dcc_transparency(pdata, ALPHA_PLANE, (p->color >> 24) & 0xFF);

	err = dcc_setcolor(pdata, p->color);
	if (err)
		goto exit;

	/* draw pixel */
	/* Note; using display width as framebuffer width */
	dcc_set_framebuffer(pdata, p->phys, p->fbwidth);
	data[0] = p->x;		/* x */
	data[1] = p->y;		/* y */
	gra_sendcmd(pdata, GRACMD_SETPIXEL, 0, data, 2);

exit:
	return err;
}

int dcc_rq_drawline(struct dcc_drvdata *p, struct dcc_rect_t *ru)
{
	struct x_rect_t rk;
	RECT_INIT(rk, ru->x, ru->y, ru->w, ru->h, dcc_get_fb_fmt(p), ru->flags);

	DCC_DBG2("line 0x%x (%d,%d) --> (%d,%d) %dx%d with color 0x%x\n",
		ru->phys, ru->x, ru->y, rk.x, rk.y, rk.w, rk.h, ru->color);

	dcc_set_framebuffer(p, ru->phys, ru->fbwidth);
	return dcc_drawline(p, &rk, ru->color);
}

/**
 * Draw line in relative coordinates from buffer base
 */
int dcc_rq_drawlinerel(struct dcc_drvdata *p, struct dcc_rect_t *ru)
{
	struct x_rect_t rk;
	unsigned int dstbuf = ru->phys;

	RECT_INIT(rk, ru->x, ru->y, ru->w, ru->h, dcc_get_fb_fmt(p), ru->flags);

	DCC_DBG2("line rel 0x%x (%d,%d) --> (%d,%d) %dx%d with color 0x%x\n",
		dstbuf, rk.x, rk.y, X2((&rk)), Y2((&rk)), rk.w, rk.h,
		ru->color);

	dcc_set_framebuffer(p, ru->phys, ru->fbwidth);
	return dcc_drawlinerel(p, &rk, ru->color);
}

/**
 * Blit area from src to dst. Source and Destination
 * are meant to have the same dimensions
 */
#define DCC_RQ_BLIT_DBG \
	"blit  @ 0x%x %dx%d fmt 0x%x -> @ 0x%x (%d,%d) %dx%d fmt 0x%x\n"

#define DCC_RQ_BLITCROP_DBG \
	"blitcrop @ 0x%x %dx%d fmt 0x%x -> @ 0x%x (%d,%d) %dx%d fmt 0x%x\n"

int dcc_rq_blit(struct dcc_drvdata *p, struct dcc_rq_t *rq)
{
	int err = 0;

	/* check constraints */
	DCC_CHECK(rq->sphys != 0, "invalid source adress 0x%08x\n", rq->sphys);
	DCC_CHECK(rq->dphys != 0, "invalid destination adress 0x%08x\n",
		  rq->sphys);
	DCC_CHECK(rq->angle == 0, "angle should be 0 but is 0x%x\n", rq->angle);

	dcc_flag_to_transparency(p, rq->flags, rq->alpha, 0);
	if (rq->sw == rq->dw) {
		struct x_rect_t rk;
		/* TODO check pixfmt */
		/* TODO handle sx and sy(force null or compute) */
		RECT_INIT(rk, rq->dx, rq->dy, rq->dw, rq->dh, rq->dfmt,
				rq->flags);

		DCC_DBG2(DCC_RQ_BLIT_DBG,
		     rq->sphys, rq->sw, rq->sh, rq->sfmt, rq->dphys, rq->dx,
		     rq->dy, rq->dw, rq->dh, rq->dfmt);

		dcc_set_framebuffer(p, rq->dphys, rq->fbwidth);
		dcc_bitblit(p, &rk, rq->sphys);

		/**
		 * Crop Blit
		 */
	} else {
		/* TODO check pixfmt */
		/* TODO handle sx and sy(force null or compute) */
		struct x_rect_t rk;
		/* initialize destiantion rectangle */
		RECT_INIT(rk, rq->dx, rq->dy, rq->dw, rq->dh, rq->dfmt,
				rq->flags);

		DCC_DBG2(DCC_RQ_BLITCROP_DBG,
		     rq->sphys, rq->sw, rq->sh, rq->sfmt, rq->dphys, rq->dx,
		     rq->dy, rq->dw, rq->dh, rq->dfmt);

		dcc_set_framebuffer(p, rq->dphys, rq->fbwidth);
		dcc_blitimage(p, rq->sphys, rq->sw, &rk);
	}

	return err;
}

#define DCC_RQ_RESIZE_DBG \
"resiz %dx%d[%dx%d(%d,%d)] @0x%x %s a:0x%x f:0x%x -> @0x%x [%dx%d(%d,%d)] %s\n"

#define DCC_CANNOT_DOWNSCALE \
"can't downscale mode than 16 times %dx%d[%dx%d(%d,%d)]-> %dx%d\n"

#define DCC_CANNOT_UPSCALE \
"can't upscale more than 4 times %dx%d[%dx%d(%d,%d)]-> %dx%d\n"

int dcc_rq_resize(struct dcc_drvdata *p, struct dcc_rq_resize_t *rq)
{
	struct x_area_t sarea;
	struct x_rect_t drect, swin;
	int err = 0;

	if (rq->angle != 0) {
		dcc_err("angle should be 0 but is 0x%x\n", rq->angle);
		return -1;
	}

	/* DrawImage(conversion+rescaling) */
	AREA_INIT(sarea, rq->sw, rq->sh, rq->sfmt);
	RECT_INIT(swin, rq->wx, rq->wy, rq->ww, rq->wh, rq->sfmt, rq->flags);
	RECT_INIT(drect, rq->dx, rq->dy, rq->dw, rq->dh, rq->dfmt, rq->flags);

	DCC_DBG2(DCC_RQ_RESIZE_DBG,
	     rq->sw, rq->sh, rq->ww, rq->wh, rq->wx, rq->wy, rq->sphys,
	     dcc_format_name(rq->sfmt), rq->alpha, rq->flags, rq->dphys, rq->dw,
	     rq->dh, rq->dx, rq->dy, dcc_format_name(rq->dfmt));

/*      if((rq->flags & DCC_BLEND_ALPHA_PLANE)&&(rq->alpha==0)){
 *              DCC_DBG2("request null global plane alpha 0x%x\n", rq->alpha);
 *              return 0;
 *      } */

	if (((rq->ww / 16) > rq->dw) || ((rq->wh / 16) > rq->dh)) {
		dcc_err(DCC_CANNOT_DOWNSCALE,
		     rq->sw, rq->sh, rq->ww, rq->wh, rq->wx, rq->wy, rq->dw,
		     rq->dh);
		return -1;
	}
	if (((rq->ww * 4) < rq->dw) || ((rq->wh * 4) < rq->dh)) {
		dcc_err(DCC_CANNOT_UPSCALE,
		     rq->sw, rq->sh, rq->ww, rq->wh, rq->wx, rq->wy, rq->dw,
		     rq->dh);
		return -1;
	}

	dcc_set_framebuffer(p, rq->dphys, rq->fbwidth);

	if (rq->flags & DCC_FLAG_DRAW2DISP)
		dcc_setdrawimagedest(p, DCC_DEST_DRAW2DISP);
	else
		dcc_setdrawimagedest(p, DCC_DEST_DRAW2MEM);

	dcc_flag_to_transparency(p, rq->flags, rq->alpha, rq->colorkey);

	err =
	    dcc_drawimage(p, &sarea, &swin, &drect, rq->sphys, rq->flags,
			  rq->alpha);

	return err;
}

int dcc_rq_scrollmove(struct dcc_drvdata *p, struct dcc_rq_t *rq)
{
	int err = 0;
	struct x_rect_t sr, dr;

	DCC_CHECK(rq->sphys == rq->dphys,
		  "source(0x%08x) and destination(0x%08x) "
		  "frame buffer must be the same\n",
		  rq->sphys, rq->dphys);
	/* init source rectangle */
	RECT_INIT(sr, rq->sx, rq->sy, rq->sw, rq->sh, rq->sfmt, rq->flags);

	/* init destination rectangle */
	RECT_INIT(dr, rq->dx, rq->dy, rq->dw, rq->dh, rq->dfmt, rq->flags);

	DCC_DBG2("scrollmove 0x%x (%d,%d) %dx%d --> 0x%x (%d,%d) %dx%d",
		rq->sphys, sr.x, sr.y, sr.w, sr.h,
		rq->dphys, dr.x, dr.y, dr.w, dr.h);

	dcc_set_framebuffer(p, rq->dphys, rq->fbwidth);
	err = dcc_scrollmove(p, &sr, &dr);

	return err;
};

int dcc_rq_rotate(struct dcc_drvdata *p, struct dcc_rq_t *rq)
{
	int err = 0;
	struct x_area_t sarea;

	/* TODO check src and dst (x,y)=(0,0) */
	DCC_DBG2("rotate 0x%x %dx%d @ 0x%x %s -> %dx%d @ 0x%x %s",
		rq->angle, rq->sw, rq->sh, rq->sphys, dcc_format_name(rq->sfmt),
		rq->dw, rq->dh, rq->dphys, dcc_format_name(rq->dfmt));

	AREA_INIT(sarea, rq->sw, rq->sh, rq->sfmt);
	dcc_set_framebuffer(p, rq->dphys, rq->fbwidth);
	err = dcc_rotateimage(p, &sarea, rq->sphys, rq->dphys, rq->angle);

	return err;
};
