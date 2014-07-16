/*
 * Component: XGOLD PCM header file
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */

#ifndef __XGOLD_PCM_H__
#define __XGOLD_PCM_H__

enum xgold_pcm_stream_type {
	STREAM_PLAY = 0,
	/*TODO PLAY2*/
	STREAM_REC,
	HW_PROBE_A,
	HW_PROBE_B,
	NR_STREAM
};

#endif /* __XGOLD_PCM_H__ */
