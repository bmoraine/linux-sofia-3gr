/*
 * Component: XGOLD6321 DSP Audio Driver
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

#ifndef _DSP_AUDIO_INTERNAL_H
#define _DSP_AUDIO_INTERNAL_H
#include <dsp_audio_platform.h>
/*----------------------------------------------------------------------*/
/*			Defines related to DSP commands			*/
/*----------------------------------------------------------------------*/
/** \brief The length of the Frame and Sample Based Audio DSP
	command-id in 16-bit words */
#ifndef CMD_ID_DEFINED
#define DSP_AUDIO_CMD_ID_LEN        1
#endif

/**
  @brief Initialization of the dsp audio sub-module

  @param list of dsp's in the system

  @return enum dsp_err_code error

 */
enum dsp_err_code dsp_audio_init(struct list_head *list);
#endif /* _DSP_AUDIO_INTERNAL_H */
