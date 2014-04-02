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

#ifndef _DSP_COMMUNICATION_INTERNAL_H
#define _DSP_COMMUNICATION_INTERNAL_H

/*
 * Use 400 us as timeout value for a DSP command.
 * Commands should be acknowledged within 100 us
 */
#define DSP_AUDIO_CMD_TIMEOUT       400

void dsp_audio_communication_init(void);
#endif /* _DSP_COMMUNICATION_INTERNAL_H */
