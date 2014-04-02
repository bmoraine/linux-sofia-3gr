/*
 * Component: XGOLD BT SCO driver header
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

#ifndef _AGOLD_BT_SCO_STREAMING_H
#define _AGOLD_BT_SCO_STREAMING_H

#ifdef CONFIG_SND_SOC_AGOLD_BT_SCO_STREAMING
int xgold_bt_sco_soc_init(struct snd_soc_platform *platform);
#endif

#endif /* _AGOLD_BT_SCO_STREAMING_H */
