/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
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
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __XGOLD_JACK_H__
#define __XGOLD_JACK_H__

#include <linux/input.h>
#include <sound/jack.h>
#include <linux/platform_device.h>


struct xgold_jack {
	struct snd_soc_jack *hs_jack;
	struct iio_channel *iio_client;
	unsigned int jack_irq;
	unsigned int button_irq;
	void __iomem *mmio_base;
	int buttons_enabled;
};

struct xgold_jack *of_xgold_jack_probe(struct platform_device *,
		struct device_node *, struct snd_soc_jack *);
void xgold_jack_remove(struct xgold_jack *);
int xgold_jack_setup(struct snd_soc_codec *, struct snd_soc_jack *);

#endif
