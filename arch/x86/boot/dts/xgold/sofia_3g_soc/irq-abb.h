/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _ABB_IRQ_H
#define _ABB_IRQ_H

/* ABB nodes from ABB domain point of view */
#define ABB_RTC_INT 0
#define ABB_PMU_INT0 1
#define ABB_PMU_INT1 2
#define ABB_SCU_INT0 3
#define ABB_SCU_INT1 4
#define ABB_BTIF_REQ_IRQ 5
#define ABB_BTIF_MULT_SRC_SRQ 6
#define ABB_BTIF_ERR_SRQ 7
#define ABB_WLAN_INT 8
#define ABB_MRDY_INT 9
#define ABB_GNSS_WUP 10
#define ABB_GNSS_NOTIFICATION 11
#define ABB_GNSS_ERR 12
#define ABB_SCU_INT2 13
#define ABB_SCU_INT3 14
#define ABB_SCU_INT4 15
#define ABB_AFE_LS_ALERT 16
#define ABB_ACI_INT 17
#define ABB_USIF_ALL_INT 18
#define ABB_I2C_RAWIRQ 19
#define ABB_IDIABB_ERR_SRQ0 20
#define ABB_IDIABB_TX_REQ_IRQ0 21
#define ABB_IDIABB_RX_REQ_IRQ0 22
#define ABB_IDIABB_ERR_SRQ1 23
#define ABB_IDIABB_TX_REQ_IRQ1 24
#define ABB_IDIABB_RX_REQ_IRQ1 25
#define ABB_IDIABB_MULT_SRC_SRQ 26
#define ABB_FMR_INT 27
#define ABB_FMRX_EV_ONESHOT 28
#define ABB_FMRX_EV_TUNE 29
#define ABB_FMTX_EV_CALIB 30
#define ABB_FMTX_EV_TUNE 31

#endif /* _ABB_IRQ_H */
