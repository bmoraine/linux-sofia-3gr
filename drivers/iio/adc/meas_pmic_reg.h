/*
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#ifndef __MEAS_PMIC_REG_H__
#define __MEAS_PMIC_REG_H__


/*****************************************************************************/

/*
	For register offset, address, bit position of measurement type,
	physical measurement channel number are referred from
	SoFIA_LTE_PMIC_V1.0_PMUHW_PS_HWA_internal.pdf.
*/

#define PMIC_SLAVE_DEVICE_1	(0x4E)
#define PMIC_SLAVE_DEVICE_2	(0x4F)
#define PMIC_SLAVE_DEVICE_3	(0x5E)
#define PMIC_SLAVE_DEVICE_4	(0x5F)

/* ADC IRQ Bit position inside IRQLVL1_REG */
#define IRQLVL1_REG_ADC_BIT_POS	(4)

/* OTP_PD_ACT Bit position inside NVM_STAT0 register */
#define NVM_STAT0_REG_OTP_PD_ACT_BIT_POS (1)
#define NVM_STAT0_REG_OTP_PD_ACT_VAL (1<<NVM_STAT0_REG_OTP_PD_ACT_BIT_POS)

/* Major number for B0 PMIC version */
#define PMIC_CHIPID_ID0_B0_MAJREV (1)

/* Bit position and length for PMIC CHIPID major revision */
#define PMIC_CHIPID_ID0_MAJREV_BIT_POS (3)
#define PMIC_CHIPID_ID0_MAJREV_BIT_LEN (3)

/* Bit position and length for PMIC CHIPID minor revision */
#define PMIC_CHIPID_ID0_MINREV_BIT_POS (0)
#define PMIC_CHIPID_ID0_MINREV_BIT_LEN (3)

#define CHIPID_ID0_REG_OFFSET	(0x00)
#define CHIPID_ID0_REG_ADDR \
		((PMIC_SLAVE_DEVICE_1 << 24) | (CHIPID_ID0_REG_OFFSET))

#define IRQLVL1_REG_OFFSET	(0x02)
#define IRQLVL1_REG_ADDR \
		((PMIC_SLAVE_DEVICE_1 << 24) | (IRQLVL1_REG_OFFSET))

#define MIRQLVL1_REG_OFFSET	(0x0E)
#define MIRQLVL1_REG_ADDR \
		((PMIC_SLAVE_DEVICE_1 << 24) | (MIRQLVL1_REG_OFFSET))

#define MADCIRQ_REG_OFFSET	(0x16)
#define MADCIRQ_REG_ADDR \
		((PMIC_SLAVE_DEVICE_1 << 24) | (MADCIRQ_REG_OFFSET))

#define GPADCIRQ_REG_OFFSET	(0x08)
#define GPADCIRQ_REG_ADDR \
		((PMIC_SLAVE_DEVICE_1 << 24) | (GPADCIRQ_REG_OFFSET))

#define OTPVERSION_REG_OFFSET	(0x01)
#define OTPVERSION_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (OTPVERSION_REG_OFFSET))

#define GPADCREQ_REG_OFFSET	(0x02)
#define GPADCREQ_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (GPADCREQ_REG_OFFSET))

#define VBATRSLTH_REG_OFFSET	(0x03)
#define VBATRSLTH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (VBATRSLTH_REG_OFFSET))

#define VBATRSLT_REG_OFFSET	(0x04)
#define VBATRSLT_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (VBATRSLT_REG_OFFSET))

#define GPADCCNTL_REG_OFFSET	(0x05)
#define GPADCCNTL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (GPADCCNTL_REG_OFFSET))

#define BATTIDRSLTH_REG_OFFSET	(0x06)
#define BATTIDRSLTH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (BATTIDRSLTH_REG_OFFSET))

#define BATTIDRSLTL_REG_OFFSET	(0x07)
#define BATTIDRSLTL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (BATTIDRSLTL_REG_OFFSET))

#define USBIDRSLTH_REG_OFFSET	(0x08)
#define USBIDRSLTH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (USBIDRSLTH_REG_OFFSET))

#define USBIDRSLTL_REG_OFFSET	(0x09)
#define USBIDRSLTL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (USBIDRSLTL_REG_OFFSET))

#define GPMEASRSLTH_REG_OFFSET	(0x0A)
#define GPMEASRSLTH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (GPMEASRSLTH_REG_OFFSET))

#define GPMEASRSLTL_REG_OFFSET	(0x0B)
#define GPMEASRSLTL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (GPMEASRSLTL_REG_OFFSET))

#define Y0DATAH_REG_OFFSET	(0x0C)
#define Y0DATAH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (Y0DATAH_REG_OFFSET))

#define Y0DATAL_REG_OFFSET	(0x0D)
#define Y0DATAL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (Y0DATAL_REG_OFFSET))

#define Y1DATAH_REG_OFFSET	(0x0E)
#define Y1DATAH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (Y1DATAH_REG_OFFSET))

#define Y1DATAL_REG_OFFSET	(0x0F)
#define Y1DATAL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (Y1DATAL_REG_OFFSET))

#define PEAKREQ_REG_OFFSET	(0x12)
#define PEAKREQ_REG_ADDR\
		((PMIC_SLAVE_DEVICE_2 << 24) | (PEAKREQ_REG_OFFSET))

#define PEAKRSLTH_REG_OFFSET	(0x13)
#define PEAKRSLTH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (PEAKRSLTH_REG_OFFSET))

#define PEAKRSLTL_REG_OFFSET	(0x14)
#define PEAKRSLTL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (PEAKRSLTL_REG_OFFSET))

#define THRMRSLT0H_REG_OFFSET	(0x38)
#define THRMRSLT0H_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT0H_REG_OFFSET))

#define THRMRSLT0L_REG_OFFSET	(0x39)
#define THRMRSLT0L_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT0L_REG_OFFSET))

#define THRMRSLT1H_REG_OFFSET	(0x3A)
#define THRMRSLT1H_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT1H_REG_OFFSET))

#define THRMRSLT1L_REG_OFFSET	(0x3B)
#define THRMRSLT1L_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT1L_REG_OFFSET))

#define THRMRSLT2H_REG_OFFSET	(0x3C)
#define THRMRSLT2H_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT2H_REG_OFFSET))

#define THRMRSLT2L_REG_OFFSET	(0x3D)
#define THRMRSLT2L_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT2L_REG_OFFSET))

#define THRMRSLT3H_REG_OFFSET	(0x3E)
#define THRMRSLT3H_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT3H_REG_OFFSET))

#define THRMRSLT3L_REG_OFFSET	(0x3F)
#define THRMRSLT3L_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT3L_REG_OFFSET))

#define THRMRSLT4H_REG_OFFSET	(0x40)
#define THRMRSLT4H_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT4H_REG_OFFSET))

#define THRMRSLT4L_REG_OFFSET	(0x41)
#define THRMRSLT4L_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT4L_REG_OFFSET))

#define THRMRSLT5H_REG_OFFSET	(0x42)
#define THRMRSLT5H_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT5H_REG_OFFSET))

#define THRMRSLT5L_REG_OFFSET	(0x43)
#define THRMRSLT5L_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (THRMRSLT5L_REG_OFFSET))

#define VBATMAXH_REG_OFFSET	(0xF6)
#define VBATMAXH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (VBATMAXH_REG_OFFSET))

#define VBATMAXL_REG_OFFSET	(0xF7)
#define VBATMAXL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (VBATMAXL_REG_OFFSET))

#define ACDRSLTH_REG_OFFSET	(0xFC)
#define ACDRSLTH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (ACDRSLTH_REG_OFFSET))

#define ACDRSLTL_REG_OFFSET	(0xFD)
#define ACDRSLTL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_2 << 24) | (ACDRSLTL_REG_OFFSET))

#define NVM_MB_ADDRH_OFFSET	(0x3A)
#define NVM_MB_ADDRH_REG_ADDR \
		((PMIC_SLAVE_DEVICE_4 << 24) | (NVM_MB_ADDRH_OFFSET))

#define NVM_MB_ADDRL_OFFSET	(0x3B)
#define NVM_MB_ADDRL_REG_ADDR \
		((PMIC_SLAVE_DEVICE_4 << 24) | (NVM_MB_ADDRL_OFFSET))

#define NVM_MB_DATA_OFFSET	(0x3C)
#define NVM_MB_DATA_REG_ADDR \
		((PMIC_SLAVE_DEVICE_4 << 24) | (NVM_MB_DATA_OFFSET))

#define NVM_STAT0_OFFSET	(0x67)
#define NVM_STAT0_REG_ADDR \
		((PMIC_SLAVE_DEVICE_4 << 24) | (NVM_STAT0_OFFSET))


/* Bit position of meausrement type inside
	GPADC interrupt register */
enum intel_adcirg_status_reg_bit {
	ADC_STA_USBID     = 0,
	ADC_STA_PEAK      = 1,
	ADC_STA_BATTEMP   = 2,
	ADC_STA_SYSTEMP   = 3,
	ADC_STA_BATID     = 4,
	ADC_STA_VBAT      = 5,
	ADC_STA_GPMEAS    = 6,
	ADC_STA_CCTICK    = 7,
	ADC_STA_IRQ_MAX   = 8
};

/* Bit position of meausrement type inside
	GPADC conversion Request Register */
enum intel_gpadc_req_reg_bit {
	ADC_REQ_BUSY      = 0,
	ADC_REQ_USBID     = 1,
	ADC_REQ_BATTEMP   = 2,
	ADC_REQ_SYSTEMP   = 3,
	ADC_REQ_BATID     = 4,
	ADC_REQ_VBAT      = 5,
	ADC_REQ_GPMEAS    = 6,
	ADC_REQ_PEAK      = 7,
	ADC_REQ_IRQ_MAX   = 8
};

/* Re-definition of the ADC physical channels to abstract HW version specific
information */
enum intel_adc_phy_channel {
	ADC_PHY_VBAT      = 0,
	ADC_PHY_BATID     = 1,
	ADC_PHY_SYS0TEMP  = 2,
	ADC_PHY_SYS1TEMP  = 3,
	ADC_PHY_SYS2TEMP  = 4,
	ADC_PHY_PMICTEMP  = 5,
	ADC_PHY_BAT0TEMP  = 6,
	ADC_PHY_BAT1TEMP  = 7,
	ADC_PHY_USBID     = 8,
	ADC_PHY_PEAK      = 9,
	ADC_PHY_AGND      = 10,
	ADC_PHY_VREF      = 11,
	ADC_PHY_ACCID     = 12,
	ADC_PHY_OCV       = 13,
	ADC_PHY_MAX       = 14
	};

#endif /* __MEAS_PMIC_REG_H__ */

