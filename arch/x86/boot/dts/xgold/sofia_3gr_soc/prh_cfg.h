/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define SF_R_ES_1_0
/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_GENERIC_MODE_START (0)
/**< Disable generic peripheral power resources and enable power saving state */
#define PRH_PER_GENERIC_DISABLE (PRH_PER_GENERIC_MODE_START + 1)
/**< Disable generic peripheral power saving state and enable power resources */
#define PRH_PER_GENERIC_ENABLE (PRH_PER_GENERIC_DISABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_GENERIC_MODE_END (PRH_PER_GENERIC_ENABLE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_ARM11_CLK_MODE_START (PRH_PER_GENERIC_MODE_END + 1)
/**< Enable PS_CPU_CLK low performance use cases (208MHz) */
#define PRH_PER_ARM11_CLK_ENABLE_LOW_PERF (PRH_PER_ARM11_CLK_MODE_START + 1)
/**< Enable PS_CPU_CLK med performance use cases (312MHz)*/
#define PRH_PER_ARM11_CLK_ENABLE_MED_PERF (PRH_PER_ARM11_CLK_ENABLE_LOW_PERF + 1)
#ifdef SOFIA_3GR_GARNET_4_LEVEL_CPUFREQ
/**< Enable PS_CPU_CLK med-high performance use cases (520MHz)*/
#define PRH_PER_ARM11_CLK_ENABLE_MED_HIGH_PERF (PRH_PER_ARM11_CLK_ENABLE_MED_PERF + 1)
/**< Enable PS_CPU_CLK high performance use cases (624MHz) */
#define PRH_PER_ARM11_CLK_ENABLE_HIGH_PERF (PRH_PER_ARM11_CLK_ENABLE_MED_HIGH_PERF + 1)
#else
/**< Enable PS_CPU_CLK high performance use cases (520MHz)*/
#define PRH_PER_ARM11_CLK_ENABLE_HIGH_PERF (PRH_PER_ARM11_CLK_ENABLE_MED_PERF + 1)
#endif
/**< Enable PS_CPU_CLK high performance use cases (624MHz) */
#define PRH_PER_ARM11_CLK_ENABLE_ULTRA_HIGH_PERF (PRH_PER_ARM11_CLK_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_ARM11_CLK_MODE_END (PRH_PER_ARM11_CLK_ENABLE_ULTRA_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_MACPHY_MODE_START (PRH_PER_ARM11_CLK_MODE_END+1)
/**< Disable MACPHY power resources and enable MACPHY power saving state */
#define PRH_PER_MACPHY_DISABLE (PRH_PER_MACPHY_MODE_START + 1)
/**< Disable MACPHY power saving state and enable MACPHY power resources without
EBU performance support */
#define PRH_PER_MACPHY_ENABLE_LOW_PERF (PRH_PER_MACPHY_DISABLE + 1)
/**< Disable MACPHY power saving state and enable MACPHY power resources without
EBU performance support */
#define PRH_PER_MACPHY_ENABLE_HIGH_PERF (PRH_PER_MACPHY_ENABLE_LOW_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MACPHY_MODE_END (PRH_PER_MACPHY_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_DMA_MODE_START (PRH_PER_MACPHY_MODE_END+1)
/**< Disable DMA power resources and enable DMA power saving state */
#define PRH_PER_DMA_DISABLE (PRH_PER_DMA_MODE_START + 1)
/**< Disable DMA power saving state and enable DMA power resources without EBU
performance support */
#define PRH_PER_DMA_ENABLE_LOW_PERF (PRH_PER_DMA_DISABLE + 1)
/**< Disable DMA power saving state and enable DMA power resources without EBU
performance support */
#define PRH_PER_DMA_ENABLE_MED_PERF (PRH_PER_DMA_ENABLE_LOW_PERF + 1)
/**< Disable DMA power saving state and enable DMA power resources without EBU
performance support */
#define PRH_PER_DMA_ENABLE_HIGH_PERF (PRH_PER_DMA_ENABLE_MED_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_DMA_MODE_END (PRH_PER_DMA_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_DSP_MODE_START (PRH_PER_DMA_MODE_END+1)
/**< Disable DSP power resources */
#define PRH_PER_DSP_DISABLE (PRH_PER_DSP_MODE_START + 1)
/**< Enable DSP power resources */
#define PRH_PER_DSP_ENABLE (PRH_PER_DSP_DISABLE + 1)
/**< Suspend DSP power resources */
#define PRH_PER_DSP_SUSPEND (PRH_PER_DSP_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_DSP_MODE_END (PRH_PER_DSP_SUSPEND + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_I2C_MODE_START (PRH_PER_DSP_MODE_END+1)
/**< Disable I2C power resources and enable I2C power saving state */
#define PRH_PER_I2C_DISABLE (PRH_PER_I2C_MODE_START + 1)
/**< Disable I2C power saving state and enable I2C power resources */
#define PRH_PER_I2C_ENABLE (PRH_PER_I2C_DISABLE + 1)
/**< Enable power save I2C */
#define PRH_PER_I2C_ENABLE_PSV (PRH_PER_I2C_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_I2C_MODE_END (PRH_PER_I2C_ENABLE_PSV + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_MMCSD_VOLT_MODE_START (PRH_PER_I2C_MODE_END+1)
/**< Disable MMCSD voltage */
#define PRH_PER_MMCSD_VOLT_DISABLE (PRH_PER_MMCSD_VOLT_MODE_START + 1)
/**< Enable MMCSD Low voltage */
#define PRH_PER_MMCSD_VOLT_ENABLE_LOW (PRH_PER_MMCSD_VOLT_DISABLE + 1)
/**< Enable MMCSD Mid voltage */
#define PRH_PER_MMCSD_VOLT_ENABLE_MID (PRH_PER_MMCSD_VOLT_ENABLE_LOW + 1)
/**< Enable MMCSD High voltage */
#define PRH_PER_MMCSD_VOLT_ENABLE_HIGH (PRH_PER_MMCSD_VOLT_ENABLE_MID + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MMCSD_VOLT_MODE_END (PRH_PER_MMCSD_VOLT_ENABLE_HIGH + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_MMCSD_CLK_MODE_START (PRH_PER_MMCSD_VOLT_MODE_END+1)
/**< Disable MMCSD clock */
#define PRH_PER_MMCSD_CLK_DISABLE (PRH_PER_MMCSD_CLK_MODE_START + 1)
/**< Enable MMCSD Low clock (48M) */
#define PRH_PER_MMCSD_CLK_ENABLE_LOW (PRH_PER_MMCSD_CLK_DISABLE + 1)
/**< Enable MMCSD Medium clock (52M) */
#define PRH_PER_MMCSD_CLK_ENABLE_MID (PRH_PER_MMCSD_CLK_ENABLE_LOW + 1)
/**< Enable MMCSD High clock (96M) */
#define PRH_PER_MMCSD_CLK_ENABLE_HIGH (PRH_PER_MMCSD_CLK_ENABLE_MID + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MMCSD_CLK_MODE_END (PRH_PER_MMCSD_CLK_ENABLE_HIGH + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_NAND_MODE_START (PRH_PER_MMCSD_CLK_MODE_END+1)
/**< Disable NANDCTRL power resources and enable NANDCTRL power saving state */
#define PRH_PER_NAND_DISABLE (PRH_PER_NAND_MODE_START + 1)
/**< Enable NANDCTRL power saving state and enable NANDCTRL power resources
(104M)*/
#define PRH_PER_NAND_ENABLE_LOW_PERF (PRH_PER_NAND_DISABLE + 1)
/**< Enable NANDCTRL power saving state and enable NANDCTRL power resources
(104M) */
#define PRH_PER_NAND_ENABLE_MID_PERF (PRH_PER_NAND_ENABLE_LOW_PERF + 1)
/**< Enable NANDCTRL power saving state and enable NANDCTRL power resources
(208M) */
#define PRH_PER_NAND_ENABLE_HIGH_PERF (PRH_PER_NAND_ENABLE_MID_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_NAND_MODE_END (PRH_PER_NAND_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_USB_MODE_START (PRH_PER_NAND_MODE_END+1)
/**< Disable USB HS power resources and enable USB HS power saving state */
#define PRH_PER_USB_DISABLE (PRH_PER_USB_MODE_START + 1)
/**< Suspend USB HS power resources and enable USB HSIC power saving state */
#define PRH_PER_USB_SUSPEND (PRH_PER_USB_DISABLE + 1)
/**< Suspend USB HS power resources but disable power saving state */
#define PRH_PER_USB_SUSPEND_NO_PSV (PRH_PER_USB_SUSPEND + 1)
/**< Enable USB HS power resources */
#define PRH_PER_USB_ENABLE (PRH_PER_USB_SUSPEND_NO_PSV + 1)
/**< Disable USB HS power saving state and disable USB HS isolations */
#define PRH_PER_USB_ENABLE_ISO (PRH_PER_USB_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_USB_MODE_END (PRH_PER_USB_ENABLE_ISO + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_USIF_MODE_START (PRH_PER_USB_MODE_END+1)
/**< Disable USIF power resources and enable USIF power saving state */
#define PRH_PER_USIF_DISABLE (PRH_PER_USIF_MODE_START + 1)
/**< Disable USIF power saving state and enable USIF power resources */
#define PRH_PER_USIF_ENABLE (PRH_PER_USIF_DISABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_USIF_MODE_END (PRH_PER_USIF_ENABLE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_UICC_VCC_MODE_START (PRH_PER_USIF_MODE_END+1)
/**< Vcc: 0V */
#define PRH_PER_UICC_VCC_OFF (PRH_PER_UICC_VCC_MODE_START + 1)
/**< Vcc: 1.8V */
#define PRH_PER_UICC_VCC_CLASS_C (PRH_PER_UICC_VCC_OFF + 1)
/**< Vcc: 3V */
#define PRH_PER_UICC_VCC_CLASS_B (PRH_PER_UICC_VCC_CLASS_C + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_UICC_VCC_MODE_END (PRH_PER_UICC_VCC_CLASS_B + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_UICC_CLK_MODE_START (PRH_PER_UICC_VCC_MODE_END+1)
/**< Kernel clk: Off;   Bus clk: Off */
#define PRH_PER_UICC_CLK_DISABLE (PRH_PER_UICC_CLK_MODE_START + 1)
/**< Kernel clk: 13MHz/4; Bus clk: Off */
#define PRH_PER_UICC_CLK_SUSPEND (PRH_PER_UICC_CLK_DISABLE + 1)
/**< Kernel clk: 13MHz/4; Bus clk: 26MHz */
#define PRH_PER_UICC_CLK_ENABLE_LOW_PERF (PRH_PER_UICC_CLK_SUSPEND + 1)
/**< Kernel clk: 15.6MHz/4; Bus clk: 26MHz */
#define PRH_PER_UICC_CLK_ENABLE_MED_PERF (PRH_PER_UICC_CLK_ENABLE_LOW_PERF + 1)
/**< Kernel clk: 19.5/4MHz; Bus clk: 26MHz */
#define PRH_PER_UICC_CLK_ENABLE_HIGH_PERF (PRH_PER_UICC_CLK_ENABLE_MED_PERF + 1)
/**< Kernel clk: On with Power Save Disabled */
#define PRH_PER_UICC_CLK_PSV_DISABLED (PRH_PER_UICC_CLK_ENABLE_HIGH_PERF + 1)
/**< Kernel clk: On with Power Save Enabled */
#define PRH_PER_UICC_CLK_PSV_ENABLED (PRH_PER_UICC_CLK_PSV_DISABLED + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_UICC_CLK_MODE_END (PRH_PER_UICC_CLK_PSV_ENABLED + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_CAPCOM_MODE_START (PRH_PER_UICC_CLK_MODE_END+1)
/**< Disable CAPCOM */
#define PRH_PER_CAPCOM_DISABLE (PRH_PER_CAPCOM_MODE_START + 1)
/**< Enable CAPCOM */
#define PRH_PER_CAPCOM_ENABLE (PRH_PER_CAPCOM_DISABLE + 1)
/**< Enable power save CAPCOM */
#define PRH_PER_CAPCOM_ENABLE_PSV (PRH_PER_CAPCOM_ENABLE + 1)
/**< Disable power save CAPCOM */
#define PRH_PER_CAPCOM_DISABLE_PSV (PRH_PER_CAPCOM_ENABLE_PSV + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_CAPCOM_MODE_END (PRH_PER_CAPCOM_DISABLE_PSV + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_GPS_MODE_START (PRH_PER_CAPCOM_MODE_END + 1)
/**< Disable GPS */
#define PRH_PER_GPS_DISABLE (PRH_PER_GPS_MODE_START + 1)
/**< Standby GPS */
#define PRH_PER_GPS_STANDBY (PRH_PER_GPS_DISABLE + 1)
/**< Enable GPS */
#define PRH_PER_GPS_ENABLE (PRH_PER_GPS_STANDBY + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_GPS_MODE_END (PRH_PER_GPS_ENABLE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_TRACE_MODE_START (PRH_PER_GPS_MODE_END + 1)
/**< Disable TRACE */
#define PRH_PER_TRACE_DISABLE (PRH_PER_TRACE_MODE_START + 1)
/**< Enable TRACE */
#define PRH_PER_TRACE_ENABLE (PRH_PER_TRACE_DISABLE + 1)
/**< Enable TRACE low performance use cases */
#define PRH_PER_TRACE_ENABLE_LOW_PERF (PRH_PER_TRACE_ENABLE + 1)
/**< Enable TRACE low performance use cases */
#define PRH_PER_TRACE_ENABLE_MED_PERF (PRH_PER_TRACE_ENABLE_LOW_PERF + 1)
/**< Enable TRACE high performance use cases */
#define PRH_PER_TRACE_ENABLE_HIGH_PERF (PRH_PER_TRACE_ENABLE_MED_PERF + 1)
/**< Enable Low Power Trace (OCT use) */
#define PRH_PER_TRACE_ENABLE_LPT (PRH_PER_TRACE_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_TRACE_MODE_END (PRH_PER_TRACE_ENABLE_LPT + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_ABB_AFE_MODE_START (PRH_PER_TRACE_MODE_END + 1)
/**< Disable ABB_AFE */
#define PRH_PER_ABB_AFE_DISABLE (PRH_PER_ABB_AFE_MODE_START + 1)
/**< Enable ABB_AFE Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_HEADSET (PRH_PER_ABB_AFE_DISABLE + 1)
/**< Enable ABB_AFE NO Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_NO_HEADSET (PRH_PER_ABB_AFE_ENABLE_HEADSET + 1)
/**< Enable Sleep retention for ABB_AFE Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET (PRH_PER_ABB_AFE_ENABLE_NO_HEADSET + 1)
/**< Enable Sleep retention for ABB_AFE NO Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_NO_HEADSET (PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET + 1)
/**< Enable ABB_AFE Headset mode for FM changes with chp =10M */
#define PRH_PER_ABB_AFE_ENABLE_HEADSET_10M (PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_NO_HEADSET + 1)
/**< Enable Sleep retention for ABB_AFE with chp=10M Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET_10M (PRH_PER_ABB_AFE_ENABLE_HEADSET_10M + 1)
/**< Enable ABB_AFE Headset mode for FM changes with chp =12M */
#define PRH_PER_ABB_AFE_ENABLE_HEADSET_12M (PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET_10M + 1)
/**< Enable Sleep retention for ABB_AFE with chp=12M Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET_12M (PRH_PER_ABB_AFE_ENABLE_HEADSET_12M + 1)
/**< Enable ABB_AFE Headset mode for FM changes with chp =12M */
#define PRH_PER_ABB_AFE_ENABLE_HEADSET_9_6M (PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET_12M + 1)
/**< Enable Sleep retention for ABB_AFE with chp=12M Headset mode */
#define PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET_9_6M (PRH_PER_ABB_AFE_ENABLE_HEADSET_9_6M + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_ABB_AFE_MODE_END (PRH_PER_ABB_AFE_ENABLE_SLEEP_RETENTION_HEADSET_9_6M + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_ABB_FMR_MODE_START (PRH_PER_ABB_AFE_MODE_END + 1)
/**< Disable ABB_FMR */
#define PRH_PER_ABB_FMR_DISABLE (PRH_PER_ABB_FMR_MODE_START + 1)
/**< Enable ABB_FMR */
#define PRH_PER_ABB_FMR_ENABLE (PRH_PER_ABB_FMR_DISABLE + 1)
/**< Enable ABB_FMR internal clock */
#define PRH_PER_ABB_FMR_ENABLE_INT_CLK (PRH_PER_ABB_FMR_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_ABB_FMR_MODE_END (PRH_PER_ABB_FMR_ENABLE_INT_CLK + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_IDI_MODE_START (PRH_PER_ABB_FMR_MODE_END + 1)
/**< Disable IDI */
#define PRH_PER_IDI_DISABLE (PRH_PER_IDI_MODE_START + 1)
/** Low 104 Mhz*/
#define PRH_PER_IDI_ENABLE_LOW (PRH_PER_IDI_DISABLE + 1)
/**< Disable IDI power saving state and enable IDI power resources without EBU
performance support (138.7Mhz) */
#define PRH_PER_IDI_ENABLE_FREQ_1 (PRH_PER_IDI_ENABLE_LOW + 1)
/**< Disable IDI power saving state and enable IDI power resources without EBU
performance support (178.3MHz) */
#define PRH_PER_IDI_ENABLE_FREQ_2 (PRH_PER_IDI_ENABLE_FREQ_1 + 1)
/**< Enable IDI to main freq (208MHz)*/
#define PRH_PER_IDI_ENABLE_HIGH (PRH_PER_IDI_ENABLE_FREQ_2 + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_IDI_MODE_END (PRH_PER_IDI_ENABLE_HIGH + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_ABB_IDI_FM_MODE_START (PRH_PER_IDI_MODE_END + 1)
/**< Disable IDI */
#define PRH_PER_ABB_IDI_FM_DISABLE (PRH_PER_ABB_IDI_FM_MODE_START + 1)
/**< Enable IDI to main freq (104MHz)*/
#define PRH_PER_ABB_IDI_FM_ENABLE_LOW_PERF (PRH_PER_ABB_IDI_FM_DISABLE + 1)
/**< Enable IDI alternate freq for FM (138.7mhz)*/
#define PRH_PER_ABB_IDI_FM_ENABLE_FREQ_1 (PRH_PER_ABB_IDI_FM_ENABLE_LOW_PERF + 1)
/**< Enable IDI to main freq (178.3MHz)*/
#define PRH_PER_ABB_IDI_FM_ENABLE_FREQ_2 (PRH_PER_ABB_IDI_FM_ENABLE_FREQ_1 + 1)
/**< Enable IDI to main freq (208MHz)*/
#define PRH_PER_ABB_IDI_FM_ENABLE_HIGH_PERF (PRH_PER_ABB_IDI_FM_ENABLE_FREQ_2 + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_ABB_IDI_FM_MODE_END (PRH_PER_ABB_IDI_FM_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_DCC_MODE_START (PRH_PER_ABB_IDI_FM_MODE_END+1)
/**< Disable DCC clock */
#define PRH_PER_DCC_DISABLE (PRH_PER_DCC_MODE_START + 1)
/**< Enable DCC Low clock (52M) */
#define PRH_PER_DCC_ENABLE_LOW_PERF (PRH_PER_DCC_DISABLE + 1)
/**< Enable DCC Medium clock (78M) */
#define PRH_PER_DCC_ENABLE_MID_PERF (PRH_PER_DCC_ENABLE_LOW_PERF + 1)
/**< Enable DCC High clock (104M) */
#define PRH_PER_DCC_ENABLE_HIGH_PERF (PRH_PER_DCC_ENABLE_MID_PERF + 1)
/**< Enable DCC Ultra High clock (156M) */
#define PRH_PER_DCC_ENABLE_ULTRA_HIGH_PERF (PRH_PER_DCC_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_DCC_MODE_END (PRH_PER_DCC_ENABLE_ULTRA_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_CIF_MODE_START (PRH_PER_DCC_MODE_END+1)
/**< Disable CIF clock */
#define PRH_PER_CIF_DISABLE (PRH_PER_CIF_MODE_START + 1)
/**< Enable CIF Low clock (78M)(Med voltage) */
#define PRH_PER_CIF_ENABLE_LOW_PERF (PRH_PER_CIF_DISABLE + 1)
/**< Enable CIF Medium clock (104M)(High voltage)  */
#define PRH_PER_CIF_ENABLE_MID_PERF (PRH_PER_CIF_ENABLE_LOW_PERF + 1)
/**< Enable CIF High clock (156M) (High voltage) */
#define PRH_PER_CIF_ENABLE_HIGH_PERF (PRH_PER_CIF_ENABLE_MID_PERF + 1)
/**< Enable CIF Ultra High clock (312M) (High voltage) */
#define PRH_PER_CIF_ENABLE_ULTRA_HIGH_PERF (PRH_PER_CIF_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_CIF_MODE_END (PRH_PER_CIF_ENABLE_ULTRA_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_GPU_MODE_START (PRH_PER_CIF_MODE_END+1)
/**< Disable GPU clock */
#define PRH_PER_GPU_DISABLE (PRH_PER_GPU_MODE_START + 1)
/**< Enable GPU Low clock (104M) */
#define PRH_PER_GPU_ENABLE_LOW_PERF (PRH_PER_GPU_DISABLE + 1)
/**< Enable GPU Medium clock (156M) */
#define PRH_PER_GPU_ENABLE_MID_PERF (PRH_PER_GPU_ENABLE_LOW_PERF + 1)
/**< Enable GPU High clock (312M) */
#define PRH_PER_GPU_ENABLE_HIGH_PERF (PRH_PER_GPU_ENABLE_MID_PERF + 1)
/**< Enable GPU Ultra High clock (420M) */
#define PRH_PER_GPU_ENABLE_ULTRA_HIGH_PERF (PRH_PER_GPU_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_GPU_MODE_END (PRH_PER_GPU_ENABLE_ULTRA_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_VIDEO_DEC_MODE_START (PRH_PER_GPU_MODE_END+1)
/**< Disable VIDEO clock */
#define PRH_PER_VIDEO_DEC_DISABLE (PRH_PER_VIDEO_DEC_MODE_START + 1)
/**< Enable VIDEO Low clock (104M) */
#define PRH_PER_VIDEO_DEC_ENABLE_LOW_PERF (PRH_PER_VIDEO_DEC_DISABLE + 1)
/**< Enable VIDEO Medium clock (156M) */
#define PRH_PER_VIDEO_DEC_ENABLE_MID_PERF (PRH_PER_VIDEO_DEC_ENABLE_LOW_PERF + 1)
/**< Enable VIDEO High clock (312M) */
#define PRH_PER_VIDEO_DEC_ENABLE_HIGH_PERF (PRH_PER_VIDEO_DEC_ENABLE_MID_PERF + 1)
/**< Enable VIDEO Ultra High clock (420M) */
#define PRH_PER_VIDEO_DEC_ENABLE_ULTRA_HIGH_PERF (PRH_PER_VIDEO_DEC_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_VIDEO_DEC_MODE_END (PRH_PER_VIDEO_DEC_ENABLE_ULTRA_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_VIDEO_ENC_MODE_START (PRH_PER_VIDEO_DEC_MODE_END+1)
/**< Disable VIDEO clock */
#define PRH_PER_VIDEO_ENC_DISABLE (PRH_PER_VIDEO_ENC_MODE_START + 1)
/**< Enable VIDEO Low clock (104M) */
#define PRH_PER_VIDEO_ENC_ENABLE_LOW_PERF (PRH_PER_VIDEO_ENC_DISABLE + 1)
/**< Enable VIDEO Medium clock (156M) */
#define PRH_PER_VIDEO_ENC_ENABLE_MID_PERF (PRH_PER_VIDEO_ENC_ENABLE_LOW_PERF + 1)
/**< Enable VIDEO High clock (312M) */
#define PRH_PER_VIDEO_ENC_ENABLE_HIGH_PERF (PRH_PER_VIDEO_ENC_ENABLE_MID_PERF + 1)
/**< Enable VIDEO Ultra High clock (420M) */
#define PRH_PER_VIDEO_ENC_ENABLE_ULTRA_HIGH_PERF (PRH_PER_VIDEO_ENC_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_VIDEO_ENC_MODE_END (PRH_PER_VIDEO_ENC_ENABLE_ULTRA_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_KPD_MODE_START (PRH_PER_VIDEO_ENC_MODE_END+1)
/**< Disable KPD power resources and enable KPD power saving state */
#define PRH_PER_KPD_DISABLE (PRH_PER_KPD_MODE_START + 1)
/**< Disable KPD power saving state and enable KPD power resources */
#define PRH_PER_KPD_ENABLE (PRH_PER_KPD_DISABLE + 1)
/**< Enable power save KPD */
#define PRH_PER_KPD_ENABLE_PSV (PRH_PER_KPD_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_KPD_MODE_END (PRH_PER_KPD_ENABLE_PSV + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_DCDC_MODE_START (PRH_PER_KPD_MODE_END + 1)
/**< Disable PMU_DCDC performance use cases  */
#define PRH_PER_DCDC_DISABLE (PRH_PER_DCDC_MODE_START + 1)
/**< Enable PMU_DCDC low performance use cases (3.104MHz) */
#define PRH_PER_DCDC_ENABLE_FREQ_1 (PRH_PER_DCDC_DISABLE + 1)
/**< Enable PMU_DCDC med performance use cases (3.152MHz)*/
#define PRH_PER_DCDC_ENABLE_FREQ_2 (PRH_PER_DCDC_ENABLE_FREQ_1 + 1)
/**< Enable PMU_DCDC high performance use cases (3.20MHz)*/
#define PRH_PER_DCDC_ENABLE_FREQ_3 (PRH_PER_DCDC_ENABLE_FREQ_2 + 1)
/**< Enable PMU_DCDC high performance use cases (3.25MHz) */
#define PRH_PER_DCDC_ENABLE_FREQ_4 (PRH_PER_DCDC_ENABLE_FREQ_3 + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_DCDC_MODE_END (PRH_PER_DCDC_ENABLE_FREQ_4 + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_CP_MODE_START (PRH_PER_DCDC_MODE_END + 1)
/**< Disable PMU_CHP  */
#define PRH_PER_CP_DISABLE (PRH_PER_CP_MODE_START + 1)
/**< Enable PMU_CHP low performance use cases (3.25MHz) */
#define PRH_PER_CP_ENABLE_FREQ_1 (PRH_PER_CP_DISABLE + 1)
/**< Enable PMU_CHP med performance use cases (5.2MHz)*/
#define PRH_PER_CP_ENABLE_FREQ_2 (PRH_PER_CP_ENABLE_FREQ_1 + 1)
/**< Enable PMU_CHP high performance use cases (6.5MHz)*/
#define PRH_PER_CP_ENABLE_FREQ_3 (PRH_PER_CP_ENABLE_FREQ_2 + 1)
/**< Enable PMU_CHP high performance use cases (13MHz) */
#define PRH_PER_CP_ENABLE_FREQ_4 (PRH_PER_CP_ENABLE_FREQ_3 + 1)
/**< Enable PMU_CHP default usecase  */
#define PRH_PER_CP_ENABLE (PRH_PER_CP_ENABLE_FREQ_4 + 1)
/**< Enable PMU_CHP high performance use cases (6MHz) */
#define PRH_PER_CP_ENABLE_FREQ_5 (PRH_PER_CP_ENABLE + 1)
/**< Enable PMU_CHP high performance use cases (4.8MHz) */
#define PRH_PER_CP_ENABLE_FREQ_6 (PRH_PER_CP_ENABLE_FREQ_5 + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_CP_MODE_END (PRH_PER_CP_ENABLE_FREQ_6 + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_AUD_SYNC_MODE_START (PRH_PER_CP_MODE_END + 1)
/**< Disable AUD_SYNC  */
#define PRH_PER_AUD_SYNC_DISABLE (PRH_PER_AUD_SYNC_MODE_START + 1)
/**< Enable AUD_SYNC low performance use cases (1KHz) */
#define PRH_PER_AUD_SYNC_ENABLE_LOW_PERF (PRH_PER_AUD_SYNC_DISABLE + 1)
/**< Enable AUD_SYNC med performance use cases (8KHz)*/
#define PRH_PER_AUD_SYNC_ENABLE_MED_PERF (PRH_PER_AUD_SYNC_ENABLE_LOW_PERF + 1)
/**< Enable AUD_SYNC high performance use cases (48KHz)*/
#define PRH_PER_AUD_SYNC_ENABLE_HIGH_PERF (PRH_PER_AUD_SYNC_ENABLE_MED_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_AUD_SYNC_MODE_END (PRH_PER_AUD_SYNC_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_ABB_BT_MODE_START (PRH_PER_AUD_SYNC_MODE_END + 1)
/**< Disable BT (IF +IP)  */
#define PRH_PER_ABB_BT_DISABLE (PRH_PER_ABB_BT_MODE_START + 1)
/**< Enable BT (IF + IP) idle use cases  */
#define PRH_PER_ABB_BT_IDLE (PRH_PER_ABB_BT_DISABLE + 1)
/**< Enable BT (IF + IP) enable with PSV use cases */
#define PRH_PER_ABB_BT_ENABLE_PSV (PRH_PER_ABB_BT_IDLE + 1)
/**< Enable BT (IF + IP) enable with no PSV use cases */
#define PRH_PER_ABB_BT_ENABLE (PRH_PER_ABB_BT_ENABLE_PSV + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_ABB_BT_MODE_END (PRH_PER_ABB_BT_ENABLE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_GNSS_MODE_START (PRH_PER_ABB_BT_MODE_END + 1)
/**< Disable IDI */
#define PRH_PER_GNSS_DISABLE (PRH_PER_GNSS_MODE_START + 1)
/** DCLCK = 83.2M  PCLK = 124.8M */
#define PRH_PER_GNSS_ENABLE (PRH_PER_GNSS_DISABLE + 1)
/**< DCLCK = 96M  PCLK = 124.8M */
#define PRH_PER_GNSS_ENABLE_FREQ_1 (PRH_PER_GNSS_ENABLE + 1)
/**< D0I3 - all resources, except clk_fix2, are up; sleep allowed */
#define PRH_PER_GNSS_IDLE (PRH_PER_GNSS_ENABLE_FREQ_1 + 1)
/**< DCLCK = 83.2M  PCLK = 104M */
#define PRH_PER_GNSS_ENABLE_FREQ_2 (PRH_PER_GNSS_IDLE + 1)
/**< DCLCK = 96M  PCLK = 104M */
#define PRH_PER_GNSS_ENABLE_FREQ_3 (PRH_PER_GNSS_ENABLE_FREQ_2 + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_GNSS_MODE_END (PRH_PER_GNSS_ENABLE_FREQ_3 + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_WLAN_MODE_START (PRH_PER_GNSS_MODE_END + 1)
/**< Disable IDI */
#define PRH_PER_WLAN_DISABLE (PRH_PER_WLAN_MODE_START + 1)
/** DCLCK = 83.2M  PCLK = 124.8M */
#define PRH_PER_WLAN_ENABLE (PRH_PER_WLAN_DISABLE + 1)
/**< D0I3 - all resources, except clk_fix2, are up; sleep allowed */
#define PRH_PER_WLAN_IDLE (PRH_PER_WLAN_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_WLAN_MODE_END (PRH_PER_WLAN_IDLE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_PWM_MODE_START (PRH_PER_WLAN_MODE_END + 1)
/**< Disable PWM clock */
#define PRH_PER_PWM_DISABLE (PRH_PER_PWM_MODE_START + 1)
/**< Enable PWM Low clock  */
#define PRH_PER_PWM_ENABLE (PRH_PER_PWM_DISABLE + 1)
/**< Enable PWM Medium clock  */
#define PRH_PER_PWM_ENABLE_PSV (PRH_PER_PWM_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_PWM_MODE_END (PRH_PER_PWM_ENABLE_PSV + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_RGA_MODE_START (PRH_PER_PWM_MODE_END + 1)
/**< Disable RGA clock */
#define PRH_PER_RGA_DISABLE (PRH_PER_RGA_MODE_START + 1)
/**< Enable RGA Low clock (104M) */
#define PRH_PER_RGA_ENABLE_LOW_PERF (PRH_PER_RGA_DISABLE + 1)
/**< Enable RGA Medium clock (156M) */
#define PRH_PER_RGA_ENABLE_MID_PERF (PRH_PER_RGA_ENABLE_LOW_PERF + 1)
/**< Enable RGA High clock (416M) */
#define PRH_PER_RGA_ENABLE_HIGH_PERF (PRH_PER_RGA_ENABLE_MID_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_RGA_MODE_END (PRH_PER_RGA_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_USIF_CLK_START (0)
/**< USIF 26 MHz kernel clock (Mode Not Supported) */
#define PRH_PER_USIF_CLK_26MHZ (PRH_PER_USIF_CLK_START + 1)
/**< USIF 96 MHz kernel clock (Mode Not Supported) */
#define PRH_PER_USIF_CLK_96MHZ (PRH_PER_USIF_CLK_26MHZ + 1)
/**< USIF 104 MHz kernel clock */
#define PRH_PER_USIF_CLK_104MHZ (PRH_PER_USIF_CLK_96MHZ + 1)
/**< Do not care USIF kernel clock */
#define PRH_PER_USIF_CLK_DONT_CARE (PRH_PER_USIF_CLK_104MHZ + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_USIF_CLK_END (PRH_PER_USIF_CLK_DONT_CARE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_USER_FIRST (0)
/**< PRH user identifier for cpu driver */
#define PRH_USER_CPU (PRH_USER_FIRST)
/**< PRH user identifier for Security Framework */
#define PRH_USER_SEC_FRAME (PRH_USER_CPU + 1)
/**< PRH user identifier for 3G Aeneas IF driver */
#define PRH_USER_3G_AENEAS_IF (PRH_USER_SEC_FRAME + 1)
/**< PRH user identifier for MACPHY driver */
#define PRH_USER_MACPHY (PRH_USER_3G_AENEAS_IF + 1)
/**< PRH user identifier for RF driver */
#define PRH_USER_RF (PRH_USER_MACPHY + 1)
/**< PRH user identifier for DSP driver */
#define PRH_USER_DSP (PRH_USER_RF + 1)
/**< PRH user identifier for GUCIPH driver */
#define PRH_USER_GUCIPH (PRH_USER_DSP + 1)
/**< PRH user identifier for GPS driver */
#define PRH_USER_GPS (PRH_USER_GUCIPH + 1)
/**< PRH user identifier for audio modem driver */
#define PRH_USER_AUD_MODEM (PRH_USER_GPS + 1)
/**< PRH user identifier for USB driver */
#define PRH_USER_USB (PRH_USER_AUD_MODEM + 1)
/**< PRH user identifier for USIF driver */
#define PRH_USER_USIF (PRH_USER_USB + 1)
/**< PRH user identifier for CAPCOM driver */
#define PRH_USER_CAPCOM (PRH_USER_USIF + 1)
/**< PRH user identifier for DMA driver */
#define PRH_USER_DMA (PRH_USER_CAPCOM + 1)
/**< PRH user identifier for I2C1 driver */
#define PRH_USER_I2C (PRH_USER_DMA + 1)
/**< PRH user identifier for PCL driver */
#define PRH_USER_PCL (PRH_USER_I2C + 1)
/**< PRH user identifier for TS driver */
#define PRH_USER_TS (PRH_USER_PCL + 1)
/**< PRH user identifier for NAND driver */
#define PRH_USER_NAND (PRH_USER_TS + 1)
/**< PRH user identifier for RTC driver */
#define PRH_USER_RTC (PRH_USER_NAND + 1)
/**< PRH user identifier for MEAS driver */
#define PRH_USER_MEAS (PRH_USER_RTC + 1)
/**< PRH user identifier for UICC driver */
#define PRH_USER_UICC (PRH_USER_MEAS + 1)
/**< PRH user identifier for MMCSD driver */
#define PRH_USER_MMCSD (PRH_USER_UICC + 1)
/**< PRH user identifier for trace debug driver */
#define PRH_USER_TRACE (PRH_USER_MMCSD + 1)
/**< PRH user identifier for fspeed driver */
#define PRH_USER_FSPEED (PRH_USER_TRACE + 1)
/**< PRH user identifier for camera driver */
#define PRH_USER_CAMERA (PRH_USER_FSPEED + 1)
/**< PRH user identifier for display driver */
#define PRH_USER_DISPLAY (PRH_USER_CAMERA + 1)
/**< PRH user identifier for graphic accelerator driver */
#define PRH_USER_GPU (PRH_USER_DISPLAY + 1)
/**< PRH user identifier for video accelerator driver */
#define PRH_USER_VPU_DEC (PRH_USER_GPU + 1)
/**< PRH user identifier for video accelerator driver */
#define PRH_USER_VPU_ENC (PRH_USER_VPU_DEC + 1)
/**< PRH user identifier for keypad driver */
#define PRH_USER_KPD (PRH_USER_VPU_ENC + 1)
/**< PRH user identifier for IDI (internal use) */
#define PRH_USER_IDI (PRH_USER_KPD + 1)
/**< PRH user identifier for BT driver */
#define PRH_USER_BT (PRH_USER_IDI + 1)
/**< PRH user identifier for FMR driver */
#define PRH_USER_FMR (PRH_USER_BT + 1)
/**< PRH user identifier for BACKLIGHT driver */
#define PRH_USER_BL (PRH_USER_FMR + 1)
/**< PRH user identifier for VIBRATOR driver */
#define PRH_USER_VIB (PRH_USER_BL + 1)
/**< PRH user identifier for WLAN driver */
#define PRH_USER_WLAN (PRH_USER_VIB + 1)
/**< PRH user identifier for GNSS driver */
#define PRH_USER_GNSS (PRH_USER_WLAN + 1)
/**< PRH user identifier for TOUCHSCREEN driver */
#define PRH_USER_TP (PRH_USER_GNSS + 1)
/**< PRH user identifier for ACCELEROMETER driver */
#define PRH_USER_ACCELEROMETER (PRH_USER_TP + 1)
/**< PRH user identifier for MAGNETOMETER driver */
#define PRH_USER_MAGNETOMETER (PRH_USER_ACCELEROMETER + 1)
/**< PRH user identifier for PROXIMITY SENSOR driver */
#define PRH_USER_PROXIMITY_SENSOR (PRH_USER_MAGNETOMETER + 1)
/**< PRH user identifier for GYROSCOPE driver */
#define PRH_USER_GYROSCOPE (PRH_USER_PROXIMITY_SENSOR + 1)

#define PRH_USER_UTA (PRH_USER_GYROSCOPE + 1)
/**< PRH user identifier for Frequency Manager */
#define PRH_USER_FM_SUBSYS_PM (PRH_USER_UTA + 1)
#if defined SF_R_ES_1_0

#define PRH_USER_RGA (PRH_USER_FM_SUBSYS_PM + 1)

#define PRH_USER_PWM (PRH_USER_RGA + 1)
/**< PRH user dummy identifier */
#define PRH_USER_DUMMY (PRH_USER_PWM + 1)
#else
/**< PRH user dummy identifier */
#define PRH_USER_DUMMY (PRH_USER_FM_SUBSYS_PM + 1)
#endif
/**< End indicator */
#define PRH_USER_NOF_ID (PRH_USER_DUMMY + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_FIRST (0)
/**< PRH peripheral identifier for PS_CPU [SPECIFIC] */
#define PRH_PER_ARM11_CLK (PRH_PER_FIRST)
/**< PRH peripheral identifier for DMAC_2CH */
#define PRH_PER_DMAC_2CH (PRH_PER_ARM11_CLK + 1)
/**< PRH peripheral identifier for CEU */
#define PRH_PER_CEU (PRH_PER_DMAC_2CH + 1)
/**< PRH peripheral identifier for CEU2 */
#define PRH_PER_CEU2 (PRH_PER_CEU + 1)
/**< PRH peripheral identifier for GSER */
#define PRH_PER_GSER (PRH_PER_CEU2 + 1)
/**< PRH peripheral identifier for 3G_COM_RAM */
#define PRH_PER_3G_COMRAM (PRH_PER_GSER + 1)
/**< PRH peripheral identifier for MACPHY (UL/DL) [SPECIFIC] */
#define PRH_PER_MACPHY (PRH_PER_3G_COMRAM + 1)
/**< PRH peripheral identifier for DIG_RF */
#define PRH_PER_DIG_RF (PRH_PER_MACPHY + 1)
/**< PRH peripheral identifier for GSI */
#define PRH_PER_GSI (PRH_PER_DIG_RF + 1)
/**< PRH peripheral identifier for RF AHB_PER2 bus frequency reduction */
#define PRH_PER_AHB_PER2_RF_BFR (PRH_PER_GSI + 1)
/**< PRH peripheral identifier for RF AHB_PER3 bus frequency reduction */
#define PRH_PER_AHB_PER3_RF_BFR (PRH_PER_AHB_PER2_RF_BFR + 1)
/**< PRH peripheral identifier for Modem 2G inner DSP [SPECIFIC] */
#define PRH_PER_DSP_2G (PRH_PER_AHB_PER3_RF_BFR + 1)
/**< PRH peripheral identifier for Audio sample DSP [SPECIFIC] */
#define PRH_PER_DSP_AUD (PRH_PER_DSP_2G + 1)
/**< PRH peripheral identifier for GUCIPH */
#define PRH_PER_GUCIPH (PRH_PER_DSP_AUD + 1)
/**< PRH peripheral identifier for GPS [SPECIFIC] */
#define PRH_PER_GPS (PRH_PER_GUCIPH + 1)
/**< PRH peripheral identifier for AUD_LIB_PSV */
#define PRH_PER_AUD_PSV (PRH_PER_GPS + 1)
/**< PRH peripheral identifier for I2S1 */
#define PRH_PER_I2S1 (PRH_PER_AUD_PSV + 1)
/**< PRH peripheral identifier for DIG_MIC */
#define PRH_PER_DIG_MIC (PRH_PER_I2S1 + 1)
/**< PRH peripheral identifier for ABB_DIG_MIC */
#define PRH_PER_ABB_DIG_MIC (PRH_PER_DIG_MIC + 1)
/**< PRH peripheral identifier for AFE  [SPECIFIC] */
#define PRH_PER_ABB_AFE (PRH_PER_ABB_DIG_MIC + 1)
/**< PRH peripheral identifier for AUD SYNC  [SPECIFIC] */
#define PRH_PER_AUD_SYNC (PRH_PER_ABB_AFE + 1)
/**< PRH peripheral identifier for USB_HS [SPECIFIC] */
#define PRH_PER_USB (PRH_PER_AUD_SYNC + 1)
/**< PRH peripheral identifier for USIF1 [SPECIFIC] */
#define PRH_PER_USIF1 (PRH_PER_USB + 1)
/**< PRH peripheral identifier for USIF2 [SPECIFIC] */
#define PRH_PER_USIF2 (PRH_PER_USIF1 + 1)
/**< PRH peripheral identifier for USIF3 (not supported) */
#define PRH_PER_USIF3 (PRH_PER_USIF2 + 1)
/**< PRH peripheral identifier for USIF5 for Artemis Trace */
#define PRH_PER_3G_USIF (PRH_PER_USIF3 + 1)
/**< PRH peripheral identifier for CAPCOM0 [SPECIFIC] */
#define PRH_PER_CAPCOM0 (PRH_PER_3G_USIF + 1)
/**< PRH peripheral identifier for CAPCOM1 [SPECIFIC] */
#define PRH_PER_CAPCOM1 (PRH_PER_CAPCOM0 + 1)
/**< PRH peripheral identifier for DMAC_8CH  [SPECIFIC] */
#define PRH_PER_DMAC_8CH (PRH_PER_CAPCOM1 + 1)
/**< PRH peripheral identifier for DMAC_8CH_2 (not supported) */
#define PRH_PER_DMAC_8CH_2 (PRH_PER_DMAC_8CH + 1)
/**< PRH peripheral identifier for I2C1 [SPECIFIC] */
#define PRH_PER_I2C1 (PRH_PER_DMAC_8CH_2 + 1)
/**< PRH peripheral identifier for I2C2 [SPECIFIC] */
#define PRH_PER_I2C2 (PRH_PER_I2C1 + 1)
/**< PRH peripheral identifier for I2C3 [SPECIFIC] */
#define PRH_PER_I2C3 (PRH_PER_I2C2 + 1)
/**< PRH peripheral identifier for I2C */
#define PRH_PER_ABB_I2C (PRH_PER_I2C3 + 1)
/**< PRH peripheral identifier for DBB PCL */
#define PRH_PER_PCL (PRH_PER_ABB_I2C + 1)
/**< PRH peripheral identifier for ABB PCL */
#define PRH_PER_ABB_PCL (PRH_PER_PCL + 1)
/**< PRH peripheral identifier for GPTU0 */
#define PRH_PER_GPTU0 (PRH_PER_ABB_PCL + 1)
/**< PRH peripheral identifier for GPTU1 */
#define PRH_PER_GPTU1 (PRH_PER_GPTU0 + 1)
/**< PRH peripheral identifier for STM */
#define PRH_PER_STM (PRH_PER_GPTU1 + 1)
/**< PRH peripheral identifier for NANDCTRL [SPECIFIC] */
#define PRH_PER_NAND (PRH_PER_STM + 1)
/**< PRH peripheral identifier for DBB RTC */
#define PRH_PER_RTC (PRH_PER_NAND + 1)
/**< PRH peripheral identifier for ABB RTC */
#define PRH_PER_ABB_RTC (PRH_PER_RTC + 1)
/**< PRH peripheral identifier for TMSU */
#define PRH_PER_TSMU (PRH_PER_ABB_RTC + 1)
/**< PRH peripheral identifier for UICC_VCC  [SPECIFIC] */
#define PRH_PER_UICC_VCC (PRH_PER_TSMU + 1)
/**< PRH peripheral identifier for UICC_CLK  [SPECIFIC] */
#define PRH_PER_UICC_CLK (PRH_PER_UICC_VCC + 1)
/**< PRH peripheral identifier for UICC2_VCC [SPECIFIC] */
#define PRH_PER_UICC2_VCC (PRH_PER_UICC_CLK + 1)
/**< PRH peripheral identifier for UICC2_CLK [SPECIFIC] */
#define PRH_PER_UICC2_CLK (PRH_PER_UICC2_VCC + 1)
/**< PRH peripheral identifier for SDMMC1_POW (NOT in Operation) */
#define PRH_PER_SDMMC1_POW (PRH_PER_UICC2_CLK + 1)
/**< PRH peripheral identifier for SDMMC1_VOLT [SPECIFIC] */
#define PRH_PER_SDMMC1_VOLT (PRH_PER_SDMMC1_POW + 1)
/**< PRH peripheral identifier for SDMMC1_CLK  [SPECIFIC] */
#define PRH_PER_SDMMC1_CLK (PRH_PER_SDMMC1_VOLT + 1)
/**< PRH peripheral identifier for SDMMC1_PSV */
#define PRH_PER_SDMMC1_PSV (PRH_PER_SDMMC1_CLK + 1)
/**< PRH peripheral identifier for SDIO_POW (NOT in Operation) */
#define PRH_PER_SDIO_POW (PRH_PER_SDMMC1_PSV + 1)
/**< PRH peripheral identifier for SDIO_VOLT [SPECIFIC](NOT in Operation) */
#define PRH_PER_SDIO_VOLT (PRH_PER_SDIO_POW + 1)
/**< PRH peripheral identifier for SDIO_CLK  [SPECIFIC] */
#define PRH_PER_SDIO_CLK (PRH_PER_SDIO_VOLT + 1)
/**< PRH peripheral identifier for SDIO_PSV */
#define PRH_PER_SDIO_PSV (PRH_PER_SDIO_CLK + 1)
/**< PRH peripheral identifier for EMMC_POW (NOT in Operation) */
#define PRH_PER_EMMC_POW (PRH_PER_SDIO_PSV + 1)
/**< PRH peripheral identifier for VEMMC_VOLT [SPECIFIC] */
#define PRH_PER_VEMMC_VOLT (PRH_PER_EMMC_POW + 1)
/**< PRH peripheral identifier for VMMC_VOLT [SPECIFIC]*/
#define PRH_PER_VMMC_VOLT (PRH_PER_VEMMC_VOLT + 1)
/**< PRH peripheral identifier for EMMC_CLK  [SPECIFIC] */
#define PRH_PER_EMMC_CLK (PRH_PER_VMMC_VOLT + 1)
/**< PRH peripheral identifier for EMMC_PSV */
#define PRH_PER_EMMC_PSV (PRH_PER_EMMC_CLK + 1)
/**< PRH peripheral identifier for DBB ST_ARB  [SPECIFIC] */
#define PRH_PER_ST_ARB (PRH_PER_EMMC_PSV + 1)
/**< PRH peripheral identifier for On-chip trace  [SPECIFIC] */
#define PRH_PER_ST_OCT (PRH_PER_ST_ARB + 1)
/**< PRH peripheral identifier for MTM1  [SPECIFIC] */
#define PRH_PER_ST_MTM1 (PRH_PER_ST_OCT + 1)
/**< PRH peripheral identifier for MTM2 */
#define PRH_PER_ST_MTM2 (PRH_PER_ST_MTM1 + 1)
/**< PRH peripheral identifier for MTM2_PAD */
#define PRH_PER_ST_MTM2_PAD (PRH_PER_ST_MTM2 + 1)
/**< PRH peripheral identifier for ST_MON  [SPECIFIC] */
#define PRH_PER_ST_MON (PRH_PER_ST_MTM2_PAD + 1)
/**< PRH peripheral identifier for ST_MON_SB 1 */
#define PRH_PER_ST_MON_SB_1 (PRH_PER_ST_MON + 1)
/**< PRH peripheral identifier for ST_MON_SB 2 */
#define PRH_PER_ST_MON_SB_2 (PRH_PER_ST_MON_SB_1 + 1)
/**< PRH peripheral identifier for ST_MON_SB 3 */
#define PRH_PER_ST_MON_SB_3 (PRH_PER_ST_MON_SB_2 + 1)
/**< PRH peripheral identifier for ST_MON_SB 4 */
#define PRH_PER_ST_MON_SB_4 (PRH_PER_ST_MON_SB_3 + 1)
/**< PRH peripheral identifier for ST_MON_SB 5 */
#define PRH_PER_ST_MON_SB_5 (PRH_PER_ST_MON_SB_4 + 1)
/**< PRH peripheral identifier for ST_MON_SB 6 */
#define PRH_PER_ST_MON_SB_6 (PRH_PER_ST_MON_SB_5 + 1)
/**< PRH peripheral identifier for ST_MON_SB 7 */
#define PRH_PER_ST_MON_SB_7 (PRH_PER_ST_MON_SB_6 + 1)
/**< PRH peripheral identifier for ST_MON_SB 8 */
#define PRH_PER_ST_MON_SB_8 (PRH_PER_ST_MON_SB_7 + 1)
/**< PRH peripheral identifier for ST_MON_SB 9 */
#define PRH_PER_ST_MON_SB_9 (PRH_PER_ST_MON_SB_8 + 1)
/**< PRH peripheral identifier for ST_MON_SB 10 */
#define PRH_PER_ST_MON_SB_10 (PRH_PER_ST_MON_SB_9 + 1)
/**< PRH peripheral identifier for ST_MON_SB 11 */
#define PRH_PER_ST_MON_SB_11 (PRH_PER_ST_MON_SB_10 + 1)
/**< PRH peripheral identifier for ST_MON_SB 12 */
#define PRH_PER_ST_MON_SB_12 (PRH_PER_ST_MON_SB_11 + 1)
/**< PRH peripheral identifier for ETM11 [SPECIFIC] */
#define PRH_PER_ARM11_ETM_TRACE (PRH_PER_ST_MON_SB_12 + 1)
/**< PRH peripheral identifier for CST  [SPECIFIC] */
#define PRH_PER_CST (PRH_PER_ARM11_ETM_TRACE + 1)
/**< PRH peripheral identifier for ABB ST_ARB */
#define PRH_PER_ABB_ST_ARB (PRH_PER_CST + 1)
/**< PRH peripheral identifier for ABB MTM */
#define PRH_PER_ABB_ST_MTM (PRH_PER_ABB_ST_ARB + 1)
/**< PRH peripheral identifier for ABB ST_MON_SB 1 */
#define PRH_PER_ABB_ST_MON_SB_1 (PRH_PER_ABB_ST_MTM + 1)
/**< PRH peripheral identifier for ABB ST_MON_SB 2 */
#define PRH_PER_ABB_ST_MON_SB_2 (PRH_PER_ABB_ST_MON_SB_1 + 1)
/**< PRH peripheral identifier for ABB ST_MON_SB 3 */
#define PRH_PER_ABB_ST_MON_SB_3 (PRH_PER_ABB_ST_MON_SB_2 + 1)
/**< PRH peripheral identifier for power save */
#define PRH_PER_FSPEED_PSV (PRH_PER_ABB_ST_MON_SB_3 + 1)
/**< PRH peripheral identifier for low voltage */
#define PRH_PER_FSPEED_LOW_VOLT (PRH_PER_FSPEED_PSV + 1)
/**< PRH peripheral identifier for IDI */
#define PRH_PER_IDI (PRH_PER_FSPEED_LOW_VOLT + 1)
/**< PRH peripheral identifier for ABB_IDI_FM [SPECIFIC] */
#define PRH_PER_ABB_IDI_FM (PRH_PER_IDI + 1)
/**< PRH peripheral identifier for camera interface */
#define PRH_PER_CIF (PRH_PER_ABB_IDI_FM + 1)
/**< PRH peripheral identifier for external primary camera */
#define PRH_PER_EXT_PRIM_CAM (PRH_PER_CIF + 1)
/**< PRH peripheral identifier for external secondary camera */
#define PRH_PER_EXT_SEC_CAM (PRH_PER_EXT_PRIM_CAM + 1)
/**< PRH peripheral identifier for display controller */
#define PRH_PER_DCC (PRH_PER_EXT_SEC_CAM + 1)
/**< PRH peripheral identifier for external display */
#define PRH_PER_EXT_DISPLAY (PRH_PER_DCC + 1)
/**< PRH peripheral identifier for keypad */
#define PRH_PER_KPD (PRH_PER_EXT_DISPLAY + 1)
/**< PRH peripheral identifier for Graphic Accelerator */
#define PRH_PER_GPU (PRH_PER_KPD + 1)
/**< PRH peripheral identifier for Video Accelerator decoder */
#define PRH_PER_VIDEO_DEC (PRH_PER_GPU + 1)
/**< PRH peripheral identifier for Video Accelerator encoder */
#define PRH_PER_VIDEO_ENC (PRH_PER_VIDEO_DEC + 1)
/**< PRH peripheral identifier for BT (IF + IP) driver */
#define PRH_PER_ABB_BT (PRH_PER_VIDEO_ENC + 1)
/**< PRH peripheral identifier for BT_AUD driver */
#define PRH_PER_ABB_BT_AUD (PRH_PER_ABB_BT + 1)
/**< PRH peripheral identifier for FMR [SPECIFIC] */
#define PRH_PER_ABB_FMR (PRH_PER_ABB_BT_AUD + 1)
/**< PRH peripheral identifier for backlight */
#define PRH_PER_ABB_BL (PRH_PER_ABB_FMR + 1)
/**< PRH peripheral identifier for vibrator */
#define PRH_PER_ABB_VIB (PRH_PER_ABB_BL + 1)
/**< PRH peripheral identifier for WLAN */
#define PRH_PER_ABB_WLAN (PRH_PER_ABB_VIB + 1)
/**< PRH peripheral identifier for GNSS */
#define PRH_PER_ABB_GNSS (PRH_PER_ABB_WLAN + 1)
/**< PRH peripheral identifier for touchscreen */
#define PRH_PER_EXT_TP (PRH_PER_ABB_GNSS + 1)
/**< PRH peripheral identifier for accelerometer */
#define PRH_PER_EXT_ACCELEROMETER (PRH_PER_EXT_TP + 1)
/**< PRH peripheral identifier for magnetometer */
#define PRH_PER_EXT_MAGNETOMETER (PRH_PER_EXT_ACCELEROMETER + 1)
/**< PRH peripheral identifier for proximity_sensor */
#define PRH_PER_EXT_PROXIMITY_SENSOR (PRH_PER_EXT_MAGNETOMETER + 1)
/**< PRH peripheral identifier for gyroscope */
#define PRH_PER_EXT_GYROSCOPE (PRH_PER_EXT_PROXIMITY_SENSOR + 1)
/**< PRH peripheral identifier for PMU DCDC from FM [SPECIFIC] */
#define PRH_PER_DCDC (PRH_PER_EXT_GYROSCOPE + 1)
/**< PRH peripheral identifier for PMU Charge Pump from FM [SPECIFIC] */
#define PRH_PER_CP (PRH_PER_DCDC + 1)
#if defined SF_R_ES_1_0
/**< PRH peripheral identifier for I2C4 [SPECIFIC] */
#define PRH_PER_I2C4 (PRH_PER_CP + 1)
/**< PRH peripheral identifier for PMU  from RGA */
#define PRH_PER_RGA (PRH_PER_I2C4 + 1)
/**< PRH peripheral identifier for PMU  from PWM */
#define PRH_PER_PWM (PRH_PER_RGA + 1)
/**< PRH peripheral dummy identifier  */
#define PRH_PER_DUMMY (PRH_PER_PWM + 1)
#else
/**< PRH peripheral dummy identifier  */
#define PRH_PER_DUMMY (PRH_PER_CP + 1)
#endif
/**< End indicator */
#define PRH_PER_NOF_ID (PRH_PER_DUMMY + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_TYPE_FIRST (0)
/**< PRH peripheral identifier for generic peripheral */
#define PRH_PER_TYPE_GENERIC (PRH_PER_TYPE_FIRST)
/**< PRH peripheral identifier for PS_CPU_CLK peripheral */
#define PRH_PER_TYPE_ARM11_CLK (PRH_PER_TYPE_GENERIC + 1)
/**< PRH peripheral identifier for MACPHY peripheral */
#define PRH_PER_TYPE_MACPHY (PRH_PER_TYPE_ARM11_CLK + 1)
/**< PRH peripheral identifier for DMA peripheral */
#define PRH_PER_TYPE_DMA (PRH_PER_TYPE_MACPHY + 1)
/**< PRH peripheral identifier for DSP peripheral */
#define PRH_PER_TYPE_DSP (PRH_PER_TYPE_DMA + 1)
/**< PRH peripheral identifier for I2C peripheral */
#define PRH_PER_TYPE_I2C (PRH_PER_TYPE_DSP + 1)
/**< PRH peripheral identifier for MMCSD_VOLT peripheral */
#define PRH_PER_TYPE_MMCSD_VOLT (PRH_PER_TYPE_I2C + 1)
/**< PRH peripheral identifier for MMCSD_CLK peripheral */
#define PRH_PER_TYPE_MMCSD_CLK (PRH_PER_TYPE_MMCSD_VOLT + 1)
/**< PRH peripheral identifier for NANDCTRL peripheral */
#define PRH_PER_TYPE_NAND (PRH_PER_TYPE_MMCSD_CLK + 1)
/**< PRH peripheral identifier for USB HS peripheral */
#define PRH_PER_TYPE_USB (PRH_PER_TYPE_NAND + 1)
/**< PRH peripheral identifier for USIF peripheral */
#define PRH_PER_TYPE_USIF (PRH_PER_TYPE_USB + 1)
/**< PRH peripheral identifier for CAPCOM peripheral */
#define PRH_PER_TYPE_CAPCOM (PRH_PER_TYPE_USIF + 1)
/**< PRH peripheral identifier for UICC peripheral */
#define PRH_PER_TYPE_UICC_VCC (PRH_PER_TYPE_CAPCOM + 1)
/**< PRH peripheral identifier for UICC peripheral */
#define PRH_PER_TYPE_UICC_CLK (PRH_PER_TYPE_UICC_VCC + 1)
/**< PRH peripheral identifier for GPS peripheral */
#define PRH_PER_TYPE_GPS (PRH_PER_TYPE_UICC_CLK + 1)
/**< PRH peripheral identifier for TRACE peripheral */
#define PRH_PER_TYPE_TRACE (PRH_PER_TYPE_GPS + 1)
/**< PRH peripheral identifier for ABB_AFE peripheral */
#define PRH_PER_TYPE_ABB_AFE (PRH_PER_TYPE_TRACE + 1)
/**< PRH peripheral identifier for ABB_FMR peripheral */
#define PRH_PER_TYPE_ABB_FMR (PRH_PER_TYPE_ABB_AFE + 1)
/**< PRH peripheral identifier for IDI peripheral */
#define PRH_PER_TYPE_IDI (PRH_PER_TYPE_ABB_FMR + 1)
/**< PRH peripheral identifier for IDI peripheral */
#define PRH_PER_TYPE_ABB_IDI_FM (PRH_PER_TYPE_IDI + 1)
/**< PRH peripheral identifier for DCC peripheral */
#define PRH_PER_TYPE_DCC (PRH_PER_TYPE_ABB_IDI_FM + 1)
/**< PRH peripheral identifier for CIF peripheral */
#define PRH_PER_TYPE_CIF (PRH_PER_TYPE_DCC + 1)
/**< PRH peripheral identifier for GPU peripheral */
#define PRH_PER_TYPE_GPU (PRH_PER_TYPE_CIF + 1)
/**< PRH peripheral identifier for VIDEO DEC peripheral */
#define PRH_PER_TYPE_VIDEO_DEC (PRH_PER_TYPE_GPU + 1)
/**< PRH peripheral identifier for VIDEO ENC peripheral */
#define PRH_PER_TYPE_VIDEO_ENC (PRH_PER_TYPE_VIDEO_DEC + 1)
/**< PRH peripheral identifier for KPD peripheral */
#define PRH_PER_TYPE_KPD (PRH_PER_TYPE_VIDEO_ENC + 1)
/**< PRH peripheral identifier for PMU DCDC peripheral */
#define PRH_PER_TYPE_DCDC (PRH_PER_TYPE_KPD + 1)
/**< PRH peripheral identifier for PMU CHP peripheral */
#define PRH_PER_TYPE_CP (PRH_PER_TYPE_DCDC + 1)
/**< PRH peripheral identifier for ABB AUD SYNC peripheral */
#define PRH_PER_TYPE_AUD_SYNC (PRH_PER_TYPE_CP + 1)
/**< PRH peripheral identifier for ABB BT (IF + IP) peripheral */
#define PRH_PER_TYPE_ABB_BT (PRH_PER_TYPE_AUD_SYNC + 1)
/**< PRH peripheral identifier for GNSS peripheral */
#define PRH_PER_TYPE_GNSS (PRH_PER_TYPE_ABB_BT + 1)
/**< PRH peripheral identifier for WLAN peripheral */
#define PRH_PER_TYPE_WLAN (PRH_PER_TYPE_GNSS + 1)
#if defined SF_R_ES_1_0
/**< PRH peripheral identifier for RGA peripheral */
#define PRH_PER_TYPE_RGA (PRH_PER_TYPE_WLAN + 1)
/**< PRH peripheral identifier for PWM peripheral */
#define PRH_PER_TYPE_PWM (PRH_PER_TYPE_RGA + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_NOF_TYPE (PRH_PER_TYPE_PWM + 1)
#else
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_NOF_TYPE (PRH_PER_TYPE_WLAN + 1)
#endif

