/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define TDSCDMA_ENABLED

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_GENERIC_MODE_START (0)
/**< Disable generic peripheral power resources and enable power saving state */
#define PRH_PER_GENERIC_DISABLE (PRH_PER_GENERIC_MODE_START + 1)
/**< Disable generic peripheral power saving state and enable power resources */
#define PRH_PER_GENERIC_ENABLE (PRH_PER_GENERIC_DISABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_GENERIC_MODE_END (PRH_PER_GENERIC_ENABLE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_MMC_CORE_CLK_MODE_START (PRH_PER_GENERIC_MODE_END + 1)
/**< Enable MMC_CORE_CLK low performance use cases */
#define PRH_PER_MMC_CORE_CLK_ENABLE_LOW_PERF (PRH_PER_MMC_CORE_CLK_MODE_START + 1)
/**< Enable MMC_CORE_CLK med performance use cases */
#define PRH_PER_MMC_CORE_CLK_ENABLE_MED_PERF (PRH_PER_MMC_CORE_CLK_ENABLE_LOW_PERF + 1)
/**< Enable MMC_CORE_CLK high performance use cases */
#define PRH_PER_MMC_CORE_CLK_ENABLE_HIGH_PERF (PRH_PER_MMC_CORE_CLK_ENABLE_MED_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MMC_CORE_CLK_MODE_END (PRH_PER_MMC_CORE_CLK_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_3G_COMRAM_MODE_START (PRH_PER_MMC_CORE_CLK_MODE_END+1)
/**< Disable 3G_COMRAM power resources (Dormant Mode) */
#define PRH_PER_3G_COMRAM_DISABLE (PRH_PER_3G_COMRAM_MODE_START + 1)
/**< Enable 3G_COMRAM power resources */
#define PRH_PER_3G_COMRAM_ENABLE (PRH_PER_3G_COMRAM_DISABLE + 1)
/**< Shutdown 3G_COMRAM power resources */
#define PRH_PER_3G_COMRAM_SHUTDOWN (PRH_PER_3G_COMRAM_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_3G_COMRAM_MODE_END (PRH_PER_3G_COMRAM_SHUTDOWN + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_EMACPHY_MODE_START (PRH_PER_3G_COMRAM_MODE_END+1)
/**< Disable DMA power resources and enable DMA power saving state */
#define PRH_PER_EMACPHY_DISABLE (PRH_PER_EMACPHY_MODE_START + 1)
/**< Disable EMACPHY power saving state and enable EMACPHY power resources */
#define PRH_PER_EMACPHY_ENABLE_LOW_PERF (PRH_PER_EMACPHY_DISABLE + 1)
/**< Disable EMACPHY power saving state and enable EMACPHY power resources */
#define PRH_PER_EMACPHY_ENABLE_MID_PERF (PRH_PER_EMACPHY_ENABLE_LOW_PERF + 1)
/**< Disable EMACPHY power saving state and enable EMACPHY power resources */
#define PRH_PER_EMACPHY_ENABLE_HIGH_PERF (PRH_PER_EMACPHY_ENABLE_MID_PERF + 1)
/**< Reset EMACPHY */
#define PRH_PER_EMACPHY_RESET (PRH_PER_EMACPHY_ENABLE_HIGH_PERF + 1)
/**< Disable EMACPHY power saving state and enable EMACPHY power resources */
#define PRH_PER_EMACPHY_ENABLE_LOW_PERF_3G (PRH_PER_EMACPHY_RESET + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_EMACPHY_MODE_END (PRH_PER_EMACPHY_ENABLE_LOW_PERF_3G + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_DMA_MODE_START (PRH_PER_EMACPHY_MODE_END+1)
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
/**< Disable DSP power resources (Dormant Mode) */
#define PRH_PER_DSP_DISABLE (PRH_PER_DSP_MODE_START + 1)
/**< Enable DSP power resources */
#define PRH_PER_DSP_ENABLE (PRH_PER_DSP_DISABLE + 1)
/**< Suspend DSP power resources */
#define PRH_PER_DSP_SUSPEND (PRH_PER_DSP_ENABLE + 1)
/**< Shutdown DSP power resources */
#define PRH_PER_DSP_SHUTDOWN (PRH_PER_DSP_SUSPEND + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_DSP_MODE_END (PRH_PER_DSP_SHUTDOWN + 1)

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
/**< Enable MMCSD Low voltage (not supported) */
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
/**< Enable MMCSD Low clock */
#define PRH_PER_MMCSD_CLK_ENABLE_LOW (PRH_PER_MMCSD_CLK_DISABLE + 1)
/**< Enable MMCSD Medium clock */
#define PRH_PER_MMCSD_CLK_ENABLE_MID (PRH_PER_MMCSD_CLK_ENABLE_LOW + 1)
/**< Enable MMCSD High clock */
#define PRH_PER_MMCSD_CLK_ENABLE_HIGH (PRH_PER_MMCSD_CLK_ENABLE_MID + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MMCSD_CLK_MODE_END (PRH_PER_MMCSD_CLK_ENABLE_HIGH + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_USB_MODE_START (PRH_PER_MMCSD_CLK_MODE_END+1)
/**< Disable USB HS power resources and enable USB HS power saving state */
#define PRH_PER_USB_DISABLE (PRH_PER_USB_MODE_START + 1)
/**< Suspend USB HS power resources and enable USB HSIC power saving state */
#define PRH_PER_USB_SUSPEND (PRH_PER_USB_DISABLE + 1)
/**< Suspend USB HS power resources but disable power saving state */
#define PRH_PER_USB_SUSPEND_NO_PSV (PRH_PER_USB_SUSPEND + 1)
/**< Disable USB HS power saving state and enable USB HS power resources */
#define PRH_PER_USB_ENABLE (PRH_PER_USB_SUSPEND_NO_PSV + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_USB_MODE_END (PRH_PER_USB_ENABLE + 1)

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
/**< Kernel clk: 13/4 MHz; Bus clk: Off */
#define PRH_PER_UICC_CLK_SUSPEND (PRH_PER_UICC_CLK_DISABLE + 1)
/**< Kernel clk: 13/4 MHz; Bus clk: 26MHz */
#define PRH_PER_UICC_CLK_ENABLE_LOW_PERF (PRH_PER_UICC_CLK_SUSPEND + 1)
/**< Kernel clk: 15.6/4 MHz; Bus clk: 26MHz */
#define PRH_PER_UICC_CLK_ENABLE_MED_PERF (PRH_PER_UICC_CLK_ENABLE_LOW_PERF + 1)
/**< Kernel clk: 19.5/4 MHz; Bus clk: 26MHz */
#define PRH_PER_UICC_CLK_ENABLE_HIGH_PERF (PRH_PER_UICC_CLK_ENABLE_MED_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_UICC_CLK_MODE_END (PRH_PER_UICC_CLK_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_CAPCOM_MODE_START (PRH_PER_UICC_CLK_MODE_END+1)
/**< Disable CAPCOM */
#define PRH_PER_CAPCOM_DISABLE (PRH_PER_CAPCOM_MODE_START + 1)
/**< Enable CAPCOM */
#define PRH_PER_CAPCOM_ENABLE (PRH_PER_CAPCOM_DISABLE + 1)
/**< Disable power save CAPCOM */
#define PRH_PER_CAPCOM_DISABLE_PSV (PRH_PER_CAPCOM_ENABLE + 1)
/**< Enable power save CAPCOM */
#define PRH_PER_CAPCOM_ENABLE_PSV (PRH_PER_CAPCOM_DISABLE_PSV + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_CAPCOM_MODE_END (PRH_PER_CAPCOM_ENABLE_PSV + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_TRACE_MODE_START (PRH_PER_CAPCOM_MODE_END + 1)
/**< Disable TRACE */
#define PRH_PER_TRACE_DISABLE (PRH_PER_TRACE_MODE_START + 1)
/**< Enable TRACE */
#define PRH_PER_TRACE_ENABLE (PRH_PER_TRACE_DISABLE + 1)
/**< Enable TRACE low performance use cases */
#define PRH_PER_TRACE_ENABLE_LOW_PERF (PRH_PER_TRACE_ENABLE + 1)
/**< Enable TRACE medium performance use cases */
#define PRH_PER_TRACE_ENABLE_MED_PERF (PRH_PER_TRACE_ENABLE_LOW_PERF + 1)
/**< Enable TRACE high performance use cases */
#define PRH_PER_TRACE_ENABLE_HIGH_PERF (PRH_PER_TRACE_ENABLE_MED_PERF + 1)
/**< Enable Low Power Trace (OCT use) */
#define PRH_PER_TRACE_ENABLE_LPT (PRH_PER_TRACE_ENABLE_HIGH_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_TRACE_MODE_END (PRH_PER_TRACE_ENABLE_LPT + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_PDB_MODE_START (PRH_PER_TRACE_MODE_END + 1)
/**< Disable PDB */
#define PRH_PER_PDB_DISABLE (PRH_PER_PDB_MODE_START + 1)
/**< Enable PDB low performance use cases */
#define PRH_PER_PDB_ENABLE_LOW_PERF (PRH_PER_PDB_DISABLE + 1)
/**< Enable PDB medium performance use cases */
#define PRH_PER_PDB_ENABLE_MED_PERF (PRH_PER_PDB_ENABLE_LOW_PERF + 1)
/**< Enable PDB high performance use cases */
#define PRH_PER_PDB_ENABLE_HIGH_PERF (PRH_PER_PDB_ENABLE_MED_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_PDB_MODE_END (PRH_PER_PDB_ENABLE_HIGH_PERF + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_DCC_MODE_START (PRH_PER_PDB_MODE_END+1)
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
/**< Enable GPU High clock (416M) */
#define PRH_PER_GPU_ENABLE_HIGH_PERF (PRH_PER_GPU_ENABLE_MID_PERF + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_GPU_MODE_END (PRH_PER_GPU_ENABLE_HIGH_PERF + 1)

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
#define PRH_PER_TDIP_MODE_START (PRH_PER_KPD_MODE_END + 1)
/**< Disable TDIP */
#define PRH_PER_TDIP_DISABLE (PRH_PER_TDIP_MODE_START + 1)
/**< Enable TDIP */
#define PRH_PER_TDIP_ENABLE (PRH_PER_TDIP_DISABLE + 1)
/**< Suspend TDIP */
#define PRH_PER_TDIP_SUSPEND (PRH_PER_TDIP_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_TDIP_MODE_END (PRH_PER_TDIP_SUSPEND + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_3G_SUBSYS_MODE_START (PRH_PER_TDIP_MODE_END + 1)
/**< Disable 3G Subsys: BootReq OFF; ShutDown ON */
#define PRH_PER_3G_SUBSYS_DISABLE (PRH_PER_3G_SUBSYS_MODE_START + 1)
/**< Enable  3G Subsys: BootReq ON;  ShutDown OFF */
#define PRH_PER_3G_SUBSYS_ENABLE (PRH_PER_3G_SUBSYS_DISABLE + 1)
/**< Suspend 3G Subsys: BootReq ON;  ShutDown ON */
#define PRH_PER_3G_SUBSYS_BOOT (PRH_PER_3G_SUBSYS_ENABLE + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_3G_SUBSYS_MODE_END (PRH_PER_3G_SUBSYS_BOOT + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_MIPI_HSI_VOLT_START (0)
/**< MIPI_HSI with 1,2 V */
#define PRH_PER_MIPI_HSI_VOLT_1V2 (PRH_PER_MIPI_HSI_VOLT_START + 1)
/**< MIPI_HSI with 1,8 V */
#define PRH_PER_MIPI_HSI_VOLT_1V8 (PRH_PER_MIPI_HSI_VOLT_1V2 + 1)
/**< Do not care MIPI_HSI V */
#define PRH_PER_MIPI_HSI_VOLT_DONT_CARE (PRH_PER_MIPI_HSI_VOLT_1V8 + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MIPI_HSI_VOLT_END (PRH_PER_MIPI_HSI_VOLT_DONT_CARE + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_MIPI_HSI_CLK_START (0)
/**< MIPI_HSI 52 KHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_52MHZ (PRH_PER_MIPI_HSI_CLK_START + 1)
/**< MIPI_HSI 78 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_78MHZ (PRH_PER_MIPI_HSI_CLK_52MHZ + 1)
/**< MIPI_HSI 96 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_96MHZ (PRH_PER_MIPI_HSI_CLK_78MHZ + 1)
/**< MIPI_HSI 104 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_104MHZ (PRH_PER_MIPI_HSI_CLK_96MHZ + 1)
/**< MIPI_HSI 138.760 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_138760KHZ (PRH_PER_MIPI_HSI_CLK_104MHZ + 1)
/**< MIPI_HSI 156 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_156MHZ (PRH_PER_MIPI_HSI_CLK_138760KHZ + 1)
/**< MIPI_HSI 208 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_208MHZ (PRH_PER_MIPI_HSI_CLK_156MHZ + 1)
/**< MIPI_HSI 228 MHz kernel clock */
#define PRH_PER_MIPI_HSI_CLK_228MHZ (PRH_PER_MIPI_HSI_CLK_208MHZ + 1)
/**< Do not care MIPI_HSI kernel clock */
#define PRH_PER_MIPI_HSI_CLK_DONT_CARE (PRH_PER_MIPI_HSI_CLK_228MHZ + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_MIPI_HSI_CLK_END (PRH_PER_MIPI_HSI_CLK_DONT_CARE + 1)

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
/**< PRH user identifier for 3G driver */
#define PRH_USER_3G (PRH_USER_3G_AENEAS_IF + 1)
/**< PRH user identifier for LTE driver */
#define PRH_USER_LTE (PRH_USER_3G + 1)
/**< PRH user identifier for EMACPHY driver */
#define PRH_USER_EMACPHY (PRH_USER_LTE + 1)
/**< PRH user identifier for EPHY driver */
#define PRH_USER_EPHY (PRH_USER_EMACPHY + 1)
/**< PRH user identifier for RF driver */
#define PRH_USER_RF (PRH_USER_EPHY + 1)
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
/**< PRH user identifier for RTT driver (time services) */
#define PRH_USER_TS (PRH_USER_PCL + 1)
/**< PRH user identifier for RTC driver */
#define PRH_USER_RTC (PRH_USER_TS + 1)
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
#if defined(TDSCDMA_ENABLED)
/**<PRH user identifier for TD-SCDMA ps driver*/
#define PRH_USER_TDS_PS (PRH_USER_FSPEED + 1)
/**< PRH user identifier for PDB driver */
#define PRH_USER_PDB (PRH_USER_TDS_PS + 1)
#else
/**< PRH user identifier for PDB driver */
#define PRH_USER_PDB (PRH_USER_FSPEED + 1)
#endif
/**< PRH user identifier for L1U */
#define PRH_USER_L1U (PRH_USER_PDB + 1)
/**< PRH user identifier for camera driver */
#define PRH_USER_CAMERA (PRH_USER_L1U + 1)
/**< PRH user identifier for keypad driver */
#define PRH_USER_KPD (PRH_USER_CAMERA + 1)
/**< PRH user identifier for graphic accelerator driver */
#define PRH_USER_GPU (PRH_USER_KPD + 1)
/**< PRH user identifier for video accelerator driver */
#define PRH_USER_VPU_DEC (PRH_USER_GPU + 1)
/**< PRH user identifier for video accelerator driver */
#define PRH_USER_VPU_ENC (PRH_USER_VPU_DEC + 1)
/**< PRH user identifier for TOUCHSCREEN driver */
#define PRH_USER_TP (PRH_USER_VPU_ENC + 1)
/**< PRH user identifier for ACCELEROMETER driver */
#define PRH_USER_ACCELEROMETER (PRH_USER_TP + 1)
/**< PRH user identifier for PROXIMITY SENSOR driver */
#define PRH_USER_PROXIMITY_SENSOR (PRH_USER_ACCELEROMETER + 1)
/**< PRH user identifier for GYROSCOPE driver */
#define PRH_USER_GYROSCOPE (PRH_USER_PROXIMITY_SENSOR + 1)
/**< PRH user identifier for display driver */
#define PRH_USER_DISPLAY (PRH_USER_GYROSCOPE + 1)
/**< PRH user identifier for MAGNETOMETER driver */
#define PRH_USER_MAGNETOMETER (PRH_USER_DISPLAY + 1)
/**< PRH user dummy identifier */
#define PRH_USER_DUMMY (PRH_USER_MAGNETOMETER + 1)
/**< End indicator */
#define PRH_USER_NOF_ID (PRH_USER_DUMMY + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_FIRST (0)
/**< PRH peripheral identifier for MMC_CORE_CLK */
#define PRH_PER_MMC_CORE_CLK (PRH_PER_FIRST)
/**< PRH peripheral identifier for DMAC_2CH */
#define PRH_PER_DMAC_2CH (PRH_PER_MMC_CORE_CLK + 1)
/** /TODO remove check yash/< PRH peripheral identifier for DMAC_2CH_2 */
#define PRH_PER_DMAC_2CH_2 (PRH_PER_DMAC_2CH + 1)
/**< PRH peripheral identifier for CEU */
#define PRH_PER_CEU (PRH_PER_DMAC_2CH_2 + 1)
/**< PRH peripheral identifier for GSER */
#define PRH_PER_GSER (PRH_PER_CEU + 1)
/**< PRH peripheral identifier for 3G_COM_RAM */
#define PRH_PER_3G_COMRAM (PRH_PER_GSER + 1)
/**< PRH peripheral identifier for EMACPHY TILE0 */
#define PRH_PER_EMACPHY_TILE0 (PRH_PER_3G_COMRAM + 1)
/**< PRH peripheral identifier for EMACPHY TILE1 */
#define PRH_PER_EMACPHY_TILE1 (PRH_PER_EMACPHY_TILE0 + 1)
/**< PRH peripheral identifier for EMACPHY TILE2 */
#define PRH_PER_EMACPHY_TILE2 (PRH_PER_EMACPHY_TILE1 + 1)
/**< PRH peripheral identifier for EMACPHY TILE3 */
#define PRH_PER_EMACPHY_TILE3 (PRH_PER_EMACPHY_TILE2 + 1)
/**< PRH peripheral identifier for EMACPHY TILE4 */
#define PRH_PER_EMACPHY_TILE4 (PRH_PER_EMACPHY_TILE3 + 1)
/**< PRH peripheral identifier for EMACPHY TILE5 */
#define PRH_PER_EMACPHY_TILE5 (PRH_PER_EMACPHY_TILE4 + 1)
/**< PRH peripheral identifier for EMACPHY TILE6 */
#define PRH_PER_EMACPHY_TILE6 (PRH_PER_EMACPHY_TILE5 + 1)
/**< PRH peripheral identifier for EMACPHY (UL/DL) */
#define PRH_PER_ECOMRAM (PRH_PER_EMACPHY_TILE6 + 1)
/**< PRH peripheral identifier for XB06_DMA */
#define PRH_PER_XB06_DMA (PRH_PER_ECOMRAM + 1)
/**< PRH peripheral identifier for DIG_RF */
#define PRH_PER_DIG_RF (PRH_PER_XB06_DMA + 1)
/**< PRH peripheral identifier for DIG_RF_AUX */
#define PRH_PER_DIG_RF_AUX (PRH_PER_DIG_RF + 1)
/**< PRH peripheral identifier for GSI */
#define PRH_PER_GSI (PRH_PER_DIG_RF_AUX + 1)
/**< PRH peripheral identifier for GSI2 */
#define PRH_PER_GSI2 (PRH_PER_GSI + 1)
/**< PRH peripheral identifier for RF AHB_PER2 bus frequency reduction */
#define PRH_PER_AHB_PER2_RF_BFR (PRH_PER_GSI2 + 1)
/**< PRH peripheral identifier for RF AHB_PER3 bus frequency reduction */
#define PRH_PER_AHB_PER3_RF_BFR (PRH_PER_AHB_PER2_RF_BFR + 1)
/**< PRH peripheral identifier for Modem 2G inner DSP */
#define PRH_PER_DSP_2G_IN (PRH_PER_AHB_PER3_RF_BFR + 1)
/**< PRH peripheral identifier for Modem 2G outer DSP */
#define PRH_PER_DSP_2G_OUT (PRH_PER_DSP_2G_IN + 1)
/**< PRH peripheral identifier for Audio sample DSP */
#define PRH_PER_DSP_AUD_SAMPLE (PRH_PER_DSP_2G_OUT + 1)
/**< PRH peripheral identifier for Audio frame DSP */
#define PRH_PER_DSP_AUD_FRAME (PRH_PER_DSP_AUD_SAMPLE + 1)
/**< PRH peripheral identifier for GUCIPH */
#define PRH_PER_GUCIPH (PRH_PER_DSP_AUD_FRAME + 1)
/**< PRH peripheral identifier for GPS */
#define PRH_PER_GPS (PRH_PER_GUCIPH + 1)
/**< PRH peripheral identifier for GPS INTERNAL */
#define PRH_PER_GPS_INTERNAL (PRH_PER_GPS + 1)
/**< PRH peripheral identifier for AUD_LIB_PSV */
#define PRH_PER_AUD_PSV (PRH_PER_GPS_INTERNAL + 1)
/**< PRH peripheral identifier for AUD_LIB_I2S1 */
#define PRH_PER_I2S1 (PRH_PER_AUD_PSV + 1)
/**< PRH peripheral identifier for AUD_LIB_I2S2 */
#define PRH_PER_I2S2 (PRH_PER_I2S1 + 1)
/**< PRH peripheral identifier for AUD_LIB_XCODEC clock */
#define PRH_PER_XCODEC (PRH_PER_I2S2 + 1)
/**< PRH peripheral identifier for AUD_LIB_AFE volt */
#define PRH_PER_AFE_VOLT (PRH_PER_XCODEC + 1)
/**< PRH peripheral identifier for USB_SS */
#define PRH_PER_USB (PRH_PER_AFE_VOLT + 1)
/**< PRH peripheral identifier for USB AHB_PER3 bus frequency reduction */
#define PRH_PER_AHB_PER3_USB_FS_BFR (PRH_PER_USB + 1)
/**< PRH peripheral identifier for USIF1 */
#define PRH_PER_USIF1 (PRH_PER_AHB_PER3_USB_FS_BFR + 1)
/**< PRH peripheral identifier for USIF2 (not supported) */
#define PRH_PER_USIF2 (PRH_PER_USIF1 + 1)
/**< PRH peripheral identifier for USIF3 */
#define PRH_PER_USIF3 (PRH_PER_USIF2 + 1)
/**< PRH peripheral identifier for USIF5 for Artemis Trace */
#define PRH_PER_3G_USIF (PRH_PER_USIF3 + 1)
/**< PRH peripheral identifier for USIF4 */
#define PRH_PER_USIF4 (PRH_PER_3G_USIF + 1)
/**< PRH peripheral identifier for CAPCOM0 */
#define PRH_PER_CAPCOM0 (PRH_PER_USIF4 + 1)
/**< PRH peripheral identifier for CAPCOM1 */
#define PRH_PER_CAPCOM1 (PRH_PER_CAPCOM0 + 1)
/**< PRH peripheral identifier for CAPCOM2 */
#define PRH_PER_CAPCOM2 (PRH_PER_CAPCOM1 + 1)
/**< PRH peripheral identifier for DMAC_8CH */
#define PRH_PER_DMAC_8CH (PRH_PER_CAPCOM2 + 1)
/**< PRH peripheral identifier for DMAC_8CH_2 */
#define PRH_PER_DMAC_8CH_2 (PRH_PER_DMAC_8CH + 1)
/**< PRH peripheral identifier for I2C1 */
#define PRH_PER_I2C1 (PRH_PER_DMAC_8CH_2 + 1)
/**< PRH peripheral identifier for I2C2 */
#define PRH_PER_I2C2 (PRH_PER_I2C1 + 1)
/**< PRH peripheral identifier for I2C3  */
#define PRH_PER_I2C3 (PRH_PER_I2C2 + 1)
/**< PRH peripheral identifier for I2C4  */
#define PRH_PER_I2C4 (PRH_PER_I2C3 + 1)
/**< PRH peripheral identifier for I2C5  */
#define PRH_PER_I2C5 (PRH_PER_I2C4 + 1)
/**< PRH peripheral identifier for PCL */
#define PRH_PER_PCL (PRH_PER_I2C5 + 1)
/**< PRH peripheral identifier for GPTU0 */
#define PRH_PER_GPTU0 (PRH_PER_PCL + 1)
/**< PRH peripheral identifier for GPTU1 */
#define PRH_PER_GPTU1 (PRH_PER_GPTU0 + 1)
/**< PRH peripheral identifier for STM */
#define PRH_PER_STM (PRH_PER_GPTU1 + 1)
/**< PRH peripheral identifier for RTC */
#define PRH_PER_RTC (PRH_PER_STM + 1)
/**< PRH peripheral identifier for TMSU */
#define PRH_PER_TSMU (PRH_PER_RTC + 1)
/**< PRH peripheral identifier for UICC_VCC */
#define PRH_PER_UICC_VCC (PRH_PER_TSMU + 1)
/**< PRH peripheral identifier for UICC_CLK */
#define PRH_PER_UICC_CLK (PRH_PER_UICC_VCC + 1)
/**< PRH peripheral identifier for UICC2_VCC */
#define PRH_PER_UICC2_VCC (PRH_PER_UICC_CLK + 1)
/**< PRH peripheral identifier for UICC2_CLK */
#define PRH_PER_UICC2_CLK (PRH_PER_UICC2_VCC + 1)
/**< PRH peripheral identifier for SDMMC1_VOLT */
#define PRH_PER_SDMMC1_VOLT (PRH_PER_UICC2_CLK + 1)
/**< PRH peripheral identifier for SDMMC1_CLK */
#define PRH_PER_SDMMC1_CLK (PRH_PER_SDMMC1_VOLT + 1)
/**< PRH peripheral identifier for SDMMC1_POW */
#define PRH_PER_SDMMC1_POW (PRH_PER_SDMMC1_CLK + 1)
/**< PRH peripheral identifier for SDMMC1_PSV */
#define PRH_PER_SDMMC1_PSV (PRH_PER_SDMMC1_POW + 1)
/**< PRH peripheral identifier for SDIO_VOLT */
#define PRH_PER_SDIO_VOLT (PRH_PER_SDMMC1_PSV + 1)
/**< PRH peripheral identifier for SDIO_CLK */
#define PRH_PER_SDIO_CLK (PRH_PER_SDIO_VOLT + 1)
/**< PRH peripheral identifier for SDIO_POW */
#define PRH_PER_SDIO_POW (PRH_PER_SDIO_CLK + 1)
/**< PRH peripheral identifier for SDIO_PSV */
#define PRH_PER_SDIO_PSV (PRH_PER_SDIO_POW + 1)
/**< PRH peripheral identifier for ST_ARB */
#define PRH_PER_ST_ARB (PRH_PER_SDIO_PSV + 1)
/**< PRH peripheral identifier for On-chip trace */
#define PRH_PER_ST_OCT (PRH_PER_ST_ARB + 1)
/**< PRH peripheral identifier for On-chip trace */
#define PRH_PER_ST_OCT_DVC (PRH_PER_ST_OCT + 1)
/**< PRH peripheral identifier for MTM1 */
#define PRH_PER_ST_MTM1 (PRH_PER_ST_OCT_DVC + 1)
/**< PRH peripheral identifier for MTM2 */
#define PRH_PER_ST_MTM2 (PRH_PER_ST_MTM1 + 1)
/**< PRH peripheral identifier for MTM2 PAD */
#define PRH_PER_ST_MTM2_PAD (PRH_PER_ST_MTM2 + 1)
/**< PRH peripheral identifier for ST_MON */
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
/**< PRH peripheral identifier for ST_MON_SB 13 */
#define PRH_PER_ST_MON_SB_13 (PRH_PER_ST_MON_SB_12 + 1)
/**< PRH peripheral identifier for ST_MON_SB 14 */
#define PRH_PER_ST_MON_SB_14 (PRH_PER_ST_MON_SB_13 + 1)
/**< PRH peripheral identifier for ST_MON_SB 16 */
#define PRH_PER_ST_MON_SB_16 (PRH_PER_ST_MON_SB_14 + 1)
/**< PRH peripheral identifier for ETM3G */
#define PRH_PER_ETM3G (PRH_PER_ST_MON_SB_16 + 1)
/**< PRH peripheral identifier for CST */
#define PRH_PER_CST (PRH_PER_ETM3G + 1)
/**< PRH peripheral identifier for power save */
#define PRH_PER_FSPEED_PSV (PRH_PER_CST + 1)
/**< PRH peripheral identifier for low voltage */
#define PRH_PER_FSPEED_LOW_VOLT (PRH_PER_FSPEED_PSV + 1)
#if defined(TDSCDMA_ENABLED)
/**< PRH peripheral identifier for TD-SCDMA coprocessor */
#define PRH_PER_EXT_XG639 (PRH_PER_FSPEED_LOW_VOLT + 1)
/**< PRH peripheral identifier for 3G subsystem */
#define PRH_PER_3G (PRH_PER_EXT_XG639 + 1)
/**< PRH peripheral identifier for TDIP */
#define PRH_PER_TDIP (PRH_PER_3G + 1)
/**< PRH peripheral identifier for TDS_COMRAM */
#define PRH_PER_TDS_COMRAM (PRH_PER_TDIP + 1)
/**< PRH user identifier for PDB driver */
#define PRH_PER_PDB (PRH_PER_TDS_COMRAM + 1)
#else
/**< PRH user identifier for PDB driver */
#define PRH_PER_PDB (PRH_PER_FSPEED_LOW_VOLT + 1)
#endif
/**< PRH user identifier for LTE driver */
#define PRH_PER_LTE (PRH_PER_PDB + 1)
/**< PRH peripheral identifier for camera interface */
#define PRH_PER_CIF (PRH_PER_LTE + 1)
/**< PRH peripheral identifier for display controller */
#define PRH_PER_DCC (PRH_PER_CIF + 1)
/**< PRH peripheral identifier for keypad */
#define PRH_PER_KPD (PRH_PER_DCC + 1)
/**< PRH peripheral identifier for Graphic Accelerator */
#define PRH_PER_GPU (PRH_PER_KPD + 1)
/**< PRH peripheral identifier for Video Accelerator decoder */
#define PRH_PER_VIDEO_DEC (PRH_PER_GPU + 1)
/**< PRH peripheral identifier for Video Accelerator encoder */
#define PRH_PER_VIDEO_ENC (PRH_PER_VIDEO_DEC + 1)
/**< PRH peripheral identifier for touchscreen */
#define PRH_PER_EXT_TP (PRH_PER_VIDEO_ENC + 1)
/**< PRH peripheral identifier for accelerometer */
#define PRH_PER_EXT_ACCELEROMETER (PRH_PER_EXT_TP + 1)
/**< PRH peripheral identifier for proximity_sensor */
#define PRH_PER_EXT_PROXIMITY_SENSOR (PRH_PER_EXT_ACCELEROMETER + 1)
/**< PRH peripheral identifier for gyroscope */
#define PRH_PER_EXT_GYROSCOPE (PRH_PER_EXT_PROXIMITY_SENSOR + 1)
/**< PRH peripheral identifier for external display */
#define PRH_PER_EXT_DISPLAY (PRH_PER_EXT_GYROSCOPE + 1)
/**< PRH peripheral identifier for EMMC_VOLT */
#define PRH_PER_VEMMC_VOLT (PRH_PER_EXT_DISPLAY + 1)
/**< PRH peripheral identifier for EMMC_CLK */
#define PRH_PER_EMMC_CLK (PRH_PER_VEMMC_VOLT + 1)
/**< PRH peripheral identifier for EMMC_POW */
#define PRH_PER_EMMC_POW (PRH_PER_EMMC_CLK + 1)
/**< PRH peripheral identifier for EMMC_PSV */
#define PRH_PER_EMMC_PSV (PRH_PER_EMMC_POW + 1)
/**< PRH peripheral identifier for external primary camera */
#define PRH_PER_EXT_PRIM_CAM (PRH_PER_EMMC_PSV + 1)
/**< PRH peripheral identifier for external secondary camera */
#define PRH_PER_EXT_SEC_CAM (PRH_PER_EXT_PRIM_CAM + 1)
/**< PRH peripheral identifier for magnetometer */
#define PRH_PER_EXT_MAGNETOMETER (PRH_PER_EXT_SEC_CAM + 1)
/**< PRH user identifier for L1U */
#define PRH_PER_3G_SUBSYS (PRH_PER_EXT_MAGNETOMETER + 1)
/**< PRH peripheral dummy identifier  */
#define PRH_PER_DUMMY (PRH_PER_3G_SUBSYS + 1)
/**< End indicator */
#define PRH_PER_NOF_ID (PRH_PER_DUMMY + 1)

/**< Start indicator (reserved for PRH driver) */
#define PRH_PER_TYPE_FIRST (0)
/**< PRH peripheral identifier for generic peripheral */
#define PRH_PER_TYPE_GENERIC (PRH_PER_TYPE_FIRST)
/**< PRH peripheral identifier for MMC_CORE_CLK peripheral */
#define PRH_PER_TYPE_MMC_CORE_CLK (PRH_PER_TYPE_GENERIC + 1)
/**< PRH peripheral identifier for 3G_COMRAM peripheral */
#define PRH_PER_TYPE_3G_COMRAM (PRH_PER_TYPE_MMC_CORE_CLK + 1)
/**< PRH peripheral identifier for 3G EMACPHY peripheral */
#define PRH_PER_TYPE_EMACPHY (PRH_PER_TYPE_3G_COMRAM + 1)
/**< PRH peripheral identifier for DMA peripheral */
#define PRH_PER_TYPE_DMA (PRH_PER_TYPE_EMACPHY + 1)
/**< PRH peripheral identifier for DSP peripheral */
#define PRH_PER_TYPE_DSP (PRH_PER_TYPE_DMA + 1)
/**< PRH peripheral identifier for I2C peripheral */
#define PRH_PER_TYPE_I2C (PRH_PER_TYPE_DSP + 1)
/**< PRH peripheral identifier for SDMMC1_VOLT peripheral */
#define PRH_PER_TYPE_MMCSD_VOLT (PRH_PER_TYPE_I2C + 1)
/**< PRH peripheral identifier for SDMMC1_CLK peripheral */
#define PRH_PER_TYPE_MMCSD_CLK (PRH_PER_TYPE_MMCSD_VOLT + 1)
/**< PRH peripheral identifier for USB SS peripheral */
#define PRH_PER_TYPE_USB (PRH_PER_TYPE_MMCSD_CLK + 1)
/**< PRH peripheral identifier for USIF peripheral */
#define PRH_PER_TYPE_USIF (PRH_PER_TYPE_USB + 1)
/**< PRH peripheral identifier for CAPCOM peripheral */
#define PRH_PER_TYPE_CAPCOM (PRH_PER_TYPE_USIF + 1)
/**< PRH peripheral identifier for UICC peripheral */
#define PRH_PER_TYPE_UICC_VCC (PRH_PER_TYPE_CAPCOM + 1)
/**< PRH peripheral identifier for UICC peripheral */
#define PRH_PER_TYPE_UICC_CLK (PRH_PER_TYPE_UICC_VCC + 1)
/**< PRH peripheral identifier for TRACE peripheral */
#define PRH_PER_TYPE_TRACE (PRH_PER_TYPE_UICC_CLK + 1)
/**< PRH peripheral identifier for PDB peripheral */
#define PRH_PER_TYPE_PDB (PRH_PER_TYPE_TRACE + 1)
/**< PRH peripheral identifier for DCC peripheral */
#define PRH_PER_TYPE_DCC (PRH_PER_TYPE_PDB + 1)
/**< PRH peripheral identifier for CIF peripheral */
#define PRH_PER_TYPE_CIF (PRH_PER_TYPE_DCC + 1)
/**< PRH peripheral identifier for KPD peripheral */
#define PRH_PER_TYPE_KPD (PRH_PER_TYPE_CIF + 1)
/**< PRH peripheral identifier for GPU peripheral */
#define PRH_PER_TYPE_GPU (PRH_PER_TYPE_KPD + 1)
/**< PRH peripheral identifier for VIDEO DEC peripheral */
#define PRH_PER_TYPE_VIDEO_DEC (PRH_PER_TYPE_GPU + 1)
/**< PRH peripheral identifier for VIDEO ENC peripheral */
#define PRH_PER_TYPE_VIDEO_ENC (PRH_PER_TYPE_VIDEO_DEC + 1)
/**< PRH peripheral identifier for TDIP peripheral */
#define PRH_PER_TYPE_TDIP (PRH_PER_TYPE_VIDEO_ENC + 1)
/**< PRH peripheral identifier for 3G Subsystem peripheral */
#define PRH_PER_TYPE_3G_SUBSYS (PRH_PER_TYPE_TDIP + 1)
/**< End indicator (reserved for PRH driver) */
#define PRH_PER_NOF_TYPE (PRH_PER_TYPE_3G_SUBSYS + 1)

