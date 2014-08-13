#ifndef _IUI_FM_MACRO_SUPPORT_H
#define _IUI_FM_MACRO_SUPPORT_H

/* This file converts each CONFIG_IUI_FM_<macro> definition from the iui_fm
   Kconfig into a boolean (0 or 1) that can be used to populate a table
   or bitfield in "iui_fm.c". */

#ifdef CONFIG_IUI_FM_EMMC
#define IUI_FM_EMMC_SUPPORT 1
#else  /* CONFIG_IUI_FM_EMMC */
#define IUI_FM_EMMC_SUPPORT 0
#endif /* CONFIG_IUI_FM_EMMC */

#ifdef CONFIG_IUI_FM_CLASS_D
#define IUI_FM_CLASS_D_SUPPORT 1
#else  /* CONFIG_IUI_FM_CLASS_D */
#define IUI_FM_CLASS_D_SUPPORT 0
#endif /* CONFIG_IUI_FM_CLASS_D */

#ifdef CONFIG_IUI_FM_PMU_CP
#define IUI_FM_PMU_CP_SUPPORT 1
#else  /* CONFIG_IUI_FM_PMU_CP */
#define IUI_FM_PMU_CP_SUPPORT 0
#endif /* CONFIG_IUI_FM_PMU_CP */

#ifdef CONFIG_IUI_FM_MS_CP
#define IUI_FM_MS_CP_SUPPORT 1
#else  /* CONFIG_IUI_FM_MS_CP */
#define IUI_FM_MS_CP_SUPPORT 0
#endif /* CONFIG_IUI_FM_MS_CP */

#ifdef CONFIG_IUI_FM_DCDC
#define IUI_FM_DCDC_SUPPORT 1
#else  /* CONFIG_IUI_FM_DCDC */
#define IUI_FM_DCDC_SUPPORT 0
#endif /* CONFIG_IUI_FM_DCDC */

#ifdef CONFIG_IUI_FM_IDI
#define IUI_FM_IDI_SUPPORT 1
#else  /* CONFIG_IUI_FM_IDI */
#define IUI_FM_IDI_SUPPORT 0
#endif /* CONFIG_IUI_FM_IDI */

#ifdef CONFIG_IUI_FM_WLAN
#define IUI_FM_WLAN_SUPPORT 1
#else  /* CONFIG_IUI_FM_WLAN */
#define IUI_FM_WLAN_SUPPORT 0
#endif /* CONFIG_IUI_FM_WLAN */

#ifdef CONFIG_IUI_FM_FMR
#define IUI_FM_FMR_SUPPORT 1
#else  /* CONFIG_IUI_FM_FMR */
#define IUI_FM_FMR_SUPPORT 0
#endif /* CONFIG_IUI_FM_FMR */

#ifdef CONFIG_IUI_FM_BT
#define IUI_FM_BT_SUPPORT 1
#else  /* CONFIG_IUI_FM_BT */
#define IUI_FM_BT_SUPPORT 0
#endif /* CONFIG_IUI_FM_BT */

#ifdef CONFIG_IUI_FM_GNSS
#define IUI_FM_GNSS_SUPPORT 1
#else  /* CONFIG_IUI_FM_GNSS */
#define IUI_FM_GNSS_SUPPORT 0
#endif /* CONFIG_IUI_FM_GNSS */



#endif /* _IUI_FM_MACRO_SUPPORT_H */
