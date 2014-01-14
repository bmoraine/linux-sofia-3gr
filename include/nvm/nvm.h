/*
 ****************************************************************
 *
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************
 */

#if defined(__cplusplus)
    extern "C" {
#endif

#if !defined _NVM_H
#define _NVM_H


/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/
#include "bastypes.h"

/*! \file */

/****************************************************************************************/
/* CONSTANTS / DEFINES                                                            BEGIN */
/****************************************************************************************/
/*!
 *  \defgroup NVM_constants Constants
 *  \ingroup NVM
 *  \{
 *  These constants are used in the interface of the NVM driver.
 */

/*
 *  These strings are used in GTI and NVM.
 *  It is recommended that the order is the same as the order of Group ID's.
 */
#define NVM_GTI_NAME_CAL_URF      "cal_urf"       /* NVM_STA_CAL_EXT_RF3G_UE1 */
#define NVM_GTI_NAME_CAL_GRF      "cal_grf"       /* NVM_STA_CAL_EXT_RF2G_UE1 */
#define NVM_GTI_NAME_CAL_RF_UE2   "cal_rf_ue2"    /* NVM_STA_CAL_EXT_RF_UE2 */
#define NVM_GTI_NAME_CAL_RF_LU    "cal_rf_lu"     /* NVM_STA_CAL_EXT_RF_LU */
#define NVM_GTI_NAME_CAL_2G_DRV   "cal_2g_drv"    /* NVM_STA_CAL_EXT_2G_DRV */
#define NVM_GTI_NAME_CAL_3G_DRV   "cal_3g_drv"    /* NVM_STA_CAL_EXT_3G_DRV */
#define NVM_GTI_NAME_CAL_4G_DRV   "cal_4g_drv"    /* NVM_STA_CAL_EXT_4G_DRV */
#define NVM_GTI_NAME_CAL_PROD     "cal_prodparm"  /* NVM_STA_CAL_PROD_PARMS */
#define NVM_GTI_NAME_CAL_AUD      "cal_aud"       /* NVM_STA_CAL_AUD */
#define NVM_GTI_NAME_CAL_RF_S4G   "cal_s4g_lut"    /* NVM_STA_CAL_EXT_RF_S4G */
#define NVM_GTI_NAME_FIX_URF      "fix_urf"       /* NVM_STA_FIX_EXT_RF3G_UE1 */
#define NVM_GTI_NAME_FIX_GRF      "fix_grf"       /* NVM_STA_FIX_EXT_RF2G_UE1 */
#define NVM_GTI_NAME_FIX_RF_UE2   "fix_rf_ue2"    /* NVM_STA_FIX_EXT_RF_UE2 */
#define NVM_GTI_NAME_FIX_2G_DRV   "fix_2g_drv"    /* NVM_STA_FIX_EXT_2G_DRV */
#define NVM_GTI_NAME_FIX_3G_DRV   "fix_3g_drv"    /* NVM_STA_FIX_EXT_3G_DRV */
#define NVM_GTI_NAME_FIX_4G_DRV   "fix_4g_drv"    /* NVM_STA_FIX_EXT_4G_DRV */
#define NVM_GTI_NAME_FIX_AUD      "fix_aud"       /* NVM_STA_FIX_AUD */
#define NVM_GTI_NAME_FIX_TONE     "fix_tone"      /* NVM_STA_FIX_TONE */
#define NVM_GTI_NAME_FIX_XL1      "fix_xl1"       /* NVM_STA_FIX_XL1 */
#define NVM_GTI_NAME_FIX_SELFTEST "fix_selftest"  /* NVM_GTI_NAME_FIX_SELFTEST */
#define NVM_GTI_NAME_FIX_RF_S4G   "fix_s4g_lut"    /* NVM_STA_FIX_EXT_RF_S4G */
#define NVM_GTI_NAME_FIX_RF_S4G_SETCAL \
                                  "fix_s4g_lut_setcal" /* NVM_STA_FIX_EXT_RF_S4G_SETCAL */
#define NVM_GTI_NAME_DYN_AUD_LIB  "dyn_aud_lib"   /* NVM_DYN_AUD_LIB */
#define NVM_GTI_NAME_DYN_BMMON_IMMEDIATE \
                                  "dyn_bmmon_immediate" /* NVM_DYN_BMMON_IMMEDIATE */
#define NVM_GTI_NAME_DYN_BMMON_DEFERRED \
                                  "dyn_bmmon_deferred" /* NVM_DYN_BMMON_DEFERRED */

#define NVM_UNUSED_VER_REV        (0xFFFFFFFF)    /* Constant to be used with version/revision number */


/*!
 * \}
 */
/****************************************************************************************/
/* CONSTANTS                                                                        END */
/****************************************************************************************/


/****************************************************************************************/
/* ENUMERATIONS                                                                   BEGIN */
/****************************************************************************************/
/*!
 * \defgroup NVM_enumerations Enumerations
 *  \ingroup NVM
 *  \{
 *  These enumerations are used in the interface of the NVM driver.
 */


/*!
 * \brief This enumeration contains the Group ID's for static and dynamic.
 * Group ID SHALL be 32 bits as it is part of sector layout.
 * The format is: 0xTGGGSSSS
 * T: Type, which is either calib (1), static (2) or dynamic (3)
 * G: Group, which is the group's ID
 * S: Subgroup, which is defined by the user of the group
 *
 *When NVM_COMMON_SWAP_LAS:
 * The format is: 0xTIGGSSSS
 * T: Type, which is either calib (1), static (2) or dynamic (3)
 * I: Info, Commonswap (8) or eraseinfo (4)
 * G: Group, which is the group's ID
 * S: Subgroup, which is defined by the user of the group
 */
typedef enum
{
  /* Reserved to the first sector in every LAS containing the Swap Update Number */
   NVM_SWAP_UPDATE_NO_SECTOR           = 0x00010000    //!< Sector layout format V1.

  /* Used in LAS reduction scheme (NVM_COMMON_SWAP_LAS) */
  ,NVM_SWAP_UPDATE_NO_SECTOR_MASK      = 0x08000000    //!< Mask to check if this is common swap sector
  ,NVM_SWAP_UPDATE_NO_SECTOR_STA_CAL   = 0x18000000    //!< Reserved to the first sector in every LAS containing the Swap Update Number. No separate struct. Used when 1 common swap sector
  ,NVM_SWAP_UPDATE_NO_SECTOR_STA_FIX   = 0x28000000    //!< Reserved to the first sector in every LAS containing the Swap Update Number. No separate struct. Used when 1 common swap sector
  ,NVM_SWAP_UPDATE_NO_SECTOR_DYN       = 0x38000000    //!< Reserved to the first sector in every LAS containing the Swap Update Number. No separate struct. Used when 1 common swap sector
  ,NVM_ERASEINFO_MASK                  = 0x04000000    //!< Mask to check if this is eraseinfo sector
  ,NVM_ERASEINFO_STA_CAL               = 0x14A1F875    //!< Written by Flashtool to request erase STA_CAL
  ,NVM_ERASEINFO_STA_FIX               = 0x24A1F875    //!< Written by Flashtool to request erase STA_FIX
  ,NVM_ERASEINFO_DYN                   = 0x34A1F875    //!< Written by Flashtool to request erase DYN

  ,NVM_STA_CAL_EEP                     = 0x10010000    //!< Static calib. for EEP.
  ,NVM_STA_CAL_XCUST                   = 0x10020000    //!< Static calib. for XDRV.
  ,NVM_STA_CAL_IBAT                    = 0x10034900    //!< Static calib. for IBAT (battery current).
  ,NVM_STA_CAL_ICHG                    = 0x10034901    //!< Static calib. for ICHG (charger current).
  ,NVM_STA_CAL_IDBAT                   = 0x10034401    //!< Static calib. for IDBAT (battery identification).
  ,NVM_STA_CAL_IDPCB                   = 0x10034402    //!< Static calib. for IDPCB (PCB identification).
  ,NVM_STA_CAL_IHW                     = 0x10034902    //!< Static calib. for IHW (BB current).
  ,NVM_STA_CAL_TBAT                    = 0x10035400    //!< Static calib. for TBAT (battery temperature).
  ,NVM_STA_CAL_TBB                     = 0x10035401    //!< Static calib. for TBB (BB temperature).
  ,NVM_STA_CAL_TPCB                    = 0x10035402    //!< Static calib. for TPCB (PCB temperature).
  ,NVM_STA_CAL_TRF                     = 0x10035403    //!< Static calib. for TRF (RF temperature).
  ,NVM_STA_CAL_TRFEXT                  = 0x10035404    //!< Static calib. for TRF EXT (RF temperature).
  ,NVM_STA_CAL_VCHG                    = 0x10035500    //!< Static calib. for VCHG (charger voltage).
  ,NVM_STA_CAL_VSYS                    = 0x10035501    //!< Static calib. for VSYS (system/battery voltage).
  ,NVM_STA_CAL_MEAS                    = 0x10035502    //!< Static calib. for MEAS (ADC channels).
  ,NVM_STA_CAL_URF_GROUP               = 0x10040000    //!< Static calib. for 3G URF.
  ,NVM_STA_CAL_SEC                     = 0x10050000    //!< Static calib. for SEC.
  ,NVM_STA_CAL_RF                      = 0x10060000    //!< Static calib. for 2G RF.
  ,NVM_STA_CAL_URF_BU_GROUP            = 0x10070000    //!< Static calib. for 3G URF backup.
  ,NVM_STA_CAL_PROD_PARMS              = 0x10080000    //!< Static calib. for PRODUCTION PARAMS.
  ,NVM_STA_CAL_RF_BU_GROUP             = 0x10090000    //!< Static calib. for 2G RF backup.
  ,NVM_STA_CAL_CSI                     = 0x100A0000    //!< Static calib. for CSI.
  ,NVM_STA_CAL_3IN1VIBRA               = 0x100B0000    //!< Static calib. for MDF.
  ,NVM_STA_CAL_NVM_INTERN              = 0x100F0000    //!< Static calib. for NVM intern status
  ,NVM_STA_CAL_AUD                     = 0x10300000    //!< Static calib. for Audio.
  ,NVM_STA_CAL_EXT_RF_S4G              = 0x10300001    //!< Static calib. for RF_S4G.
  ,NVM_STA_CAL_BOOTCORE                = 0x10310000    //!< Static calib. for Bootcore.
  ,NVM_STA_CAL_EMGLIST                 = 0x10311000    //!< Static calib. for Bootcore.
  ,NVM_STA_CAL_CALIBSIG                = 0x10312000    //!< Static calib. for Bootcore.
  ,NVM_STA_CAL_EXT_RF3G_UE1            = 0x10320000    //!< Static calib. for RF3G_UE1.
  ,NVM_STA_CAL_EXT_RF2G_UE1            = 0x10330000    //!< Static calib. for RF2G_UE1.
  ,NVM_STA_CAL_EXT_RF_UE2_BU           = 0x10340000    //!< Static calib. for RF UE2 BU.
  ,NVM_STA_CAL_EXT_RF_UE2              = 0x10350000    //!< Static calib. for RF UE2.
  ,NVM_STA_CAL_EXT_RF_LU               = 0x10350001    //!< Static calib. for RF LU.
  ,NVM_STA_CAL_EXT_2G_DRV              = 0x10351000    //!< Static calib. for 2G_DRV.
  ,NVM_STA_CAL_EXT_3G_DRV              = 0x10352000    //!< Static calib. for 3G_DRV.
  ,NVM_STA_CAL_EXT_4G_DRV              = 0x10353000    //!< Static calib. for 4G_DRV.
  ,NVM_STA_CAL_BT                      = 0x10360000    //!< Static calib. for BT.
  ,NVM_STA_CAL_CPS                     = 0x10370000    //!< Static calib. for CPS.
  ,NVM_STA_CAL_UICC                    = 0x10380000    //!< Static calib. for UICC.
  ,NVM_STA_CAL_TDS_PS                  = 0x10390000    //!< Static calib. for TDS_PS.
  ,NVM_STA_CAL_IDC                     = 0x103A0000    //!< Static calib. for IDC.
  ,NVM_STA_CAL_ADC_SENSORS             = 0x103B0000    //!< Static calib. for ADC sensors.

  ,NVM_STA_CAL_VARSIZE_START           = 0x1E000000    //!< Static calib. VARSIZE start of range
   /* Reserved area for VARSIZE groups */
  ,NVM_STA_CAL_VARSIZE_STOP            = 0x1EFFFFFF    //!< Static calib. VARSIZE stop of range
  ,NVM_STA_CAL_UTA_1                   = 0x1F000000    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_2                   = 0x1F000001    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_3                   = 0x1F000002    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_4                   = 0x1F000003    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_5                   = 0x1F000004    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_6                   = 0x1F000005    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_7                   = 0x1F000006    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_8                   = 0x1F000007    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_9                   = 0x1F000008    //!< Static calib. for UTA_NVM
  ,NVM_STA_CAL_UTA_10                  = 0x1F000009    //!< Static calib. for UTA_NVM
#if defined CONFIG_NVMTEST
  ,NVM_STA_CAL_GROUP_0                 = 0x10001000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_1                 = 0x10002000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_2                 = 0x10003000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_3                 = 0x10004000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_4                 = 0x10005000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_5                 = 0x10006000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_6                 = 0x10007000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_7                 = 0x10008000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_8                 = 0x10009000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_9                 = 0x1000A000    //!< Static calib. test group.
  ,NVM_STA_CAL_GROUP_10                = 0x1000B000    //!< Static calib. test group.
#endif

  ,NVM_STA_FIX_EEP                     = 0x20010000    //!< Static fixed for EEP.
  ,NVM_STA_FIX_AUD                     = 0x20020000    //!< Static fixed for Audio.
  ,NVM_STA_FIX_STARTUP                 = 0x20030000    //!< Static fixed for Startup.
  ,NVM_STA_FIX_COREDUMP                = 0x20040000    //!< Static fixed for Coredump data.
  ,NVM_STA_FIX_TONE                    = 0x20050000    //!< Static fixed for Tone.
  ,NVM_STA_FIX_URF_GROUP               = 0x20060000    //!< Static fixed for 3G URF.
  ,NVM_STA_FIX_SEC                     = 0x20070000    //!< Static fixed for SEC.
  ,NVM_STA_FIX_RF                      = 0x20080000    //!< Static fixed for 2G RF.
  ,NVM_STA_FIX_USB                     = 0x20090000    //!< Static fixed for USB.
  ,NVM_STA_FIX_CPS                     = 0x200A0000    //!< Static fixed for CPS.
  ,NVM_STA_FIX_BMBATDRV                = 0x200B0000    //!< Static fixed for (not Ice3) BATT.
  ,NVM_STA_FIX_BMMON                   = 0x200C0000    //!< Static fixed for (not Ice3) BATT MON.
  ,NVM_STA_FIX_PROD_PARMS              = 0x200D0000    //!< Static fixed for PRODUCTION PARAMS.
  ,NVM_STA_FIX_EXT_RF3G_UE1            = 0x200E0000    //!< Static fixed for RF3G_UE1.
  ,NVM_STA_FIX_EXT_RF2G_UE1            = 0x200F0000    //!< Static fixed for RF2G_UE1.
  ,NVM_STA_FIX_EXT_RF_UE2              = 0x20100000    //!< Static fixed for RF_UE2.
  ,NVM_STA_FIX_EXT_2G_DRV              = 0x20110000    //!< Static fixed for 2G_DRV.
  ,NVM_STA_FIX_EXT_3G_DRV              = 0x20120000    //!< Static fixed for 3G_DRV.
  ,NVM_STA_FIX_EXT_4G_DRV              = 0x20130000    //!< Static fixed for 4G_DRV.
  ,NVM_STA_FIX_AFE                     = 0x20200000    //!< Static fixed for AFE.
  ,NVM_STA_FIX_FMR                     = 0x20210000    //!< Static fixed for FMR (EHS2AGR).
  ,NVM_STA_FIX_XL1                     = 0x20220000    //!< Static fixed for XL1
  ,NVM_STA_FIX_UICC                    = 0x20230000    //!< Static fixed for UICC
  ,NVM_STA_FIX_BT                      = 0x20240000    //!< Static fixed for BT
  ,NVM_STA_FIX_CPS_FEAT_CONFIG         = 0x20250000    //!< Static fixed for CPS FEAT CONFIG
  ,NVM_STA_FIX_SELFTEST                = 0x20260000    //!< Static fixed for SELFTEST
  ,NVM_STA_FIX_BOOTCORE_DUMMY          = 0x20270000    //!< Dummy for bootcore reader
  ,NVM_STA_FIX_EXT_RF_S4G              = 0x20280000    //!< Static fixed for RF_S4G.
  ,NVM_STA_FIX_EXT_RF_S4G_SETCAL       = 0x20281000    //!< Static fixed for RF_S4G_SETCAL
  ,NVM_STA_FIX_CSI_1                   = 0x20290000    //!< Static fixed. for user customization.
  ,NVM_STA_FIX_CSI_2                   = 0x202A0000    //!< Static fixed. for user customization.
  ,NVM_STA_FIX_WLAN                    = 0x202B0000    //!< Static fixed. for WLAN.
  ,NVM_STA_FIX_ACM                     = 0x202C0000    //!< Static fixed for ACM
  ,NVM_STA_FIX_VARSIZE_START           = 0x2E000000    //!< Static fixed VARSIZE start of range
   /* Reserved area for VARSIZE groups */
  ,NVM_STA_FIX_VARSIZE_STOP            = 0x2EFFFFFF    //!< Static fixed VARSIZE stop of range
  ,NVM_STA_FIX_UTA_1                   = 0x2F000000    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_2                   = 0x2F000001    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_3                   = 0x2F000002    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_4                   = 0x2F000003    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_5                   = 0x2F000004    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_6                   = 0x2F000005    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_7                   = 0x2F000006    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_8                   = 0x2F000007    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_9                   = 0x2F000008    //!< Static fixed for UTA_NVM
  ,NVM_STA_FIX_UTA_10                  = 0x2F000009    //!< Static fixed for UTA_NVM
#if defined CONFIG_NVMTEST
  ,NVM_STA_FIX_GROUP_0                 = 0x20001000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_1                 = 0x20002000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_2                 = 0x20003000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_3                 = 0x20004000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_4                 = 0x20005000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_5                 = 0x20006000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_6                 = 0x20007000    //!< Static fixed test group.
  ,NVM_STA_FIX_GROUP_7                 = 0x20008000    //!< Static fixed test group.
#endif

  ,NVM_DYN_EEP                         = 0x30010000    //!< Dynamic for EEP.
  ,NVM_DYN_RTC                         = 0x30020000    //!< Dynamic for RTC.
  ,NVM_DYN_DAM_RTC_PARMS               = 0x30020001    //!< Dynamic for Quantum RTC.
  ,NVM_DYN_LLT                         = 0x30030000    //!< Dynamic for LLT.
  ,NVM_DYN_SEC                         = 0x30040000    //!< Dynamic for SEC.
  ,NVM_DYN_ADR                         = 0x30050000    //!< Dynamic for AGPS.
  ,NVM_DYN_AGPS                        = 0x30060000    //!< Dynamic for AGPS BBData.
  ,NVM_DYN_TRAP                        = 0x30070000    //!< Dynamic for TRAP data.
  ,NVM_DYN_SAH                         = 0x30071000    //!< Dynamic for SAH data.
  ,NVM_DYN_DAM_SCU_PARAMS              = 0x30080000    //!< Dynamic for Quantum DAM SCU.
  ,NVM_DYN_CAT_NVM_PARAMS              = 0x30090000    //!< Dynamic for EHS2AGR C-AT.
  ,NVM_DYN_CPS                         = 0x300A0000    //!< Dynamic for CPS.
  ,NVM_DYN_CPS_PSSI                    = 0x300A1000    //!< Dynamic for CPS PSSI.
  ,NVM_DYN_BMCHRDRV                    = 0x300B0000    //!< Dynamic for (not Ice3) BATT.
  /* Reserved for old NVM_DYN_PROD_PARMS = 0x300C0000 */
  ,NVM_DYN_PAM_AT_NVM_PARAMS           = 0x300D0000    //!< Dynamic for Quantum PAM AT.
  ,NVM_DYN_PAM_CALL_PROFILE_PARAMS     = 0x300E0000    //!< Dynamic for Quantum PAM CALL PROFILE.
  ,NVM_DYN_PAM_NET_PROFILE_PARAMS      = 0x300F0000    //!< Dynamic for Quantum PAM NET PROFILE.
  ,NVM_DYN_PAM_SMS_PROFILE_PARAMS      = 0x30100000    //!< Dynamic for Quantum PAM SMS PROFILE.
  ,NVM_DYN_COREDUMP                    = 0x30110000    //!< Dynamic for Coredump.
  ,NVM_DYN_CSI                         = 0x30200000    //!< Dynamic for CSI.
  ,NVM_DYN_AFC                         = 0x30400000    //!< Dynamic for AFC, (rf).
  ,NVM_DYN_GDD                         = 0x30500000    //!< Dynamic for graphics driver (GDD).
  ,NVM_DYN_I2S                         = 0x30600000    //!< Dynamic for I2S module (Audio Modem).
  ,NVM_DYN_PAM_GSS_PROFILE_PARAMS      = 0x30610000    //!< Dynamic for GSS server.
  ,NVM_DYN_AUD_LIB                     = 0x30620000    //!< Dynamic for Audio library.
  ,NVM_DYN_PAM_MON_PROFILE_PARAMS      = 0x30630000    //!< Dynamic for MON tracing
  ,NVM_DYN_PAM_GPDS_PROFILE_PARAMS     = 0x30640000    //!< Dynamic for PAM GPDS Server
  ,NVM_DYN_BOOTCORE                    = 0x30700000    //!< Dynamic for Bootcore.
  ,NVM_DYN_UTA_SWU_BOOT                = 0x30710000    //!< Dynamic for UTA SWU boot.
  ,NVM_DYN_UTA_SWU_AGENT               = 0x30710010    //!< Dynamic for UTA SWU agent.
  ,NVM_DYN_TRC_DBG                     = 0x30720000    //!< Dynamic for TRACE DEBUG.
  ,NVM_DYN_BT                          = 0x30730000    //!< Dynamic for BT.
  ,NVM_DYN_BOOTCORE_DUMMY              = 0x30740000    //!< Dummy for bootcore reader
  ,NVM_DYN_KEY_PARMS                   = 0x30750000    //!< Dynamic for keypad parameters
  ,NVM_DYN_MODE_MANAGER                = 0x30800000    //!< Dynamic boot and mode manager state
  ,NVM_DYN_STARTUP                     = 0x30810000    //!< Dynamic for Startup.
  ,NVM_DYN_BMADMIN                     = 0x30820000    //!< Dynamic for batt manager.
  ,NVM_DYN_IMS_VOLTE                   = 0x30830000    //!< Dynamic for IMS.
  ,NVM_DYN_ME                          = 0x30840000    //!< Dynamic for ME.
  ,NVM_DYN_BMMON_IMMEDIATE             = 0x30850000    //!< Dynamic for batt monitoring immediate
  ,NVM_DYN_BMMON_DEFERRED              = 0x30850001    //!< Dynamic for batt monitoring deferred
  ,NVM_DYN_TDS_PS                      = 0x30860000    //!< Dynamic for TDS_PS.
  ,NVM_DYN_ESL                         = 0x30870000    //!< Dynamic for ESL
  ,NVM_DYN_SW_FUEL_GAUGE               = 0x30880000    //!< Dynamic for fuel gauge.

  ,NVM_DYN_VARSIZE_START               = 0x3E000000    //!< Dynamic VARSIZE start of range
   /* Reserved area for VARSIZE groups */
  ,NVM_DYN_VARSIZE_STOP                = 0x3EFFFFFF    //!< Dynamic VARSIZE stop of range
  ,NVM_DYN_UTA_1                       = 0x3F000000    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_2                       = 0x3F000001    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_3                       = 0x3F000002    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_4                       = 0x3F000003    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_5                       = 0x3F000004    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_6                       = 0x3F000005    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_7                       = 0x3F000006    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_8                       = 0x3F000007    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_9                       = 0x3F000008    //!< Dynamic for UTA_NVM
  ,NVM_DYN_UTA_10                      = 0x3F000009    //!< Dynamic for UTA_NVM
#if defined CONFIG_NVMTEST
  ,NVM_DYN_GROUP_0                     = 0x30002000    //!< Dynamic test group.
  ,NVM_DYN_GROUP_1                     = 0x30003000    //!< Dynamic test group.
  ,NVM_DYN_GROUP_2                     = 0x30000400    //!< Dynamic test group.
  ,NVM_DYN_GROUP_3                     = 0x30000050    //!< Dynamic test group.
  ,NVM_DYN_GROUP_4                     = 0x30000006    //!< Dynamic test group.
  ,NVM_DYN_GROUP_5                     = 0x30000007    //!< Dynamic test group.
  ,NVM_DYN_GROUP_6                     = 0x30000008    //!< Dynamic test group.
  ,NVM_DYN_GROUP_7                     = 0x30000009    //!< Dynamic test group.
  ,NVM_DYN_GROUP_8                     = 0x3000000A    //!< Dynamic test group.
#endif

  ,NVM_ILLEGAL_GROUP_ID                = 0x0FFFFFFF    //!< Used to identify an illegal group_id. Enum is SIGNED !
} T_NVM_GROUP;

#if defined NVM_COMMON_SWAP_LAS
#define ATC_STARTUP_MODE_REQUEST_TEST    0x19837465    /* Magic number for flashtool requesting test mode */
#endif

/*!
  \brief This enumeration indicates the status of an operation.
 * It is in order of increasing severity.
 */
typedef enum
{
   NVM_OK                                  = 0    //!< = UTA_SUCCESS The function completed as expected.
  ,NVM_ERROR                               = 1    //!< General error.
  ,NVM_MEMORY_CANNOT_BE_ALLOCATED          = 2    //!< The memory could not allocated using malloc.
  ,NVM_ILLEGAL_GROUP                       = 3    //!< The group_id is illegal to use in the current context.
  ,NVM_INVALID_PARAMETERS                  = 4    //!< The function was called with invalid parameters.
  ,NVM_NOT_USED                            = 5    //!< Not used.
  ,NVM_DEACTIVATION_IN_PROGRESS            = 6    //!< A function was called while the driver was being deactivated.
  ,NVM_IN_PROGRESS                         = 7    //!< The result is being processed. Please wait...
  ,NVM_DRIVER_NOT_INITIALIZED              = 8    //!< A function was called before the driver was initialized.
  ,NVM_UNKNOWN_TEST_CMD                    = 9    //!< An unknown test command was received in the test interface.
  ,NVM_CHECKSUM_ERROR                      = 10   //!< Checksum error.
  ,NVM_COMMUNICATION_FAILED                = 11   //!< Communication failed on the test interface between target and PC.
  ,NVM_WRONG_NOF_BYTES_RECEIVED            = 12   //!< More or less data was received than expected.
  ,NVM_UPDATES_PENDING                     = 13   //!< There are still one or more operations running of type read, write, clear or check_op.
  ,NVM_NO_UPDATES_PENDING                  = 14   //!< There are no more operations running of type read, write, clear or check_op.
  ,NVM_FUNCTION_CALL_NOT_ALLOWED           = 15   //!< Function call not allowed
  ,NVM_CONSISTENCY_CHECK_FAILED            = 16   //!< Consistency check failed in nvm_initialize_driver().
  ,NVM_READ_SECTOR_ERROR                   = 17   //!< Sector could not be read.
  ,NVM_WRITE_SECTOR_ERROR                  = 18   //!< Sector could not be written.
  ,NVM_CLEAR_LAS_ERROR                     = 19   //!< The LAS could not be erased.
  ,NVM_LAS_NOT_YET_READ                    = 20   //!< The LAS has not been read yet.
  ,NVM_INSERT_WRONG_CHECKSUM_ERROR         = 21   //!< The wrong checksum could not be inserted (for test purposes).
  ,NVM_FCT_INVALID_FOR_HOST_TEST           = 22   //!< The function was called in host test but can only be used in target test.
  ,NVM_FCT_INVALID_FOR_TARGET_TEST         = 23   //!< The function was called in target test but can only be used in host test.
  ,NVM_NOT_USED_2                          = 24   //!< A new write was initiated while the first write was not yet finished.
  ,NVM_UNKNOWN_MAIL_RECEIVED               = 25   //!< Unknown mail received.
  ,NVM_OS_NOT_RUNNING                      = 26   //!< OS is not running as expected.
  ,NVM_EXCEPTIONS_GENERATED                = 27   //!< Exceptions were generated.
  ,NVM_PLR_TEST                            = 28   //!< Used for testing PLR.
  ,NVM_GTI_FLASH_TO_MIRROR_ERROR           = 29   //!< gti flash to mirror failed.
  ,NVM_GTI_MIRROR_TO_FLASH_ERROR           = 30   //!< gti mirror to flash failed.
  ,NVM_GTI_INIT_ERROR                      = 31   //!< gti_init_nvm_system failed.
  ,NVM_WRONG_PAYLOAD_TYPE                  = 32   //!< Wrong payload type.
  ,NVM_WRONG_COMMIT_TYPE                   = 33   //!< Wrong commit type.
  ,NVM_CALLER_CHECK_ERROR                  = 34   //!< Return code when the caller check fails.
  ,NVM_CLEAN_LAS                           = 35   //!< Used to tell if LAS has to be cleaned at initialisation in a "Read only before OS" system.
  ,NVM_OS_NOT_INITIALIZED                  = 36   //!< OS is not initialised as expected.
  ,NVM_SECTOR_HEADER_ERROR                 = 37   //!< Error in sector header.
  ,NVM_SECTOR_EMPTY                        = 38   //!< Sector is empty (contains 0xFF).
  ,NVM_NON_ATC_TASK                        = 39   //!< A non-ATC task request was received
  ,NVM_MEM_OVERWRITE_ERROR                 = 40   //!< An overwrite error was reported from flash driver
  ,NVM_NULL_POINTER_NOT_ALLOWED            = 41   //!< NULL pointer not allowed
  ,NVM_OUT_OF_KERNEL_MIRROR_MEMORY         = 42   //!< Out of kernel mirror memory
  ,NVM_FLASH_PLUGIN_NOT_REGISTERED         = 43   //!< Flash plugin has not yet been registered.
  ,NVM_KERNEL_MIRROR_NOT_YET_READ          = 44   //!< Kernel mirror not yet read
  ,NVM_HASH_ERROR                          = 45   //!< HASH error
  ,NVM_SIGNATURE_ERROR                     = 46   //!< Signature error
  ,NVM_GROUP_SIZE_ERROR                    = 47   //!< The size of a group as indicated in the group header has wrong value
  ,NVM_UNKNOWN_HEADER_VERSION              = 48   //!< The version of the group header is unknown
  ,NVM_READ_ONLY                           = 49   //!< NVM can't write this parameter. The parameter are temporarely read only
  ,NVM_TOO_FEW_SECTORS_AVAILABLE           = 50   //!< There are not as many sectors available to read as requested
  ,NVM_UNEXPECTED_UPDATE_NO                = 51   //!< A sector was read that has an unexpected update number
  ,NVM_NOT_ENOUGH_SPACE_IN_LAS             = 52   //!< Even after a swap, there is not enough room in the LAS to keep the data
  ,NVM_VARSIZE_EVENT_ILLEGAL               = 53   //!< Illegal event was received in VARSIZE state machine
  ,NVM_VARSIZE_STATE_ILLEGAL               = 54   //!< Illegal state in VARSIZE state machine
  ,NVM_VARSIZE_GROUP_NOT_EXIST             = 55   //!< The VARSIZE group has not been created with nvm_create()
  ,NVM_VARSIZE_GROUP_EXIST                 = 56   //!< The VARSIZE group already exists
  ,NVM_VARSIZE_STRING_TOO_LONG             = 57   //!< String too long
  ,NVM_VARSIZE_ILLEGAL_CHAR                = 58   //!< Illegal char
  ,NVM_ERROR_CODE_FOR_TEST                 = 59   //!< Error code used by test suite
  ,NVM_STATUS_SECURITY_SPECIFIC            = 0x10000   //!< Specific status codes for security module start from (NVM_STATUS_SECURITY_SPECIFIC + 1).
/* UtaCommonReturnCodes: */
  ,NVM_UTA_FAILURE                         = -1   //!< Unspecified error.
  ,NVM_UTA_ERROR_OUT_OF_MEMORY             = -2   //!< Out of memory.
  ,NVM_UTA_ERROR_INVALID_HANDLE            = -3   //!< The given handle does not identify a valid object.
  ,NVM_UTA_ERROR_OUT_OF_RANGE_PARAM        = -4   //!< A parameter passed to the function is outside the valid range.
  ,NVM_UTA_ERROR_INVALID_PARAM             = -5   //!< A parameter passed to the function is invalid.
  ,NVM_UTA_ERROR_TOO_SMALL_BUF_PARAM       = -6   //!< A buffer parameter passed to the function is too small to take the result.
  ,NVM_UTA_ERROR_NOT_SUPPORTED             = -7   //!< The requested operation is not supported.
  ,NVM_UTA_ERROR_TIMEOUT                   = -8   //!< Timeout has occurred.
  ,NVM_UTA_ERROR_WRONG_STATE               = -9   //!< The requested operation is not possible/allowed in the current state.
  ,NVM_UTA_ERROR_BAD_FORMAT                = -10  //!< Data is not in the expected format.
  ,NVM_UTA_ERROR_INSUFFICIENT_PERMISSIONS  = -11  //!< The requester of the operation does not have sufficient permissions (access rights etc).
  ,NVM_UTA_ERROR_IO_ERROR                  = -12  //!< An I/O error occurred.
  ,NVM_UTA_ERROR_OUT_OF_HANDLES            = -13  //!< There is no free handle available.
  ,NVM_UTA_ERROR_OPERATION_PENDING         = -14  //!< Another operation is pending - it is not allowed to run more than one operation concurrently.
  ,NVM_UTA_ERROR_SPECIFIC                  = -100 //!< Component specific error codes start from (UTA_ERROR_SPECIFIC - 1).
/* UtaOsReturnCodes: */
  ,NVM_UTA_OS_INVALID_CONTROL_STRUCTURE    = -101 //!< The given OS control structure is invalid (a thread struct is passed to a semaphore function for example).
  ,NVM_UTA_OS_NO_THREAD_CONTEXT            = -102 //!< A OS function which has to be called in a thread context was called form non thread context (ISR).
  ,NVM_UTA_OS_THREAD_WRONG_STACK_ALIGNMENT = -103 //!< The stack is not aligned correctly (for example 8 byte alignment for ARM RVDS compiler).
  ,NVM_UTA_OS_QUEUE_EMPTY                  = -104 //!< The queue is empty.
  ,NVM_UTA_OS_QUEUE_FULL                   = -105 //!< The queue is full.
  ,NVM_UTA_OS_EVENT_GROUP_NOT_PRESENT      = -106 //!< The requested events are not present.
  ,FORCE_TO_S32                            = 0x0FFFFFFF //!< Force to S32.
} T_NVM_RETURNCODE;


/*!
 * \brief This enumeration contains the Type ID's for static and dynamic. Used with nvm_api_clear(). Do not change !
 */
typedef enum
{
   NVM_STA_CAL      = 0x10000000    //!< Static Cal.
  ,NVM_STA_FIX      = 0x20000000    //!< Static Fix.
  ,NVM_DYN          = 0x30000000    //!< Dynamic.
  ,NVM_ILLEGAL_TYPE = 0x0FFFFFFF    //!< Illegal type
} T_NVM_TYPE;


/*!
 * \brief This enumeration contains the Type Index ID's for static and dynamic. Do not change !
 */
typedef enum
{
   NVM_TYPE_INDEX_STA_CAL      = 0x00000000    /**< Static Cal */
  ,NVM_TYPE_INDEX_STA_FIX      = 0x00000001    /**< Static Fix */
  ,NVM_TYPE_INDEX_DYN          = 0x00000002    /**< Dynamic */
  ,NVM_TYPE_INDEX_FORCE_TO_S32 = 0x0FFFFFFF
} T_NVM_TYPE_INDEX;


/*!
  \brief This enumeration indicates which parameters are set to default
 */
typedef enum
{
   NVM_SET_FULL = 0                //!< The whole EEP is set to default values except internal, eep_sec_status.
  ,NVM_SET_CUSTOM                  //!< The whole EEP is set to default values except internal, eep_sec_status and sp_unlock.
  ,NVM_SET_SERVICE                 //!< Corresponds to NVM_SET_CUSTOM.
  ,NVM_SET_DEFAULT_WITH_FILTER     //!< Only those fields are set to default that are indicated by a filter.
  ,NVM_SET_DEFAULT_WITHOUT_FILTER  //!< The whole EEP is set to default values.
  ,NOF_NVM_SET_FACTORY             //!< Number of factory settings.
} T_NVM_SET_FACTORY;

/* NVM device states */
typedef enum
{
     NVM_IDLE                   /* NVM has not been initialized. Reads and writes are not possible.*/
    ,NVM_INIT_TASK              /* NVM has been initialized with nvm_task, NVM mailbox and semaphores. Reads and writes are not possible.*/
    ,NVM_INIT                   /* NVM has been initialized totally and is ready for reads and writes, but nvm_task hasn't been activated.*/
    ,NVM_ACTIVE                 /* nvm_task has been activated. NVM is now totally functional and open for reads and writes.*/
    ,NVM_DEACTIVATE             /* Deactivation task has initiated deactivation of NVM. Reads are possible but it is not possible to write.*/
    ,NVM_DEACTIVATED_READ_ONLY  /* Deactivation completed. NVM will need totally reinitialisation to run again. Reads are still possible but writes are not possible.*/
} T_NVM_STATE;


/*!
  \brief This enumeration contains the two types of Logical Adress Space
 */
typedef enum
{
   NVM_LAS_0 = 0
  ,NVM_LAS_1
} T_NVM_LAS;


/*!
  \brief This enumeration contains the type of memory used by NVM
 */
typedef enum
{
   memory_type_NA = 0,
   nor_flash,
   nand_flash,
   flash_less
} T_NVM_MEMORY_TYPE;


/*!
  \brief This enumeration contains the vendor of memory used by NVM
 */
typedef enum
{
   memory_vendor_NA = 0,
   spansion,
   numonyxL18,
   numonyxM18,
   pcm,
   samsung
} T_NVM_MEMORY_VENDOR;


/*!
  \brief This enumeration gives the calibration status of NVM static calibration LAS
 */
typedef enum
{
   NON_CALIBRATED = 0
  ,CALIBRATED     = 1
  ,CHECKSUM_ERROR = 2
} T_NVM_CALIB_STATUS;


/*!
 * \brief This enumeration is placed here because it is used by nvm_cfg.h
 */
#if 0
typedef enum
{
   EXPOSED_MIN  /* Exposed group, without translation info. A minimum set of AT@ functionality is supported */
  ,EXPOSED_FULL  /* Exposed group with translation info. The full set of AT@ functionality is supported */
  ,NON_EXPOSED  /* Non exposed group. AT@ interface is not supported */
  ,NON_EXPOSED_VARSIZE /* Non exposed group. AT@ interface is not supported. This enum is used for variable sized groups */
  ,MIRROR_TYPE_FORCE_TO_S32 = 0x0FFFFFFF
} T_MIRROR_TYPE;
#endif

/*!
 * \}
 */
/****************************************************************************************/
/* ENUMERATIONS                                                                     END */
/****************************************************************************************/


/****************************************************************************************/
/* DATA STRUCTURES                                                                BEGIN */
/****************************************************************************************/
/*!
 *  \defgroup NVM_data_structures Data Structures
 *  \ingroup NVM
 *  \{
 *  These type definitions are used in the interface of the NVM driver.
 */

/*!
 * \brief Version type
 */
#define T_VERSION U32 //!< Version of group structure.


/*!
 * \brief Revision type
 */
#define T_REVISION U32 //!< Revision of group data.


/*!
 * \brief Revision type
 */
#define T_NOF_BYTES U32 //!< Number of bytes of group data


/*!
 * \brief Header for each group in the RAM mirror. Do not change !
 */
typedef struct
{
  T_VERSION     version;                    //!< Version of group structure
  T_REVISION    revision;                   //!< Revision of group data
  T_NOF_BYTES   nof_bytes;                  //!< # bytes in the user group's struct, ie. # bytes of mirror excl. transl. info and excl. header
} T_NVM_VER_REV_LGT;


/*!
 * \brief Struct containing parameters to be returned by nvm_get_info.
 */
typedef struct
{
  T_NVM_MEMORY_TYPE memory_type;
  T_NVM_MEMORY_VENDOR memory_vendor;
  U8 memory_unique_id[16];
} T_NVM_INFO;


/*!
 * \brief Struct containing parameters to be returned by nvm_get_group_mirror_properties()
 */
typedef struct
{
  T_NVM_VER_REV_LGT *p_mirror;        //!< Pointer to first byte of RAM mirror, ie. version.
  U32               nof_bytes_mirror; //!< Number of bytes of group mirror.
  BOOL              hash_mirror;      //!< Is TRUE if a hash mirror exists that is not a NULL pointer
} T_NVM_MIRROR_PROPERTIES;

typedef T_NVM_RETURNCODE T_NVM_CALLBACK(void *p_context);

/* Definition of state change notification callback function */
typedef void (*P_NVM_STATE_CHANGE_NOTIFY_CB) (T_NVM_STATE nvmdev_state);

typedef struct {
    /* If the notify flag is set for particular state; it has to be notified when nvm changed to that state */
    unsigned char                   flags;
#define NVM_ACTIVE_NOTIFY       0x01
#define NVM_DEACTIVE_NOTIFY     0x02
#define NVM_NOTIFY_ALL          (NVM_ACTIVE_NOTIFY | NVM_DEACTIVE_NOTIFY)
#define NVM_NOTIFY_ALWAYS       0x80
    P_NVM_STATE_CHANGE_NOTIFY_CB    p_func;
} T_NVM_STATE_NOTIFY;

#define NVM_NOTIFIED_SHIFT_LOC  4

#define NVM_ACTIVE_NOTIFIED     (NVM_ACTIVE_NOTIFY << NVM_NOTIFIED_SHIFT_LOC)
#define NVM_DEACTIVE_NOTIFIED   (NVM_DEACTIVE_NOTIFY << NVM_NOTIFIED_SHIFT_LOC)

/*!
 * \brief Sets dynamic parameters to Factory Default values
 *
 * \param[in]  nvm_set_factory      Determines which kind af Factory Default values shall be applied.
 *
 * \return     T_NVM_RETURNCODE     NVM_OK, NVM_DRIVER_NOT_INITIALIZED
 */
T_NVM_RETURNCODE nvm_set_factory_default(T_NVM_SET_FACTORY nvm_set_factory);


/*!
 * \brief Each user module can contain a function that sets it's dynamic parameters to default values.
 * The user module shall register this function at NVM (which uses it in nvm_set_factory_default).
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 * \param[in]  func                 Function located in the user module that sets group_id's parameters to default values
 *
 * \return     T_NVM_RETURNCODE     NVM_OK or NVM_ILLEGAL_GROUP
 */
T_NVM_RETURNCODE nvm_register_callback_set_default_dynamic(T_NVM_GROUP group_id, void (*func)(T_NVM_SET_FACTORY nvm_set_factory));


/*!
 * \brief Each user module can contain a function that shall register at NVM, and shall be triggered whenever there is a change in device state.
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 * \param[in]  p_func               Function located in the user module that will be called during a nvm device state change
 * \param[in]  flags                Flags specify what all notifications required.
 *
 * \return     T_NVM_RETURNCODE     NVM_OK or NVM_ILLEGAL_GROUP
 */
T_NVM_RETURNCODE nvm_register_state_change_notify(T_NVM_GROUP group_id, void (*p_func)(T_NVM_STATE nvmdev_state), unsigned char flags);


/*!
 * \brief Unregister the registered state change notification callback.
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 *
 * \return     T_NVM_RETURNCODE     NVM_OK or NVM_ILLEGAL_GROUP
 */
T_NVM_RETURNCODE nvm_unregister_state_change_notify(T_NVM_GROUP group_id);


/*!
 * \brief This function finds and returns the latest version and revision of the indicated group (struct).
 * Earlier versions are disregarded.
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 * \param[in]  p_version            The latest version of the struct.
 * \param[in]  p_revision           The latest revision of the struct.
 * \param[in]  p_nof_bytes          The total number of bytes of the group's structure at the time it was stored rounded up to a multiple of 4.
 *
 * \return     T_NVM_RETURNCODE     NVM_OK, NVM_ILLEGAL_GROUP or NVM_DRIVER_NOT_INITIALIZED
 */
T_NVM_RETURNCODE nvm_get_ver_rev_lgt(T_NVM_GROUP group_id,
                                     T_VERSION   *p_version,
                                     T_REVISION  *p_revision,
                                     U32         *p_nof_bytes);


/*!
 * \brief Checks if any write, read or clear operations are ongoing and returns the result.
 *
 * \return     T_NVM_RETURNCODE     NVM_UPDATES_PENDING, NVM_NO_UPDATES_PENDING, NVM_ERROR, NVM_DRIVER_NOT_INITIALIZED
 */
T_NVM_RETURNCODE nvm_check_updates_pending(void);



/*!
 * \brief This function shall be called from the user module during boot-up to initialise its NVM group,
 * in case the data currently stored in NVM group is no longer compatible. The update is done asynchronously.
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 * \param[in]  src                  Absolute address where to copy data from
 * \param[in]  offset               Offset in local NVM data structure.
 * \param[in]  nof_bytes            Number of bytes to write
 * \param[in]  version              The version to store
 * \param[in]  revision             The revision to store
 *
 * \return     T_NVM_RETURNCODE     NVM_OK, NVM_ILLEGAL_GROUP, NVM_DRIVER_NOT_INITIALIZED, NVM_DEACTIVATION_IN_PROGRESS, NVM_INVALID_PARAMETERS or NVM_ERROR
 */
T_NVM_RETURNCODE nvm_init(T_NVM_GROUP group_id, U8 *src, U32 offset, U32 nof_bytes, T_VERSION version, T_REVISION revision);



/*!
 * \brief Writes data asynchronously.
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 * \param[in]  src                  Absolute address where to copy data from
 * \param[in]  offset               Offset in local NVM data structure.
 * \param[in]  nof_bytes            Number of bytes to write
 *
 * \return     T_NVM_RETURNCODE     NVM_OK, NVM_ILLEGAL_GROUP, NVM_DRIVER_NOT_INITIALIZED, NVM_DEACTIVATION_IN_PROGRESS, NVM_INVALID_PARAMETERS or NVM_ERROR
 */
T_NVM_RETURNCODE nvm_write(T_NVM_GROUP group_id, U8 *src, U32 offset, U32 nof_bytes);


/*!
 * \brief Reads data from the NVM group in RAM mirror.
 * The group can be dynamic or static.
 *
 * \param[in]  group_id             Identifies the NVM structure belonging to the user.
 * \param[in]  dst                  Absolute address where to copy data to
 * \param[in]  offset               Offset in local NVM data structure.
 * \param[in]  nof_bytes            Number of bytes to read
 *
 * \return     T_NVM_RETURNCODE     NVM_OK, NVM_ILLEGAL_GROUP, NVM_DRIVER_NOT_INITIALIZED, NVM_INVALID_PARAMETERS
 */
T_NVM_RETURNCODE nvm_read(T_NVM_GROUP group_id, U8 *dst, U32 offset, U32 nof_bytes);


/*!
 * \}
 */
/****************************************************************************************/
/* INTERFACE FUNCTIONS                                                              END */
/****************************************************************************************/

#endif // _NVM_H

#if defined(__cplusplus)
    }   /* extern "C" */
#endif
