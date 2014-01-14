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

#if !defined _NVM_CFG_LIN_H
#define _NVM_CFG_LIN_H

/* Modified nvm_cfg.h to work with linux */

#include <nvm/nvm.h>

#include <aud_app_nvm_sta_fix_driverif.h>
#include <sw_fuel_gauge_nvm_dyn_driverif.h>
#include <adc_sensors_nvm_sta_cal_driverif.h>

typedef struct
{
  T_NVM_GROUP                           group_id;
  U8                                    *p_data;            /* Points to ver of the group */
  U32                                   nof_bytes_mirror;   /* Number of bytes in the mirror including ver, rev, length */
  CHAR                                  *gti_name;          /* Used only with external mirrors when converting with GTI */
} T_GROUP_PROPERTIES;

/****************************************************************************************/
/* DATA STRUCTURES                                                                BEGIN */
/****************************************************************************************/

/******************* Group Calib definitions *********************/
#if defined CONFIG_NVMTEST
typedef struct
{
  U8 test[1];
} T_GROUP0_NVM_STA_CAL;

typedef struct
{
  U8 test[2];
} T_GROUP1_NVM_STA_CAL;

typedef struct
{
  U8 test[3];
} T_GROUP2_NVM_STA_CAL;

typedef struct
{
  U8 test[4];
} T_GROUP3_NVM_STA_CAL;

typedef struct
{
  U8 test[220];
} T_GROUP4_NVM_STA_CAL;

typedef struct
{
  U8 test[20000];
} T_GROUP5_NVM_STA_CAL;

typedef struct
{
  U8 test[10000];
} T_GROUP6_NVM_STA_CAL;

/* External group definitions */
typedef struct
{
  U8 test[2000];
} T_GROUP7_NVM_STA_CAL;


/******************* Group Fixed definitions *********************/
typedef struct
{
  U8 test[1];
} T_GROUP0_NVM_STA_FIX;

typedef struct
{
  U8 test[1007];
} T_GROUP1_NVM_STA_FIX;

typedef struct
{
  U8 test[1008];
} T_GROUP2_NVM_STA_FIX;

typedef struct
{
  U8 test[1009];
} T_GROUP3_NVM_STA_FIX;

typedef struct
{
  U8 test[4000];
} T_GROUP4_NVM_STA_FIX;

typedef struct
{
  U8 test[3000];
} T_GROUP5_NVM_STA_FIX;

/* External group definitions */
typedef struct
{
  U8 test[25];
} T_GROUP6_NVM_STA_FIX;


/******************* Group Dynamic definitions *********************/
typedef struct
{
  U8 test[300];
} T_GROUP0_NVM_DYN;

typedef struct
{
  U8 test[3023];
} T_GROUP1_NVM_DYN;

typedef struct
{
  U8 test[3024];
} T_GROUP2_NVM_DYN;

typedef struct
{
  U8 test[3025];
} T_GROUP3_NVM_DYN;

typedef struct
{
  U8 test[10000];
} T_GROUP4_NVM_DYN;

/* External group definitions */
typedef struct
{
  U8 test[5000];
} T_GROUP5_NVM_DYN;

#endif /* CONFIG_NVMTEST */

/******************* Mirror Calib definitions *********************/
#if defined CONFIG_NVMTEST
  typedef struct
  {
    T_NVM_VER_REV_LGT             group_0_ver_rev;
    T_GROUP0_NVM_STA_CAL          group_0;
  } T_GROUP0_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_1_ver_rev;
    T_GROUP1_NVM_STA_CAL          group_1;
  } T_GROUP1_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_2_ver_rev;
    T_GROUP2_NVM_STA_CAL          group_2;
  } T_GROUP2_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_3_ver_rev;
    T_GROUP3_NVM_STA_CAL          group_3;
  } T_GROUP3_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_4_ver_rev;
    T_GROUP4_NVM_STA_CAL          group_4;
  } T_GROUP4_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_5_ver_rev;
    T_GROUP5_NVM_STA_CAL          group_5;
  } T_GROUP5_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_6_ver_rev;
    T_GROUP6_NVM_STA_CAL          group_6;
  } T_GROUP6_MIRROR_NVM_STA_CAL;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_7_ver_rev;
    T_GROUP7_NVM_STA_CAL          group_7;
  } T_GROUP7_MIRROR_NVM_STA_CAL;
#endif /* CONFIG_NVMTEST */

  typedef struct
  {
    T_NVM_VER_REV_LGT             adc_sensors_ver_rev;
    T_ADC_SENSORS_NVM_STA_CAL     adc_sensors;
  } T_ADC_SENSORS_MIRROR_NVM_STA_CAL;

  /******************* Mirror Fixed definitions *********************/
#if defined CONFIG_NVMTEST
  typedef struct
  {
    T_NVM_VER_REV_LGT             group_0_ver_rev;
    T_GROUP0_NVM_STA_FIX          group_0;
  } T_GROUP0_MIRROR_NVM_STA_FIX;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_1_ver_rev;
    T_GROUP1_NVM_STA_FIX          group_1;
  } T_GROUP1_MIRROR_NVM_STA_FIX;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_2_ver_rev;
    T_GROUP2_NVM_STA_FIX          group_2;
  } T_GROUP2_MIRROR_NVM_STA_FIX;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_3_ver_rev;
    T_GROUP3_NVM_STA_FIX          group_3;
  } T_GROUP3_MIRROR_NVM_STA_FIX;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_4_ver_rev;
    T_GROUP4_NVM_STA_FIX          group_4;
  } T_GROUP4_MIRROR_NVM_STA_FIX;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_5_ver_rev;
    T_GROUP5_NVM_STA_FIX          group_5;
  } T_GROUP5_MIRROR_NVM_STA_FIX;

  typedef struct
  {
    T_NVM_VER_REV_LGT             group_6_ver_rev;
    T_GROUP6_NVM_STA_FIX          group_6;
  } T_GROUP6_MIRROR_NVM_STA_FIX;
#endif /* CONFIG_NVMTEST */

typedef struct
{
  T_NVM_VER_REV_LGT             fmr_nvm_sta_fix_ver_rev;
  T_FMR_NVM_STA_FIX             fmr_nvm_sta_fix;
} T_FMR_MIRROR_NVM_STA_FIX;

  /******************* Mirror Dynamic definitions *********************/
#if defined CONFIG_NVMTEST
  typedef struct
  {
    T_NVM_VER_REV_LGT               group_0_ver_rev;
    T_GROUP0_NVM_DYN                group_0;
  } T_GROUP0_MIRROR_NVM_DYN;

  typedef struct
  {
    T_NVM_VER_REV_LGT               group_1_ver_rev;
    T_GROUP1_NVM_DYN                group_1;
  } T_GROUP1_MIRROR_NVM_DYN;

  typedef struct
  {
    T_NVM_VER_REV_LGT               group_2_ver_rev;
    T_GROUP2_NVM_DYN                group_2;
  } T_GROUP2_MIRROR_NVM_DYN;

  typedef struct
  {
    T_NVM_VER_REV_LGT               group_3_ver_rev;
    T_GROUP3_NVM_DYN                group_3;
  } T_GROUP3_MIRROR_NVM_DYN;

  typedef struct
  {
    T_NVM_VER_REV_LGT               group_4_ver_rev;
    T_GROUP4_NVM_DYN                group_4;
  } T_GROUP4_MIRROR_NVM_DYN;

  typedef struct
  {
    T_NVM_VER_REV_LGT               group_5_ver_rev;
    T_GROUP5_NVM_DYN                group_5;
  } T_GROUP5_MIRROR_NVM_DYN;
#endif /* CONFIG_NVMTEST */

  typedef struct
  {
    T_NVM_VER_REV_LGT              sw_fuel_gauge_ver_rev;
    T_SW_FUEL_GAUGE_NVM_DYN        sw_fuel_gauge;
  } T_SW_FUEL_GAUGE_MIRROR_NVM_DYN;

/* Mirror required only for kernel driver */
#if defined CONFIG_NVM
  #if defined CONFIG_NVMTEST
  #define NVM_DECLARE_MIRROR_STA_CAL_TEST \
  T_GROUP0_MIRROR_NVM_STA_CAL     group0_mirror_nvm_sta_cal; \
  T_GROUP1_MIRROR_NVM_STA_CAL     group1_mirror_nvm_sta_cal; \
  T_GROUP2_MIRROR_NVM_STA_CAL     group2_mirror_nvm_sta_cal; \
  T_GROUP3_MIRROR_NVM_STA_CAL     group3_mirror_nvm_sta_cal; \
  T_GROUP4_MIRROR_NVM_STA_CAL     group4_mirror_nvm_sta_cal; \
  T_GROUP5_MIRROR_NVM_STA_CAL     group5_mirror_nvm_sta_cal; \
  T_GROUP6_MIRROR_NVM_STA_CAL     group6_mirror_nvm_sta_cal; \
  T_GROUP7_MIRROR_NVM_STA_CAL     group7_mirror_nvm_sta_cal;
  #endif /* CONFIG_NVMTEST */

  #define NVM_DECLARE_MIRROR_STA_CAL \
  T_ADC_SENSORS_MIRROR_NVM_STA_CAL     adc_sensors_mirror_nvm_sta_cal;

  /******************* Mirror Fixed declarations *********************/

  #if defined CONFIG_NVMTEST
  #define NVM_DECLARE_MIRROR_STA_FIX_TEST \
  T_GROUP0_MIRROR_NVM_STA_FIX     group0_mirror_nvm_sta_fix; \
  T_GROUP1_MIRROR_NVM_STA_FIX     group1_mirror_nvm_sta_fix; \
  T_GROUP2_MIRROR_NVM_STA_FIX     group2_mirror_nvm_sta_fix; \
  T_GROUP3_MIRROR_NVM_STA_FIX     group3_mirror_nvm_sta_fix; \
  T_GROUP4_MIRROR_NVM_STA_FIX     group4_mirror_nvm_sta_fix; \
  T_GROUP5_MIRROR_NVM_STA_FIX     group5_mirror_nvm_sta_fix; \
  T_GROUP6_MIRROR_NVM_STA_FIX     group6_mirror_nvm_sta_fix;
  #endif /* CONFIG_NVMTEST */

  #define NVM_DECLARE_MIRROR_STA_FIX \
    T_FMR_MIRROR_NVM_STA_FIX        fmr_mirror_nvm_sta_fix;

  /******************* Mirror Dynamic declarations *********************/

  #if defined CONFIG_NVMTEST
  #define NVM_DECLARE_MIRROR_DYN_TEST \
  T_GROUP0_MIRROR_NVM_DYN        group0_mirror_nvm_dyn; \
  T_GROUP1_MIRROR_NVM_DYN        group1_mirror_nvm_dyn; \
  T_GROUP2_MIRROR_NVM_DYN        group2_mirror_nvm_dyn; \
  T_GROUP3_MIRROR_NVM_DYN        group3_mirror_nvm_dyn; \
  T_GROUP4_MIRROR_NVM_DYN        group4_mirror_nvm_dyn; \
  T_GROUP5_MIRROR_NVM_DYN        group5_mirror_nvm_dyn;
  #endif /* CONFIG_NVMTEST */

  #define NVM_DECLARE_MIRROR_DYN \
    T_SW_FUEL_GAUGE_MIRROR_NVM_DYN       sw_fuel_gauge_mirror_nvm_dyn;

  /* External Dynamic Group */
  #define NVM_DECLARE_MIRROR_DYN_EXTERN
#endif /* CONFIG_NVM */

/* Mirror only required for kernel driver */
#if defined CONFIG_NVM
  #if defined CONFIG_NVMTEST
    #define P_GROUP0_MIRROR_NVM_STA_CAL         ((U8*)&group0_mirror_nvm_sta_cal)
    #define P_GROUP1_MIRROR_NVM_STA_CAL         ((U8*)&group1_mirror_nvm_sta_cal)
    #define P_GROUP2_MIRROR_NVM_STA_CAL         ((U8*)&group2_mirror_nvm_sta_cal)
    #define P_GROUP3_MIRROR_NVM_STA_CAL         ((U8*)&group3_mirror_nvm_sta_cal)
    #define P_GROUP4_MIRROR_NVM_STA_CAL         ((U8*)&group4_mirror_nvm_sta_cal)
    #define P_GROUP5_MIRROR_NVM_STA_CAL         ((U8*)&group5_mirror_nvm_sta_cal)
    #define P_GROUP6_MIRROR_NVM_STA_CAL         ((U8*)&group6_mirror_nvm_sta_cal)
    #define P_GROUP7_MIRROR_NVM_STA_CAL         ((U8*)&group7_mirror_nvm_sta_cal)

    #define P_GROUP0_MIRROR_NVM_STA_FIX         ((U8*)&group0_mirror_nvm_sta_fix)
    #define P_GROUP1_MIRROR_NVM_STA_FIX         ((U8*)&group1_mirror_nvm_sta_fix)
    #define P_GROUP2_MIRROR_NVM_STA_FIX         ((U8*)&group2_mirror_nvm_sta_fix)
    #define P_GROUP3_MIRROR_NVM_STA_FIX         ((U8*)&group3_mirror_nvm_sta_fix)
    #define P_GROUP4_MIRROR_NVM_STA_FIX         ((U8*)&group4_mirror_nvm_sta_fix)
    #define P_GROUP5_MIRROR_NVM_STA_FIX         ((U8*)&group5_mirror_nvm_sta_fix)
    #define P_GROUP6_MIRROR_NVM_STA_FIX         ((U8*)&group6_mirror_nvm_sta_fix)

    #define P_GROUP0_MIRROR_NVM_DYN             ((U8*)&group0_mirror_nvm_dyn)
    #define P_GROUP1_MIRROR_NVM_DYN             ((U8*)&group1_mirror_nvm_dyn)
    #define P_GROUP2_MIRROR_NVM_DYN             ((U8*)&group2_mirror_nvm_dyn)
    #define P_GROUP3_MIRROR_NVM_DYN             ((U8*)&group3_mirror_nvm_dyn)
    #define P_GROUP4_MIRROR_NVM_DYN             ((U8*)&group4_mirror_nvm_dyn)
    #define P_GROUP5_MIRROR_NVM_DYN             ((U8*)&group5_mirror_nvm_dyn)
  #endif /* CONFIG_NVMTEST */

    #define P_ADC_SENSORS_NVM_STA_CAL           ((U8*)&adc_sensors_mirror_nvm_sta_cal)

    #define P_FMR_MIRROR_NVM_STA_FIX            ((U8*)&fmr_mirror_nvm_sta_fix)

    #define P_SW_FUEL_GAUGE_MIRROR_NVM_DYN      ((U8*)&sw_fuel_gauge_mirror_nvm_dyn)
#else
  #if defined CONFIG_NVMTEST
    #define P_GROUP0_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP1_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP2_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP3_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP4_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP5_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP6_MIRROR_NVM_STA_CAL         NULL
    #define P_GROUP7_MIRROR_NVM_STA_CAL         NULL

    #define P_GROUP0_MIRROR_NVM_STA_FIX         NULL
    #define P_GROUP1_MIRROR_NVM_STA_FIX         NULL
    #define P_GROUP2_MIRROR_NVM_STA_FIX         NULL
    #define P_GROUP3_MIRROR_NVM_STA_FIX         NULL
    #define P_GROUP4_MIRROR_NVM_STA_FIX         NULL
    #define P_GROUP5_MIRROR_NVM_STA_FIX         NULL
    #define P_GROUP6_MIRROR_NVM_STA_FIX         NULL

    #define P_GROUP0_MIRROR_NVM_DYN             NULL
    #define P_GROUP1_MIRROR_NVM_DYN             NULL
    #define P_GROUP2_MIRROR_NVM_DYN             NULL
    #define P_GROUP3_MIRROR_NVM_DYN             NULL
    #define P_GROUP4_MIRROR_NVM_DYN             NULL
    #define P_GROUP5_MIRROR_NVM_DYN             NULL
  #endif /* CONFIG_NVMTEST */
    #define P_ADC_SENSORS_NVM_STA_CAL           NULL

    #define P_FMR_MIRROR_NVM_STA_FIX            NULL

    #define P_SW_FUEL_GAUGE_MIRROR_NVM_DYN      NULL
#endif /* CONFIG_NVM */


#define NVM_INIT_PROP_STA_CAL \
  {NVM_STA_CAL_ADC_SENSORS, P_ADC_SENSORS_NVM_STA_CAL, sizeof(T_ADC_SENSORS_MIRROR_NVM_STA_CAL), "cal_adc_sensors"}

#if defined CONFIG_NVMTEST
#define NVM_INIT_PROP_STA_CAL_TEST \
  {NVM_STA_CAL_GROUP_0,  P_GROUP0_MIRROR_NVM_STA_CAL, sizeof(T_GROUP0_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_1,  P_GROUP1_MIRROR_NVM_STA_CAL, sizeof(T_GROUP1_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_2,  P_GROUP2_MIRROR_NVM_STA_CAL, sizeof(T_GROUP2_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_3,  P_GROUP3_MIRROR_NVM_STA_CAL, sizeof(T_GROUP3_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_4,  P_GROUP4_MIRROR_NVM_STA_CAL, sizeof(T_GROUP4_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_5,  P_GROUP5_MIRROR_NVM_STA_CAL, sizeof(T_GROUP5_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_6,  P_GROUP6_MIRROR_NVM_STA_CAL, sizeof(T_GROUP6_MIRROR_NVM_STA_CAL), ""          }, \
  {NVM_STA_CAL_GROUP_7,  P_GROUP7_MIRROR_NVM_STA_CAL, sizeof(T_GROUP7_MIRROR_NVM_STA_CAL), "group7_cal"}
#endif /* CONFIG_NVMTEST */

#if defined CONFIG_NVMTEST
#define NVM_INIT_PROP_STA_FIX_TEST \
  {NVM_STA_FIX_GROUP_0,  P_GROUP0_MIRROR_NVM_STA_FIX, sizeof(T_GROUP0_MIRROR_NVM_STA_FIX), ""          }, \
  {NVM_STA_FIX_GROUP_1,  P_GROUP1_MIRROR_NVM_STA_FIX, sizeof(T_GROUP1_MIRROR_NVM_STA_FIX), ""          }, \
  {NVM_STA_FIX_GROUP_2,  P_GROUP2_MIRROR_NVM_STA_FIX, sizeof(T_GROUP2_MIRROR_NVM_STA_FIX), ""          }, \
  {NVM_STA_FIX_GROUP_3,  P_GROUP3_MIRROR_NVM_STA_FIX, sizeof(T_GROUP3_MIRROR_NVM_STA_FIX), ""          }, \
  {NVM_STA_FIX_GROUP_4,  P_GROUP4_MIRROR_NVM_STA_FIX, sizeof(T_GROUP4_MIRROR_NVM_STA_FIX), ""          }, \
  {NVM_STA_FIX_GROUP_5,  P_GROUP5_MIRROR_NVM_STA_FIX, sizeof(T_GROUP5_MIRROR_NVM_STA_FIX), ""          }, \
  {NVM_STA_FIX_GROUP_6,  P_GROUP6_MIRROR_NVM_STA_FIX, sizeof(T_GROUP6_MIRROR_NVM_STA_FIX), "group6_fix"}
#endif /* CONFIG_NVMTEST */

#define NVM_INIT_PROP_STA_FIX \
  {NVM_STA_FIX_FMR,     P_FMR_MIRROR_NVM_STA_FIX,    sizeof(T_FMR_MIRROR_NVM_STA_FIX), "fix_fmr"}

#if defined CONFIG_NVMTEST
#define NVM_INIT_PROP_DYN_TEST \
  {NVM_DYN_GROUP_0,     P_GROUP0_MIRROR_NVM_DYN,     sizeof(T_GROUP0_MIRROR_NVM_DYN), ""          }, \
  {NVM_DYN_GROUP_1,     P_GROUP1_MIRROR_NVM_DYN,     sizeof(T_GROUP1_MIRROR_NVM_DYN), ""          }, \
  {NVM_DYN_GROUP_2,     P_GROUP2_MIRROR_NVM_DYN,     sizeof(T_GROUP2_MIRROR_NVM_DYN), ""          }, \
  {NVM_DYN_GROUP_3,     P_GROUP3_MIRROR_NVM_DYN,     sizeof(T_GROUP3_MIRROR_NVM_DYN), ""          }, \
  {NVM_DYN_GROUP_4,     P_GROUP4_MIRROR_NVM_DYN,     sizeof(T_GROUP4_MIRROR_NVM_DYN), ""          }, \
  {NVM_DYN_GROUP_5,     P_GROUP5_MIRROR_NVM_DYN,     sizeof(T_GROUP5_MIRROR_NVM_DYN), "group5_dyn"}
#endif /* CONFIG_NVMTEST */

#define NVM_INIT_PROP_DYN \
    {NVM_DYN_SW_FUEL_GAUGE,    P_SW_FUEL_GAUGE_MIRROR_NVM_DYN,   sizeof(T_SW_FUEL_GAUGE_MIRROR_NVM_DYN), "dyn_sw_fuel_gauge"}
//#define NVM_INIT_PROP_DYN_EXTERN

/******************* Number of groups *********************/

/* No. of test groups in Linux kernel space, i.e. groups only used by NVM test suite. */
#if defined CONFIG_NVMTEST
    #define NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_CAL    8
    #define NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_FIX    7
    #define NOF_NVM_GROUPS_LIN_KER_SPC_TEST_DYN        6
#else
    #define NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_CAL    0
    #define NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_FIX    0
    #define NOF_NVM_GROUPS_LIN_KER_SPC_TEST_DYN        0
#endif /* CONFIG_NVMTEST */

#define NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_STA_CAL  1
#define NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_STA_FIX  1
#define NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_DYN      1

#define NOF_NVM_GROUPS_ALL_STA_CAL          (NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_CAL + NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_STA_CAL)
#define NOF_NVM_GROUPS_ALL_STA_FIX          (NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_FIX + NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_STA_FIX)
#define NOF_NVM_GROUPS_ALL_DYN              (NOF_NVM_GROUPS_LIN_KER_SPC_TEST_DYN     + NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_DYN)

#define TOT_NOF_NVM_LIN_KER_SPC_TEST_GROUPS ((NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_CAL)  +  \
                                             (NOF_NVM_GROUPS_LIN_KER_SPC_TEST_STA_FIX)  +  \
                                             (NOF_NVM_GROUPS_LIN_KER_SPC_TEST_DYN))

#define TOT_NOF_NVM_LIN_KER_SPC_GROUPS      ((NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_STA_CAL)  +  \
                                             (NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_STA_FIX)  +  \
                                             (NOF_NVM_GROUPS_LIN_KER_SPC_NORMAL_DYN))

#define TOT_NOF_NVM_GROUPS                  ((TOT_NOF_NVM_LIN_KER_SPC_TEST_GROUPS) + (TOT_NOF_NVM_LIN_KER_SPC_GROUPS))

/*!
 * \brief Number of NVM Types. Do not change !
 */
#define NOF_NVM_TYPES      3


/*!
 * \brief Location of Type. Do not change !
 */
#define NVM_TYPE_POS      28


/*!
 * NVM_cfg_constants END
 * \}
 */
/****************************************************************************************/
/* CONSTANTS                                                                        END */
/****************************************************************************************/


#endif /* _NVM_CFG_LIN_H */
