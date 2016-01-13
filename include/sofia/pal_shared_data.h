/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

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
  ---------------------------------------------------------------------------*/
/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/
/*
 * WARNING:
 * If this file is  modified,
 * ensure the regenerated lib_mobilevisor_platform_service.a into
 * pal/release/$(PROJECTFOLDER)/lib_guest/ folder for commit
 * Use command: "make release"
*/

#ifndef PAL_SHARED_DATA_H
#define PAL_SHARED_DATA_H

#ifdef __KERNEL__
#include "linux/types.h"
#else
#include "stdint.h"
#endif

struct pm_control_shared_data {
       volatile uint32_t emif_curr_freq;
       volatile uint32_t emif_clk_src;
       volatile uint32_t prh_user_id;
       volatile uint32_t prh_per_id;
       volatile uint32_t prh_param[20];
       volatile uint32_t reserved0[4];
       volatile uint32_t vm_pow_state_param[20];
       volatile uint32_t vm_cpu_freq_param[20];
       volatile uint32_t prh_request_control_flag;
       volatile uint32_t prh_request_return_value;
       volatile uint32_t target_power_state;
       volatile uint32_t actual_power_state;
       volatile uint32_t modem_state;
       volatile uint32_t exit_latency;
       volatile uint32_t vm_blocker_id;
       volatile uint32_t vm_blocking_reason;
       volatile uint32_t calibration_state;
       volatile uint32_t gsm_sleep_timer_frames_in;
       volatile uint32_t gsm_sleep_timer_frames_out;
       volatile uint32_t gsm_sleep_timer_stopped;
       volatile uint32_t cpu_drv_param;
       volatile uint32_t cpu_scaling_states[7];
       volatile uint32_t cpu_clk;
       volatile uint32_t vcpu_c0[12];
       volatile uint32_t num_dev_sleep_blockers;
};

#define HWID_GPIO_SHARED_MAX_SIZE 4
#define HWID_ADC_SHARED_MAX_SIZE  2

struct hwid_boot_shared_data {
	uint32_t num_of_gpio;
	struct exchange_boot_gpio {
		uint32_t id;
		uint32_t gpio_status;
	} gpio[HWID_GPIO_SHARED_MAX_SIZE];

	uint32_t num_of_adc;
	struct exchange_boot_adc {
		uint32_t id;
		uint32_t sensor_value_ohm;
	} adc[HWID_ADC_SHARED_MAX_SIZE];
};

#define PMIC_ACCESS_MAX_SIZE 64

struct pmic_access_shared_data {
	uint32_t status;
	uint8_t  data[PMIC_ACCESS_MAX_SIZE];
};

/**
Data structure for keeping date and time\n
*/
struct rtc_datetime_shared_data {
	uint16_t   m_year;
	uint8_t    m_month;
	uint8_t    m_day;
	uint8_t    m_hour;
	uint8_t    m_minute;
	uint8_t    m_second;
	uint16_t   m_msecond;
};

enum spcu_hwwup {
	SPCU_HWWUP_SRC_UNDEFINED,
	SPCU_HWWUP_DBB_KPD,         /**< Hardware wakeup by Keypad */
	SPCU_HWWUP_DBB_RES,         /**< Reserved bit */
	SPCU_HWWUP_DBB_SIM1,        /**< Hardware wakeup by SIM1 */
	SPCU_HWWUP_DBB_SIM2,        /**< Hardware wakeup by SIM2 */
	SPCU_HWWUP_DBB_DAP,         /**< Hardware wakeup by DAP */
	SPCU_HWWUP_DBB_OCT,         /**< Hardware wakeup by On-Chip Trace Module */
	SPCU_HWWUP_DBB_CPM,         /**< Hardware wakeup by CAPCOMs */
	SPCU_HWWUP_DBB_DSP,         /**< Hardware wakeup by DSP and FMRadio */
	SPCU_HWWUP_DBB_3G,          /**< Hardware wakeup 3G */
	SPCU_HWWUP_DBB_EXT0,        /**< Hardware wakeup EXTINT 0 */
	SPCU_HWWUP_DBB_EXT1,        /**< Hardware wakeup EXTINT 1 */
	SPCU_HWWUP_DBB_EXT2,        /**< Hardware wakeup EXTINT 2 */
	SPCU_HWWUP_DBB_EXT3,        /**< Hardware wakeup EXTINT 3 */
	SPCU_HWWUP_DBB_EXT4,        /**< Hardware wakeup EXTINT 4 */
	SPCU_HWWUP_DBB_EXT5,        /**< Hardware wakeup EXTINT 5 */
	SPCU_HWWUP_DBB_EXT6,        /**< Hardware wakeup EXTINT 6 */
	SPCU_HWWUP_DBB_EXT7,        /**< Hardware wakeup EXTINT 7 */
	SPCU_HWWUP_DBB_EXT8,        /**< Hardware wakeup EXTINT 8 */
	SPCU_HWWUP_DBB_EXT9,        /**< Hardware wakeup EXTINT 9 */
	SPCU_HWWUP_DBB_EXT10,       /**< Hardware wakeup EXTINT 10 */
	SPCU_HWWUP_DBB_EXT11,       /**< Hardware wakeup EXTINT 11 */
	SPCU_HWWUP_DBB_EXT12,       /**< Hardware wakeup EXTINT 12 */
	SPCU_HWWUP_DBB_EXT13,       /**< Hardware wakeup EXTINT 13 */
	SPCU_HWWUP_DBB_EXT14,       /**< Hardware wakeup EXTINT 14 */
	SPCU_HWWUP_DBB_EXT15,       /**< Hardware wakeup EXTINT 15 */
	SPCU_HWWUP_DBB_EXT16,       /**< Hardware wakeup EXTINT 16 : USB_HS */
	SPCU_HWWUP_DBB_EXT17,       /**< Hardware wakeup EXTINT 17 : SDMMC */
	SPCU_HWWUP_DBB_EXT18,       /**< Hardware wakeup EXTINT 18 : SDIO */
	SPCU_HWWUP_DBB_EXT19,       /**< Hardware wakeup EXTINT 19 : SDIO */
	SPCU_HWWUP_DBB_EXT20,       /**< Hardware wakeup EXTINT 20 : SDIO */
	SPCU_HWWUP_DBB_EXT21,       /**< Hardware wakeup EXTINT 21 : USIF1 */
	SPCU_HWWUP_DBB_EXT22,       /**< Hardware wakeup EXTINT 22 : USIF2 */
	SPCU_HWWUP_DBB_GST_WKUP,    /**< Hardware wakeup GST_WKUP */
	SPCU_HWWUP_DBB_nIRQOUT0,    /**< Hardware wakeup nIRQOUT0 */
	SPCU_HWWUP_DBB_nIRQOUT1,    /**< Hardware wakeup nIRQOUT1 */
	SPCU_HWWUP_DBB_nFIQOUT2,    /**< Hardware wakeup nFIQOUT0 */
	SPCU_HWWUP_DBB_nFIQOUT3,    /**< Hardware wakeup nFIQOUT1 */
	SPCU_HWWUP_DBB_USB_ID,    /**< Hardware wakeup nFIQOUT1 */
	SPCU_HWWUP_DBB_END,
	/**** SPCU ABB bits follow ****/
	SPCU_HWWUP_ABB_WLAN, /**< Hardware wakeup from ABB WLAN  */
	SPCU_HWWUP_ABB_DAP,         /**< Hardware wakeup from ABB DAP  */
	SPCU_HWWUP_ABB_FMR,         /**< Hardware wakeup by FMR */
	SPCU_HWWUP_ABB_PMU,         /**< Hardware wakeup by PMU */
	SPCU_HWWUP_ABB_FSYS1,       /**< Hardware wakeup from FSYS1_EN */
	SPCU_HWWUP_ABB_RTC,         /**< Hardware wakeup by ABB RTC */
	SPCU_HWWUP_ABB_BT,          /**< Hardware wakeup by Bluetooth */
	SPCU_HWWUP_ABB_GLDO,        /**< Hardware wakeup by GLDO */
	SPCU_HWWUP_ABB_ACI_EN,      /**< Hardware wakeup by Accessory In */
	SPCU_HWWUP_DBB_REF_CLK_EN,  /**< Hardware wakeup by DBB REF CLK EN */
	SPCU_HWWUP_ABB_PEN_IRQ,     /**< Hardware wakeup by PEN IRQ */
	SPCU_HWWUP_ABB_FSYS2,       /**< Hardware wakeup by FSYS2_EN */
	SPCU_HWWUP_ABB_G2ARM,       /**< Hardware wakeup by G2ARM_EN */
	SPCU_HWWUP_ABB_GWDG,        /**< Hardware wakeup by GWDG_EN */
	SPCU_HWWUP_ABB_GGPIO,       /**< Hardware wakeup by GGPIO_EN */
	SPCU_WAKEUP_NOT_SLEPT,
	SPCU_HWWUP_END             /**< No Hardware wake up */
};

struct pm_state_shared_data {
	uint32_t s3_count;
	uint32_t pal_power_sleep_disabled[4];
	uint64_t s3_total_res;
	uint32_t last_wakeup_src;
	uint32_t wakeup_counts[SPCU_HWWUP_END-2];
};

#define MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP 64
/**
  @brief pinctrl group information
**/
struct pinctrl_group_info {
	uint32_t pinctrl_pad; /*!< PCL pad index */
	uint32_t pinctrl_pad_value; /*!< pad value */
};

struct pal_shared_data {
	struct pm_control_shared_data pm_control_shared_data;
	struct pmic_access_shared_data pmic_access_shared_data;
	struct rtc_datetime_shared_data rtc_shared_data;
	struct hwid_boot_shared_data hwid_shared_data;
	struct pm_state_shared_data pm_state_shared_data;
	struct pinctrl_group_info
	    pincontrol_group_shared_data[MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP];
};

enum VMM_POWER_STATE_E {
	PM_S0,
	PM_S1,
	PM_S0i3 = 4,
	PM_S3 = 4,
	PM_S5 = 5,
};

enum vmm_pm_opcode {
	PM_OPCODE_START = 0,
	ENTER_IDLE = 1,
	ENTER_SLEEP = 2,
	SYSTEM_READY = 3,
	EMIC_INIT = 4,
	PM_PRH_PRE_INIT_SET_MODE = 5,
	PM_PRH_INIT_SET_MODE = 6,
	PM_PRH_LATE_INIT_SET_MODE = 7,
	PM_PRH_SET_PER_MODE = 8,
	PM_WAKEUP_CONTROL = 9,
	PM_CALIB_CONTROL = 10,
	PM_PRH_SET_PER_MODE_SYNC = 11,
	PM_PRH_SET_PER_MODE_ASYNC = 12,
	PM_CPU_DIS_PRF_INT = 13,
	PM_CPU_EN_PRF_INT = 14,
	PM_CPU_SCALING_START = 15,
	PM_CPU_LOAD2 = 16,
	PM_OMP_SET_POLICY = 17,
	PM_GET_VCPU_C0 = 18,
	PM_REQ_FREQ_CHNG = 19,
	PM_GSM_SLEEP_TIMER_START = 20,
	PM_GSM_SLEEP_TIMER_STOP = 21,
	PM_GSM_SLEEP_TIMER_GET_SLEPT_FRAMES = 22,
	PM_GSM_SLEEP_TIMER_IS_STOPPED = 23,
	PM_CPU_GET_NUM_FREQ_STEPS = 24,
	PM_CPU_GET_FREQ_STEPS = 25,
	PM_S3_COUNTER_UPDATE = 26,
	PM_OPCODE_END = 27,
};


enum boot_mode {
	ENUM_BOOT_MODE_POWER_OFF = 1,
	ENUM_BOOT_MODE_NORMAL,
	ENUM_BOOT_MODE_WEAK_BATTERY_CHARGING,
	ENUM_BOOT_MODE_WEAK_BATTERY_CHARGING_STEP2,
	ENUM_BOOT_MODE_RECOVERY,
	ENUM_BOOT_MODE_RECOVERY_CLEAR,
	ENUM_BOOT_MODE_FASTBOOT,
	ENUM_BOOT_MODE_PTEST,
	ENUM_BOOT_MODE_PTEST_CLEAR,
	ENUM_BOOT_MODE_CHARGEONLY,
	ENUM_BOOT_MODE_SILENT_RESET,
	ENUM_BOOT_MODE_KDUMP,
	ENUM_BOOT_MODE_SHUT_DOWN
};

enum pow_control_id {
	POW_CONTROL_NOT_AVAIL_ID,        /**< wrong id used, hard coded value */
	POW_CONTROL_USIF1_ID,                        /**< USIF1 peripheral ID */
	POW_CONTROL_USIF2_ID,                        /**< USIF2 peripheral ID */
	POW_CONTROL_I2C1_ID,                         /**< I2C1 peripheral ID */
	POW_CONTROL_I2C2_ID,                         /**< I2C2 peripheral ID */
	POW_CONTROL_I2C3_ID,                         /**< I2C3 peripheral ID */
	POW_CONTROL_I2C4_ID,                         /**< I2C4 peripheral ID */
	POW_CONTROL_PWM_ID,                          /**< PWM  peripheral ID */
	POW_CONTROL_RGA_ID,                          /**< RGA  peripheral ID */
	POW_CONTROL_SDMMC1_ID,                       /**< SDMMC1 peripheral ID */
	POW_CONTROL_EMMC_ID,                         /**< EMMC peripheral ID */
	POW_CONTROL_SDIO_ID,                         /**< SDIO peripheral ID */
	POW_CONTROL_CIF_ID,                          /**< CIF peripheral ID */
	POW_CONTROL_DCC_ID,                          /**< DCC peripheral ID */
	POW_CONTROL_KEYPAD_ID,                       /**< KEYPAD peripheral ID */
	POW_CONTROL_CEU_ID,                          /**< CEU peripheral ID */
	POW_CONTROL_CEU2_ID,                         /**< CEU2 peripheral ID */
	POW_CONTROL_USB_HS_ID,                       /**< USB_HS peripheral ID */
	POW_CONTROL_CAPCOM0_ID,                      /**< CAPCOM0 peripheral ID */
	POW_CONTROL_CAPCOM1_ID,                      /**< CAPCOM1 peripheral ID */
	POW_CONTROL_STM_ID,                          /**< STM peripheral ID */
	POW_CONTROL_GPTU0_ID,                        /**< GPTU0 peripheral ID */
	POW_CONTROL_GPTU1_ID,                        /**< GPTU1 peripheral ID */
	POW_CONTROL_PCL_ID,                          /**< PCL peripheral ID */
	POW_CONTROL_RTC_ID,                          /**< RTC peripheral ID */
	POW_CONTROL_USIM_ID,                         /**< USIM peripheral ID */
	POW_CONTROL_USIM2_ID,                        /**< USIM2 peripheral ID */
	POW_CONTROL_PLL_ID,                          /**< PLL peripheral ID */
	POW_CONTROL_CAM_PRIM_ID,                     /**< CAM_PRIM peripheral ID */
	POW_CONTROL_CAM_SEC_ID,                      /**< CAM_SEC peripheral ID */
	POW_CONTROL_PRIM_DISPLAY_ID,                 /**< PRIM_DISPLAY peripheral ID */
	POW_CONTROL_PRIM_DISP_BACKLIGHT_ID,          /**< PRIM_DISP_BACKLIGHT peripheral ID */
	POW_CONTROL_TOUCHSCREEN_ID,                  /**< TOUCHSCREEN peripheral ID */
	POW_CONTROL_TOUCH_SENSOR_ID,                 /**< TOUCH_SENSOR peripheral ID */
	POW_CONTROL_PROXIMITY_SENSOR_ID,             /**< PROXIMITY_SENSOR peripheral ID */
	POW_CONTROL_ACCELEROMETER_ID,                /**< ACCELEROMETER peripheral ID */
	POW_CONTROL_MAGNETOMETER_ID,                 /**< MAGNETOMETER peripheral ID */
	POW_CONTROL_GYROSCOPE_ID,                    /**< GYROSCOPE peripheral ID */
	POW_CONTROL_GSI_ID,                          /**< GSI peripheral ID */
	POW_CONTROL_SHMEM_ID,                        /**< SHMEM peripheral ID */
	POW_CONTROL_GUCIPH_ID,                       /**< GUCIPH peripheral ID */
	POW_CONTROL_ST_ARB_ID,                       /**< ST_ARB peripheral ID */
	POW_CONTROL_ST_OCT_ID,                       /**< ST_OCT peripheral ID */
	POW_CONTROL_ST_MON_ID,                       /**< ST_MON peripheral ID */
	POW_CONTROL_ST_MTM1_ID,                      /**< ST_MTM1 peripheral ID */
	POW_CONTROL_ST_MTM2_ID,                      /**< ST_MTM2 peripheral ID */
	POW_CONTROL_DMA4_CH_ID,                      /**< DMA4_CH peripheral ID */
	POW_CONTROL_DMA8_CH_ID,                      /**< DMA8_CH peripheral ID */
	POW_CONTROL_PS_CPU_ID,                       /**< PS_CPU peripheral ID */
	POW_CONTROL_DSP_2G_ID,                       /**< DSP_2G peripheral ID */
	POW_CONTROL_DSP_AUDIO_ID,                    /**< DSP_AUDIO peripheral ID */
	POW_CONTROL_3G_COMRAM_CPHY_ID,               /**< 3G_COMRAM_CPHY peripheral ID */
	POW_CONTROL_3G_COMRAM_PHY_ID,                /**< 3G_COMRAM_PHY peripheral ID */
	POW_CONTROL_MACPHY_ID,                       /**< MACPHY peripheral ID */
	POW_CONTROL_DIG_RF_ID,                       /**< DIG_RF peripheral ID */
	POW_CONTROL_ST_MON_SB_1_ID,                  /**< ST_MON_SB_1 peripheral ID */
	POW_CONTROL_ST_MON_SB_2_ID,                  /**< ST_MON_SB_2 peripheral ID */
	POW_CONTROL_ST_MON_SB_3_ID,                  /**< ST_MON_SB_3 peripheral ID */
	POW_CONTROL_ST_MON_SB_4_ID,                  /**< ST_MON_SB_4 peripheral ID */
	POW_CONTROL_ST_MON_SB_5_ID,                  /**< ST_MON_SB_5 peripheral ID */
	POW_CONTROL_ST_MON_SB_6_ID,                  /**< ST_MON_SB_6 peripheral ID */
	POW_CONTROL_ST_MON_SB_7_ID,                  /**< ST_MON_SB_7 peripheral ID */
	POW_CONTROL_ST_MON_SB_8_ID,                  /**< ST_MON_SB_8 peripheral ID */
	POW_CONTROL_ST_MON_SB_9_ID,                  /**< ST_MON_SB_9 peripheral ID */
	POW_CONTROL_ST_MON_SB_10_ID,                 /**< ST_MON_SB_10 peripheral ID */
	POW_CONTROL_ST_MON_SB_11_ID,                 /**< ST_MON_SB_11 peripheral ID */
	POW_CONTROL_ST_MON_SB_12_ID,                 /**< ST_MON_SB_12 peripheral ID */
	POW_CONTROL_NANDCTRL_ID,                     /**< NANDCTRL peripheral ID */
	POW_CONTROL_CST_ID,                          /**< CST peripheral ID */
	POW_CONTROL_ETMA5_ID,                        /**< ETMA5 peripheral ID */
	POW_CONTROL_AUDIO_IDI_ID,                    /**< AUDIO_IDI peripheral ID */
	POW_CONTROL_EMIC_ID,                         /**< EMIC peripheral ID */
	POW_CONTROL_USB_PLL_E_480M_ID,               /**< USB_PLL_E_480M peripheral ID */
	POW_CONTROL_GSER_ID,                         /**< GSER peripheral ID */
	POW_CONTROL_IDI_ID,                          /**< IDI peripheral ID */
	POW_CONTROL_OUT0_ID,                         /**< OUT0 peripheral ID */
	POW_CONTROL_OUT1_ID,                         /**< OUT1 peripheral ID */
	POW_CONTROL_ATCPTEST_ID,                     /**< ATCPTEST peripheral ID */
	POW_CONTROL_PMU_IF_ID,                       /**< PMU_IF peripheral ID */
	POW_CONTROL_TSMU_ID,                         /**< TSMU peripheral ID */
	POW_CONTROL_GPU_ID,                          /**< GPU peripheral ID */
	POW_CONTROL_VIDEO_DECODER_ID,                /**< VIDEO_DECODER peripheral ID */
	POW_CONTROL_VIDEO_ENCODER_ID,                /**< VIDEO_ENCODER peripheral ID */
	POW_CONTROL_ABB_BT_IP_ID,                    /**< ABB_BT_IP peripheral ID */
	POW_CONTROL_ABB_BT_IF_ID,                    /**< ABB_BT_IF peripheral ID */
	POW_CONTROL_ABB_BT_AUD_ID,                   /**< ABB_BT_AUD peripheral ID */
	POW_CONTROL_ABB_FMR_ID,                      /**< ABB_FMR peripheral ID */
	POW_CONTROL_ABB_AFE_ID,                      /**< ABB_AFE peripheral ID */
	POW_CONTROL_ABB_IDI_ID,                      /**< ABB_IDI peripheral ID */
	POW_CONTROL_ABB_RTC_ID,                      /**< ABB_RTC peripheral ID */
	POW_CONTROL_ABB_PCL_ID,                      /**< ABB_PCL peripheral ID */
	POW_CONTROL_ABB_VIBRATOR_ID,                 /**< ABB_VIBRATOR peripheral ID */
	POW_CONTROL_ABB_BACKLIGHT_ID,                /**< ABB_BACKLIGHT peripheral ID */
	POW_CONTROL_ABB_DIG_MIC_ID,                  /**< ABB_DIG_MIC peripheral ID */
	POW_CONTROL_ABB_MTM_ID,                      /**< ABB_MTM peripheral ID */
	POW_CONTROL_ABB_ST_ARB_ID,                   /**< ABB_ST_ARB peripheral ID */
	POW_CONTROL_ABB_ST_MON_ID,                   /**< ABB_ST_MON peripheral ID */
	POW_CONTROL_ABB_DCDC_ID,                     /**< ABB_DCDC peripheral ID */
	POW_CONTROL_ABB_PMU_CHP_ID,                  /**< ABB_PMU_CHP peripheral ID */
	POW_CONTROL_ABB_MS_CHP_ID,                   /**< ABB_MS_CHP peripheral ID */
	POW_CONTROL_ABB_I2C_ID,                      /**< ABB_I2C peripheral ID */
	POW_CONTROL_ABB_WLAN_ID,                     /**< ABB_WLAN peripheral ID */
	POW_CONTROL_ABB_USIF_ID,                     /**< ABB_USIF peripheral ID */
	POW_CONTROL_ABB_GNSS_ID,                     /**< ABB_GNSS peripheral ID */
	POW_CONTROL_ABB_AUD_SYNC_ID,                 /**< ABB_AUD_SYNC peripheral ID */
	POW_CONTROL_ABB_ST_MON_SB_1_ID,              /**< ABB_ST_MON_SB_1 peripheral ID */
	POW_CONTROL_ABB_ST_MON_SB_2_ID,              /**< ABB_ST_MON_SB_2 peripheral ID */
	POW_CONTROL_ABB_ST_MON_SB_3_ID,              /**< ABB_ST_MON_SB_3 peripheral ID */
	POW_CONTROL_DEPRECATED_ID,                   /**< DEPRECATED peripheral ID */
	POW_CONTROL_IGNORE_ID,
	POW_CONTROL_NUMBER_PERIPHERALS_ID, /**< available peripheral IDse */
	POW_CONTROL_MAX_PERIPHERALS_ID = 0x7FFFFFFF  /**< hard coded value */
};

struct slb_iram_mode_info {
	uint32_t magic;
	uint32_t version;
	uint32_t next_mode; /* T_ENUM_BOOT_MODE */
	uint32_t check_sum;
};

#define IRAM_MODE_INFO_MAGIC 0x9876abcd
#define IRAM_MODE_INFO_VERSION 1

/*#################################################################*/
#endif
