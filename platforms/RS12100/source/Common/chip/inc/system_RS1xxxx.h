/*******************************************************************************
* @file  system_RS1xxxx.h
* @brief 
*******************************************************************************
* # License
* <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/

#include <stdint.h>

#ifndef __SYSTEM_ARMCM4_H__
#define __SYSTEM_ARMCM4_H__

/**
 * 
 */
#include "rsi_error.h"
#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock; /*!< System Clock Frequency (Core Clock)  */
extern uint32_t npssIntrState;
extern uint32_t __sp;
extern uint8_t SiliconRev;
extern uint8_t package_type;

void RSI_PS_ConfigureTaMemories(void);
/*WiSeAOC specific VTOR values*/
#if defined(WISE_AOC_MODE)
#define VECTOR_OFF_SET_TA_ROM        (0x100000 + 0x70100) /*<! M4 execution from TA ROM                  */
#define VECTOR_OFF_SET_M4_ROM        (0x300000 + 0xB100)  /*<! M4 execution from M4 ROM                  */
#define VECTOR_OFF_SET_TA_ROM_DIRECT (0x80000 + 0x70100)  /*<! M4 execution from TA ROM in directed mode */
#endif                                                    //WISE_AOC_MODE

typedef enum SLEEP_TYPE {
  SLEEP_WITH_RETENTION,    /*Saves the CPU context while entering into sleep */
  SLEEP_WITHOUT_RETENTION, /*do not Save the CPU context while entering into sleep */
} SLEEP_TYPE_T;

#define DISABLE_LF_MODE           0x00
#define LF_32_KHZ_RC              BIT(0)
#define LF_32_KHZ_XTAL            BIT(1)
#define EXTERNAL_CAP_MODE         BIT(2)
#define HF_MHZ_RO                 3
#define BG_SLEEP_TIMER_REG_OFFSET 0x125
/*System default clocks*/
#define DEFAULT_SOC_PLL_CLOCK    80000000
#define DEFAULT_INTF_PLL_CLOCK   80000000
#define DEFAULT_MODEM_PLL_CLOCK  80000000
#define DEFAULT_32MHZ_RC_CLOCK   32000000
#define DEFAULT_20MHZ_RO_CLOCK   20000000
#define DEFAULT_DOUBLER_CLOCK    (2 * DEFAULT_20MHZ_RO_CLOCK)
#define DEFAULT_32KHZ_RC_CLOCK   32000
#define DEFAULT_32KHZ_RO_CLOCK   32000
#define DEFAULT_32KHZ_XTAL_CLOCK 32000
#define DEFAULT_RF_REF_CLOCK     40000000
#define DEFAULT_MEMS_REF_CLOCK   40000000
#define DEFAULT_BYP_RC_CLOCK     32000000
#define DEFAULT_I2S_PLL_CLOCK    6144000
#define DEFAULT_REF_CLOCK        2
/* Flash offset to read Silicon revision */
#define TA_FLASH_BASE     0x04000000
#define SILICON_REV_WMCU  (*(volatile uint32_t *)(TA_FLASH_BASE + 1024 + 55))
#define PACKAGE_TYPE_WMCU (*(volatile uint32_t *)(TA_FLASH_BASE + 992 + 86))

#define M4_FLASH_BASE    0x08000000
#define SILICON_REV_MCU  (*(volatile uint32_t *)(M4_FLASH_BASE + 1024 + 55))
#define PACKAGE_TYPE_MCU (*(volatile uint32_t *)(M4_FLASH_BASE + 992 + 86))

/* system clock source look up table*/
typedef struct SYSTEM_CLOCK_SOURCE_FREQUENCIES {
  uint32_t m4ss_ref_clk;
  uint32_t ulpss_ref_clk;
  uint32_t soc_pll_clock;
  uint32_t soc_clock;
  uint32_t modem_pll_clock;
  uint32_t intf_pll_clock;
  uint32_t i2s_pll_clock;
  uint32_t modem_pll_clock2;
  uint32_t ro_32khz_clock;
  uint32_t rc_32khz_clock;
  uint32_t xtal_32khz_clock;
  uint32_t rc_32mhz_clock;
  uint32_t ro_20mhz_clock;
  uint32_t doubler_clock;
  uint32_t sleep_clock;
  uint32_t rf_ref_clock;
  uint32_t mems_ref_clock;
  uint32_t byp_rc_ref_clock;
  uint32_t m4_ref_clock_source;
  uint32_t ulp_ref_clock_source;
} SYSTEM_CLOCK_SOURCE_FREQUENCIES_T;

extern SYSTEM_CLOCK_SOURCE_FREQUENCIES_T system_clocks;

/**
 * \ingroup   RSI_SPECIFIC_DRIVERS
 */

/**
 * \defgroup RSI_SYSTEM_RS1XXXX RSI:SYSTEM RS1XXXX  
 *  @{
 *
 */

extern void SystemInit(void);

extern void SystemCoreClockUpdate(void);

// this function is used to shift soc clock to pll clock

// extern void shift_soc_clk_to_160(void);

error_t RSI_PS_EnterDeepSleep(SLEEP_TYPE_T sleepType, uint8_t lf_clk_mode);

void RSI_PS_SaveCpuContext(void);
void RSI_PS_RestoreCpuContext(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_ARMCM4_H__ */
