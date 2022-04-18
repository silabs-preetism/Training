/*******************************************************************************
* @file  rsi_pwm.c
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
/*************************************************************************
 *
 */

/***********************************************************************
													INCLUDE FILES														  
 ***********************************************************************/
#include "rsi_ccp_user_config.h"
#ifndef ROMDRIVER_PRESENT
#include "rsi_rom_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn          RSI_DRIVER_VERSION RSI_MCPWM_GetVersion(void)
 * @brief		    This API is used to get version information of the driver implementation
 * @param[in]	  none
 * @return 		    structure of type RSI_DRIVER_VERSION and its members are as below
								 - \ref RSI_MCPWM_API_VERSION : Version of the CMSIS-Driver specification used to implement this driver.
								 - \ref RSI_MCPWM_DRV_VERSION : MCPWM peripheral source code version of the actual driver implementation. 
 */
RSI_DRIVER_VERSION RSI_MCPWM_GetVersion(void)
{
  RSI_DRIVER_VERSION vsDriverVersion;

  vsDriverVersion.api = RSI_MCPWM_API_VERSION;
  vsDriverVersion.drv = RSI_MCPWM_API_VERSION;
  return vsDriverVersion;
}
/**
 * @fn          RSI_MCPWM_CAPABILITIES RSI_MCPWM_GetCapabilities(void)
 * @brief		    The function RSI_MCPWM_GetCapabilities returns information about capabilities of this driver implementation. 
								The data fields of the struct RSI_MCPWM_CAPABILITIES encodes the driver capabilities.
 * @param[in]	  none
 * @return 		    structure of type RSI_MCPWM_CAPABILITIES and its members are as below
							  - \ref PWMOutputs    : Number of PWM outputs
							  - \ref FaultPins     : Number of fault input pins							
 */
RSI_MCPWM_CAPABILITIES_T RSI_MCPWM_GetCapabilities(void)
{
  RSI_MCPWM_CAPABILITIES_T vsDriverCapabilities;

  vsDriverCapabilities.faultPins  = 2;
  vsDriverCapabilities.pwmOutputs = 8;
  return vsDriverCapabilities;
}

/* This API is used to start the MCPWM operation for required channel */
error_t mcpwm_start(RSI_MCPWM_T *pMCPWM, uint8_t chnlNum)
{
  /* starts base timer operation */
  switch (chnlNum) {
    case PWM_CHNL_0:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH0_b.PWM_TIME_BASE_EN_FRM_REG_CH0 = 0x1;
      break;
    case PWM_CHNL_1:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH1_b.PWM_TIME_BASE_EN_FRM_REG_CH1 = 0x1;
      break;
    case PWM_CHNL_2:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH2_b.PWM_TIME_BASE_EN_FRM_REG_CH2 = 0x1;
      break;
    case PWM_CHNL_3:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH3_b.PWM_TIME_BASE_EN_FRM_REG_CH3 = 0x1;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/*This API is used to stops the MCPWM operation for required channel */
error_t mcpwm_stop(RSI_MCPWM_T *pMCPWM, uint8_t chnlNum)
{
  /* Stops base timer operation */
  switch (chnlNum) {
    case PWM_CHNL_0:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH0_b.PWM_TIME_BASE_EN_FRM_REG_CH0 = DISABLE;
      break;
    case PWM_CHNL_1:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH1_b.PWM_TIME_BASE_EN_FRM_REG_CH1 = DISABLE;
      break;
    case PWM_CHNL_2:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH2_b.PWM_TIME_BASE_EN_FRM_REG_CH2 = DISABLE;
      break;
    case PWM_CHNL_3:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH3_b.PWM_TIME_BASE_EN_FRM_REG_CH3 = DISABLE;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to set time period and counter initial,
   value for the required MCPWM channel */
error_t mcpwm_set_time_period(RSI_MCPWM_T *pMCPWM, uint8_t chnlNum, uint16_t period, uint16_t initVal)
{
  /* Sets initial value and timer period for time base counter */
  switch (chnlNum) {
    case PWM_CHNL_0:
      pMCPWM->PWM_TIME_PRD_CNTR_WR_REG_CH0_b.PWM_TIME_PRD_CNTR_WR_REG_CH0 = initVal;
      pMCPWM->PWM_TIME_PRD_WR_REG_CH0_b.PWM_TIME_PRD_REG_WR_VALUE_CH0     = period;
      break;
    case PWM_CHNL_1:
      pMCPWM->PWM_TIME_PRD_CNTR_WR_REG_CH1_b.PWM_TIME_PRD_CNTR_WR_REG_CH1 = initVal;
      pMCPWM->PWM_TIME_PRD_WR_REG_CH1_b.PWM_TIME_PRD_REG_WR_VALUE_CH1     = period;
      break;
    case PWM_CHNL_2:
      pMCPWM->PWM_TIME_PRD_CNTR_WR_REG_CH2_b.PWM_TIME_PRD_CNTR_WR_REG_CH2 = initVal;
      pMCPWM->PWM_TIME_PRD_WR_REG_CH2_b.PWM_TIME_PRD_REG_WR_VALUE_CH2     = period;
      break;
    case PWM_CHNL_3:
      pMCPWM->PWM_TIME_PRD_CNTR_WR_REG_CH3_b.PWM_TIME_PRD_CNTR_WR_REG_CH3 = initVal;
      pMCPWM->PWM_TIME_PRD_WR_REG_CH3_b.PWM_TIME_PRD_REG_WR_VALUE_CH3     = period;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to configure special event trigger generation for required MCPWM 
   channel which allows the A/D converter to be synchronized to the PWM time base */
void mcpwm_special_event_trigger_config(RSI_MCPWM_T *pMCPWM, boolean_t svtDir, RSI_MCPWM_SVT_CONFIG_T *pMCPWMSVTConfig)
{
  /* sets counter direction to generate special event trigger */
  if (svtDir == COUNTDOWN) {
    pMCPWM->PWM_SVT_CTRL_SET_REG = (1 << 1);
  }
  if (svtDir == COUNTUP) {
    pMCPWM->PWM_SVT_CTRL_RESET_REG = (1 << 1);
  }
  /* selct the channel to generate SVT */
  pMCPWM->PWM_TIME_PRD_COMMON_REG_b.PWM_TIME_PRD_COMMON_TIMER_VALUE = pMCPWMSVTConfig->svtChannel;
  /* sets post scalar */
  pMCPWM->PWM_SVT_PARAM_REG_b.SVT_POSTSCALER_SELECT = pMCPWMSVTConfig->svtPostscalar;
  /* compare value for SVT */
  pMCPWM->PWM_SVT_COMPARE_VALUE_REG = pMCPWMSVTConfig->svtCompareVal;
}

/* This API is used to Configure Dead time insertion parameters for MCPWM */
error_t mcpwm_dead_time_value_set(RSI_MCPWM_T *pMCPWM, RSI_MCPWM_DT_CONFIG_T *pMCPWMDeadTimeConfig, uint8_t chnlNum)
{
  if ((pMCPWMDeadTimeConfig->counterSelect == COUNTER_A) || (pMCPWMDeadTimeConfig->counterSelect == COUNTER_B)) {
    /* Sets prescale for counter A or counter B */
    switch (chnlNum) {
      case PWM_CHNL_0: {
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_A) {
          if ((pMCPWMDeadTimeConfig->preScaleA >= 0) && (pMCPWMDeadTimeConfig->preScaleA <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_A_b.DEADTIME_PRESCALE_SELECT_A =
              (pMCPWMDeadTimeConfig->preScaleA << 0);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_B) {
          if ((pMCPWMDeadTimeConfig->preScaleB >= 0) && (pMCPWMDeadTimeConfig->preScaleB <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_B_b.DEADTIME_PRESCALE_SELECT_B =
              (pMCPWMDeadTimeConfig->preScaleB << 0);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
      } break;
      case PWM_CHNL_1: {
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_A) {
          if ((pMCPWMDeadTimeConfig->preScaleA >= 0) && (pMCPWMDeadTimeConfig->preScaleA <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_A_b.DEADTIME_PRESCALE_SELECT_A =
              (pMCPWMDeadTimeConfig->preScaleA << 2);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_B) {
          if ((pMCPWMDeadTimeConfig->preScaleB >= 0) && (pMCPWMDeadTimeConfig->preScaleB <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_B_b.DEADTIME_PRESCALE_SELECT_B =
              (pMCPWMDeadTimeConfig->preScaleB << 2);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
      } break;
      case PWM_CHNL_2: {
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_A) {
          if ((pMCPWMDeadTimeConfig->preScaleA >= 0) && (pMCPWMDeadTimeConfig->preScaleA <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_A_b.DEADTIME_PRESCALE_SELECT_A =
              (pMCPWMDeadTimeConfig->preScaleA << 4);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_B) {
          if ((pMCPWMDeadTimeConfig->preScaleB >= 0) && (pMCPWMDeadTimeConfig->preScaleB <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_B_b.DEADTIME_PRESCALE_SELECT_B =
              (pMCPWMDeadTimeConfig->preScaleB << 4);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
      } break;
      case PWM_CHNL_3: {
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_A) {
          if ((pMCPWMDeadTimeConfig->preScaleA >= 0) && (pMCPWMDeadTimeConfig->preScaleA <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_A_b.DEADTIME_PRESCALE_SELECT_A =
              (pMCPWMDeadTimeConfig->preScaleA << 6);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
        if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_B) {
          if ((pMCPWMDeadTimeConfig->preScaleB >= 0) && (pMCPWMDeadTimeConfig->preScaleB <= 3)) {
            pMCPWM->PWM_DEADTIME_PRESCALE_SELECT_B_b.DEADTIME_PRESCALE_SELECT_B =
              (pMCPWMDeadTimeConfig->preScaleB << 6);
          } else {
            return ERROR_PWM_INVALID_ARG;
          }
        }
      } break;
      default:
        return ERROR_PWM_INVALID_CHNLNUM;
    }
    /* Sets DeadTime for counter A */
    if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_A) {
      pMCPWM->PWM_DEADTIME[chnlNum].PWM_DEADTIME_A_b.DEADTIME_A_CH = pMCPWMDeadTimeConfig->deadTimeA;
    }
    /* Sets DeadTime for counter B */
    if (pMCPWMDeadTimeConfig->counterSelect == COUNTER_B) {
      pMCPWM->PWM_DEADTIME[chnlNum].PWM_DEADTIME_B_b.DEADTIME_B_CH = pMCPWMDeadTimeConfig->deadTimeB;
    }
  } else {
    return ERROR_PWM_INVALID_ARG;
  }
  return RSI_OK;
}

/* This API is used to reset the required channel of MCPWM */
error_t mcpwm_channel_reset(RSI_MCPWM_T *pMCPWM, uint8_t chnlNum)
{
  /* Resets operation of MCPWM channel */
  switch (chnlNum) {
    case PWM_CHNL_0:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH0_b.PWM_SFT_RST = ENABLE;
      break;
    case PWM_CHNL_1:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH1_b.PWM_SFT_RST = ENABLE;
      break;
    case PWM_CHNL_2:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH2_b.PWM_SFT_RST = ENABLE;
      break;
    case PWM_CHNL_3:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH3_b.PWM_SFT_RST = ENABLE;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to reset the counter from required channel of MCPWM */
error_t mcpwm_counter_reset(RSI_MCPWM_T *pMCPWM, uint8_t chnlNum)
{
  /* resets counter operations */
  switch (chnlNum) {
    case PWM_CHNL_0:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH0_b.PWM_TIME_PRD_CNTR_RST_FRM_REG = ENABLE;
      break;
    case PWM_CHNL_1:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH1_b.PWM_TIME_PRD_CNTR_RST_FRM_REG = ENABLE;
      break;
    case PWM_CHNL_2:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH2_b.PWM_TIME_PRD_CNTR_RST_FRM_REG = ENABLE;
      break;
    case PWM_CHNL_3:
      pMCPWM->PWM_TIME_PRD_CTRL_REG_CH3_b.PWM_TIME_PRD_CNTR_RST_FRM_REG = ENABLE;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to set base time period control for the required MCPWM channel */
error_t mcpwm_period_control_config(RSI_MCPWM_T *pMCPWM, uint32_t postScale, uint32_t preScale, uint8_t chnlNum)
{
  /* sets prescale and post scale values */
  switch (chnlNum) {
    case PWM_CHNL_0:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH0_b.PWM_TIME_PRD_POST_SCALAR_VALUE_CH0 = postScale;
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH0_b.PWM_TIME_PRD_PRE_SCALAR_VALUE_CH0  = preScale;
      break;
    case PWM_CHNL_1:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH1_b.PWM_TIME_PRD_POST_SCALAR_VALUE_CH1 = postScale;
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH1_b.PWM_TIME_PRD_PRE_SCALAR_VALUE_CH1  = preScale;
      break;
    case PWM_CHNL_2:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH2_b.PWM_TIME_PRD_POST_SCALAR_VALUE_CH2 = postScale;
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH2_b.PWM_TIME_PRD_PRE_SCALAR_VALUE_CH2  = preScale;
      break;
    case PWM_CHNL_3:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH3_b.PWM_TIME_PRD_POST_SCALAR_VALUE_CH3 = postScale;
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH3_b.PWM_TIME_PRD_PRE_SCALAR_VALUE_CH3  = preScale;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to set fault A pin output value to be overridden 
   when fault condition occurs */
error_t mcpwm_fault_avalue_set(RSI_MCPWM_T *pMCPWM, uint8_t pwmOutput, uint8_t value)
{
  switch (pwmOutput) {
    case PWM_OUTPUT_L0:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_L0 = value;
      break;
    case PWM_OUTPUT_H0:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_H0 = value;
      break;
    case PWM_OUTPUT_L1:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_L1 = value;
      break;
    case PWM_OUTPUT_H1:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_H1 = value;
      break;
    case PWM_OUTPUT_L2:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_L2 = value;
      break;
    case PWM_OUTPUT_H2:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_H2 = value;
      break;
    case PWM_OUTPUT_L3:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_L3 = value;
      break;
    case PWM_OUTPUT_H3:
      pMCPWM->PWM_FLT_A_OVERRIDE_VALUE_REG_b.PWM_FLT_A_OVERRIDE_VALUE_H3 = value;
      break;
    default:
      return ERROR_PWM_INVALID_PWMOUT;
  }
  return RSI_OK;
}

/* This API is used to set fault B pin output value to be overridden 
   when fault condition occurs */
error_t mcpwm_fault_bvalue_set(RSI_MCPWM_T *pMCPWM, uint8_t pwmOutput, uint8_t value)
{
  switch (pwmOutput) {
    case PWM_OUTPUT_L0:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_L0 = value;
      break;
    case PWM_OUTPUT_H0:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_H0 = value;
      break;
    case PWM_OUTPUT_L1:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_L1 = value;
      break;
    case PWM_OUTPUT_H1:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_H1 = value;
      break;
    case PWM_OUTPUT_L2:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_L2 = value;
      break;
    case PWM_OUTPUT_H2:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_H2 = value;
      break;
    case PWM_OUTPUT_L3:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_L3 = value;
      break;
    case PWM_OUTPUT_H3:
      pMCPWM->PWM_FLT_B_OVERRIDE_VALUE_REG_b.PWM_FLT_B_OVERRIDE_VALUE_H3 = value;
      break;
    default:
      return ERROR_PWM_INVALID_PWMOUT;
  }
  return RSI_OK;
}

/* This API is used to set the mode of base timer for required channel */
error_t mcpwm_set_base_timer_mode(RSI_MCPWM_T *pMCPWM, uint8_t mode, uint8_t chnlNum)
{
  switch (chnlNum) {
    case 0:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH0_b.TMR_OPEARATING_MODE_CH0 = mode;
      break;
    case 1:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH1_b.TMR_OPEARATING_MODE_CH1 = mode;
      break;
    case 2:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH2_b.TMR_OPEARATING_MODE_CH2 = mode;
      break;
    case 3:
      pMCPWM->PWM_TIME_PRD_PARAM_REG_CH3_b.TMR_OPEARATING_MODE_CH3 = mode;
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to set output mode for the MCPWM */
error_t mcpwm_set_output_mode(RSI_MCPWM_T *pMCPWM, boolean_t mode, uint8_t chnlNum)
{
  switch (chnlNum) {
    case 0:
      if (mode) {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_SET_REG = (1 << 12);
      } else {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_RESET_REG = (1 << 12);
      }
      break;
    case 1:
      if (mode) {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_SET_REG = (1 << 13);
      } else {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_RESET_REG = (1 << 13);
      }
      break;
    case 2:
      if (mode) {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_SET_REG = (1 << 14);
      } else {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_RESET_REG = (1 << 14);
      }
      break;
    case 3:
      if (mode) {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_SET_REG = (1 << 15);
      } else {
        pMCPWM->PWM_FLT_OVERRIDE_CTRL_RESET_REG = (1 << 15);
      }
      break;
    default:
      return ERROR_PWM_INVALID_CHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to set output polarity for MCPWM */
void mcpwm_set_output_polarity(RSI_MCPWM_T *pMCPWM, boolean_t polL, boolean_t polH)
{
  if (polL) {
    pMCPWM->PWM_FLT_OVERRIDE_CTRL_SET_REG = (1 << 3);
  } else {
    pMCPWM->PWM_FLT_OVERRIDE_CTRL_RESET_REG = (1 << 3);
  }
  if (polH) {
    pMCPWM->PWM_FLT_OVERRIDE_CTRL_SET_REG = (1 << 2);
  } else {
    pMCPWM->PWM_FLT_OVERRIDE_CTRL_RESET_REG = (1 << 2);
  }
}

/* Handles all interrupt flags of MCPWM */
void mcpwm_interrupt_handler(RSI_MCPWM_T *pMCPWM, RSI_MCPWM_CALLBACK_T *pCallBack)
{
  uint32_t intrStat = 0;

  intrStat = pMCPWM->PWM_INTR_STS;
  if ((pCallBack->cbFunc) != NULL) {
    if (intrStat & RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH0) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH0);
      pCallBack->cbFunc(RISE_TIME_PERIOD_MATCH_CH0);
    }
    if (intrStat & RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH0) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH0);
      pCallBack->cbFunc(TIME_PERIOD_MATCH_CH0);
    }
    if (intrStat & RSI_MCPWM_EVENT_FAULT_A) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_FAULT_A);
      pCallBack->cbFunc(FAULT_A);
    }
    if (intrStat & RSI_MCPWM_EVENT_FAULT_B) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_FAULT_B);
      pCallBack->cbFunc(FAULT_B);
    }
    if (intrStat & RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH1) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH1);
      pCallBack->cbFunc(RISE_TIME_PERIOD_MATCH_CH1);
    }
    if (intrStat & RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH1) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH1);
      pCallBack->cbFunc(TIME_PERIOD_MATCH_CH1);
    }
    if (intrStat & RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH2) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH2);
      pCallBack->cbFunc(RISE_TIME_PERIOD_MATCH_CH2);
    }
    if (intrStat & RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH2) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH2);
      pCallBack->cbFunc(TIME_PERIOD_MATCH_CH2);
    }
    if (intrStat & RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH3) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_RISE_TIME_PERIOD_MATCH_CH3);
      pCallBack->cbFunc(RISE_TIME_PERIOD_MATCH_CH3);
    }
    if (intrStat & RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH3) {
      RSI_MCPWM_InterruptClear(pMCPWM, RSI_MCPWM_EVENT_TIME_PERIOD_MATCH_CH3);
      pCallBack->cbFunc(TIME_PERIOD_MATCH_CH3);
    }
  } else {
  }
}
/*ROM API Structure
const ROM_MCPWM_API_T mcpwm_api = {
		&mcpwm_counter_reset,
		&mcpwm_channel_reset,
		&mcpwm_start,
		&mcpwm_stop,
		&mcpwm_set_time_period,
		&mcpwm_special_event_trigger_config,
		&mcpwm_dead_time_value_set,
		&mcpwm_period_control_config,
		&mcpwm_fault_avalue_set,
		&mcpwm_fault_bvalue_set,
		&mcpwm_set_base_timer_mode,			
		&mcpwm_set_output_mode,
		&mcpwm_set_output_polarity,
		&mcpwm_interrupt_handler	
};
#endif
*/
#ifdef __cplusplus
}
#endif
#endif //ROMDRIVER_PRESENT
/************** END OF FILE *************/
