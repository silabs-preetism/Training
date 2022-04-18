/*******************************************************************************
* @file  rsi_temp_sensor.c
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

/**
 * Includes
 */
#include "rsi_chip.h"

/** 
 * @brief       This API is used to set the count of reference clock on which ptat clock counts
 * @param[in]    pstcTempSens  : pointer to the temperature sensor register instance
 * @param[in]    u32CountFreez : count of reference clock on which ptat clock counts
 * @return      none
 * 
 */
void RSI_TS_SetCntFreez(NPSS_TEMPSENSOR_Type *pstcTempSens, uint32_t u32CountFreez)
{
  pstcTempSens->TS_ENABLE_AND_TEMPERATURE_DONE_b.CONT_COUNT_FREEZE = u32CountFreez;
  return;
}

/** 
 * @brief       This API is used to select the reference clock to the temperature sensor
 * @param[in]    pstcTempSens : pointer to the temperature sensor register instance
 * @param[in]    bRefClk      : reference clock selection
 *                - 0: reference RO clock from analog
 *                - 1: MCU FSM clock
 * @return      none
 * */
void RSI_TS_RefClkSel(NPSS_TEMPSENSOR_Type *pstcTempSens, boolean_t bRefClk)
{

  pstcTempSens->TS_ENABLE_AND_TEMPERATURE_DONE_b.REF_CLK_SEL = bRefClk;
  return;
}

/** 
 * @brief       This API is used to enable / disable the temperature sensor
 * @param[in]    pstcTempSens  : pointer to the temperature sensor register instance
 * @param[in]    bEn           : enable / disable parameter
 *                - 0: Disable
 *                - 1: Enable
 * @return      none
 * */
void RSI_TS_Enable(NPSS_TEMPSENSOR_Type *pstcTempSens, boolean_t bEn)
{
  pstcTempSens->TS_ENABLE_AND_TEMPERATURE_DONE_b.TEMP_SENS_EN = bEn;
  return;
}

/**
 * @brief       This API is used to set the slope of the temperature sensor
 * @param[in]    pstcTempSens  : pointer to the temperature sensor register instance
 * @param[in]    u32Slope      : slope value to be programmed
 * @param[in]    u32Nomial     : calibrated temperature value
 * @param[in]    u32F2Nominal  : ptat clock count during calibration. This will vary with chip to chip
 * @return      none
 * */
void RSI_TS_Config(NPSS_TEMPSENSOR_Type *pstcTempSens, uint32_t u32Nomial)
{
  volatile uint32_t reg_write_data                                  = 0;
  reg_write_data                                                    = RSI_IPMU_RO_TsConfig();
  pstcTempSens->TS_SLOPE_SET_b.SLOPE                                = reg_write_data;
  pstcTempSens->TS_FE_COUNTS_NOMINAL_SETTINGS_b.NOMINAL_TEMPERATURE = u32Nomial;
  reg_write_data                                                    = RSI_IPMU_RO_TsEfuse();
  pstcTempSens->TS_FE_COUNTS_NOMINAL_SETTINGS_b.F2_NOMINAL          = reg_write_data;
  return;
}

/**
 * @brief       This API is used to read the temperature value
 * @param[in]    pstcTempSens   : pointer to the temperature sensor register instance
 * @return      returns the temperature value
 * */
uint32_t RSI_TS_ReadTemp(NPSS_TEMPSENSOR_Type *pstcTempSens)
{
  /*Wait for done */
  while (pstcTempSens->TS_ENABLE_AND_TEMPERATURE_DONE_b.TEMP_MEASUREMENT_DONE != 1)
    ;
  /*Return the temperature value*/
  return pstcTempSens->TEMPERATURE_READ_b.TEMPERATURE_RD;
}

/** 
 * @brief       This API is used to read the temperature value
 * @param[in]    pstcTempSens   : pointer to the temperature sensor register instance
 * @param[in]    enable         : enable or disable bjt based temp sensor
 *                - 0: Disable
 *                - 1: Enable
 * @return      returns the temperature value
 * 
 */
void RSI_TS_RoBjtEnable(NPSS_TEMPSENSOR_Type *pstcTempSens, boolean_t enable)
{
  uint32_t i;

  if (pstcTempSens->TS_SLOPE_SET_b.BJT_BASED_TEMP == 1U) {
    if (enable == 1U) {
      return;
    } else {
      pstcTempSens->TS_SLOPE_SET_b.BJT_BASED_TEMP = enable; //0
    }
  } else {
    if (pstcTempSens->TS_SLOPE_SET_b.BJT_BASED_TEMP == 0U) {
      if (enable == 1U) {
        pstcTempSens->TS_SLOPE_SET_b.BJT_BASED_TEMP = 1;
        for (i = 100; i; i--)
          ; // wait for 100 us
      } else {
        pstcTempSens->TS_SLOPE_SET_b.BJT_BASED_TEMP = enable; //0
      }
    }
  }
}

/**
 * @brief       This API is used to read the temperature value
 * @param[in]    pstcTempSens   : pointer to the temperature sensor register instance
 * @param[in]    temp           : known temprature
 * @return      returns the temperature value
 */
void RSI_TS_LoadBjt(NPSS_TEMPSENSOR_Type *pstcTempSens, uint8_t temp)
{
  pstcTempSens->TS_SLOPE_SET_b.TEMPERATURE_SPI = temp; // update the temp value 	temperature_spi = temp;
  pstcTempSens->TS_SLOPE_SET_b.TEMP_UPDATED    = 1;    // temp_updated =1;
}

/**
 * @brief      This API is used to read the reference clock count
 * @param[in]   pstcTempSens   : pointer to the temperature sensor register instance
 * @return     returns the reference clock count
 * */
uint32_t RSI_TS_GetRefClkCnt(NPSS_TEMPSENSOR_Type *pstcTempSens)
{
  /*Return the count value*/
  return pstcTempSens->TS_COUNTS_READ_B.COUNT_F1;
}

/**
 * @brief      This API is used to read the ptat  clock count
 * @param[in]   pstcTempSens   : pointer to the temperature sensor register instance
 * @return     returns the ptat  clock count
 * */
uint32_t RSI_TS_GetPtatClkCnt(NPSS_TEMPSENSOR_Type *pstcTempSens)
{
  /*Return the count value*/
  return pstcTempSens->TS_COUNTS_READ_B.COUNT_F2;
}

/* @brief      This API is used to update the temp periodically after some time
 * @parm[in]   temp         : pointer to the timeperiod register instance
 * @parm[in]   enable       : enable periodic checking of temp
 * @parm[in]   trigger_time : periodic check time in sec
 * 0 -1 sec
 * 1 -2 sec
 * 2 -4 sec
 * 3 -5 sec
 * */
void RSI_Periodic_TempUpdate(TimePeriodCalib_t *temp, uint8_t enable, uint8_t trigger_time)
{
  temp->MCU_CAL_TEMP_PROG_REG_b.PERIODIC_TEMP_CALIB_EN = enable;
  temp->MCU_CAL_TEMP_PROG_REG_b.TEMP_TRIGGER_TIME_SEL  = trigger_time;
  return;
}

/*End of file not truncated */
