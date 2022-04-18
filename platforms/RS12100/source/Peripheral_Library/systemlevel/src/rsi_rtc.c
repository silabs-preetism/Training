/*******************************************************************************
* @file  rsi_rtc.c
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
#define CENTURTY_START 0

/**
 * @fn					void RSI_RTC_Start(RTC_Type *Cal)
 * @brief   	 	This API is used to start the RTC
 * @param[in] 	Cal :pointer to the rtc register instance
 * @return   		None
 */
void RSI_RTC_Start(RTC_Type *Cal)
{
#ifdef CHIP_9117
  if (0x1 == STATIC_COMBI_RTC_PG_ENABLE) {
    /* Enable static combi rtc powergate */
    Cal->MCU_CAL_POWERGATE_REG_b.DISABLE_COMBI_DYN_PWRGATE_EN = 0x1;
    Cal->MCU_CAL_POWERGATE_REG_b.STATIC_COMBI_RTC_PG_EN       = 0x1;
  } else {
    /* Disable static combi rtc powergate*/
    Cal->MCU_CAL_POWERGATE_REG_b.STATIC_COMBI_RTC_PG_EN = 0x0;
  }
#endif
  Cal->MCU_CAL_POWERGATE_REG_b.ENABLE_CALENDER_COMBI = 1u;
  Cal->MCU_CAL_POWERGATE_REG_b.PG_EN_CALENDER        = 1u;
  return;
}

/**
 * @fn					void RSI_RTC_Init(RTC_Type *Cal)
 * @brief    		This API is used to init the rtc block
 * @param[in] 	Cal : pointer to the rtc register instance
 * @return  	 	None
 * */
void RSI_RTC_Init(RTC_Type *Cal)
{
  /* Power-Up RTC Domain */
  RSI_PS_NpssPeriPowerUp(SLPSS_PWRGATE_ULP_MCURTC | SLPSS_PWRGATE_ULP_TIMEPERIOD);

  /*Wait for RTC-ON pluse synchronization*/
  do {
    RTC->MCU_CAL_ALARM_PROG_1 = 6U;
  } while (RTC->MCU_CAL_ALARM_PROG_1 != 6U);

  //TODO: Update the RTC clock INIT code here
  /* Time Period Programming */
  RSI_TIMEPERIOD_TimerClkSel(TIME_PERIOD, 0x003E7FFF);
  /*Enable power gates*/
  RSI_RTC_Start(Cal);
  return;
}

/**
 * @fn					void RSI_RTC_Stop(RTC_Type *Cal)
 * @brief   		This API is used to stop the rtc operation
 * @param[in]		Cal : pointer to the rtc register instance
 * @return  		None
 * */
void RSI_RTC_Stop(RTC_Type *Cal)
{
  Cal->MCU_CAL_POWERGATE_REG_b.ENABLE_CALENDER_COMBI = 0u;
  Cal->MCU_CAL_POWERGATE_REG_b.PG_EN_CALENDER        = 0u;
  return;
}

/**
 * @fn		error_t RSI_RTC_SetDateTime(RTC_Type *Cal , RTC_TIME_CONFIG_T *date)	
 * @brief   	   This API is used to set the rtc configuration
 * @param[in]    Cal   : pointer to the rtc register instance
 * @param[in]    date  : pointer to the rtc configuration structure
 * @return       Non zero on success and error code on failure
 * */
error_t RSI_RTC_SetDateTime(RTC_Type *Cal, RTC_TIME_CONFIG_T *date)
{
  if ((Cal == NULL) || (date == NULL)) {
    return ERROR_CAL_INVALID_PARAMETERS;
  }
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_DAY      = date->Day;
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_WEEK_DAY = date->DayOfWeek;
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_MONTH    = date->Month;
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_YEAR     = date->Year;
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_CENTURY  = date->Century;

  Cal->MCU_CAL_PROG_TIME_1_b.PROG_MSEC      = date->MilliSeconds;
  Cal->MCU_CAL_PROG_TIME_1_b.PROG_SEC       = date->Second;
  Cal->MCU_CAL_PROG_TIME_1_b.PROG_MIN       = date->Minute;
  Cal->MCU_CAL_PROG_TIME_1_b.PROG_HOUR      = date->Hour;
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_TIME_TRIG = 1U;
  /*Wait for applied values to updated to registers */
  while (Cal->MCU_CAL_READ_TIME_LSB_b.MILLISECONDS_COUNT != date->MilliSeconds)
    ;
  while (Cal->MCU_CAL_READ_TIME_LSB_b.SECONDS_COUNT != date->Second)
    ;
  while (Cal->MCU_CAL_READ_TIME_LSB_b.MINS_COUNT != date->Minute)
    ;
  while (Cal->MCU_CAL_READ_TIME_LSB_b.HOURS_COUNT != date->Hour)
    ;
  return RSI_OK;
}

/**
 *@fn						error_t RSI_RTC_GetDateTime(RTC_Type *Cal ,  RTC_TIME_CONFIG_T *date)
 *@brief    	 	This API is used to Get the RTC time
 *@param[in]  	Cal  : pointer to the rtc register instance
 *@param[in] 		date : pointer to the rtc structure to hold the current time parameters
 *@return    		Non zero on success and error code on failure
 * */
error_t RSI_RTC_GetDateTime(RTC_Type *Cal, RTC_TIME_CONFIG_T *date)
{
  if ((Cal == NULL) || (date == NULL)) {
    return ERROR_CAL_INVALID_PARAMETERS;
  }
  date->MilliSeconds = Cal->MCU_CAL_READ_TIME_LSB_b.MILLISECONDS_COUNT;
  date->Second       = Cal->MCU_CAL_READ_TIME_LSB_b.SECONDS_COUNT;
  date->Minute       = Cal->MCU_CAL_READ_TIME_LSB_b.MINS_COUNT;
  date->Hour         = Cal->MCU_CAL_READ_TIME_LSB_b.HOURS_COUNT;
  date->DayOfWeek    = (RTC_DAY_OF_WEEK_T)Cal->MCU_CAL_READ_TIME_MSB_b.WEEK_DAY;
  date->Day          = Cal->MCU_CAL_READ_TIME_LSB_b.DAYS_COUNT;
  date->Month        = (RTC_MONTH_T)Cal->MCU_CAL_READ_TIME_MSB_b.MONTHS_COUNT;
  date->Year         = Cal->MCU_CAL_READ_TIME_MSB_b.YEAR_COUNT;
  date->Century      = Cal->MCU_CAL_READ_TIME_MSB_b.CENTURY_COUNT;

  return RSI_OK;
}

/**
 * @fn						 error_t RSI_RTC_SetAlarmDateTime(RTC_Type *Cal , RTC_TIME_CONFIG_T *alarm)			
 * @brief    			 This API is used to Set the alarm for RTC module
 * @param[in]  		 Cal   : pointer to the rtc register instance
 * @param[in]  		 alarm : pointer to alarm configuration structure
 * @return    		 Non zero on success and error code on failure
 * */
error_t RSI_RTC_SetAlarmDateTime(RTC_Type *Cal, RTC_TIME_CONFIG_T *alarm)
{
  if ((Cal == NULL) || (alarm == NULL)) {
    return ERROR_CAL_INVALID_PARAMETERS;
  }

  Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_HOUR = alarm->Hour;         //HH
  Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_MIN  = alarm->Minute;       //MM
  Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_SEC  = alarm->Second;       //SS
  Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_MSEC = alarm->MilliSeconds; // MSEC

  Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_CENTURY = alarm->Century; // CC
  Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_DAY     = alarm->Day;     // DAY
  Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_MONTH   = alarm->Month;   //MONTH
  Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_YEAR    = alarm->Year;    // YEAR
  /*Wait for applied values to updated to registers */
  while (Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_MIN != alarm->Minute)
    ;
  while (Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_SEC != alarm->Second)
    ;
  while (Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_MSEC != alarm->MilliSeconds)
    ;
  return RSI_OK;
}

/**
 * @fn						void RSI_RTC_AlamEnable(RTC_Type *Cal , boolean_t val)
 * @brief   		  This API is used to Enable alarm in RTC
 * @param[in] 		Cal : pointer to the rtc register instance
 * @param[in] 		val    : to enable or disable the alarm
 *           				-\b 0 : Disable the alarm
 *            			-\b 1 : Enable the alarm
 * @return  			None
 * */
void RSI_RTC_AlamEnable(RTC_Type *Cal, boolean_t val)
{
  Cal->MCU_CAL_ALARM_PROG_2_b.ALARM_EN = val;
  return;
}

/**
 * @fn			error_t RSI_RTC_GetAlarmDateTime(RTC_Type *Cal , RTC_TIME_CONFIG_T *alarm)
 * @brief   		 This API is used to Get alarm configurations for RTC
 * @param[in]		 Cal 	 : pointer to the rtc register instance
 * @param[in]		 alarm : pointer to the rtc alarm configuration structure
 * @return			 Non zero on success and error code on failure
 * */
error_t RSI_RTC_GetAlarmDateTime(RTC_Type *Cal, RTC_TIME_CONFIG_T *alarm)
{
  if ((Cal == NULL) || (alarm == NULL)) {
    return ERROR_CAL_INVALID_PARAMETERS;
  }

  alarm->Hour         = Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_HOUR; //HH
  alarm->Minute       = Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_MIN;  //MM
  alarm->Second       = Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_SEC;  //SS
  alarm->MilliSeconds = Cal->MCU_CAL_ALARM_PROG_1_b.PROG_ALARM_MSEC; // MSEC

  alarm->Century = Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_CENTURY;            // CC
  alarm->Day     = Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_DAY;                // DAY
  alarm->Month   = (RTC_MONTH_T)Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_MONTH; //MONTH
  alarm->Year    = Cal->MCU_CAL_ALARM_PROG_2_b.PROG_ALARM_YEAR;               // YEAR
  return RSI_OK;
}

/**
 * @fn					 void RSI_RTC_SetDayOfWeek(RTC_Type *Cal , RTC_DAY_OF_WEEK_T dayInWeek)
 * @brief   		 This API is used to set the Day in a week
 * @param[in]		 Cal 	   : pointer to the rtc register instance
 * @param[in]		 dayInWeek : enum value of days in a week
 *														- \ref Sunday      
 *   													- \ref Monday
 *   													- \ref Tuesday       
 *  													- \ref Wednesday
 *														- \ref Thursday
 *  													- \ref Friday         
 *  													- \ref Saturday     
 * @return			 None
 * */
void RSI_RTC_SetDayOfWeek(RTC_Type *Cal, RTC_DAY_OF_WEEK_T dayInWeek)
{
  Cal->MCU_CAL_PROG_TIME_2_b.PROG_WEEK_DAY = dayInWeek;
  return;
}

/**
 * @fn					 void RSI_RTC_IntrUnMask(uint32_t intr)
 * @brief   		 This API is used to enable the RTC alarm interrupts
 * @param[in]		 intr  : \b Ored value of interrupt to be enabled/Un-mask
 *                       - \ref RTC_MSEC_INTR  : Use this macro to enable msec interrupt
 *                       - \ref RTC_SEC_INTR   : Use this macro to enable sec interrupt
 *                       - \ref RTC_ALARM_INTR : Use this macro to enable alarm interrupt 
 * @note         Usage of this APIs 
 *                       - RSI_RTC_IntrUnMask(RTC_MSEC_INTR | RTC_SEC_INTR |RTC_ALARM_INTR);
 * @return			 None
 */
void RSI_RTC_IntrUnMask(uint32_t intr)
{
  NPSS_INTR_MASK_CLR_REG = intr;
}

/**
 * @fn					void RSI_RTC_IntrMask(uint32_t intr)
 * @brief   		This API is used to disable the RTC alarm interrupts
 * @param[in]		intr  : \b Ored value of interrupt to be disabled/Mask
 *                       - \ref RTC_MSEC_INTR  : Use this macro to disable msec interrupt
 *                       - \ref RTC_SEC_INTR   : Use this macro to disable sec interrupt
 *                       - \ref RTC_ALARM_INTR : Use this macro to disable alarm interrupt 
 * @note        Usage of this APIs is as follows
 *                       - RSI_RTC_IntrMask(RTC_MSEC_INTR | RTC_SEC_INTR |RTC_ALARM_INTR);
 * @return			None
 */
void RSI_RTC_IntrMask(uint32_t intr)
{
  NPSS_INTR_MASK_SET_REG = intr;
}

/**
 * @fn					void RSI_RTC_IntrClear(uint32_t intr)
 * @brief   		This API is used to clear the RTC alarm interrupts
 * @param[in]		intr  : \b Ored value of interrupt to be cleared
 *                       - \ref RTC_MSEC_INTR  : Use this macro to clear msec interrupt
 *                       - \ref RTC_SEC_INTR   : Use this macro to clear sec interrupt
 *                       - \ref RTC_ALARM_INTR : Use this macro to clear alarm interrupt 
 * @note        Usage of this APIs is as follows
 *                       - RSI_RTC_IntrClear(RTC_MSEC_INTR | RTC_SEC_INTR |RTC_ALARM_INTR);
 * @return			None
 */
void RSI_RTC_IntrClear(uint32_t intr)
{
  /*Clear FSM registers */
  if (intr & RTC_MSEC_INTR) {
    NPSS_INTR_CLEAR_REG                  = NPSS_TO_MCU_MSEC_INTR;
    MCU_FSM->MCU_FSM_WAKEUP_STATUS_CLEAR = MILLI_SEC_BASED_STATUS_CLEAR;
  }
  if (intr & RTC_SEC_INTR) {
    NPSS_INTR_CLEAR_REG                  = NPSS_TO_MCU_SEC_INTR;
    MCU_FSM->MCU_FSM_WAKEUP_STATUS_CLEAR = RTC_SEC_BASED_STATUS_CLEAR;
  }
  if (intr & RTC_ALARM_INTR) {
    NPSS_INTR_CLEAR_REG                  = NPSS_TO_MCU_ALARM_INTR;
    MCU_FSM->MCU_FSM_WAKEUP_STATUS_CLEAR = RTC_ALARM_BASED_WAKEUP_STATUS_CLEAR;
  }
}

/*Initilization RTC CALIBRATION*/
void RSI_RTC_CalibInitilization(void)
{
  /* Uncomment below 2 lines of code if it is MCU only mode */
  //	RF_AFE_PWRCTRL_REG |= BIT(4);
  //	XO_PROG_REG =0xA95A ;  // Dummy Read
  //	XO_PROG_REG |= BIT(1); // Calin Clk EN in XO BG
  ULP_SPI_MEM_MAP(0x106) |= (BIT(21) | BIT(19));
}

/**
 * @fn					void RSI_RTC_RCCLK_Calib(uint8_t enable,uint8_t periodic_en,uint8_t trigger_time)
 * @brief   		This API is used to rc calibration
 * @param[in]		enable      : 1 to start rc calibration
 *              periodic_en : 1 to start periodically calibrate
 *              trigger_time : rc_trigger time
 *              0 -5 sec
                1 -10 sec 
                2 -15 sec
                3 -30 sec
                4 -1 minute
                5 -2minute          
 * @return			None
 */
void RSI_RTC_RCCLK_Calib(TimePeriodCalib_t *rtc, uint8_t enable, uint8_t periodic_en, uint8_t trigger_time)
{
  rtc->MCU_CAL_TEMP_PROG_REG_b.RTC_TIMER_PERIOD_MUX_SEL = 1;
  rtc->MCU_CAL_START_REG_b.START_CALIB_RC               = enable;
  rtc->MCU_CAL_START_REG_b.PERIODIC_RC_CALIB_EN         = periodic_en;
  rtc->MCU_CAL_START_REG_b.RC_TRIGGER_TIME_SEL          = trigger_time;
  return;
}

/**
 * @fn					void RSI_RTC_ROCLK_Calib(uint8_t enable,uint8_t periodic_en,uint8_t trigger_time,
	                          uint8_t ro_enable,uint8_t ro_periodic_en,uint8_t ro_trigger_time)
 * @brief   		This API is used to rc and ro calibration
 * @param[in]		enable      : 1 to start rc calibration
 *              periodic_en : 1 to start periodically calibrate
 *              trigger_time : rc_trigger time
 *              0 -5 sec
                1 -10 sec 
                2 -15 sec
                3 -30 sec
                4 -1 minute
                5 -2minute   

							 ro_enable  :1 to enavle ro calib
							 periodic_en:1 to enable periodic ro calib
							 trigger_time : ro_trigger time
                0 -1 time in sec
                1 -2 time in sec 
                2 -4 time in sec
                3 -8 time in sec
                
 * @return			None
 */
void RSI_RTC_ROCLK_Calib(TimePeriodCalib_t *rtc,
                         uint8_t enable,
                         uint8_t periodic_en,
                         uint8_t trigger_time,
                         uint8_t ro_enable,
                         uint8_t ro_periodic_en,
                         uint8_t ro_trigger_time)
{
  RSI_RTC_RCCLK_Calib(rtc, enable, periodic_en, trigger_time);
  rtc->MCU_CAL_TEMP_PROG_REG_b.RTC_TIMER_PERIOD_MUX_SEL = 1;
  rtc->MCU_CAL_START_REG_b.START_CALIB_RO               = ro_enable;
  rtc->MCU_CAL_START_REG_b.PERIODIC_RO_CALIB_EN         = ro_periodic_en;
  rtc->MCU_CAL_START_REG_b.RO_TRIGGER_TIME_SEL          = ro_trigger_time;
  return;
}
/**
 * @fn					uint32_t  RSI_RTC_GetIntrStatus(void)
 * @brief   		This API is used to get the RTC alarm interrupt status
 * @return[in]  intr status  : \b Ored value of interrupt status
 *                       - \ref RTC_MSEC_INTR  : Use this macro to check the msec interrupt status
 *                       - \ref RTC_SEC_INTR   : Use this macro to check the sec interrupt status
 *                       - \ref RTC_ALARM_INTR : Use this macro to check the alarm interrupt status
 * @note        Usage of this APIs is as follows
 *                       - intr_status = RSI_RTC_GetIntrStatus();
 *                         if(intr_status & RTC_MSEC_INTR){
 *                           // Msec interrupt status is set
 *                         }
 *                         if(intr_status & RTC_SEC_INTR){
 *                           // Sec interrupt status is set
 *                         }
 *                         if(intr_status & RTC_ALARM_INTR){
 *                           // Alarm interrupt status is set
 *                         }
 */
uint32_t RSI_RTC_GetIntrStatus(void)
{
  return NPSS_INTR_STATUS_REG;
}
/*End of file not truncated*/
