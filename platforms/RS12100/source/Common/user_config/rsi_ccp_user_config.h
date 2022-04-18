/*******************************************************************************
* @file  rsi_ccp_user_config.h
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

/*rsi_ccp_user_config.h --> user configurations w.r.t CCP*/

/* To enable 9117 chip  --> Add #define CHIP_9117 */
/* To enable 9116 chip -->  Add #define CHIP_9118 */
#define CHIP_9118 
//#define CHIP_9117

/*1. By default A10 ROM is enabled */
/*2. To enable A11 ROM      --> Add #define A11_ROM (this disables A10 ROM) */
/*3. To enable OS           --> Add RSI_WITH_OS in preprocessor */
/*4. To enable ROM Wireless --> Add #define ROM_WIRELESS */
//#define A11_ROM
//#define ROMDRIVER_PRESENT
//#define ROM_WIRELESS

/* To enable uart prints  --> Add #define DEBUG_UART as below */
/* To disable uart prints --> Remove /Comment #define DEBUG_UART */
#define DEBUG_UART

/*To compile code for simulations --> Add #define SIMULATION*/
//#define SIMULATION

/*To compile code for simulations with RAM based settings --> Add #define RAM_COMPILATION */
//#define RAM_COMPILATION
