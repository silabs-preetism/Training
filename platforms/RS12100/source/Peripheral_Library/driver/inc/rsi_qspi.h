/*******************************************************************************
* @file  rsi_qspi.h
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

#ifndef __RSI_QSPI_H__
#define __RSI_QSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "rsi_ccp_common.h"
#include "base_types.h"
#include "stdint.h"
/**
 * \cond HIDDEN_SYMBOLS 
 */

//! QSPI defines
#define TA_QSPI_BASE_ADDRESS        0x10000000
#define M4_QSPI_BASE_ADDRESS        0x12000000
#define TA_QSPI_AUTOM_CHIP0_ADDRESS 0x04000000
#define M4_QSPI_AUTOM_CHIP0_ADDRESS 0x08000000

#define NWP_FSM_BASE_ADDR     0x41300000
#define MCU_NPSS_BASE_ADDR    0x24048000
#define MCU_FSM_BASE_ADDR     MCU_NPSS_BASE_ADDR + 0x100
#define M4_BBFF_STORAGE1      *(volatile uint32_t *)(MCU_NPSS_BASE_ADDR + 0x0580)
#define M4_BBFF_STORAGE2      *(volatile uint32_t *)(MCU_NPSS_BASE_ADDR + 0x0584)
#define MCURET_BOOTSTATUS_REG *(volatile uint32_t *)(MCU_NPSS_BASE_ADDR + 0x604)
#define TA_BBFF_STORAGE1      *(volatile uint32_t *)(NWP_FSM_BASE_ADDR + 0x580)
#define TA_BBFF_STORAGE2      *(volatile uint32_t *)(NWP_FSM_BASE_ADDR + 0x584)

//! This structure contains qspi registers
struct qspi_reg_s {
  volatile uint32_t QSPI_CLK_CONFIG_REG;         //!< qspi reg
  volatile uint32_t QSPI_BUS_MODE_REG;           //!< qspi reg
  volatile uint32_t QSPI_AUTO_CTRL_CONFIG_1_REG; //!< qspi reg
  volatile uint32_t QSPI_AUTO_CTRL_CONFIG_2_REG; //!< qspi reg
  volatile uint32_t QSPI_MANUAL_CONFIG_REG;      //!< qspi reg
  volatile uint32_t QSPI_MANUAL_CONFIG_2_REG;    //!< qspi reg

  volatile uint32_t RESERVED_1;                   //!< qspi reg
  volatile uint32_t QSPI_FIFO_THRESHOLD_REG;      //!< qspi reg
  volatile uint32_t QSPI_STATUS_REG;              //!< qspi reg
  volatile uint32_t QSPI_INTR_MASK_REG;           //!< qspi reg
  volatile uint32_t QSPI_INTR_UNMASK_REG;         //!< qspi reg
  volatile uint32_t QSPI_INTR_STS_REG;            //!< qspi reg
  volatile uint32_t QSPI_INTR_ACK_REG;            //!< qspi reg
  volatile uint32_t QSPI_STS_MC_REG;              //!< qspi reg
  volatile uint32_t QSPI_AUTO_CONFIG_1_CSN1_REG;  //!< qspi reg
  volatile uint32_t QSPI_AUTO_CONFIG_2_CSN1_REG;  //!< qspi reg
  volatile uint32_t QSPI_MANUAL_RD_WR_DATA_REG;   //!< qspi reg
  volatile uint32_t RESERVED_2[15];               //!< qspi reg
  volatile uint32_t QSPI_MANUAL_WRITE_DATA_2_REG; //!< qspi reg
  volatile uint32_t RESERVED_3[3];                //!< qspi reg
  volatile uint32_t QSPI_AUTO_CONFIG3;
  volatile uint32_t QSPI_AUTO_CONFIG3_CSN1;
  volatile uint32_t RESERVED_4[6];           //!< qspi reg
  volatile uint32_t OCTA_SPI_BUS_CONTROLLER; //!< qspi reg :2c
  volatile uint32_t QSPI_AUTO_BASE_ADDR_UNMASK_CSN0;
  volatile uint32_t RESERVED_7[3];            //!< qspi reg
  volatile uint32_t OCTA_SPI_BUS_CONTROLLER2; //!< qspi reg  :31
#ifdef CHIP_9117
  volatile uint32_t QSPI_AES_CONFIG;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY_IV_VALID; //!< qspi reg
  volatile uint32_t QSPI_AES_IV1_0_3;      //!< qspi reg
  volatile uint32_t QSPI_AES_IV1_4_7;      //!< qspi reg
  volatile uint32_t QSPI_AES_IV1_8_B;      //!< qspi reg
  volatile uint32_t QSPI_AES_IV1_C_F;      //!< qspi reg
  volatile uint32_t RESERVED_8;            //!< qspi reg
#else
  volatile uint32_t QSPI_AES_KEY_0_3;   //!< qspi reg
  volatile uint32_t QSPI_AES_KEY_4_7;   //!< qspi reg
  volatile uint32_t QSPI_AES_KEY_8_B;   //!< qspi reg
  volatile uint32_t QSPI_AES_KEY_C_F;   //!< qspi reg
  volatile uint32_t QSPI_AES_NONCE_0_3; //!< qspi reg
  volatile uint32_t QSPI_AES_NONCE_4_7; //!< qspi reg
  volatile uint32_t QSPI_AES_NONCE_8_B; //!< qspi reg
#endif
  volatile uint32_t QSPI_AES_SEC_SEG_ADDR[4 * 2]; //!< qspi reg
  volatile uint32_t RESERVED_6[6];                //!< qspi reg
  volatile uint32_t QSPI_SEMI_AUTO_ADDR_REG;
  volatile uint32_t QSPI_SEMI_AUTO_MODE_CONFIG_REG;
  volatile uint32_t QSPI_SEMI_AUTO_MODE_CONFIG2_REG;
  volatile uint32_t QSPI_BUS_MODE2_REG;
  volatile uint32_t QSPI_AES_SEC_KEY_FRM_KH;
  volatile uint32_t QSPI_AUTO_CONITNUE_FETCH_CTRL_REG; //!< qspi reg
#ifdef CHIP_9117
  volatile uint32_t QSPI_AES_KEY1_0_3;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_4_7;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_8_B;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_C_F;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_10_13;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_14_17;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_18_1B;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY1_1C_1F;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_0_3;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_4_7;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_8_B;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_C_F;       //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_10_13;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_14_17;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_18_1B;     //!< qspi reg
  volatile uint32_t QSPI_AES_KEY2_1C_1F;     //!< qspi reg
  volatile uint32_t QSPI_AES_IV2_0_3;        //!< qspi reg
  volatile uint32_t QSPI_AES_IV2_4_7;        //!< qspi reg
  volatile uint32_t QSPI_AES_IV2_8_B;        //!< qspi reg
  volatile uint32_t QSPI_AES_IV2_C_F;        //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV1_0_3; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV1_4_7; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV1_8_B; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV1_C_F; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV2_0_3; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV2_4_7; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV2_8_B; //!< qspi reg
  volatile uint32_t QSPI_AES_CTXOUT_IV2_C_F; //!< qspi reg
#endif
};

#define XMAX(x, y) (((x) > (y)) ? (x) : (y))
#define XMIN(x, y) (((x) < (y)) ? (x) : (y))

#define QSPI_BASE_ADDRESS             0x12000000
#define QSPI_AUTOM_CHIP0_ADDRESS      0x08000000
#define QSPI_AUTOM_CHIP1_ADDRESS      0x09000000
#define PAD_CONFIGURATION_BASE_ADDR   0x41380000
#define TASS_CLK_PWR_CTRL_BASE_ADDR   0x41400000
#define GPIO_BASE_ADDR                0x40200000
#define DMA_BASE_ADDR                 0x21000000
#define M4SS_CLK_PWR_CTRL_BASE_ADDR   0x46000000
#define M4GPIO_BASE_ADDR              0x46130000
#define M4PAD_CONFIGURATION_BASE_ADDR 0x46004000
#define M4PAD_SELECTION_BASE_ADDR     0x41300000

#define MISC_CONFIGURATION_REG    0x46008000
#define M4SS_QSPI_OCTA_MODE_CTRL  *(volatile uint32_t *)(MISC_CONFIGURATION_REG + 0x1B4)
#define M4SS_QSPI_RX_DLL_TEST_REG *(volatile uint32_t *)(MISC_CONFIGURATION_REG + 0x1C0)
#define M4SS_QSPI_TX_DLL_TEST_REG *(volatile uint32_t *)(MISC_CONFIGURATION_REG + 0x1BC)

#define qspi_ddr_data_0 0
#define qspi_ddr_data_1 1
#define qspi_ddr_data_2 2
#define qspi_ddr_data_3 3
#define qspi_ddr_data_4 4
#define qspi_ddr_data_5 5
#define qspi_ddr_data_6 6
#define qspi_ddr_data_7 7
#define qspi_ddr_csn    8
#define qspi_ddr_clk    9
#define qspi_ddr_dqs    10
#define smih_wp         11

#define CHNL_21 21
#define CHNL_20 20
/*****************************************
 *        Qspi register defines
 *****************************************/
//! cmd len will be 8 bits
#define CMD_LEN 8
//! reg bit
#define RD_FIFO_EMPTY BIT(7)
//! reg bit
#define Q_QSPI_BUSY BIT(0)
//! QSPI_MANUAL_CONFIG_REG bits
#define FULL_DUPLEX_EN   BIT(22)
#define TAKE_LEN_FRM_REG BIT(21)
#define HW_CTRL_MODE     BIT(25)
#define READ_TRIGGER     BIT(2)
#define WRITE_TRIGGER    BIT(1)
#define CSN_ACTIVE       BIT(0)

//! QSPI_MANUAL_CONFIG_2_REG bits
#define QSPI_LOOP_BACK_MODE_EN BIT(14)
#define QSPI_MANUAL_DDR_PHASSE BIT(15)
#define QSPI_DDR_CLK_EN        BIT(16)

//! QSPI_CLK_CONFIG_REG bits
#define QSPI_DLL_CALIB BIT(28)
#define QSPI_DLL_TX_EN BIT(21)
#define QSPI_DLL_RX_EN BIT(19)

//! QSPI_AUTO_CTRL_CONFIG_1_REG bits
#define EXTRA_BYTE_EN BIT(18)

//! QSPI_AUTO_CTRL_CONFIG_2_REG bits
#define AUTO_RD_SWAP BIT(0)

//! QSPI_SEMI_AUTO_MODE_CONFIG2_REG
#define SEMI_AUTO_MODE_EN      BIT(12)
#define QSPI_SEMI_AUTO_RD_BUSY BIT(13)

#define _1BYTE 0
#define _2BYTE 1
#define _4BYTE 3

//! QSPI_STATUS_REG bits
#define HW_CTRLD_QSPI_MODE_CTRL_SCLK BIT(14)
#define AUTO_MODE_ENABLED            BIT(12)
#define QSPI_AUTO_MODE               BIT(11)
#define AUTO_MODE_FSM_IDLE_SCLK      BIT(10)
#define QSPI_MANUAL_RD_CNT           BIT(9)
#define QSPI_FIFO_AEMPTY_RFIFO_S     BIT(8)
#define QSPI_FIFO_EMPTY_RFIFO_S      BIT(7)
#define QSPI_FIFO_AFULL_RFIFO        BIT(6)
#define QSPI_FIFO_FULL_RFIFO         BIT(5)
#define QSPI_FIFO_AEMPTY_WFIFO       BIT(4)
#define QSPI_FIFO_EMPTY_WFIFO        BIT(3)
#define QSPI_FIFO_AFULL_WFIFO_S      BIT(2)
#define QSPI_FIFO_FULL_WFIFO_S       BIT(1)
//#define BUSY                         BIT(0)

#define QSPI_FIFO_DEPTH     16
#define QSPI_FIFO_AFULL_TH  3
#define QSPI_FIFO_AEMPTY_TH 3

//! QSPI_BUS_MODE_REG bits
#define AUTO_CSN_BASED_ADDR_ENABLE    BIT(7)
#define AUTO_MODE                     BIT(6)
#define QSPI_WRAP_EN                  BIT(5)
#define QSPI_PREFETCH_EN              BIT(4)
#define QSPI_ULTRA_HIGH_SPEED_MODE_EN BIT(0)

//! QSPI_AUTO_CONFIG3 bits
#define QSPI_RD_INST_CSN0_MSB         24
#define QSPI_CMD_SIZE_16BIT_CSN0      BIT(18)
#define QSPI_ADR_SIZE_32BIT_AUTO_MODE BIT(19)

//! QSPI_MANUAL_WRITE_DATA_2_REG bits
#define USE_PREV_LEN BIT(7)

//! OCTA_SPI_BUS_CONTRLLER_2 bits
#define DUAL_FLASH_MODE BIT(3)

/******************************************
 *              FLASH CMDS
 ******************************************/
//! Write enable cmd
#define WREN  0x06
#define WREN2 0xF9
//! Write disable cmd
#define WRDI  0x04
#define WRDI2 0xFB
//! Read status reg cmd
#define RDSR  0x05
#define RDSR2 0xFA
//! chip erase cmd
#define CHIP_ERASE 0xC7
//! block erase cmd
#define BLOCK_ERASE 0xD8
//! sector erase cmd
#define SECTOR_ERASE 0x20
//! high speed rd cmd
#define HISPEED_READ 0x0B
//! rd cmd
#define READ 0x03
//!write config2
#define WCFG2 0x72

/****************************************
 *        SST25 specific cmds
 ****************************************/
//! Write status reg cmd
#define WRSR  0x01
#define WRSR2 0xFE
//! Enable Write status reg cmd
#define EWSR 0x50
//! Auto address incremental rd cmd
#define AAI 0xAF
//! Byte program cmd
#define BYTE_PROGRAM 0x02

/****************************************
 *       SST26 specific cmds
 ***************************************/
//! Enable quad IO
#define EQIO 0x38
//! Reset quad IO
#define RSTQIO 0xFF
//! Enable STR octa
#define OPI_ENABLE 0X01
//! Enable DDR octa
#define DOPI_ENABLE 0x02
//! wrap : set burst
#define SET_BURST 0xC0
//! wrap : read cmd
#define READ_BURST 0x0C
//! Jump : page index read
#define READ_PI 0x08
//! Jump : Index read
#define READ_I 0x09
//! Jump : Block Index read
#define READ_BI 0x10
//! Page program cmd
#define PAGE_PROGRAM 0x02
//! write suspend cmd
#define Write_Suspend 0xB0
//! write resume cmd
#define Write_Resume 0x30
//! read block protection reg
#define RBPR 0x72
//! Write block protection reg
#define WBPR 0x42
//! Lockdown block protection reg
#define LBPR 0x8D

/****************************************
 * WINBOND + AT + MACRONIX specific cmds
 ***************************************/
//! fast read dual output
#define FREAD_DUAL_O 0x3B
//! fast read quad output
#define FREAD_QUAD_O 0x6B

#define FREAD_QUAD_O_EB 0xEB

/****************************************
 * WINBOND + MACRONIX specific cmds
 ***************************************/
//! fast read dual IO
#define FREAD_DUAL_IO 0xBB
//! fast read quad IO
#define FREAD_QUAD_IO 0xEB

/****************************************
 *       WINBOND specific cmds
 ***************************************/
//! Octal word read (A7-A0 must be 00)
#define OCTAL_WREAD 0xE3
//! Enable high performance cmd
#define HI_PERFMNC 0xA3

/****************************************
 *        ATMEL specific cmds
 ***************************************/
//! write config reg
#define WCON 0x3E
//! read config reg
#define RCON 0x3F
//! supported upto 100MHz
#define HI_FREQ_SPI_READ 0x1B

#define ATMEL_QEN BIT(7)

/****************************************
 *    MACRONIX specific write cmds
 ***************************************/
//! Address and data in quad
#define QUAD_PAGE_PROGRAM 0x38
#define DDR_DATA          16
#define DDR_EXTRA_BYTE    15
#define DDR_DUMMY         14
#define DDR_ADDR          13
#define DDR_CMD           17

#define OCTA_DDR_READ      0xEE
#define OCTA_DDR_READ_CMD2 0x11

#define OCTA_SDR_READ      0xEC
#define OCTA_SDR_READ_CMD2 0x13

/****************************************
 * ATMEL + WINBOND specific write cmds
 ***************************************/
//! Only data in quad mode
#define QUAD_IN_PAGE_PROGRAM 0x32

/****************************************
 * ATMEL specific write cmds
 ***************************************/
//! Data in dual
#define DUAL_IN_PAGE_PROGRAM 0xA2

/****************************************
 * MICRON specific cmds
 ***************************************/
#define RD_LOCK_REG         0xE8
#define WR_LOCK_REG         0xE5
#define RD_FLAG_STS_REG     0x70
#define CLR_FLAG_STS_REG    0x50
#define RD_NONVOL_CON_REG   0xB5
#define WR_NONVOL_CON_REG   0xB1
#define RD_VOL_CON_REG      0x85
#define WR_VOL_CON_REG      0x81
#define RD_ENHN_VOL_CON_REG 0x65
#define WR_ENHN_VOL_CON_REG 0x61
#define DIS_XIP             BIT(3)
#define XIP_MODE            0

/****************************************
 * ADESTO specific cmds
 ***************************************/
#define STS_BYT2 0x31
#define STS_CTRL 0x71

#define ADEST_PROTECT_CMD   0x36
#define ADEST_UNPROTECT_CMD 0x39

/***************************************
 *        Defines for arguments
 **************************************/

//! disable hw ctrl
#define DIS_HW_CTRL 1
//! donot disable hw ctrl
#define DNT_DIS_HW_CTRL 0

//! 32bit hsize
#define _32BIT 3
//! 24bit hsize is not supported, so reserved
//      reserved                     2
//! 16bit hsize
#define _16BIT 1
//! 8bit hsize
#define _8BIT 0

//! cmd len will be 8 bits
#define QSPI_8BIT_LEN 8
//! cmd + 1 byte len
#define QSPI_16BIT_LEN 16
//! cmd + 24bit addr len
#define QSPI_32BIT_LEN 0

//! OCTA_SPI_BUS_CONTROLLER2 bits
//! initialises NONCE
#define NONCE_INIT BIT(1)
//! enables global security
#define EN_SECURITY BIT(2)
//! enables of sec per segment
#define EN_SEG_SEC 12
//! enable qspi to use key from kh
#define EN_KH_KEY BIT(18)

//! QSPI AES SEC KEY FROM KH bits
//! secure key loading interval
#define LOAD_SEC_KEY_FRM_KH BIT(0)

/***************************************
 *        LIST OF MACRO USED
 **************************************/
//!  Macro to Deassert CS
#define DEASSERT_CSN qspi_reg->QSPI_MANUAL_CONFIG_REG = ((qspi_reg->QSPI_MANUAL_CONFIG_REG & ~0x1FFF) | 0x1)

//! Macro to check Quad mode
#define CHK_QUAD_MODE (spi_config->spi_config_1.data_mode == QUAD_MODE)
//! Macro to check DUAL mode
#define CHK_DUAL_MODE (spi_config->spi_config_1.data_mode == DUAL_MODE)
#define CHK_OCTA_MODE (spi_config->spi_config_1.data_mode == OCTA_MODE)

//#define IS_QSPI_IN_OCTA              (((qspi_reg->QSPI_BUS_MODE_REG & 0x6) >> 1) == OCTA_MODE)

//! Macro to provide protection byte for SST
#define SST_PROTECTION ((spi_config->spi_config_2.protection == EN_WR_PROT) ? 0xFF : 0)
//! Macro to provide protection byte for WB
#define WB_PROT ((spi_config->spi_config_2.protection == EN_WR_PROT) ? 0x1C00 : 0)
//! Macro to provide protection byte for MX
#define MX_PROT ((spi_config->spi_config_2.protection == EN_WR_PROT) ? 0x3C : 0)
//! Macro to provide protection byte for AT
#define AT_PROT ((spi_config->spi_config_2.protection == EN_WR_PROT) ? 0xC : 0)
//! Macro to get the position for D2 line data
#define GET_POS                                   \
  ((spi_config->spi_config_2.cs_no == CHIP_THREE) \
     ? 30                                         \
     : (spi_config->spi_config_2.cs_no == CHIP_TWO) ? 26 : (spi_config->spi_config_2.cs_no == CHIP_ONE) ? 14 : 10)

#define GET_POS_D7_D4                             \
  ((spi_config->spi_config_2.cs_no == CHIP_THREE) \
     ? 24                                         \
     : (spi_config->spi_config_2.cs_no == CHIP_TWO) ? 16 : (spi_config->spi_config_2.cs_no == CHIP_ONE) ? 8 : 0)
//! Macro to mask the d3 and d2 data bits
#define MASK_D3_D2(_POS) ~(0x3 << _POS)
#define MASK_D7_D4(_POS) ~(0xf << _POS)
//! for 0xEB 0xE3 and 0xBB, 7th bit will be high so check for it
#define HI_PERFORMANCE_REQ (spi_config->spi_config_1.read_cmd & BIT(7))

//! Macro to trigger QSPI to read from flash
#define READ_4M_FLASH(_NUM_BYTES, _CS_NO, Hsize)                                                                      \
  qspi_reg->QSPI_MANUAL_CONFIG_REG = (qspi_reg->QSPI_MANUAL_CONFIG_REG & ~0xF8387FFF) | READ_TRIGGER | (_CS_NO << 13) \
                                     | ((_NUM_BYTES & 0x3FF) << 3) | (((_NUM_BYTES >> 10) & 0x1F) << 27)              \
                                     | ((spi_config->spi_config_3.ddr_mode_en == 1) ? (BIT(21) | (Hsize << 19)) : 0)

//! Macro to return A8 bit in case 9bit addressing is required
#define A8_BIT ((spi_config->spi_config_2.addr_width == _9BIT_ADDR) ? ((addr & BIT(8)) << 3) : 0)

//! Macro to RETURN QSPI_BUS_SIZE from manual regs
#define QSPI_MANUAL_BUS_SIZE(_CHIP_SELECT)                                 \
  (_CHIP_SELECT == CHIP_ZERO) ? ((qspi_reg->QSPI_BUS_MODE_REG & 0x6) >> 1) \
                              : ((qspi_reg->QSPI_MANUAL_CONFIG_2_REG >> (8 + ((_CHIP_SELECT - 1) * 2))) & 0x3)

#define QSPI_CMD_DDR_MODE                                                             \
  (spi_config->spi_config_2.cs_no ? (qspi_reg->QSPI_AUTO_CONFIG3_CSN1 & BIT(DDR_CMD)) \
                                  : (qspi_reg->QSPI_AUTO_CONFIG3 & BIT(DDR_CMD)))
#define QSPI_DATA_DDR_MODE                                                            \
  (spi_config->spi_config_2.cs_no ? (qspi_reg->QSPI_AUTO_CONFIG3_CSN1 & BIT(DDR_CMD)) \
                                  : (qspi_reg->QSPI_AUTO_CONFIG3 & BIT(DDR_DATA)))
#define QSPI_DUAL_FLASH_MODE (qspi_reg->OCTA_SPI_BUS_CONTROLLER2 & DUAL_FLASH_MODE)
//! Macro to compute address width for manual mode based upon the addressing specified
#define ADDR_LEN        (spi_config->spi_config_2.addr_width ? (spi_config->spi_config_2.addr_width * 8) : 8)
#define QSPI_32BIT_ADDR 32
#define QSPI_8BIT_ADDR  8
//! Macro to compute address width for auto mode based upon addressing specified
#define AUTO_ADDR_WIDTH (((spi_config->spi_config_2.addr_width + 1) & 0x1) << 1)

//! Macro to check whether the addr violates page boundary
#define ODD_PAGE_BOUNDARY (addr & (page_size - 1))

//! EXTRA_BYTE is currently hardcoded to 0xA0
#define EXTRA_BYTE ((spi_config->spi_config_1.continuous) ? 0xA0 : 0x00)

//! number of dummy bytes required by the flash during a wrap/burst read command
#define NUM_DUMMY_BYTES_WRAP 1
#define PROT_FROM_TOP        BIT(3)

#define HIGH_PERF_MODE BIT(1)
#define HSIZE_IN_BITS  ((hsize + 1) * 8)

//! QSPI AES Decryption Defines
#ifdef CHIP_9117
#define CTR_MODE        0x04
#define XTS_MODE        0x80
#define IV_VALID        (0xf << 16)
#define KEY1_VALID      (0xf << 0)
#define KEY2_VALID      (0xf << 8)
#define DECRYPT_KEY_CAL BIT(9)
#endif
/**
 * \endcond
 */

#ifdef __cplusplus
}
#endif

#ifdef GIGA_FLASH_SUPPORT
#define SR2_READ 0x35
#define QUAD_EN  BIT(1)
#endif
#endif // __RSI_QSPI_H__
