
/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2016 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *  
 * $Date:        1 AUG 2017
 * $Revision:    V1.0
 *  
 * Driver:       Driver_ETH_PHYn (default: Driver_ETH_PHY0)
 * Project:      Ethernet Physical Layer Transceiver (PHY)
 *               Driver for LAN8742A
 * ---------------------------------------------------------------------- 
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 * 
 *   Configuration Setting                     Value
 *   ---------------------                     -----
 *   Connect to hardware via Driver_ETH_PHY# = n (default: 0)
 * -------------------------------------------------------------------- 
 *
 */

#include "rsi_ccp_user_config.h"
#if defined(CHIP_9118)
#include "PHY_LAN8742A.h"
/* driver version */


#define ARM_ETH_PHY_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,1)
#ifndef ETH_PHY_NUM
#define ETH_PHY_NUM     0        /* Default driver number */
#endif

#ifndef ETH_PHY_ADDR
#define ETH_PHY_ADDR    0x00     /* Default device address */
#endif


/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
		ARM_ETH_PHY_API_VERSION,
		ARM_ETH_PHY_DRV_VERSION
};

/* Ethernet PHY control structure */
static PHY_CTRL PHY = { NULL, NULL, 0, 0 };


/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
 */
static ARM_DRIVER_VERSION GetVersion (void) {
	return DriverVersion;
}


/**
  \fn          int32_t Initialize (ARM_ETH_PHY_Read_t  fn_read,
                                   ARM_ETH_PHY_Write_t fn_write)
  \brief       Initialize Ethernet PHY Device.
  \param[in]   fn_read  : Pointer to \ref ARM_ETH_MAC_PHY_Read
  \param[in]   fn_write : Pointer to \ref ARM_ETH_MAC_PHY_Write
  \return      \ref execution_status
 */
static int32_t Initialize (ARM_ETH_PHY_Read_t fn_read, ARM_ETH_PHY_Write_t fn_write) {

	if ((fn_read == NULL) || (fn_write == NULL)) { return ARM_DRIVER_ERROR_PARAMETER; }

	if ((PHY.flags & PHY_INIT) == 0U) {
		/* Register PHY read/write functions. */
		PHY.reg_rd = fn_read;
		PHY.reg_wr = fn_write;

		PHY.bcr    = 0U;
		PHY.flags  = PHY_INIT;
	}

	return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet PHY Device.
  \return      \ref execution_status
 */
static int32_t Uninitialize (void) {

	PHY.reg_rd = NULL;
	PHY.reg_wr = NULL;
	PHY.bcr    = 0U;
	PHY.flags  = 0U;

	return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet PHY Device Power.
  \param[in]   state : Power state
  \return      \ref execution_status
 */
static int32_t PowerControl (ARM_POWER_STATE state) {
	uint16_t val;

	switch (state) {
	case ARM_POWER_OFF:
		if ((PHY.flags & PHY_INIT) == 0U) {
			/* Initialize must provide register access function pointers */
			return ARM_DRIVER_ERROR;
		}

		PHY.flags &= ~PHY_POWER;
		PHY.bcr    =  BCR_POWER_DOWN;

		return (PHY.reg_wr(ETH_PHY_ADDR, REG_BCR, PHY.bcr));

	case ARM_POWER_FULL:
		if ((PHY.flags & PHY_INIT) == 0U) {
			return ARM_DRIVER_ERROR;
		}
		if (PHY.flags & PHY_POWER) {
			return ARM_DRIVER_OK;
		}

		/* Check Device Identification. */
		PHY.reg_rd(ETH_PHY_ADDR, REG_PHYIDR1, &val);

		if (val != PHY_ID1) {
			/* Invalid PHY ID */
			return ARM_DRIVER_ERROR_UNSUPPORTED;
		}

		PHY.reg_rd(ETH_PHY_ADDR, REG_PHYIDR2, &val);

		if ((val & 0xFFF0) != PHY_ID2) {
			/* Invalid PHY ID */
			return ARM_DRIVER_ERROR_UNSUPPORTED;
		}

		PHY.bcr = 0U;

		if (PHY.reg_wr(ETH_PHY_ADDR, REG_BCR, PHY.bcr) != ARM_DRIVER_OK) {
			return ARM_DRIVER_ERROR;
		}

		PHY.flags |=  PHY_POWER;

		return ARM_DRIVER_OK;

	case ARM_POWER_LOW:
	default:
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
}

/**
  \fn          int32_t SetInterface (uint32_t interface)
  \brief       Set Ethernet Media Interface.
  \param[in]   interface : Media Interface type
  \return      \ref execution_status
 */
static int32_t SetInterface (uint32_t interface) {

	if ((PHY.flags & PHY_POWER) == 0U) { return ARM_DRIVER_ERROR; }

	switch (interface) {
	case ARM_ETH_INTERFACE_RMII:
		break;
	case ARM_ETH_INTERFACE_MII:
	default:
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}
	return(0);
}

/**
  \fn          int32_t SetMode (uint32_t mode)
  \brief       Set Ethernet PHY Device Operation mode.
  \param[in]   mode : Operation Mode
  \return      \ref execution_status
 */
static int32_t SetMode (uint32_t mode) {
	uint16_t val;

	if ((PHY.flags & PHY_POWER) == 0U) { return ARM_DRIVER_ERROR; }

	val = PHY.bcr & BCR_POWER_DOWN;

	switch (mode & ARM_ETH_PHY_SPEED_Msk) {
	case ARM_ETH_PHY_SPEED_10M:
		break;
	case ARM_ETH_PHY_SPEED_100M:
		val |=  BCR_SPEED_SEL;
		break;
	default:
		return ARM_DRIVER_ERROR_UNSUPPORTED;
	}

	switch (mode & ARM_ETH_PHY_DUPLEX_Msk) {
	case ARM_ETH_PHY_DUPLEX_HALF:
		break;
	case ARM_ETH_PHY_DUPLEX_FULL:
		val |=  BCR_DUPLEX;
		break;
	}

	if (mode & ARM_ETH_PHY_AUTO_NEGOTIATE) {
		val |= BCR_ANEG_EN;
	}

	if (mode & ARM_ETH_PHY_LOOPBACK) {
		val |= BCR_LOOPBACK;
	}

	if (mode & ARM_ETH_PHY_ISOLATE) {
		val |= BCR_ISOLATE;
	}

	PHY.bcr = val;

	return (PHY.reg_wr(ETH_PHY_ADDR, REG_BCR, PHY.bcr));
}

/**
  \fn          ARM_ETH_LINK_STATE GetLinkState (void)
  \brief       Get Ethernet PHY Device Link state.
  \return      current link status \ref ARM_ETH_LINK_STATE
 */
static ARM_ETH_LINK_STATE GetLinkState (void) {
	ARM_ETH_LINK_STATE state;
	uint16_t           val = 0U;

	if (PHY.flags & PHY_POWER) {
		PHY.reg_rd(ETH_PHY_ADDR, REG_BSR, &val);
	}
	state = (val & BSR_LINK_STAT) ? ARM_ETH_LINK_UP : ARM_ETH_LINK_DOWN;

	return (state);
}

/**
  \fn          ARM_ETH_LINK_INFO GetLinkInfo (void)
  \brief       Get Ethernet PHY Device Link information.
  \return      current link parameters \ref ARM_ETH_LINK_INFO
 */
static ARM_ETH_LINK_INFO GetLinkInfo (void) {
	ARM_ETH_LINK_INFO info;
	uint16_t          val = 0U;

	if (PHY.flags & PHY_POWER) {
		PHY.reg_rd(ETH_PHY_ADDR, REG_PSCS, &val);
	}

	info.speed  = (val & PSCS_SPEED)  ? ARM_ETH_SPEED_10M   : ARM_ETH_SPEED_100M;
	info.duplex = (val & PSCS_DUPLEX) ? ARM_ETH_DUPLEX_FULL : ARM_ETH_DUPLEX_HALF;

	return (info);
}


/* PHY Driver Control Block */
extern
ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY0;
ARM_DRIVER_ETH_PHY ARM_Driver_ETH_PHY0= {
		GetVersion,
		Initialize,
		Uninitialize,
		PowerControl,
		SetInterface,
		SetMode,
		GetLinkState,
		GetLinkInfo
};
#endif
