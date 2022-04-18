/*******************************************************************************
* @file  rsi_gpdma.c
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
#include "rsi_ccp_user_config.h"

#ifndef ROMDRIVER_PRESENT
#include "rsi_rom_gpdma.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn          RSI_DRIVER_VERSION RSI_GPDMA_GetVersion(void)
 * @brief		    This API is used to get version information of the driver implementation
 * @param[in]	  none
 * @return 		  structure of type RSI_DRIVER_VERSION and its members are as below
								- \ref RSI_GPDMA_API_VERSION   : Version of the CMSIS-Driver specification used to implement this driver.
								- \ref RSI_GPDMA_DRV_VERSION   : GPDMA peripheral source code version of the actual driver implementation.   
 */
RSI_DRIVER_VERSION RSI_GPDMA_GetVersion(void)
{
  RSI_DRIVER_VERSION vsDriverVersion;

  vsDriverVersion.api = RSI_GPDMA_API_VERSION;
  vsDriverVersion.drv = RSI_GPDMA_DRV_VERSION;
  return vsDriverVersion;
}

/**
 * @fn           RSI_GPDMA_CAPABILITIES_T RSI_GPDMA_GetCapabilities(void)
 * @brief        This API get the capabilities of the GPDMA
 * @param[in]    none       
 * @return       GPDMA capabilities structure
 */
RSI_GPDMA_CAPABILITIES_T RSI_GPDMA_GetCapabilities(void)
{
  RSI_GPDMA_CAPABILITIES_T vsDriverCapabilities;

  vsDriverCapabilities.noOfChannels         = 8;
  vsDriverCapabilities.noOfMasterInterfaces = 2;
  vsDriverCapabilities.noOfPeriSupport      = 64;
  vsDriverCapabilities.noOfPriorityLevels   = 4;
  return vsDriverCapabilities;
}

/* This API is used to enable the required channel of GPDMA */
error_t gpdma_dma_channel_trigger(RSI_GPDMA_HANDLE_T pHandle, uint8_t dmaCh)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (dmaCh <= GPDMA_CHNL7) {
    /* Enables the required channel */
    pDrv->baseG->GLOBAL.DMA_CHNL_ENABLE_REG = SET_BIT(dmaCh);
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to clear the interrupts of required GPDMA channel */
error_t gpdma_interrupt_clear(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (pCfg->dmaCh <= GPDMA_CHNL7) {
    if (pCfg->descFetchDoneIntr == 1) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = SET_BIT(((4 * pCfg->dmaCh) + 1));
    }
    if (pCfg->xferDoneIntr == 1) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = SET_BIT(((4 * pCfg->dmaCh) + 2));
    }
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to get the interrupt status of required GPDMA channel */
uint32_t gpdma_interrupt_status(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (pCfg->dmaCh <= GPDMA_CHNL7) {
    if (pCfg->descFetchDoneIntr == 1) {
      return (pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG & SET_BIT(((4 * pCfg->dmaCh) + 1)));
    }
    if (pCfg->xferDoneIntr == 1) {
      return (pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG & SET_BIT(((4 * pCfg->dmaCh) + 2)));
    }
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to get the error status of required GPDMA channel */
uint32_t gpdma_get_error_status(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (pCfg->dmaCh <= GPDMA_CHNL7) {
    if (pCfg->hrespErr == 1) {
      return (pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG & SET_BIT(pCfg->dmaCh * 4));
    }
    if (pCfg->gpdmacErr == 1) {
      return (pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG & SET_BIT(((pCfg->dmaCh * 4) + 3)));
    }
  }
  return RSI_OK;
}

/* This API is used to clear the errors for the required GPDMA channel */
error_t gpdma_error_status_clear(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (pCfg->dmaCh <= GPDMA_CHNL7) {
    if (pCfg->hrespErr == 1) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = SET_BIT(pCfg->dmaCh * 4);
    }
    if (pCfg->gpdmacErr == 1) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = SET_BIT(((pCfg->dmaCh * 4) + 3));
    }
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API get current activity of channel */
uint8_t RSI_GPDMA_GetChannelActivity(RSI_GPDMA_HANDLE_T pHandle, uint8_t dmaCh)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;
  return (pDrv->baseG->GLOBAL.DMA_CHNL_ENABLE_REG & SET_BIT(dmaCh));
}

/* This API is used to enable interrupt flags for required GPDMA channel */
error_t gpdma_interrupt_enable(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  uint32_t maskReg          = 0;
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (pCfg->dmaCh <= GPDMA_CHNL7) {
    M4SS_GPDMA_INTR_SEL |= SET_BIT(pCfg->dmaCh);
    maskReg = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if (pCfg->descFetchDoneIntr == 1) {
      maskReg &= (CLR_BIT(pCfg->dmaCh + 8));
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = maskReg;
    }
    if (pCfg->xferDoneIntr == 1) {
      maskReg &= (CLR_BIT(pCfg->dmaCh + 16));
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = maskReg;
    }
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to disable the interrupt for required GPDMA channel */
error_t gpdma_interrupt_disable(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  uint32_t maskReg          = 0;
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (pCfg == NULL) {
    return ERROR_GPDMA_INVALID_ARG;
  }
  if (pCfg->dmaCh <= GPDMA_CHNL7) {
    M4SS_GPDMA_INTR_SEL |= SET_BIT(pCfg->dmaCh);
    maskReg = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if (pCfg->descFetchDoneIntr == 1) {
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | SET_BIT((pCfg->dmaCh + 8)));
    }
    if (pCfg->xferDoneIntr == 1) {
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | SET_BIT((pCfg->dmaCh + 16)));
    }
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to Abort the channel transfer */
error_t gpdma_abort_channel(RSI_GPDMA_HANDLE_T pHandle, uint8_t dmaCh)
{
  GPDMA_DATACONTEXT_T *pDrv               = (GPDMA_DATACONTEXT_T *)pHandle;
  pDrv->baseG->GLOBAL.DMA_CHNL_SQUASH_REG = SET_BIT(dmaCh);
  /* Waiting till squash bit becomes '0' */
  while (pDrv->baseG->GLOBAL.DMA_CHNL_SQUASH_REG & SET_BIT(dmaCh))
    ;
  return RSI_OK;
}

/* Get memory size in bytes needed for GPDMA controller driver context */
uint32_t gpdma_get_mem_size(void)
{
  return sizeof(GPDMA_DATACONTEXT_T);
}

/* This API is used to initialize driver context parameters */
RSI_GPDMA_HANDLE_T gpdma_init(void *mem, const RSI_GPDMA_INIT_T *pInit)
{
  GPDMA_DATACONTEXT_T *pDrv;

  /* Verify alignment is at least 4 bytes */
  if (((uint32_t)mem & 0x3) != 0) {
    return NULL;
  }

  pDrv = (GPDMA_DATACONTEXT_T *)mem;
  memset(pDrv, 0, sizeof(GPDMA_DATACONTEXT_T));

  /* Save pointer to user data */
  pDrv->pUserData = pInit->pUserData;
  pDrv->baseG     = (RSI_GPDMAG_T *)pInit->baseG;
  pDrv->baseC     = (RSI_GPDMAC_T *)pInit->baseC;
  pDrv->sramBase  = (RSI_GPDMA_DESC_T *)pInit->sramBase;

  return (RSI_GPDMA_HANDLE_T)pDrv;
}

/* This API is used to configure required channel for transfer */
error_t gpdma_setup_channel(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  uint32_t maskReg          = 0;
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  /* Parameter checks */
  if ((pCfg != NULL) && (pCfg->dmaCh <= GPDMA_CHNL7) && ((pCfg->channelPrio) <= PRIO_LEVEL_4)) {
    if (RSI_GPDMA_GetChannelActivity(pDrv, pCfg->dmaCh) != 1) {
      pDrv->baseC->CHANNEL_CONFIG[pCfg->dmaCh].FIFO_CONFIG_REGS_b.FIFO_STRT_ADDR = (8 * pCfg->dmaCh);
      pDrv->baseC->CHANNEL_CONFIG[pCfg->dmaCh].FIFO_CONFIG_REGS_b.FIFO_SIZE      = 8;

      pDrv->baseC->CHANNEL_CONFIG[pCfg->dmaCh].PRIORITY_CHNL_REGS_b.PRIORITY_CH = pCfg->channelPrio;

      /* Error status clear */
      if (pCfg->hrespErr == 1) {
        pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = SET_BIT(pCfg->dmaCh * 4);
      }
      if (pCfg->gpdmacErr == 1) {
        pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = SET_BIT(((pCfg->dmaCh * 4) + 3));
      }

      M4SS_GPDMA_INTR_SEL |= SET_BIT(pCfg->dmaCh);
      maskReg = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

      if ((pCfg->descFetchDoneIntr == 1) && (pCfg->xferDoneIntr == 1)) {
        maskReg &= (CLR_BIT(pCfg->dmaCh + 8));
        maskReg &= (CLR_BIT(pCfg->dmaCh + 16));
        pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = maskReg;
      } else if (pCfg->descFetchDoneIntr == 1) {
        maskReg &= (CLR_BIT(pCfg->dmaCh + 8));
        pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = maskReg;
      } else if (pCfg->xferDoneIntr == 1) {
        maskReg &= (CLR_BIT(pCfg->dmaCh + 16));
        pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = maskReg;
      }
    } else {
      return ERROR_GPDMA_CHNL_BUSY;
    }
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  return RSI_OK;
}

/* This API is used to resgister interrupt and error callbacks */
void gpdma_register_callback(RSI_GPDMA_HANDLE_T pHandle, uint32_t cbIndex, void *pCB)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;
  switch (cbIndex) {
    case RSI_GPDMA_XFERCOMPLETE_CB:
      pDrv->gpdmaCompCB = (gpdmaTransferCompleteCB)pCB;
      break;
    case RSI_GPDMA_XFERDESCFETCHCOMPLETE_CB:
      pDrv->gpdmaDescFetchCompCB = (gpdmaTransferDescFetchCompleteCB)pCB;
      break;
    case RSI_GPDMA_XFERHRESPERROR_CB:
      pDrv->gpdmaHrespErrorCB = (gpdmaTransferHrespErrorCB)pCB;
      break;
    case RSI_GPDMA_XFERGPDMACERROR_CB:
      pDrv->gpdmaRpdmacErrorCB = (gpdmaTransferRpdmacErrorCB)pCB;
      break;
  }
}

/* This API is used to build the descriptor parameters */
error_t gpdma_build_descriptors(RSI_GPDMA_HANDLE_T pHandle,
                                RSI_GPDMA_DESC_T *pXferCfg,
                                RSI_GPDMA_DESC_T *pDesc,
                                RSI_GPDMA_DESC_T *pDescPrev)
{
  /* Parameter checks */
  if (pDesc == NULL) {
    return ERROR_GPDMA_INVALID_ARG;
  }

  /* Parameter checks */
  if (pXferCfg->chnlCtrlConfig.srcDataWidth > SRC_32_DATA_WIDTH) {
    return ERROR_GPDMA_INVALID_ARG;
  }

  if (pXferCfg->chnlCtrlConfig.destDataWidth > DST_32_DATA_WIDTH) {
    return ERROR_GPDMA_INVALID_ARG;
  }

  if (pXferCfg->chnlCtrlConfig.transSize <= MAX_TRANS_SIZE) {
    pDesc->chnlCtrlConfig.transSize = pXferCfg->chnlCtrlConfig.transSize;
  } else {
    return ERROR_GPDMA_INVALID_TRANS_LEN;
  }

  /* Assigns transfer mode */
  if ((pXferCfg->chnlCtrlConfig.transType == MEMORY_MEMORY) || (pXferCfg->chnlCtrlConfig.transType == MEMORY_PERIPHERAL)
      || (pXferCfg->chnlCtrlConfig.transType == PERIPHERAL_MEMORY)
      || (pXferCfg->chnlCtrlConfig.transType == PERIPHERAL_PERIPHERAL)) {
    pDesc->chnlCtrlConfig.transType = pXferCfg->chnlCtrlConfig.transType;
  } else {
    return ERROR_GPDMA_INVALID_XFERMODE;
  }

  /* Set flow controller */
  if ((pXferCfg->chnlCtrlConfig.dmaFlwCtrl == DMA_FLW_CTRL) || (pXferCfg->chnlCtrlConfig.dmaFlwCtrl == DST_PERI_CTRL)
      || (pXferCfg->chnlCtrlConfig.dmaFlwCtrl == SRC_PERI_CTRL)
      || (pXferCfg->chnlCtrlConfig.dmaFlwCtrl == SRC_DST_PERI_CTRL)) {
    pDesc->chnlCtrlConfig.dmaFlwCtrl = pXferCfg->chnlCtrlConfig.dmaFlwCtrl;
  } else {
    return ERROR_GPDMA_FLW_CTRL;
  }
  /* Checks master interface 0/1 select for fetch and send of data */
  if ((pXferCfg->chnlCtrlConfig.mastrIfFetchSel == MASTER0_FETCH_IFSEL)
      || (pXferCfg->chnlCtrlConfig.mastrIfFetchSel == MASTER1_FETCH_IFSEL)) {
    pDesc->chnlCtrlConfig.mastrIfFetchSel = pXferCfg->chnlCtrlConfig.mastrIfFetchSel;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }

  if ((pXferCfg->chnlCtrlConfig.mastrIfSendSel == MASTER0_SEND_IFSEL)
      || (pXferCfg->chnlCtrlConfig.mastrIfSendSel == MASTER1_SEND_IFSEL)) {
    pDesc->chnlCtrlConfig.mastrIfSendSel = pXferCfg->chnlCtrlConfig.mastrIfSendSel;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }

  /* checks Destination data width */
  if ((pXferCfg->chnlCtrlConfig.destDataWidth == DST_8_DATA_WIDTH)
      || (pXferCfg->chnlCtrlConfig.destDataWidth == DST_16_DATA_WIDTH)
      || (pXferCfg->chnlCtrlConfig.destDataWidth == DST_32_DATA_WIDTH)) {
    pDesc->chnlCtrlConfig.destDataWidth = pXferCfg->chnlCtrlConfig.destDataWidth;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }

  /* checks source data width */
  if ((pXferCfg->chnlCtrlConfig.srcDataWidth == SRC_8_DATA_WIDTH)
      || (pXferCfg->chnlCtrlConfig.srcDataWidth == SRC_16_DATA_WIDTH)
      || (pXferCfg->chnlCtrlConfig.srcDataWidth == SRC_32_DATA_WIDTH)) {
    pDesc->chnlCtrlConfig.srcDataWidth = pXferCfg->chnlCtrlConfig.srcDataWidth;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* Src align or unalign flag */
  pDesc->chnlCtrlConfig.srcAlign = pXferCfg->chnlCtrlConfig.srcAlign;
  /* enable for non link or link transfers */
  pDesc->chnlCtrlConfig.linkListOn = pXferCfg->chnlCtrlConfig.linkListOn;

  /* master fetch selection */
  if ((pXferCfg->chnlCtrlConfig.linkListMstrSel == LINK_MASTER_0_FTCH)
      || (pXferCfg->chnlCtrlConfig.linkListMstrSel == LINK_MASTER_1_FTCH)) {
    pDesc->chnlCtrlConfig.linkListMstrSel = pXferCfg->chnlCtrlConfig.linkListMstrSel;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* source address contiguous */
  pDesc->chnlCtrlConfig.srcAddContiguous = pXferCfg->chnlCtrlConfig.srcAddContiguous;
  /* destination address contiguous */
  pDesc->chnlCtrlConfig.dstAddContiguous = pXferCfg->chnlCtrlConfig.dstAddContiguous;
  /* Retry on error bit flag */
  pDesc->chnlCtrlConfig.retryOnErr = pXferCfg->chnlCtrlConfig.retryOnErr;
  /* Link inerrupt enable */
  pDesc->chnlCtrlConfig.linkInterrupt = pXferCfg->chnlCtrlConfig.linkInterrupt;
  /* Set source FIFO mode in peripheral transfers */
  pDesc->chnlCtrlConfig.srcFifoMode = pXferCfg->chnlCtrlConfig.srcFifoMode;
  /* Set destination FIFO mode in peripheral transfers */
  pDesc->chnlCtrlConfig.dstFifoMode = pXferCfg->chnlCtrlConfig.dstFifoMode;

  /* Misc Configurations */
  if (pXferCfg->miscChnlCtrlConfig.ahbBurstSize <= AHBBURST_SIZE_MAX) {
    /* Set AHB burst size */
    pDesc->miscChnlCtrlConfig.ahbBurstSize = pXferCfg->miscChnlCtrlConfig.ahbBurstSize;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* Set source burst size */
  if (pXferCfg->miscChnlCtrlConfig.srcDataBurst <= SRC_BURST_SIZE_MAX) {
    pDesc->miscChnlCtrlConfig.srcDataBurst = pXferCfg->miscChnlCtrlConfig.srcDataBurst;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* Set dest burst size */
  if (pXferCfg->miscChnlCtrlConfig.destDataBurst <= DST_BURST_SIZE_MAX) {
    pDesc->miscChnlCtrlConfig.destDataBurst = pXferCfg->miscChnlCtrlConfig.destDataBurst;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* Set dest channel ID */
  if (pXferCfg->miscChnlCtrlConfig.destChannelId <= DST_CHNL_ID_MAX) {
    pDesc->miscChnlCtrlConfig.destChannelId = pXferCfg->miscChnlCtrlConfig.destChannelId;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* Set source  channel ID */
  if (pXferCfg->miscChnlCtrlConfig.srcChannelId <= SRC_CHNL_ID_MAX) {
    pDesc->miscChnlCtrlConfig.srcChannelId = pXferCfg->miscChnlCtrlConfig.srcChannelId;
  } else {
    return ERROR_GPDMA_INVALID_ARG;
  }
  /* DMA protection flag */
  pDesc->miscChnlCtrlConfig.dmaProt = pXferCfg->miscChnlCtrlConfig.dmaProt;
  /* Memory fill enable */
  pDesc->miscChnlCtrlConfig.memoryFillEn = pXferCfg->miscChnlCtrlConfig.memoryFillEn;
  /* Memory fill with either 0 or 1 */
  pDesc->miscChnlCtrlConfig.memoryOneFill = pXferCfg->miscChnlCtrlConfig.memoryOneFill;

  pDesc->src  = (uint32_t *)pXferCfg->src;
  pDesc->dest = (uint32_t *)pXferCfg->dest;

  /* Is the descriptor linked from a previous descriptor? */
  if (pDescPrev) {
    pDescPrev->pNextLink = (uint32_t *)pDesc;
  } else {
    pDesc->pNextLink = NULL;
  }

  return RSI_OK;
}

/* This API is used to prepare descriptor with required configurations and addresses */
error_t gpdma_setup_channelTransfer(RSI_GPDMA_HANDLE_T pHandle, uint8_t dmaCh, RSI_GPDMA_DESC_T *pDesc)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;

  if (dmaCh <= GPDMA_CHNL7) {
    /* Channel control register fields */

    /* Number of bytes per transfer; max = 4096 bytes(4KB) */
    if (pDesc->chnlCtrlConfig.transSize <= MAX_TRANS_SIZE) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.DMA_BLK_SIZE = pDesc->chnlCtrlConfig.transSize;
    } else {
      return ERROR_GPDMA_INVALID_TRANS_LEN;
    }
    /* Assigns transfer mode */
    if ((pDesc->chnlCtrlConfig.transType == MEMORY_MEMORY) || (pDesc->chnlCtrlConfig.transType == MEMORY_PERIPHERAL)
        || (pDesc->chnlCtrlConfig.transType == PERIPHERAL_MEMORY)
        || (pDesc->chnlCtrlConfig.transType == PERIPHERAL_PERIPHERAL)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.TRNS_TYPE = pDesc->chnlCtrlConfig.transType;
    } else {
      return ERROR_GPDMA_INVALID_XFERMODE;
    }
    /* Set flow controller */
    if ((pDesc->chnlCtrlConfig.dmaFlwCtrl == DMA_FLW_CTRL) || (pDesc->chnlCtrlConfig.dmaFlwCtrl == DST_PERI_CTRL)
        || (pDesc->chnlCtrlConfig.dmaFlwCtrl == SRC_PERI_CTRL)
        || (pDesc->chnlCtrlConfig.dmaFlwCtrl == SRC_DST_PERI_CTRL)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.DMA_FLOW_CTRL = pDesc->chnlCtrlConfig.dmaFlwCtrl;
    } else {
      return ERROR_GPDMA_FLW_CTRL;
    }
    /* Checks master interface 0/1 select for fetch and send of data */
    if ((pDesc->chnlCtrlConfig.mastrIfFetchSel == MASTER0_FETCH_IFSEL)
        || (pDesc->chnlCtrlConfig.mastrIfFetchSel == MASTER1_FETCH_IFSEL)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.MSTR_IF_FETCH_SEL =
        pDesc->chnlCtrlConfig.mastrIfFetchSel;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    if ((pDesc->chnlCtrlConfig.mastrIfSendSel == MASTER0_SEND_IFSEL)
        || (pDesc->chnlCtrlConfig.mastrIfSendSel == MASTER1_SEND_IFSEL)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.MSTR_IF_SEND_SEL =
        pDesc->chnlCtrlConfig.mastrIfSendSel;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    /* checks source data width */
    if ((pDesc->chnlCtrlConfig.srcDataWidth == SRC_8_DATA_WIDTH)
        || (pDesc->chnlCtrlConfig.srcDataWidth == SRC_16_DATA_WIDTH)
        || (pDesc->chnlCtrlConfig.srcDataWidth == SRC_32_DATA_WIDTH)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.SRC_DATA_WIDTH = pDesc->chnlCtrlConfig.srcDataWidth;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    /* checks Destination data width */
    if ((pDesc->chnlCtrlConfig.destDataWidth == DST_8_DATA_WIDTH)
        || (pDesc->chnlCtrlConfig.destDataWidth == DST_16_DATA_WIDTH)
        || (pDesc->chnlCtrlConfig.destDataWidth == DST_32_DATA_WIDTH)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.DEST_DATA_WIDTH = pDesc->chnlCtrlConfig.destDataWidth;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.SRC_ALIGN    = pDesc->chnlCtrlConfig.srcAlign;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.LINK_LIST_ON = pDesc->chnlCtrlConfig.linkListOn;

    if ((pDesc->chnlCtrlConfig.linkListMstrSel == LINK_MASTER_0_FTCH)
        || (pDesc->chnlCtrlConfig.linkListMstrSel == LINK_MASTER_1_FTCH)) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.LINK_LIST_MSTR_SEL =
        pDesc->chnlCtrlConfig.linkListMstrSel;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.SRC_ADDR_CONTIGUOUS =
      pDesc->chnlCtrlConfig.srcAddContiguous;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.DEST_ADDR_CONTIGUOUS =
      pDesc->chnlCtrlConfig.dstAddContiguous;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.RETRY_ON_ERROR = pDesc->chnlCtrlConfig.retryOnErr;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.LINK_INTERRUPT = pDesc->chnlCtrlConfig.linkInterrupt;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.SRC_FIFO_MODE  = pDesc->chnlCtrlConfig.srcFifoMode;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].CHANNEL_CTRL_REG_CHNL_b.DEST_FIFO_MODE = pDesc->chnlCtrlConfig.dstFifoMode;

    /* Misc channel control register fields */
    /* Set AHB Burst size */
    if (pDesc->miscChnlCtrlConfig.ahbBurstSize <= AHBBURST_SIZE_MAX) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.AHB_BURST_SIZE =
        pDesc->miscChnlCtrlConfig.ahbBurstSize;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    /* Set Source burst size */
    if (pDesc->miscChnlCtrlConfig.srcDataBurst <= SRC_BURST_SIZE_MAX) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.SRC_DATA_BURST =
        pDesc->miscChnlCtrlConfig.srcDataBurst;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    /* Set Dest burst size */
    if (pDesc->miscChnlCtrlConfig.destDataBurst <= DST_BURST_SIZE_MAX) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.DEST_DATA_BURST =
        pDesc->miscChnlCtrlConfig.destDataBurst;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    /* Set dest channel ID */
    if (pDesc->miscChnlCtrlConfig.destChannelId <= DST_CHNL_ID_MAX) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.DEST_CHNL_ID =
        pDesc->miscChnlCtrlConfig.destChannelId;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }

    /* Set source channel ID */
    if (pDesc->miscChnlCtrlConfig.srcChannelId <= SRC_CHNL_ID_MAX) {
      pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.SRC_CHNL_ID =
        pDesc->miscChnlCtrlConfig.srcChannelId;
    } else {
      return ERROR_GPDMA_INVALID_ARG;
    }
    /* DMA protection flag */
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.DMA_PROT = pDesc->miscChnlCtrlConfig.dmaProt;
    /* Memory fill enable */
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.MEM_FILL_ENABLE =
      pDesc->miscChnlCtrlConfig.memoryFillEn;
    /* Memory fill with either 0 or 1 */
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].MISC_CHANNEL_CTRL_REG_CHNL_b.MEM_ONE_FILL =
      pDesc->miscChnlCtrlConfig.memoryOneFill;

    pDrv->baseC->CHANNEL_CONFIG[dmaCh].SRC_ADDR_REG_CHNL  = (uint32_t)pDesc->src;
    pDrv->baseC->CHANNEL_CONFIG[dmaCh].DEST_ADDR_REG_CHNL = (uint32_t)pDesc->dest;

    pDrv->baseC->CHANNEL_CONFIG[dmaCh].LINK_LIST_PTR_REGS = (uint32_t)pDesc;
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
  return RSI_OK;
}

/* This API is used to handle interrupts for all channels  */
void gpdma_interrupt_handler(RSI_GPDMA_HANDLE_T pHandle)
{
  uint32_t status           = 0;
  RSI_GPDMA_DESC_T *pDesc   = NULL;
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;
  uint32_t maskReg          = 0;

  status = pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG;

  /* Channel 0 */
  if ((status & GPDMA_STAT_HRESP_ERROR0) || (status & GPDMA_STAT_LINK_LIST_DONE0)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER0) || (status & GPDMA_STAT_GPDMAC_ERROR0)) {
    pDrv->dmaCh = 0;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR0) || (status & GPDMA_STAT_GPDMAC_ERROR0)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER0 | GPDMA_STAT_LINK_LIST_DONE0);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 0);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR0 | GPDMA_STAT_GPDMAC_ERROR0;

      if (status & GPDMA_STAT_HRESP_ERROR0) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 0);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR0) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 0);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE0) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE0;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 0);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER0) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER0;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 0);
      }
    }
  }

  /* Channel 1 */
  if ((status & GPDMA_STAT_HRESP_ERROR1) || (status & GPDMA_STAT_LINK_LIST_DONE1)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER1) || (status & GPDMA_STAT_GPDMAC_ERROR1)) {
    pDrv->dmaCh = 1;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR1) || (status & GPDMA_STAT_GPDMAC_ERROR1)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER1 | GPDMA_STAT_LINK_LIST_DONE1);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 1);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR1 | GPDMA_STAT_GPDMAC_ERROR1;

      if (status & GPDMA_STAT_HRESP_ERROR1) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 1);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR1) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 1);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE1) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE1;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 1);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER1) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER1;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 1);
      }
    }
  }

  /* Channel 2 */
  if ((status & GPDMA_STAT_HRESP_ERROR2) || (status & GPDMA_STAT_LINK_LIST_DONE2)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER2) || (status & GPDMA_STAT_GPDMAC_ERROR2)) {
    pDrv->dmaCh = 2;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR2) || (status & GPDMA_STAT_GPDMAC_ERROR2)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER2 | GPDMA_STAT_LINK_LIST_DONE2);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 2);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR2 | GPDMA_STAT_GPDMAC_ERROR2;

      if (status & GPDMA_STAT_HRESP_ERROR2) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 2);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR0) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 2);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE2) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE2;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 2);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER2) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER2;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 2);
      }
    }
  }

  /* Channel 3 */
  if ((status & GPDMA_STAT_HRESP_ERROR3) || (status & GPDMA_STAT_LINK_LIST_DONE3)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER3) || (status & GPDMA_STAT_GPDMAC_ERROR3)) {
    pDrv->dmaCh = 3;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR3) || (status & GPDMA_STAT_GPDMAC_ERROR3)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER3 | GPDMA_STAT_LINK_LIST_DONE3);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 3);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR3 | GPDMA_STAT_GPDMAC_ERROR3;

      if (status & GPDMA_STAT_HRESP_ERROR3) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 3);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR3) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 3);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE3) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE3;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 3);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER3) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER3;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 3);
      }
    }
  }
  /* Channel 4 */
  if ((status & GPDMA_STAT_HRESP_ERROR4) || (status & GPDMA_STAT_LINK_LIST_DONE4)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER4) || (status & GPDMA_STAT_GPDMAC_ERROR4)) {
    pDrv->dmaCh = 4;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR4) || (status & GPDMA_STAT_GPDMAC_ERROR4)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER4 | GPDMA_STAT_LINK_LIST_DONE4);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 4);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR0 | GPDMA_STAT_GPDMAC_ERROR4;

      if (status & GPDMA_STAT_HRESP_ERROR4) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 4);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR4) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 4);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE4) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE4;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 4);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER4) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER4;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 4);
      }
    }
  }
  /* Channel 5 */
  if ((status & GPDMA_STAT_HRESP_ERROR5) || (status & GPDMA_STAT_LINK_LIST_DONE5)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER5) || (status & GPDMA_STAT_GPDMAC_ERROR5)) {
    pDrv->dmaCh = 5;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR5) || (status & GPDMA_STAT_GPDMAC_ERROR5)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER5 | GPDMA_STAT_LINK_LIST_DONE5);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 5);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR5 | GPDMA_STAT_GPDMAC_ERROR5;

      if (status & GPDMA_STAT_HRESP_ERROR5) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 5);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR0) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 5);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE5) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE5;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 5);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER5) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER5;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 5);
      }
    }
  }
  /* Channel 6 */
  if ((status & GPDMA_STAT_HRESP_ERROR6) || (status & GPDMA_STAT_LINK_LIST_DONE6)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER6) || (status & GPDMA_STAT_GPDMAC_ERROR6)) {
    pDrv->dmaCh = 6;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR6) || (status & GPDMA_STAT_GPDMAC_ERROR6)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER6 | GPDMA_STAT_LINK_LIST_DONE6);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 6);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR6 | GPDMA_STAT_GPDMAC_ERROR6;

      if (status & GPDMA_STAT_HRESP_ERROR6) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 6);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR6) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 6);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE6) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE6;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 6);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER6) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER6;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 6);
      }
    }
  }
  /* Channel 7 */
  if ((status & GPDMA_STAT_HRESP_ERROR7) || (status & GPDMA_STAT_LINK_LIST_DONE7)
      || (status & GPDMA_STAT_PHRL_END_OF_XFER7) || (status & GPDMA_STAT_GPDMAC_ERROR7)) {
    pDrv->dmaCh = 7;
    maskReg     = pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG;

    if ((status & GPDMA_STAT_HRESP_ERROR7) || (status & GPDMA_STAT_GPDMAC_ERROR7)) {
      /* Disable interrupts for this channel */
      pDrv->baseG->GLOBAL.INTERRUPT_MASK_REG = (maskReg | GPDMA_STAT_PHRL_END_OF_XFER7 | GPDMA_STAT_LINK_LIST_DONE7);

      /* Abort current descriptor */
      gpdma_abort_channel(pHandle, 7);

      /* Error status clear */
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_HRESP_ERROR7 | GPDMA_STAT_GPDMAC_ERROR7;

      if (status & GPDMA_STAT_HRESP_ERROR7) {
        /* Call error callback for channel */
        if (pDrv->gpdmaHrespErrorCB) {
          pDrv->gpdmaHrespErrorCB(pHandle, pDesc, 7);
        }
      }
      if (status & GPDMA_STAT_GPDMAC_ERROR7) {
        /* Call error callback for channel */
        if (pDrv->gpdmaRpdmacErrorCB) {
          pDrv->gpdmaRpdmacErrorCB(pHandle, pDesc, 7);
        }
      }
    }
    if (status & GPDMA_STAT_LINK_LIST_DONE7) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_LINK_LIST_DONE7;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaDescFetchCompCB) {
        pDrv->gpdmaDescFetchCompCB(pHandle, pDesc, 7);
      }
    }
    if (status & GPDMA_STAT_PHRL_END_OF_XFER7) {
      pDrv->baseG->GLOBAL.INTERRUPT_STAT_REG = GPDMA_STAT_PHRL_END_OF_XFER7;
      /* Call transfer descriptor completion for channel */
      if (pDrv->gpdmaCompCB) {
        pDrv->gpdmaCompCB(pHandle, pDesc, 7);
      }
    }
  }
}
/*  This API is used to Uninitialize RPMDA configurations */
void gpdma_deInit(RSI_GPDMA_HANDLE_T pHandle, RSI_GPDMA_CHA_CFG_T *pCfg)
{
  gpdma_interrupt_clear(pHandle, pCfg);
  gpdma_error_status_clear(pHandle, pCfg);
}

/* This API is used to check the required GPDMA channel is enabled or not */
uint32_t gpdma_channel_is_enabled(RSI_GPDMA_HANDLE_T pHandle, uint8_t dmaCh)
{
  GPDMA_DATACONTEXT_T *pDrv = (GPDMA_DATACONTEXT_T *)pHandle;
  if (dmaCh <= GPDMA_CHNL7) {
    /* Enables the required channel */
    return (pDrv->baseG->GLOBAL.DMA_CHNL_ENABLE_REG & SET_BIT(dmaCh));
  } else {
    return ERROR_GPDMA_INVALIDCHNLNUM;
  }
}

/*ROM API Structure
const ROM_RPDMA_API_T gpdma_api = {
		&gpdma_get_mem_size,
		&gpdma_init,
		&gpdma_register_callback,
		&gpdma_abort_channel,
		&gpdma_setup_channel,
		&gpdma_build_descriptors,
		&gpdma_setup_channelTransfer,
		&gpdma_interrupt_handler,
		&gpdma_deInit,
		&gpdma_dma_channel_trigger,
		&gpdma_channel_is_enabled,
		&gpdma_interrupt_disable,
		&gpdma_interrupt_enable,
		&gpdma_error_status_clear,
		&gpdma_get_error_status,
		&gpdma_interrupt_clear,
		&gpdma_interrupt_status
};
*/
#ifdef __cplusplus
}
#endif
#endif //ROMDRIVER_PRESENT
/*************** EOF ****************/
