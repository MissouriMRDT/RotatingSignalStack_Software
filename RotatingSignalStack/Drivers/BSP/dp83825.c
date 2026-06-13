/**
  ******************************************************************************
  * @file    DP83825.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the LAN742
  *          PHY devices.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "DP83825.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup DP83825 DP83825
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup DP83825_Private_Defines DP83825 Private Defines
  * @{
  */
#define DP83825_MAX_DEV_ADDR   ((uint32_t)31U)
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @defgroup DP83825_Private_Functions DP83825 Private Functions
  * @{
  */

/**
  * @brief  Register IO functions to component object
  * @param  pObj: device object  of DP83825_Object_t.
  * @param  ioctx: holds device IO functions.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_ERROR if missing mandatory function
  */
int32_t  DP83825_RegisterBusIO(DP83825_Object_t *pObj, DP83825_IOCtx_t *ioctx)
{
  if(!pObj || !ioctx->ReadReg || !ioctx->WriteReg || !ioctx->GetTick)
  {
    return DP83825_STATUS_ERROR;
  }

  pObj->IO.Init = ioctx->Init;
  pObj->IO.DeInit = ioctx->DeInit;
  pObj->IO.ReadReg = ioctx->ReadReg;
  pObj->IO.WriteReg = ioctx->WriteReg;
  pObj->IO.GetTick = ioctx->GetTick;

  return DP83825_STATUS_OK;
}

/**
  * @brief  Initialize the DP83825 and configure the needed hardware resources
  * @param  pObj: device object DP83825_Object_t.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_ADDRESS_ERROR if cannot find device address
  *         DP83825_STATUS_READ_ERROR if cannot read register
  */
 int32_t DP83825_Init(DP83825_Object_t *pObj)
 {
   uint32_t regvalue = 0, addr = 0;
   int32_t status = DP83825_STATUS_OK;

   if(pObj->Is_Initialized == 0)
   {
     if(pObj->IO.Init != 0)
     {
       /* GPIO and Clocks initialization */
       pObj->IO.Init();
     }

     /* for later check */
     pObj->DevAddr = DP83825_MAX_DEV_ADDR + 1;

     /* Get the device address from special mode register */
     for(addr = 0; addr <= DP83825_MAX_DEV_ADDR; addr ++)
     {
       if(pObj->IO.ReadReg(addr, DP83825_BMCR, &regvalue) < 0)
       {
         status = DP83825_STATUS_READ_ERROR;
         /* Can't read from this device address
            continue with next address */
         continue;
       }

       if((regvalue & DP83825_PHYCR_PHY_ADDR) == addr)
       {
         pObj->DevAddr = addr;
         status = DP83825_STATUS_OK;
         break;
       }
     }

     if(pObj->DevAddr > DP83825_MAX_DEV_ADDR)
     {
       status = DP83825_STATUS_ADDRESS_ERROR;
     }

     /* if device address is matched */
     if(status == DP83825_STATUS_OK)
     {
       pObj->Is_Initialized = 1;
     }
   }

   return status;
 }

/**
  * @brief  De-Initialize the DP83825 and it's hardware resources
  * @param  pObj: device object DP83825_Object_t.
  * @retval None
  */
int32_t DP83825_DeInit(DP83825_Object_t *pObj)
{
  if(pObj->Is_Initialized)
  {
    if(pObj->IO.DeInit != 0)
    {
      if(pObj->IO.DeInit() < 0)
      {
        return DP83825_STATUS_ERROR;
      }
    }

    pObj->Is_Initialized = 0;
  }

  return DP83825_STATUS_OK;
}

/**
  * @brief  Disable the DP83825 power down mode.
  * @param  pObj: device object DP83825_Object_t.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_DisablePowerDownMode(DP83825_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &readval) >= 0)
  {
    readval &= ~DP83825_BMCR_POWER_DOWN;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_BMCR, readval) < 0)
    {
      status =  DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Enable the DP83825 power down mode.
  * @param  pObj: device object DP83825_Object_t.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_EnablePowerDownMode(DP83825_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &readval) >= 0)
  {
    readval |= DP83825_BMCR_POWER_DOWN;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_BMCR, readval) < 0)
    {
      status =  DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Start the auto negotiation process.
  * @param  pObj: device object DP83825_Object_t.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_StartAN(DP83825_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &readval) >= 0)
  {
    readval |= DP83825_BMCR_AN_EN;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_BMCR, readval) < 0)
    {
      status =  DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Get the link state of DP83825 device.
  * @param  pObj: Pointer to device object.
  * @param  pLinkState: Pointer to link state
  * @retval DP83825_STATUS_LINK_DOWN  if link is down
  *         DP83825_STATUS_AN_NOTDONE if Auto nego not completed
  *         DP83825_STATUS_100MBITS_FULLDUPLEX if 100Mb/s FD
  *         DP83825_STATUS_100MBITS_HALFDUPLEX if 100Mb/s HD
  *         DP83825_STATUS_10MBITS_FULLDUPLEX  if 10Mb/s FD
  *         DP83825_STATUS_10MBITS_HALFDUPLEX  if 10Mb/s HD
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_GetLinkState(DP83825_Object_t *pObj)
{
  uint32_t readval = 0;

  /* Read Status register  */
  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMSR, &readval) < 0)
  {
    return DP83825_STATUS_READ_ERROR;
  }

  /* Read Status register again */
  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMSR, &readval) < 0)
  {
    return DP83825_STATUS_READ_ERROR;
  }

  if((readval & DP83825_BMSR_LINK_STATUS) == 0)
  {
    /* Return Link Down status */
    return DP83825_STATUS_LINK_DOWN;
  }

  /* Check Auto negotiation */
  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &readval) < 0)
  {
    return DP83825_STATUS_READ_ERROR;
  }

  if((readval & DP83825_BMCR_AN_EN) != DP83825_BMCR_AN_EN)
  {
    if(((readval & DP83825_BMCR_SPEED_SEL) == DP83825_BMCR_SPEED_SEL) && ((readval & DP83825_BMCR_DUPLEX_MODE) == DP83825_BMCR_DUPLEX_MODE))
    {
      return DP83825_STATUS_100MBITS_FULLDUPLEX;
    }
    else if ((readval & DP83825_BMCR_SPEED_SEL) == DP83825_BMCR_SPEED_SEL)
    {
      return DP83825_STATUS_100MBITS_HALFDUPLEX;
    }
    else if ((readval & DP83825_BMCR_DUPLEX_MODE) == DP83825_BMCR_DUPLEX_MODE)
    {
      return DP83825_STATUS_10MBITS_FULLDUPLEX;
    }
    else
    {
      return DP83825_STATUS_10MBITS_HALFDUPLEX;
    }
  }
  else /* Auto Nego enabled */
  {
    if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_PHYSCR, &readval) < 0)
    {
      return DP83825_STATUS_READ_ERROR;
    }

    /* Check if auto nego not done */
    if((readval & DP83825_BMSR_AN_COMPLETE) == 0)
    {
      return DP83825_STATUS_AUTONEGO_NOTDONE;
    }


    if(readval & DP83825_BMSR_100BTX_FD)
    {
      return DP83825_STATUS_100MBITS_FULLDUPLEX;
    }
    else if (readval & DP83825_BMSR_100BTX_HD)
    {
      return DP83825_STATUS_100MBITS_HALFDUPLEX;
    }
    else if (readval & DP83825_BMSR_10BT_FD)
    {
      return DP83825_STATUS_10MBITS_FULLDUPLEX;
    }
    else
    {
      return DP83825_STATUS_10MBITS_HALFDUPLEX;
    }
  }
}

/**
  * @brief  Set the link state of DP83825 device.
  * @param  pObj: Pointer to device object.
  * @param  pLinkState: link state can be one of the following
  *         DP83825_STATUS_100MBITS_FULLDUPLEX if 100Mb/s FD
  *         DP83825_STATUS_100MBITS_HALFDUPLEX if 100Mb/s HD
  *         DP83825_STATUS_10MBITS_FULLDUPLEX  if 10Mb/s FD
  *         DP83825_STATUS_10MBITS_HALFDUPLEX  if 10Mb/s HD
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_ERROR  if parameter error
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_SetLinkState(DP83825_Object_t *pObj, uint32_t LinkState)
{
  uint32_t BMCRvalue = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &BMCRvalue) >= 0)
  {
    /* Disable link config (Auto nego, speed and duplex) */
    BMCRvalue &= ~(DP83825_BMCR_AN_EN | DP83825_BMCR_SPEED_SEL | DP83825_BMCR_DUPLEX_MODE);

    if(LinkState == DP83825_STATUS_100MBITS_FULLDUPLEX)
    {
      BMCRvalue |= (DP83825_BMCR_SPEED_SEL | DP83825_BMCR_DUPLEX_MODE);
    }
    else if (LinkState == DP83825_STATUS_100MBITS_HALFDUPLEX)
    {
      BMCRvalue |= DP83825_BMCR_SPEED_SEL;
    }
    else if (LinkState == DP83825_STATUS_10MBITS_FULLDUPLEX)
    {
      BMCRvalue |= DP83825_BMCR_DUPLEX_MODE;
    }
    else
    {
      /* Wrong link status parameter */
      status = DP83825_STATUS_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  if(status == DP83825_STATUS_OK)
  {
    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_BMCR, BMCRvalue) < 0)
    {
      status = DP83825_STATUS_WRITE_ERROR;
    }
  }

  return status;
}

/**
  * @brief  Enable loopback mode.
  * @param  pObj: Pointer to device object.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_EnableLoopbackMode(DP83825_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &readval) >= 0)
  {
    readval |= DP83825_BMCR_LOOPBACK;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_BMCR, readval) < 0)
    {
      status = DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Disable loopback mode.
  * @param  pObj: Pointer to device object.
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_DisableLoopbackMode(DP83825_Object_t *pObj)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_BMCR, &readval) >= 0)
  {
    readval &= ~DP83825_BMCR_LOOPBACK;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_BMCR, readval) < 0)
    {
      status =  DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Enable IT source.
  * @param  pObj: Pointer to device object.
  * @param  Interrupt: IT source to be enabled
  *         should be a value or a combination of the following:
  *         DP83825_LINK_QUALITY_INTERRUPT_ENABLE
  *         DP83825_ENERGY_DETECT_INTERRUPT_ENABLE
  *         DP83825_LINK_INTERRUPT_ENABLE
  *         DP83825_SPEED_INTERRUPT_ENABLE
  *         DP83825_DUPLEX_INTERRUPT_ENABLE
  *         DP83825_AUTO_NEGOTIATION_COMPLETE_INT_ENABLE
  *         DP83825_FALSE_CARRIER_HALF_FULL_INT_ENABLE
  *         DP83825_RHF_INTERRUPT_ENABLE
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_EnableIT(DP83825_Object_t *pObj, uint32_t Interrupt)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_REG_12, &readval) >= 0)
  {
    readval |= Interrupt;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_REG_12, readval) < 0)
    {
      status =  DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Disable IT source.
  * @param  pObj: Pointer to device object.
  * @param  Interrupt: IT source to be disabled
  *         should be a value or a combination of the following:
  *         DP83825_LINK_QUALITY_INTERRUPT_ENABLE
  *         DP83825_ENERGY_DETECT_INTERRUPT_ENABLE
  *         DP83825_LINK_INTERRUPT_ENABLE
  *         DP83825_SPEED_INTERRUPT_ENABLE
  *         DP83825_DUPLEX_INTERRUPT_ENABLE
  *         DP83825_AUTO_NEGOTIATION_COMPLETE_INT_ENABLE
  *         DP83825_FALSE_CARRIER_HALF_FULL_INT_ENABLE
  *         DP83825_RHF_INTERRUPT_ENABLE
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  *         DP83825_STATUS_WRITE_ERROR if cannot write to register
  */
int32_t DP83825_DisableIT(DP83825_Object_t *pObj, uint32_t Interrupt)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_REG_12, &readval) >= 0)
  {
    readval &= ~Interrupt;

    /* Apply configuration */
    if(pObj->IO.WriteReg(pObj->DevAddr, DP83825_REG_12, readval) < 0)
    {
      status = DP83825_STATUS_WRITE_ERROR;
    }
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Clear IT flag.
  * @param  pObj: Pointer to device object.
  * @param  Interrupt: IT flag to be cleared
  *         should be a value or a combination of the following:
  *         DP83825_WOL_IT
  *         DP83825_ENERGYON_IT
  *         DP83825_AN_COMPLETE_IT
  *         DP83825_REMOTE_FAULT_IT
  *         DP83825_LINK_DOWN_IT
  *         DP83825_AN_LP_ACK_IT
  *         DP83825_PARALLEL_DETECTION_FAULT_IT
  *         DP83825_AN_PAGE_RECEIVED_IT
  * @retval DP83825_STATUS_OK  if OK
  *         DP83825_STATUS_READ_ERROR if cannot read register
  */
int32_t  DP83825_ClearIT(DP83825_Object_t *pObj, uint32_t Interrupt)
{
  uint32_t readval = 0;
  int32_t status = DP83825_STATUS_OK;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_REG_12, &readval) < 0)
  {
    status =  DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @brief  Get IT Flag status.
  * @param  pObj: Pointer to device object.
  * @param  Interrupt: IT Flag to be checked,
  *         should be a value or a combination of the following:
  *             DP83825_LINK_QUALITY_INTERRUPT
  *             DP83825_ENERGY_DETECT_INTERRUPT
  *             DP83825_LINK_INTERRUPT
  *             DP83825_SPEED_INTERRUPT
  *             DP83825_DUPLEX_INTERRUPT
  *             DP83825_AUTO_NEGOTIATION_COMPLETE_INTERRUPT
  *             DP83825_FALSE_CARRIER_HALF_FULL_INTERRUPT
  *             DP83825_RHF_INTERRUPT
  * @retval 1 IT flag is SET
  *         0 IT flag is RESET
  *         DP83825_STATUS_READ_ERROR if cannot read register
  */
int32_t DP83825_GetITStatus(DP83825_Object_t *pObj, uint32_t Interrupt)
{
  uint32_t readval = 0;
  int32_t status = 0;

  if(pObj->IO.ReadReg(pObj->DevAddr, DP83825_REG_12, &readval) >= 0)
  {
    status = ((readval & Interrupt) == Interrupt);
  }
  else
  {
    status = DP83825_STATUS_READ_ERROR;
  }

  return status;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
