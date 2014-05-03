/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Examples/STM32_CPAL_I2C/Advanced_Example/cpal_usercallback.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file provides all the CPAL UserCallback functions .
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"



/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern I2C_TypeDef* CPAL_I2C_DEVICE[];
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*------------------------------------------------------------------------------
                     CPAL User Callbacks implementations 
------------------------------------------------------------------------------*/


/*=========== Timeout UserCallback ===========*/


/**
  * @brief  User callback that manages the Timeout error.
  * @param  pDevInitStruct .
  * @retval None.
  */
uint32_t CPAL_TIMEOUT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
  /* Generate STOP */
  __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
  
  //LCD_DisplayStringLine(Line7, (uint8_t*)"Timeout Err occurred ");
  rt_kprintf("I2C timeout error!\n");
//  /* Deinitialize peripheral */
//  sEE_DeInit(sEE_DevStructures[pDevInitStruct->CPAL_Dev]) ;
//  
//  /* Initialize peripheral to communication with sEE and IOE and TempSensor */
//  sEE_StructInit(sEE_DevStructures[pDevInitStruct->CPAL_Dev]);
//  sEE_Init(sEE_DevStructures[pDevInitStruct->CPAL_Dev]); 
//  
//  /* Initialize sEE state */
//  if ((sEE_DevStructures[pDevInitStruct->CPAL_Dev]->sEEState == sEE_STATE_WRITING)\
//    || (sEE_DevStructures[pDevInitStruct->CPAL_Dev]->sEEState == sEE_STATE_READING))
//  {    
//    sEE_DevStructures[pDevInitStruct->CPAL_Dev]->sEEState = sEE_STATE_ERROR;
//  }  
  
  return CPAL_PASS;
}


/*=========== Transfer UserCallback ===========*/


/**
  * @brief  Manages the End of Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
//  if (sEE_DevStructures[pDevInitStruct->CPAL_Dev]->sEEState == sEE_STATE_WRITING)
//  {
//    sEE_WriteHandler(pDevInitStruct->CPAL_Dev);
//  }
}


/**
  * @brief  Manages the End of Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
//  if (sEE_DevStructures[pDevInitStruct->CPAL_Dev]->sEEState == sEE_STATE_READING)
//  {
//    sEE_ReadHandler(pDevInitStruct->CPAL_Dev);
//  }
}

/**
  * @brief  Manages Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_TX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
 
}*/


/**
  * @brief  Manages Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_RX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
 
}*/


/**
  * @brief  Manages the End of DMA Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
 
}*/


/**
  * @brief  Manages the Half of DMA Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMATXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  Manages Error of DMA Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMATXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  Manages the End of DMA Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMARXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
 
}*/


/**
  * @brief  Manages the Half of DMA Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_DMARXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  Manages Error of DMA Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_DMARXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/



/*=========== Error UserCallback ===========*/


/**
  * @brief  User callback that manages the I2C device errors.
  * @note   Make sure that the define USE_SINGLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInitStruct. 
  * @param  DeviceError.
  * @retval None
  */ 
void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)
{  
  /* Generate STOP */
  __CPAL_I2C_HAL_STOP(pDevInstance);
  
 // LCD_DisplayStringLine(Line6, (uint8_t*)" Device Err occurred ");
  rt_kprintf("I2C device error\n");
  /* Deinitialize peripheral */
//  sEE_DeInit(sEE_DevStructures[pDevInstance]) ;
//  
//  /* Initialize peripheral To communication with sEE and IOE and TempSensor*/
//  sEE_StructInit(sEE_DevStructures[pDevInstance]);
//  sEE_Init(sEE_DevStructures[pDevInstance]); 
//  
//  /* Initialize sEE state */
//  if ((sEE_DevStructures[pDevInstance]->sEEState == sEE_STATE_WRITING)\
//    || (sEE_DevStructures[pDevInstance]->sEEState == sEE_STATE_READING))
//  {      
//    sEE_DevStructures[pDevInstance]->sEEState = sEE_STATE_ERROR;
//  }  
}


/**
  * @brief  User callback that manages BERR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */  
/*void CPAL_I2C_BERR_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages ARLO I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_ARLO_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages OVR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_OVR_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages AF I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */ 
/*void CPAL_I2C_AF_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/*=========== Addressing Mode UserCallback ===========*/


/**
  * @brief  User callback that manage General Call Addressing mode.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_GENCALL_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  User callback that manage Dual Address Addressing mode.
  * @param  pDevInitStruct 
  * @retval None
  */  
/*void CPAL_I2C_DUALF_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/
 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
