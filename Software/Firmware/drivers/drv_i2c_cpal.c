/**
  ******************************************************************************
  * @file    STM32_EVAL_CPAL/Common/drv_i2c_cpal.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file provides a set of functions needed to manage the I2C LM75 
  *          temperature sensor mounted on STM32xx-EVAL board (refer to stm32_eval.h
  *          to know about the boards supporting this sensor). 
  *          It implements a high level communication layer for read and write 
  *          from/to this sensor. The needed STM32 hardware resources (I2C and 
  *          GPIO) are defined in stm32xx_eval.h file, and the initialization is 
  *          performed in I2C1_LowLevel_Init() function declared in stm32xx_eval.c 
  *          file.
  *            
  *          @note This file is a modified version of stm32_eval_i2c_tsensor.c driver;
  *                I2C CPAL library drivers are used instead of the Standard Peripherals
  *                I2C driver.
  *                 
  *          You can easily tailor this driver to any other development board, 
  *          by just adapting the defines for hardware resources and 
  *          I2C1_LowLevel_Init() function.
  *
  *     +-----------------------------------------------------------------+
  *     |                        Pin assignment                           |                 
  *     +---------------------------------------+-----------+-------------+
  *     |  STM32 I2C Pins                       |   STLM75  |   Pin       |
  *     +---------------------------------------+-----------+-------------+
  *     | I2C1_I2C_SDA_PIN/ SDA                 |   SDA     |    1        |
  *     | I2C1_I2C_SCL_PIN/ SCL                 |   SCL     |    2        |
  *     |                                       |   OS/INT  |    3        |
  *     | .                                     |   GND     |    4  (0V)  |
  *     | .                                     |   GND     |    5  (0V)  |
  *     | .                                     |   GND     |    6  (0V)  |
  *     | .                                     |   GND     |    7  (0V)  |
  *     | .                                     |   VDD     |    8  (3.3V)|
  *     +---------------------------------------+-----------+-------------+
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
#include "drv_i2c_cpal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define I2C1_SD_SET      0x01 /*!< Set SD bit in the configuration register */
#define I2C1_SD_RESET    0xFE /*!< Reset SD bit in the configuration register */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  
CPAL_TransferTypeDef  I2C1_RXTransfer = { 
                        /* Initialize TX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

CPAL_TransferTypeDef  I2C1_TXTransfer = {
                        /* Initialize RX Transfer structure */
                        pNULL,
                        0,
                        0,
                        0};

uint8_t I2C1_Buffer[2] = {0x00,0x00}; 

extern CPAL_InitTypeDef I2C1_DevStructure;


__IO uint32_t  I2C1_Timeout = I2C1_TIMEOUT; 

/* Private function prototypes -----------------------------------------------*/
static void I2C1_StructInit(void);
static void I2C1_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the I2C1_I2C.
  * @param  None
  * @retval None
  */
void I2C1_Config(void)
{
  I2C1_StructInit ();
  I2C1_Init();
}

/**
  * @brief  Deinitialize the I2C1_I2C.
  * @param  None
  * @retval None
  */
void I2C1_DeInit(void)
{
    /* Initialize CPAL peripheral */
  CPAL_I2C_DeInit(&I2C1_DevStructure);
}

/**
  * @brief  Initializes the I2C1_I2C.
  * @param  None
  * @retval None
  */
static void I2C1_Init(void)
{
  /* Initialize CPAL peripheral */
  CPAL_I2C_Init(&I2C1_DevStructure);
}

/**
  * @brief  Initializes the I2C1_I2C.
  * @param  None
  * @retval None
  */
static void I2C1_StructInit(void)
{
  /* Set CPAL structure parameters to their default values */  
  CPAL_I2C_StructInit(&I2C1_DevStructure);
    
  /* Set I2C clock speed */
  I2C1_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C1_SPEED;
  			 
#ifdef I2C1_IT
  /* Select Interrupt programming model and disable all options */
  I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
  I2C1_DevStructure.wCPAL_Options  = 0;
#else
  /* Select DMA programming model and activate TX_DMA_TC and RX_DMA_TC interrupts */
  I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
  I2C1_DevStructure.wCPAL_Options  = CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMARX_TCIT ;
#endif /* I2C1_IT */  

  /* point to CPAL_TransferTypeDef structure */
  I2C1_DevStructure.pCPAL_TransferTx = &I2C1_TXTransfer;
  I2C1_DevStructure.pCPAL_TransferRx = &I2C1_RXTransfer;
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

uint8_t i2c_rx_byte(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg)
{
    uint8_t rx_data = 0;
  
    /* Disable all options */
    I2C1_DevStructure.wCPAL_Options = 0;

    /* point to CPAL_TransferTypeDef structure */
    I2C1_DevStructure.pCPAL_TransferRx = &I2C1_RXTransfer;
  
    /* Configure transfer parameters */  
    I2C1_DevStructure.pCPAL_TransferRx->wNumData = 1;
    I2C1_DevStructure.pCPAL_TransferRx->pbBuffer = &rx_data ;
    I2C1_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)(addr << 1);
    I2C1_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)reg;
  
    /* Read Operation */
    if(CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS)
    {
        while ((I2C1_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C1_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
        { }
    }
  
    /* return a Reg value */
    return rx_data;  
}

I2C1_Status_TypDef i2c_rx_byte_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t* buf)
{   
    return I2C1_OK;
}

I2C1_Status_TypDef i2c_rx_mbytes_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    /* Disable all options */
    I2C1_DevStructure.wCPAL_Options = 0;

    /* point to CPAL_TransferTypeDef structure */
    I2C1_DevStructure.pCPAL_TransferRx = &I2C1_RXTransfer;
  
    /* Configure transfer parameters */  
    I2C1_DevStructure.pCPAL_TransferRx->wNumData = len;
    I2C1_DevStructure.pCPAL_TransferRx->pbBuffer = buf ;
    I2C1_DevStructure.pCPAL_TransferRx->wAddr1   = (uint32_t)(addr << 1);
    I2C1_DevStructure.pCPAL_TransferRx->wAddr2   = (uint32_t)reg;
  
    /* Read Operation */
    if(CPAL_I2C_Read(&I2C1_DevStructure) == CPAL_PASS)
    {
        while ((I2C1_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C1_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
        { }
    }

    return I2C1_OK;
}

I2C1_Status_TypDef i2c_tx_cmd(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t cmd)
{
    uint8_t tx_cmd = cmd;
     
    /* Disable all options */
    I2C1_DevStructure.wCPAL_Options = CPAL_OPT_NO_MEM_ADDR;
  
    /* point to CPAL_TransferTypeDef structure */
    I2C1_DevStructure.pCPAL_TransferTx = &I2C1_TXTransfer;
  
    /* Configure transfer parameters */  
    I2C1_DevStructure.pCPAL_TransferTx->wNumData = 1;
    I2C1_DevStructure.pCPAL_TransferTx->pbBuffer = &tx_cmd ;
    I2C1_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)(addr << 1);
  
    /* Write Operation */
    if(CPAL_I2C_Write(&I2C1_DevStructure) == CPAL_PASS)
    {
        while ((I2C1_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C1_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
        { }
    
        if (I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
        {
            return I2C1_FAIL;
        }
    }
    else
    {
        return I2C1_FAIL;
    }
  
    return I2C1_OK;
}

I2C1_Status_TypDef i2c_tx_byte(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t tx_data = data;
     
    /* Disable all options */
    I2C1_DevStructure.wCPAL_Options = 0;
  
    /* point to CPAL_TransferTypeDef structure */
    I2C1_DevStructure.pCPAL_TransferTx = &I2C1_TXTransfer;
  
    /* Configure transfer parameters */  
    I2C1_DevStructure.pCPAL_TransferTx->wNumData = 1;
    I2C1_DevStructure.pCPAL_TransferTx->pbBuffer = &tx_data ;
    I2C1_DevStructure.pCPAL_TransferTx->wAddr1   = (uint32_t)(addr << 1);
    I2C1_DevStructure.pCPAL_TransferTx->wAddr2   = (uint32_t)reg;
  
    /* Write Operation */
    if(CPAL_I2C_Write(&I2C1_DevStructure) == CPAL_PASS)
    {
        while ((I2C1_DevStructure.CPAL_State != CPAL_STATE_READY) && (I2C1_DevStructure.CPAL_State != CPAL_STATE_ERROR) )
        { }
    
        if (I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
        {
            return I2C1_FAIL;
        }
    }
    else
    {
        return I2C1_FAIL;
    }
  
    return I2C1_OK;
}

I2C1_Status_TypDef i2c_tx_byte_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t* buf)
{
    return I2C1_OK;
}

I2C1_Status_TypDef i2c_tx_mdatas_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf)
{
    return I2C1_OK;
}

