/**
  ******************************************************************************
  * @file    STM32_EVAL_CPAL/Common/drv_i2c_cpal.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-June-2011
  * @brief   This file contains all the functions prototypes for the 
  *          drv_i2c_cpal.c firmware driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_I2C_CPAL_H
#define __DRV_I2C_CPAL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
   
/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"
   
/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  TSENSOR Status  
  */ 
typedef enum
{
  I2C1_OK = 0,
  I2C1_FAIL
}I2C1_Status_TypDef;

/* Exported constants --------------------------------------------------------*/
    
/* CPAL Structure configuration : Select I2C device (uncomment relative define) */

#define I2C1_DevStructure                I2C1_DevStructure   
//#define I2C2_DevStructure                I2C2_DevStructure  
//#define I2C2_DevStructure                I2C3_DevStructure 

   
/* Select clock Speed */
/* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
   input clock) must be a multiple of 10 MHz */

#define I2C1_SPEED                        300000

/* Select interrupt programming model : By default DMA programming model is selected.
 To select interrupt programming model uncomment this define */
#define I2C1_IT

/* Maximum Timeout values for waiting until device is ready for communication.*/
   
#define I2C1_TIMEOUT        ((uint32_t)0x3FFFF)

/**
  * @brief  Internal register Memory
  */
#define I2C2_REG_TEMP       0x00  /*!< Temperature Register of LM75 */
#define I2C2_REG_CONF       0x01  /*!< Configuration Register of LM75 */
#define I2C2_REG_THYS       0x02  /*!< Temperature Register of LM75 */
#define I2C2_REG_TOS        0x03  /*!< Over-temp Shutdown threshold Register of LM75 */
#define I2C2_ADDR           0x90   /*!< LM75 address */
   

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

void I2C2_DeInit(void);
void I2C2_Config(void);
ErrorStatus I2C2_GetStatus(void);
uint16_t I2C2_ReadTemp(void);
uint16_t I2C2_ReadReg(uint8_t RegName);
uint8_t I2C2_WriteReg(uint8_t RegName, uint16_t RegValue);
uint8_t I2C2_ReadConfReg(void);
uint8_t I2C2_WriteConfReg(uint8_t RegValue);
uint8_t I2C2_ShutDown(FunctionalState NewState);

uint8_t i2c_rx_byte(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg);
I2C1_Status_TypDef i2c_rx_byte_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t* buf);
I2C1_Status_TypDef i2c_rx_mbytes_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);

I2C1_Status_TypDef i2c_tx_cmd(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t cmd);
I2C1_Status_TypDef i2c_tx_byte(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t data);
I2C1_Status_TypDef i2c_tx_byte_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t* buf);
I2C1_Status_TypDef i2c_tx_mbytes_buf(CPAL_DevTypeDef i2cx, uint8_t addr, uint8_t reg, uint8_t len, uint16_t* buf);
 
#ifdef __cplusplus
}
#endif

#endif /* __STM32_EVAL_I2C_TSENSOR_CPAL_H */
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

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
