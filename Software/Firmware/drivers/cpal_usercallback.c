

/* Includes ------------------------------------------------------------------*/
#include "drv_mpu6050.h"
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern I2C_TypeDef* I2C_DEVICE[];
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern i2c_dev_t MPU6050_i2c;

/*------------------------------------------------------------------------------
                     CPAL User Callbacks implementations 
------------------------------------------------------------------------------*/


/*=========== Timeout UserCallback ===========*/


/**
  * @brief  User callback that manages the Timeout error.
  * @param  i2c_dev .
  * @retval None.
  */
uint32_t I2C_TIMEOUT_UserCallback(i2c_dev_t* i2c_dev)
{
  /* Generate STOP */
  i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
  	MPU6050_DeInit();
  MPU6050_StructInit();
  I2CDev_Init(&MPU6050_i2c);

  printf("I2C_TIMEOUT_UserCallback\r\n");
	
  return I2C_PASS;
}


/*=========== Transfer UserCallback ===========*/


/**
  * @brief  Manages the End of Tx transfer event.
  * @param  i2c_dev 
  * @retval None
  */
void I2C_TXTC_UserCallback(i2c_dev_t* i2c_dev)
{
//  if (MPU6050_DevStructure.state== sEE_STATE_WRITING)
//  {
//    sEE_WriteHandler(i2c_dev->Dev);
//  }
//	printf("I2C_TXTC_UserCallback\r\n");
}


/**
  * @brief  Manages the End of Rx transfer event.
  * @param  i2c_dev 
  * @retval None
  */ 
void I2C_RXTC_UserCallback(i2c_dev_t* i2c_dev)
{

//	printf("I2C_RXTC_UserCallback\r\n");
}

/**
  * @brief  Manages Tx transfer event.
  * @param  i2c_dev 
  * @retval None
  */
/*void I2C_TX_UserCallback(i2c_dev_t* i2c_dev)
{
 
}*/


/**
  * @brief  Manages Rx transfer event.
  * @param  i2c_dev 
  * @retval None
  */ 
/*void I2C_RX_UserCallback(i2c_dev_t* i2c_dev)
{
 
}*/


/**
  * @brief  Manages the End of DMA Tx transfer event.
  * @param  i2c_dev 
  * @retval None
  */
/*void CPAL_I2C_DMATXTC_UserCallback(i2c_dev_t* i2c_dev)
{
 
}*/


/**
  * @brief  Manages the Half of DMA Tx transfer event.
  * @param  i2c_dev 
  * @retval None
  */
/*void CPAL_I2C_DMATXHT_UserCallback(i2c_dev_t* i2c_dev)
{
   
}*/


/**
  * @brief  Manages Error of DMA Tx transfer event.
  * @param  i2c_dev 
  * @retval None
  */
/*void CPAL_I2C_DMATXTE_UserCallback(i2c_dev_t* i2c_dev)
{
   
}*/


/**
  * @brief  Manages the End of DMA Rx transfer event.
  * @param  i2c_dev 
  * @retval None
  */
/*void CPAL_I2C_DMARXTC_UserCallback(i2c_dev_t* i2c_dev)
{
 
}*/


/**
  * @brief  Manages the Half of DMA Rx transfer event.
  * @param  i2c_dev 
  * @retval None
  */ 
/*void CPAL_I2C_DMARXHT_UserCallback(i2c_dev_t* i2c_dev)
{
   
}*/


/**
  * @brief  Manages Error of DMA Rx transfer event.
  * @param  i2c_dev 
  * @retval None
  */ 
/*void CPAL_I2C_DMARXTE_UserCallback(i2c_dev_t* i2c_dev)
{
   
}*/



/*=========== Error UserCallback ===========*/


/**
  * @brief  User callback that manages the I2C device errors.
  * @note   Make sure that the define USE_SINGLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  i2c_dev. 
  * @param  DeviceError.
  * @retval None
  */ 
void I2C_ERR_UserCallback(i2c_dev_t* i2c_dev, uint32_t DeviceError)
{  
  /* Generate STOP */
  i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
  
	MPU6050_DeInit();
  MPU6050_StructInit();
  I2CDev_Init(&MPU6050_i2c);
 
	printf("I2C_ERR_UserCallback\r\n");
}


/**
  * @brief  User callback that manages BERR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */  
/*void CPAL_I2C_BERR_UserCallback(I2C_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages ARLO I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_ARLO_UserCallback(I2C_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages OVR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_OVR_UserCallback(I2C_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages AF I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */ 
/*void CPAL_I2C_AF_UserCallback(I2C_DevTypeDef pDevInstance)
{
   
}*/


/*=========== Addressing Mode UserCallback ===========*/


/**
  * @brief  User callback that manage General Call Addressing mode.
  * @param  i2c_dev 
  * @retval None
  */ 
/*void I2C_GENCALL_UserCallback(i2c_dev_t* i2c_dev)
{
   
}*/


/**
  * @brief  User callback that manage Dual Address Addressing mode.
  * @param  i2c_dev 
  * @retval None
  */  
/*void I2C_DUALF_UserCallback(i2c_dev_t* i2c_dev)
{
   
}*/
 