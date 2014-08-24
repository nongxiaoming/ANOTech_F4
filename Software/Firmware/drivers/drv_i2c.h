/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#include "cpal_i2c_hal_stm32f4xx.h"


#ifdef __cplusplus
 extern "C" {
#endif

   
typedef enum
{
  I2C_ERR_NONE      = 0x0000, /*!<No Error: This is the default state for an Idle peripheral */

  I2C_ERR_TIMEOUT   = 0x00FF, /*!<Timeout error: The specified timeout */

  I2C_ERR_BERR      = 0x0100, /*!<Bus error: This error occurs when I2C peripheral detects an external
                                       Stop or Start condition during address or data transfer.  */                                        
  I2C_ERR_ARLO        = 0x0200, /*!<Arbitration Lost error: This error occurs when the I2C interface detects 
                                         an arbitration lost condition. */                                                  
  I2C_ERR_AF          = 0x0400, /*!<Acknowledge Failure : This error occurs when the interface detects 
                                         a non-acknowledge bit. */                                                                                            
  I2C_ERR_OVR          = 0x0800, /*!<Overrun/Underrun error: An overrun error can occur in slave mode when clock 
                                          stretching is disabled and the I2C interface is receiving data. */
                                                  
 }I2CErrorTypeDef;



uint32_t  I2CDev_Init         (i2c_dev_t* i2c_dev); /*<!This function Initializes the selected I2C device 
                                                                        and all needed resources (GPIOs, clocks, DMA, 
                                                                        interrupts …) */
                                                       
uint32_t  I2CDev_DeInit       (i2c_dev_t* i2c_dev); /*<!This function free the resources used by the I2C 
                                                                        device (GPIOs, clocks, DMA, interrupts …) and 
                                                                        deinitialize the device itself */
                                                       
uint32_t  I2CDev_StructInit   (i2c_dev_t* i2c_dev); /*<!This function Initializes I2C device structure 
                                                                        by filling all fields with their default values.
                                                                        Warning: Pointer fields are filled with CPAL local variables
                                                                        pointer. To avoid all risks, it is recommended to declare
                                                                        application local variables and fill these fields with their
                                                                        pointers. */



uint32_t  I2C_Write        (i2c_dev_t* i2c_dev);
uint32_t  I2C_Read         (i2c_dev_t* i2c_dev); 


uint32_t  I2C_IsDeviceReady(i2c_dev_t* i2c_dev); /*<!This function can be used to wait until target device is ready 
                                                                        for communication (ie. for memories after write operations) */

uint32_t I2C_EV_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function manages all I2C device events */ 

uint32_t I2C_ER_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function manages all I2C device errors  */ 

#ifdef I2C_DMA_PROGMODEL
uint32_t I2C_DMA_TX_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function Handles DMA TX Interrupts */

uint32_t I2C_DMA_RX_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function Handles DMA RX Interrupts */
#endif /* I2C_DMA_PROGMODEL */



uint32_t I2C_Enable_DMA_IT (i2c_dev_t* i2c_dev, I2C_DirectionTypeDef Direction); /* This function Configure I2C DMA 
                                                                                                    and Interrupts before starting 
                                                                                                    transfer phase */


#ifndef I2C_TX_UserCallback
void I2C_TX_UserCallback   (i2c_dev_t* i2c_dev); /*<!This function is called when (in Interrupt mode) 
                                                                        the peripheral is preparing to send data */ 
#endif

#ifndef I2C_RX_UserCallback
void I2C_RX_UserCallback   (i2c_dev_t* i2c_dev); /*<!This function is called when (in Interrupt mode) 
                                                                        the peripheral has received data */ 
#endif

#ifndef I2C_TXTC_UserCallback
void I2C_TXTC_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when (in DMA or Interrupt mode)
                                                                      TX Transfer is complete (to use in DMA mode, Transfer complete 
                                                                      interrupt must be enabled) */ 
#endif

#ifndef I2C_RXTC_UserCallback
void I2C_RXTC_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when (in DMA or Interrupt mode)
                                                                       RX Transfer is complete (to use in DMA mode, Transfer complete 
                                                                       interrupt must be enabled) */ 
#endif

#ifndef I2C_DMATXTC_UserCallback
void I2C_DMATXTC_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called (in DMA mode) when 
                                                                          DMA Transmission is finished (If Transfer Complete 
                                                                          interrupt is enabled) */
#endif

#ifndef I2C_DMATXHT_UserCallback
void I2C_DMATXHT_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called (in DMA mode) when the 
                                                                          DMA Transmission has reached the half of the 
                                                                          buffer (If Half Transfer interrupt is enabled) */
#endif

#ifndef I2C_DMATXTE_UserCallback
void I2C_DMATXTE_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when (in DMA mode) a 
                                                                          DMA Transmission transfer error has occurred 
                                                                          (If Transfer Error interrupt is enabled ) */
#endif

#ifndef I2C_DMARXTC_UserCallback
void I2C_DMARXTC_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when (in DMA mode) when 
                                                                          DMA Reception is finished (If Transfer Complete 
                                                                          interrupt is enabled) */
#endif

#ifndef I2C_DMARXHT_UserCallback
void I2C_DMARXHT_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when (in DMA mode) the
                                                                          DMA Reception has reached the half of the 
                                                                          buffer (If Half Transfer interrupt is enabled) */
#endif

#ifndef I2C_DMARXTE_UserCallback
void I2C_DMARXTE_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when (in DMA mode) a 
                                                                          DMA Reception transfer error has occurred 
                                                                          (If Transfer Error interrupt is enabled ) */
#endif

#ifndef I2C_GENCALL_UserCallback
void I2C_GENCALL_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when General Call flag
                                                                          is set (used in General Call Mode only ) */
#endif

#ifndef I2C_DUALF_UserCallback
void I2C_DUALF_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when Dual Address flag
                                                                        is set (used in Dual Address Mode only ) */
#endif

#ifndef I2C_SLAVE_READ_UserCallback
void I2C_SLAVE_READ_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when a read operation is
                                                                             requested in Listen mode only */
#endif

#ifndef I2C_SLAVE_WRITE_UserCallback
void I2C_SLAVE_WRITE_UserCallback(i2c_dev_t* i2c_dev); /*<!This function is called when a write operation is
                                                                              requested in Listen mode only */
#endif

/*========= User_ErrorCallback_Prototypes =========*/
/* User can use two types of Callback:
    - Single Error Callback : All error are handled by one Callback (I2C_ERR_UserCallback()).
    - Multiple Error Callback : Each error has its own Callback ( I2C_ERRTYPE_UserCallback () , 
        ERRTYPE : can be one of I2C Errors (BERR, ARLO, OVR and AF)).
   To select one of this type, user should comment or uncomment adequate defines in cpal_conf.h file. */  

#ifdef USE_SINGLE_ERROR_CALLBACK

  #ifndef I2C_ERR_UserCallback
  void  I2C_ERR_UserCallback(i2c_dev_t* i2c_dev, uint32_t DeviceError); /*<!This callback is called when an error 
                                                                                           occurred on the peripheral while transferring
                                                                                           (If I2C Error interrupt is enabled). Device 
                                                                                           instance and error type (DeviceError) are 
                                                                                           passed as argument. Device_Error value can be
                                                                                           one of I2CErrorTypeDef enumeration */
  #endif
#endif /* USE_SINGLE_ERROR_CALLBACK */
  
#ifdef USE_MULTIPLE_ERROR_CALLBACK

  #ifndef I2C_BERR_UserCallback
   void  I2C_BERR_UserCallback(I2C_DevTypeDef pDevInstance); /*<!This callback is called when an Bus ERROR
                                                                       occurred on the peripheral while transferring
                                                                       (If I2C Error interrupt is enabled) */
  #endif  
 
  #ifndef I2C_ARLO_UserCallback 
   void  I2C_ARLO_UserCallback(I2C_DevTypeDef pDevInstance); /*<!This callback is called when an Arbitration Lost 
                                                                       ERROR occurred on the peripheral while transferring 
                                                                       (If I2C Error interrupt is enabled) */
  #endif    
 
  #ifndef I2C_OVR_UserCallback 
   void  I2C_OVR_UserCallback(I2C_DevTypeDef pDevInstance); /*<!This callback is called when an Overrun/Underrun 
                                                                      ERROR occurred on the peripheral while transferring 
                                                                      (If I2C Error interrupt is enabled) */
  #endif   
 
  #ifndef I2C_AF_UserCallback
   void  I2C_AF_UserCallback(I2C_DevTypeDef pDevInstance); /*<!This callback is called when an Acknowledge 
                                                                     Failure occurred on the peripheral while transferring 
                                                                     (If I2C Error interrupt is enabled) */
  #endif 
 
#endif /* USE_SINGLE_ERROR_CALLBACK */
   

#ifdef __cplusplus
}
#endif

#endif /*__DRV_I2C_H */

