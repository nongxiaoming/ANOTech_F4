

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CPAL_H
#define __CPAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cpal_conf.h"
#include "stm32f4xx.h"
  

  
typedef enum   
{
  I2C1_Dev       =   0x00,	    /*!< Use I2C1 device */
 
  I2C2_Dev       =   0x01,     /*!< Use I2C2 device */
 
  I2C3_Dev       =   0x02      /*!< Use I2C3 device */

}I2C_DevTypeDef;



typedef enum
{ 
  I2C_DIRECTION_TX        = 0x01,         /*!<Transmitter only direction */

  I2C_DIRECTION_RX        = 0x02,         /*!<Receiver only direction */

  I2C_DIRECTION_TXRX      = 0x03,         /*!<Transmitter and Receiver direction */

}I2C_DirectionTypeDef;


typedef enum
{
  I2C_PROGMODEL_INTERRUPT = 0x01,         /*!<Interrupt transfer programming model */

  I2C_PROGMODEL_DMA       = 0x02          /*!<DMA transfer programming model */

}I2C_ProgModelTypeDef;



/*========= CPAL_State_TypeDef =========*/
/* CPAL global State enumeration contains the current state of CPAL communication. 
   Before starting each operation this state is tested. After each operation 
   CPAL_State is updated with the new value resulting from the relative operation.*/

typedef enum
{
  I2C_STATE_DISABLED = 0x00,        /*!<The Disabled state indicates that the device 
                                         is not configured. */
    
  I2C_STATE_READY    = 0x01,        /*!<The Ready state indicates that the device is configured
                                         correctly and is ready for read or write operation and/or 
                                         the last transaction has been successfully completed */  
                                           
  I2C_STATE_READY_TX = 0x03,        /*!<The Ready_TX state indicates that the device is ready for 
                                         transmission operation */
                                          
  I2C_STATE_READY_RX = 0x05,        /*!<The Ready_RX state indicates that the device is ready for 
                                         reception operation */
                                         
  I2C_STATE_BUSY     = 0x02,        /*!<The Busy state indicates that a Write or Read 
                                         operation started */
  
  I2C_STATE_BUSY_TX  = 0x06,        /*!<The Busy_TX state indicates that a transmission 
                                         operation is on going */
  
  I2C_STATE_BUSY_RX  = 0x0A,        /*!<The Busy_RX state indicates that a reception 
                                         operation is on going */
  
  I2C_STATE_ERROR    = 0x10,        /*!<The Error state indicates that the last operation failed. 
                                         To determine which error caused the failure, read the 
                                         device status structure. */
}I2C_StateTypeDef;


/*========= CPAL_Dev_TypeDef =========*/
/* CPAL Device structure definition */

typedef struct 
{  
  I2C_DevTypeDef         dev;          /*!<Instance of the device. This parameter can be one 
                                                 of the following values: CPAL_Dev_TypeDef */
   I2C_TypeDef*         I2C;
#ifdef I2C_DMA_PROGMODEL
	 DMA_TypeDef *        DMA;
	 uint32_t DMA_Channel;
   DMA_Stream_TypeDef	 *DMA_TX_Stream;
	 DMA_Stream_TypeDef	 *DMA_RX_Stream;
#endif
	
  I2C_DirectionTypeDef   direction;    /*!<Specifies the direction for the device transfers. 
                                                 It can be one of the following values: CPAL_Direction_TypeDef */                                         

  I2C_ProgModelTypeDef   mode;    /*!<Specifies the programming model for the device transfers. 
                                                 It can be one of the following values:  CPAL_ProgModel_Enum */

  uint8_t*        buffer;               /*!<The address of the buffer from/to which the transfer should start */ 

  volatile uint32_t   bytes_to_read;   /*!<Number of data to be transferred for the current transaction */ 
  volatile uint32_t   bytes_to_write;	
                   
  uint32_t        addr;                /*!<Contains the Target device Address (optional)*/

  uint32_t        reg;                   /*!<Contains the Register/Physical Address into the device (optional) */
 
  volatile I2C_StateTypeDef  state;        /*!<Holds the current State of the CPAL driver relative to the device 
                                                 instantiated by CPAL_Dev field. The state parameter can be one of 
                                                 the following values: State_Enum */

  volatile uint32_t           error;    /*!<Specifies the error code for the current operation.The error codes 
                                                 are specified for each device type as follows: 
                                                 CPAL_I2CError_Enum for I2C devices */

  uint32_t                options;     /*!<Bit-field value specifying additional options for the configuration 
                                                 of the device: The bit-field value can be any combination of following 
                                                 values: CPAL_Options_Enum. When a value is not selected the relative 
                                                 feature is disabled */    
  
 volatile uint32_t           timeout;     /*!<This field is with timeout procedure. its used to detect timeout */    

  I2C_InitTypeDef*        I2C_InitStruct;  /*!<Pointer to a device Initialization structure as described 
                                                 in the standard device library driver.  */  
}i2c_dev_t;


/*========= Table containing all I2C device structures =========*/

extern i2c_dev_t i2c1_dev;




/* Exported constants --------------------------------------------------------*/
   
/*========= Options_TypeDef =========*/
/* Options defines contains configuration options which can be affected 
   to Options which is a bit-field argument so any combination of these 
   parameters can be selected. If one option is not selected then the relative 
   feature is disabled.
   There are common options and device specific options.*/

#define OPT_I2C_DUALADDR           ((uint32_t)0x00000001)  /*!<Use Dual Address Mode (available in Slave Mode only). 
                                                                    To use this option enable it by affecting this define 
                                                                    and own address2 to wCPAL_Options */

#define OPT_DMATX_HTIT             ((uint32_t)0x00000200)  /*!<Enable the Transmitter DMA Half Transfer Complete interrupt */

#define OPT_DMARX_HTIT             ((uint32_t)0x00001000)  /*!<Enable the Receiver DMA Half Transfer Complete interrupt */

#define OPT_DMATX_CIRCULAR         ((uint32_t)0x00004000)  /*!<Enable the Circular Mode for DMA Transmitter */
  
#define OPT_DMARX_CIRCULAR         ((uint32_t)0x00008000)  /*!<Enable the Circular Mode for DMA Receiver */
  
#define OPT_NO_MEM_ADDR            ((uint32_t)0x00010000)  /*!<Enable No Memory addressing mode: only slave device address sent 
                                                                    No Register/Physical address to be sent after slave address */  
  
#define OPT_16BIT_REG              ((uint32_t)0x00020000)  /*!<Enable 16-Bit Register/Physical addressing mode (two bytes, 
                                                                   MSB first). This option is supported only when OPT_NO_MEM_ADDR 
                                                                   option is not set */  

#define OPT_I2C_GENCALL            ((uint32_t)0x00100000)  /*!<Use General Call Address Mode (available in Slave Mode only) 
                                                                    (General Call Address = 0x00) */ 
  
#define DMA_1BYTE_CASE             ((uint32_t)0x00200000)  /*!<This define is used internally in the library
                                                                    (not by user) and handle 1Byte transfer by IT 
                                                                    when DMA Programming Model is selected for reception */
  
#define OPT_I2C_ERRIT_DISABLE      ((uint32_t)0x00400000)  /*!<Disable I2C Errors interrupt (Bus Error, Arbitration Loss, 
                                                                    Acknowledge Failure and Overrun/Underrun Errors).
                                                                    By default, errors interrupt is enabled to manage errors efficiently */
  
#define OPT_I2C_NOSTOP             ((uint32_t)0x00800000)  /*!<Use communication mode with no STOP generation at the end 
                                                                    of data transfer (for multi-read/write operations) */

#define OPT_I2C_NOSTOP_MODE        ((uint32_t)0x01000000)  /*!<Start communication in No STOP generation mode */

#define OPT_I2C_NACK_ADD           ((uint32_t)0x40000000)  /*!<Initialize the I2C Slave device without enabling the acknowledgement of its 
                                                                    own address. This option must not be used with No Stop generation mode */





/*========= CPAL_TIMEOUT_Callback =========*/

#ifndef I2C_TIMEOUT_UserCallback 
 uint32_t I2C_TIMEOUT_UserCallback(i2c_dev_t* i2c_dev);   /*<!This function is called when a Timeout 
                                                                             occurs during communication with devices */ 
#endif



#ifdef __cplusplus
}
#endif

#endif /*__CPAL_H */


