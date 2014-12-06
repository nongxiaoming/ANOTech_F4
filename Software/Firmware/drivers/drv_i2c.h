#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#include <rtthread.h>
#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"  
#include "stm32f4xx_dma.h" 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"


/* I2C1 Hardware Device */
  
#define I2C1_SCL_GPIO_PORT         GPIOB       
#define I2C1_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C1_SCL_GPIO_PIN          GPIO_Pin_6
#define I2C1_SCL_GPIO_PINSOURCE    GPIO_PinSource6 
  
#define I2C1_SDA_GPIO_PORT         GPIOB       
#define I2C1_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C1_SDA_GPIO_PIN          GPIO_Pin_7 
#define I2C1_SDA_GPIO_PINSOURCE    GPIO_PinSource7 
  
#define I2C1_CLK                   RCC_APB1Periph_I2C1
#define I2C1_AF                    GPIO_AF_I2C1  
  
#define I2C1_DMA                   DMA1
#define I2C1_DMA_CLK               RCC_AHB1Periph_DMA1 
#define I2C1_DMA_CHANNEL           DMA_Channel_1  
   
#define I2C1_IT_EVT_IRQn           I2C1_EV_IRQn  
#define I2C1_IT_ERR_IRQn           I2C1_ER_IRQn   
    
#define I2C1_DMA_TX_Stream         DMA1_Stream6
#define I2C1_DMA_TX_IRQn           DMA1_Stream6_IRQn
#define I2C1_DMA_TX_IRQHandler     DMA1_Stream6_IRQHandler
#define I2C1_DMA_TX_TC_FLAG        DMA_FLAG_TCIF6
#define I2C1_DMA_TX_HT_FLAG        DMA_FLAG_HTIF6
#define I2C1_DMA_TX_TE_FLAG        DMA_FLAG_TEIF6

#define I2C1_DMA_RX_Stream         DMA1_Stream0
#define I2C1_DMA_RX_IRQn           DMA1_Stream0_IRQn
#define I2C1_DMA_RX_IRQHandler     DMA1_Stream0_IRQHandler
#define I2C1_DMA_RX_TC_FLAG        DMA_FLAG_TCIF0
#define I2C1_DMA_RX_HT_FLAG        DMA_FLAG_HTIF0
#define I2C1_DMA_RX_TE_FLAG        DMA_FLAG_TEIF0 
  
/* I2C1 Device */
#define I2C_IT_EVT_SUBPRIO             I2C_IT_OFFSET_SUBPRIO + 0   /* I2C EVT SUB-PRIORITY */ 
#define I2C_IT_EVT_PREPRIO             I2C_IT_OFFSET_PREPRIO + 2   /* I2C EVT PREEMPTION PRIORITY */ 
#define I2C_IT_ERR_SUBPRIO             I2C_IT_OFFSET_SUBPRIO + 0   /* I2C ERR SUB-PRIORITY */
#define I2C_IT_ERR_PREPRIO             I2C_IT_OFFSET_PREPRIO + 0   /* I2C ERR PREEMPTION PRIORITY */
#define I2C_IT_DMATX_SUBPRIO           I2C_IT_OFFSET_SUBPRIO + 0   /* I2C DMA TX SUB-PRIORITY */
#define I2C_IT_DMATX_PREPRIO           I2C_IT_OFFSET_PREPRIO + 1   /* I2C DMA TX PREEMPTION PRIORITY */
#define I2C_IT_DMARX_SUBPRIO           I2C_IT_OFFSET_SUBPRIO + 0   /* I2C DMA RX SUB-PRIORITY */
#define I2C_IT_DMARX_PREPRIO           I2C_IT_OFFSET_PREPRIO + 1   /* I2C DMA RX PREEMPTION PRIORITY */


/* This define is used to enable DMA Channel */
#define DMA_CR_EN                       ((uint32_t)0x00000001)
  
/* This define is used to get Interrupt status */
#define DMA_HIGH_ISR_MASK               ((uint32_t)0x20000000)
  
/* This define is used to check if DMA interrupt option are enabled */
#define OPT_DMA_IT_MASK            ((uint32_t)0x00003F00)
  
/* This define is used to check I2C errors ( BERR, ARLO, AF and OVR) */ 
#define I2C_STATUS_ERR_MASK        ((uint16_t)0x0F00) 
  
/* This define is used to check I2C events ( TXE, RXNE, STOPF, ADD10, BTF, ADDR and SB) */ 
#define I2C_STATUS1_EVT_MASK       ((uint16_t)0x00DF)  
  
/* This define is used to check I2C events ( DUALF, GENCALL, TRA, BUSY and MSL) */ 
#define I2C_STATUS2_EVT_MASK       ((uint16_t)0x0097) 
  
/* This define is used to check if DMA TX interrupt are selected */  
#define OPT_I2C_DMA_TX_IT_MASK     ((uint32_t)0x00000700) 
  
/* This define is used to check if DMA RX interrupt are selected */   
#define OPT_I2C_DMA_RX_IT_MASK     ((uint32_t)0x00003800) 
    
   
/* DMA interrupts flag management */

  
#define I2C_GET_DMATX_TCIT(device)    ((I2C1_DMA_TX_TC_FLAG & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C1_DMA_TX_TC_FLAG) :\
                                                 (uint32_t)((device)->DMA->LISR & I2C1_DMA_TX_TC_FLAG)
  
#define I2C_GET_DMATX_HTIT(device)    ((I2C1_DMA_TX_HT_FLAG & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C1_DMA_TX_HT_FLAG):\
                                                 (uint32_t)((device)->DMA->LISR & I2C1_DMA_TX_HT_FLAG))

#define I2C_GET_DMATX_TEIT(device)    ((I2C1_DMA_TX_TE_FLAG & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C1_DMA_TX_TE_FLAG):\
                                                 (uint32_t)((device)->DMA->LISR & I2C1_DMA_TX_TE_FLAG))

#define I2C_GET_DMARX_TCIT(device)    ((I2C1_DMA_RX_TC_FLAG & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C1_DMA_RX_TC_FLAG):\
                                                 (uint32_t)((device)->DMA->LISR & I2C1_DMA_RX_TC_FLAG))  

#define I2C_GET_DMARX_HTIT(device)    ((I2C1_DMA_RX_HT_FLAG & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C1_DMA_RX_HT_FLAG):\
                                                 (uint32_t)((device)->DMA->LISR & I2C1_DMA_RX_HT_FLAG))

#define I2C_GET_DMARX_TEIT(device)    ((I2C1_DMA_RX_TE_FLAG & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C1_DMA_RX_TE_FLAG):\
                                                 (uint32_t)((device)->DMA->LISR & I2C1_DMA_RX_TE_FLAG))
  
#define I2C_CLEAR_DMATX_IT(device)    ((I2C1_DMA_TX_TC_FLAG & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 ((device)->DMA->HIFCR = (I2C1_DMA_TX_TC_FLAG |\
                                                 I2C1_DMA_TX_HT_FLAG | I2C1_DMA_TX_TE_FLAG)) :\
                                                 ((device)->DMA->LIFCR = (I2C1_DMA_TX_TC_FLAG|\
                                                 I2C1_DMA_TX_HT_FLAG | I2C1_DMA_TX_TE_FLAG))
                                                   
#define I2C_CLEAR_DMARX_IT(device)    ((I2C1_DMA_RX_TC_FLAG & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 ((device)->DMA->HIFCR = (I2C1_DMA_RX_TC_FLAG |\
                                                 I2C1_DMA_RX_HT_FLAG | I2C1_DMA_RX_TE_FLAG)) :\
                                                 ((device)->DMA->LIFCR = (I2C1_DMA_RX_TC_FLAG|\
                                                 I2C1_DMA_RX_HT_FLAG | I2C1_DMA_RX_TE_FLAG))
																								 
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



typedef enum
{
  I2C_STATE_DISABLED = 0x00,       
    
  I2C_STATE_READY    = 0x01,      
                                           
  I2C_STATE_READY_TX = 0x03,        
                                          
  I2C_STATE_READY_RX = 0x05,        
                                         
  I2C_STATE_BUSY     = 0x02,       
  
  I2C_STATE_BUSY_TX  = 0x06,        
  
  I2C_STATE_BUSY_RX  = 0x0A,        
  
  I2C_STATE_ERROR    = 0x10,       
	
}I2C_StateTypeDef;


typedef struct 
{  
  I2C_TypeDef*                I2C;
	DMA_TypeDef*                DMA;
	uint32_t                    DMA_Channel;
  DMA_Stream_TypeDef*	        DMA_TX_Stream;
	DMA_Stream_TypeDef*	        DMA_RX_Stream;
I2C_DirectionTypeDef   direction;                                  
  I2C_ProgModelTypeDef        mode;   
  uint8_t*                    buffer;               
  volatile uint32_t           bytes_to_read;   
  volatile uint32_t           bytes_to_write;	                  
  uint32_t                    addr;               
  uint32_t                    reg;                  
  volatile I2C_StateTypeDef   state;        
  volatile uint32_t           error;   
  uint32_t                    options;     
 volatile uint32_t            timeout;      
}i2c_dev_t;

extern i2c_dev_t i2c1_dev;


#define OPT_DMATX_HTIT             ((uint32_t)0x00000200)  /*!<Enable the Transmitter DMA Half Transfer Complete interrupt */
#define OPT_DMARX_HTIT             ((uint32_t)0x00001000)  /*!<Enable the Receiver DMA Half Transfer Complete interrupt */
#define OPT_DMATX_CIRCULAR         ((uint32_t)0x00004000)  /*!<Enable the Circular Mode for DMA Transmitter */
#define OPT_DMARX_CIRCULAR         ((uint32_t)0x00008000)  /*!<Enable the Circular Mode for DMA Receiver */
#define OPT_16BIT_REG              ((uint32_t)0x00020000)  /*!<Enable 16-Bit Register/Physical addressing mode (two bytes, MSB first). */  
#define OPT_I2C_ERRIT_DISABLE      ((uint32_t)0x00400000)  /*!<Disable I2C Errors interrupt (Bus Error, Arbitration Loss*/
#define OPT_I2C_NOSTOP             ((uint32_t)0x00800000)  /*!<Use communication mode with no STOP generation at the end */
#define OPT_I2C_NOSTOP_MODE        ((uint32_t)0x01000000)  /*!<Start communication in No STOP generation mode */
#define OPT_I2C_NACK_ADD           ((uint32_t)0x40000000)  /*!<Initialize the I2C Slave device without enabling the acknowledgement of its */
#define OPT_NO_MEM_ADDR            ((uint32_t)0x00010000)  /*!<Enable No Memory addressing mode: only slave device address sent 
                                                                    No Register/Physical address to be sent after slave address */  

#define DMA_1BYTE_CASE             ((uint32_t)0x00200000)  /*!<This define is used internally in the library */

 uint32_t I2C_TIMEOUT_UserCallback(i2c_dev_t* i2c_dev);   /*<!This function is called when a Timeout 
                                                                             occurs during communication with devices */ 


uint32_t  I2CDev_Init         (i2c_dev_t* i2c_dev); 
uint32_t  I2CDev_DeInit       (i2c_dev_t* i2c_dev); 
uint32_t  I2CDev_StructInit   (i2c_dev_t* i2c_dev);

uint32_t  I2C_Write        (i2c_dev_t* i2c_dev);
uint32_t  I2C_Read         (i2c_dev_t* i2c_dev); 


uint32_t  I2C_IsDeviceReady(i2c_dev_t* i2c_dev); /*<!This function can be used to wait until target device is ready 
                                                                        for communication (ie. for memories after write operations) */

uint32_t I2C_EV_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function manages all I2C device events */ 

uint32_t I2C_ER_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function manages all I2C device errors  */ 

uint32_t I2C_DMA_TX_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function Handles DMA TX Interrupts */

uint32_t I2C_DMA_RX_IRQHandler(i2c_dev_t* i2c_dev); /*<!This function Handles DMA RX Interrupts */

uint32_t I2C_Enable_DMA_IT (i2c_dev_t* i2c_dev); /* This function Configure I2C DMA and Interrupts before starting */

/* Enable the use of DMA Programming Model */
#define I2C_DMA_PROGMODEL

/* Enable 1 Byte reception with DMA Programming Model */
#define I2C_DMA_1BYTE_CASE

/* Method1 used for closing communication with master receiver */
#define I2C_CLOSECOM_METHOD1 

void i2c_timeout(void);

/* Maximum Timeout values for each communication operation (preferably, Time base should be 1 Millisecond).
   The exact maximum value is the sum of event timeout value and the I2C_TIMEOUT_MIN value defined below */
#define I2C_TIMEOUT_SB             30             
#define I2C_TIMEOUT_ADDR           3
#define I2C_Timeout_ADD10          3
#define I2C_TIMEOUT_TXE            2
#define I2C_TIMEOUT_RXNE           2
#define I2C_TIMEOUT_BTF            4
#define I2C_TIMEOUT_BUSY           10

/* DO NOT MODIFY THESE VALUES ---------------------------------------------------------*/
#define I2C_TIMEOUT_DEFAULT        ((uint32_t)0xFFFFFFFF)


#define I2C_IT_OFFSET_SUBPRIO          0      /* I2C SUB-PRIORITY Offset */ 
#define I2C_IT_OFFSET_PREPRIO          0      /* I2C PREEMPTION PRIORITY Offset */ 


#define I2C_TIMEOUT_DETECT                (i2c_dev->timeout < rt_tick_get())

#define I2C_TIMEOUT(cmd, tout)         i2c_dev->timeout = rt_tick_get()+ (tout);\
                                                 while (((cmd) == 0) && (!I2C_TIMEOUT_DETECT));\
                                                 if (I2C_TIMEOUT_DETECT)\
                                                 {\
                                                   return I2C_Timeout (i2c_dev); \
                                                 }\
                                                 i2c_dev->timeout = I2C_TIMEOUT_DEFAULT
   


#endif /*__DRV_I2C_H */

