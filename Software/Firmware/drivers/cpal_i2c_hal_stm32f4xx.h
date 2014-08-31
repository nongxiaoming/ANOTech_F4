
#ifndef __I2C_HAL_STM32F4XX_H
#define __I2C_HAL_STM32F4XX_H

#ifdef __cplusplus
extern "C" {
#endif
  

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"  
#include "stm32f4xx_dma.h" 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
  
/*========= CPAL library files includes =========*/
#include "cpal.h" 


/* IO Pins selection possibilities 
  
|--------|---------|--------------|-----------|------------------|-------------------------|
| Device | I2C PIN |   GPIO_PIN   | GPIO_PORT |  GPIO_PinSource  |        GPIO_CLK         |                                          
|--------|---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_6  |   GPIOB   | GPIO_PinSource6  |   RCC_AHB1Periph_GPIOB  |                                          
|        |   SCL   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOB   | GPIO_PinSource8  |   RCC_AHB1Periph_GPIOB  |                                          
|  I2C1  |---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_7  |   GPIOB   | GPIO_PinSource7  |   RCC_AHB1Periph_GPIOB  |                                          
|        |   SDA   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_9  |   GPIOB   | GPIO_PinSource9  |   RCC_AHB1Periph_GPIOB  |                                          
|--------|---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_10 |   GPIOB   | GPIO_PinSource10 |   RCC_AHB1Periph_GPIOB  |                                          
|        |         |--------------|-----------|------------------|-------------------------|
|        |   SCL   |  GPIO_Pin_1  |   GPIOF   | GPIO_PinSource1  |   RCC_AHB1Periph_GPIOF  |                                           
|        |         |--------------|-----------|------------------|-------------------------|    
|        |         |  GPIO_Pin_4  |   GPIOH   | GPIO_PinSource4  |   RCC_AHB1Periph_GPIOH  |                                           
|  I2C2  |---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_11 |   GPIOB   | GPIO_PinSource11 |   RCC_AHB1Periph_GPIOB  |                                           
|        |         |--------------|-----------|------------------|-------------------------|
|        |   SDA   |  GPIO_Pin_0  |   GPIOF   | GPIO_PinSource0  |   RCC_AHB1Periph_GPIOF  |                                           
|        |         |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_5  |   GPIOH   | GPIO_PinSource5  |   RCC_AHB1Periph_GPIOH  |                                           
|--------|---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_7  |   GPIOH   | GPIO_PinSource7  |   RCC_AHB1Periph_GPIOH  |                                           
|        |   SCL   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOA   | GPIO_PinSource8  |   RCC_AHB1Periph_GPIOA  |                                           
|  I2C3  |---------|--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOH   | GPIO_PinSource8  |   RCC_AHB1Periph_GPIOH  |                                           
|        |   SDA   |--------------|-----------|------------------|-------------------------|
|        |         |  GPIO_Pin_9  |   GPIOC   | GPIO_PinSource9  |   RCC_AHB1Periph_GPIOC  |                                           
|--------|---------|--------------|-----------|------------------|-------------------------|  */ 
  
  
/*----------- I2C1 Device -----------*/
  
#define I2C1_SCL_GPIO_PORT         GPIOB       
#define I2C1_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C1_SCL_GPIO_PIN          GPIO_Pin_6
#define I2C1_SCL_GPIO_PINSOURCE    GPIO_PinSource6 
  
#define I2C1_SDA_GPIO_PORT         GPIOB       
#define I2C1_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C1_SDA_GPIO_PIN          GPIO_Pin_7 
#define I2C1_SDA_GPIO_PINSOURCE    GPIO_PinSource7 
  
/*-----------I2C2 Device -----------*/
  
#define I2C2_SCL_GPIO_PORT         GPIOB       
#define I2C2_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C2_SCL_GPIO_PIN          GPIO_Pin_10
#define I2C2_SCL_GPIO_PINSOURCE    GPIO_PinSource10 

#define I2C2_SDA_GPIO_PORT         GPIOB       
#define I2C2_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C2_SDA_GPIO_PIN          GPIO_Pin_11 
#define I2C2_SDA_GPIO_PINSOURCE    GPIO_PinSource11  

/*-----------I2C3 Device -----------*/
  
#define I2C3_SCL_GPIO_PORT         GPIOH
#define I2C3_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOH
#define I2C3_SCL_GPIO_PIN          GPIO_Pin_7
#define I2C3_SCL_GPIO_PINSOURCE    GPIO_PinSource7
  
#define I2C3_SDA_GPIO_PORT         GPIOH
#define I2C3_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOH
#define I2C3_SDA_GPIO_PIN          GPIO_Pin_8
#define I2C3_SDA_GPIO_PINSOURCE    GPIO_PinSource8
  
/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*    -- Section 2 :           **** Device TX and RX DMA Channels Selection ****
  
  Description: This section allows user to choose TX and RX DMA Channels if possible (in accordance with 
               used product) for each device.
               Each device instance (I2C1, I2C2 ..) has its specific defines: one for DMA TX Channel and 
               another one for DMA RX Channel.
               For each device instance, you find all TX an RX DMA Channel possibilities ( Refer to Product
               Reference Manual).*/
 
/* DMA Stream selection possibilities 
  
|--------|-----------|-------------------------|--------------|
| Device | Direction |    Define Configuration |  DMA Stream  |
|--------|-----------|-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_TX_CONFIG1| DMA1_Stream6 |
|        |     TX    |-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_TX_CONFIG2| DMA1_Stream7 |
|  I2C1  |-----------|-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_RX_CONFIG1| DMA1_Stream0 |
|        |     RX    |-------------------------|--------------|
|        |           | CPAL_I2C1_DMA_RX_CONFIG2| DMA1_Stream5 |
|--------|-----------|-------------------------|--------------|
|        |     TX    | CPAL_I2C2_DMA_TX_CONFIG1| DMA1_Stream7 |
|        |-----------|-------------------------|--------------|
|  I2C2  |           | CPAL_I2C2_DMA_RX_CONFIG1| DMA1_Stream3 |
|        |     RX    |-------------------------|--------------|
|        |           | CPAL_I2C2_DMA_RX_CONFIG2| DMA1_Stream2 |
|--------|-----------|-------------------------|--------------|
|        |     TX    | CPAL_I2C3_DMA_TX_CONFIG1| DMA1_Stream4 |
|  I2C3  |-----------|-------------------------|--------------|
|        |     RX    | CPAL_I2C3_DMA_TX_CONFIG1| DMA1_Stream2 |
|--------|-----------|-------------------------|--------------| */
  

  
/*----------- I2C1 Device -----------*/
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
#define I2C_STATUS_ERR_MASK        ((uint16_t)0x0F00)  /*!< I2C errors Mask  */
  
/* This define is used to check I2C events ( TXE, RXNE, STOPF, ADD10, BTF, ADDR and SB) */ 
#define I2C_STATUS1_EVT_MASK       ((uint16_t)0x00DF)  /*!< I2C event Mask for Status Register 1  */
  
/* This define is used to check I2C events ( DUALF, GENCALL, TRA, BUSY and MSL) */ 
#define I2C_STATUS2_EVT_MASK       ((uint16_t)0x0097)  /*!< I2C event Mask for Status Register 2  */
  
/* This define is used to check if DMA TX interrupt are selected */  
#define OPT_I2C_DMA_TX_IT_MASK     ((uint32_t)0x00000700) 
  
/* This define is used to check if DMA RX interrupt are selected */   
#define OPT_I2C_DMA_RX_IT_MASK     ((uint32_t)0x00003800) 
  
 /* I2C Event Defines */  
#define I2C_EVT_SB                 I2C_SR1_SB       /*!<Start Bit (Master mode) */
#define I2C_EVT_ADDR               I2C_SR1_ADDR     /*!<Address sent (master mode)/matched (slave mode) */
#define I2C_EVT_ADD10              I2C_SR1_ADD10    /*!<10-bit header sent (Master mode) */
#define I2C_EVT_STOPF              I2C_SR1_STOPF    /*!<Stop detection (Slave mode) */
#define I2C_EVT_RXNE               I2C_SR1_RXNE     /*!<Data Register not Empty (receivers) */
#define I2C_EVT_TXE                I2C_SR1_TXE      /*!<Data Register Empty (transmitters) */
  
  
/*========= I2C1 specific defines (Clocks and DMA) =========*/   
  
#define I2C1_CLK                   RCC_APB1Periph_I2C1
#define I2C1_DR                    ((uint32_t)0x40005410)
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
   

  
#define I2C_CLK_CMD(clk,cmd)                   RCC_APB1PeriphClockCmd((clk),(cmd))

#define I2C_RCC_RESET(clk)                     RCC_APB1PeriphResetCmd((clk),ENABLE);\
                                                 RCC_APB1PeriphResetCmd((clk),DISABLE)  

  
#define I2C_GPIO_CLK_CMD(clk,cmd)              RCC_AHB1PeriphClockCmd((clk),(cmd))
    
#define DMA_CLK_CMD(clk,cmd)                   RCC_AHB1PeriphClockCmd((clk),(cmd))

#define DMA_RESET_CMD(clk,cmd)                 RCC_AHB1PeriphResetCmd((clk),(cmd)) 
  
  

/* DMA interrupts flag management */

  
#define I2C_HAL_GET_DMATX_TCIT(device)    ((I2C_DMA_TX_TC_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C_DMA_TX_TC_FLAG [(device)->dev]) :\
                                                 (uint32_t)((device)->DMA->LISR & I2C_DMA_TX_TC_FLAG [(device)->dev])
  
#define I2C_HAL_GET_DMATX_HTIT(device)    ((I2C_DMA_TX_HT_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C_DMA_TX_HT_FLAG [(device)->dev]):\
                                                 (uint32_t)((device)->DMA->LISR & I2C_DMA_TX_HT_FLAG [(device)->dev]))

#define I2C_HAL_GET_DMATX_TEIT(device)    ((I2C_DMA_TX_TE_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C_DMA_TX_TE_FLAG [(device)->dev]):\
                                                 (uint32_t)((device)->DMA->LISR & I2C_DMA_TX_TE_FLAG [(device)->dev]))

#define I2C_HAL_GET_DMARX_TCIT(device)    ((I2C_DMA_RX_TC_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C_DMA_RX_TC_FLAG [(device)->dev]):\
                                                 (uint32_t)((device)->DMA->LISR & I2C_DMA_RX_TC_FLAG [(device)->dev]))  

#define I2C_HAL_GET_DMARX_HTIT(device)    ((I2C_DMA_RX_HT_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C_DMA_RX_HT_FLAG [(device)->dev]):\
                                                 (uint32_t)((device)->DMA->LISR & I2C_DMA_RX_HT_FLAG [(device)->dev]))

#define I2C_HAL_GET_DMARX_TEIT(device)    ((I2C_DMA_RX_TE_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0 ? \
                                                 (uint32_t)((device)->DMA->HISR & I2C_DMA_RX_TE_FLAG [(device)->dev]):\
                                                 (uint32_t)((device)->DMA->LISR & I2C_DMA_RX_TE_FLAG [(device)->dev]))
  
#define I2C_HAL_CLEAR_DMATX_IT(device)    ((I2C_DMA_TX_TC_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 ((device)->DMA->HIFCR = (I2C_DMA_TX_TC_FLAG[(device)->dev] |\
                                                 I2C_DMA_TX_HT_FLAG[(device)->dev] | I2C_DMA_TX_TE_FLAG[(device)->dev])) :\
                                                 ((device)->DMA->LIFCR = (I2C_DMA_TX_TC_FLAG[(device)->dev]|\
                                                 I2C_DMA_TX_HT_FLAG[(device)->dev] | I2C_DMA_TX_TE_FLAG[(device)->dev]))
                                                   
#define I2C_HAL_CLEAR_DMARX_IT(device)    ((I2C_DMA_RX_TC_FLAG[(device)->dev] & DMA_HIGH_ISR_MASK) != 0) ? \
                                                 ((device)->DMA->HIFCR = (I2C_DMA_RX_TC_FLAG[(device)->dev] |\
                                                 I2C_DMA_RX_HT_FLAG[(device)->dev] | I2C_DMA_RX_TE_FLAG[(device)->dev])) :\
                                                 ((device)->DMA->LIFCR = (I2C_DMA_RX_TC_FLAG[(device)->dev]|\
                                                 I2C_DMA_RX_HT_FLAG[(device)->dev] | I2C_DMA_RX_TE_FLAG[(device)->dev]))
                                                   
 
uint32_t I2C1_EV_IRQHandler(void); 
uint32_t I2C1_ER_IRQHandler(void);

   
#ifdef I2C_DMA_PROGMODEL   
uint32_t I2C1_DMA_TX_IRQHandler(void);
uint32_t I2C1_DMA_RX_IRQHandler(void); 
#endif /* I2C_DMA_PROGMODEL */   

/*========= Hardware Abstraction Layer local =========*/      

  void I2C_HAL_CLKInit(I2C_DevTypeDef Device); /*<!This function resets then enable the I2C device clock */
  
  void I2C_HAL_CLKDeInit(I2C_DevTypeDef Device); /*<!This function resets then disable the I2C device clock */
  
  void I2C_HAL_GPIOInit(I2C_DevTypeDef Device);  /*<!This function configures the IO pins used by the I2C device */
  
  void I2C_HAL_GPIODeInit(I2C_DevTypeDef Device); /*<!This function deinitialize the IO pins used by the I2C device 
                                                            (configured to their default state) */ 
  
#ifdef I2C_DMA_PROGMODEL
  void I2C_HAL_DMAInit(i2c_dev_t* i2c_dev); /*<!This function enable the DMA clock and initialize 
                                                                                                            needed DMA Streams used by the I2C device   */
  
  void I2C_HAL_DMATXConfig(i2c_dev_t* i2c_dev); /*<!This function configures the DMA Stream specific */
                                     
  
  void I2C_HAL_DMARXConfig(i2c_dev_t* i2c_dev); /*<!This function configures the DMA Stream specific  */
                                                                                                                    
  
  void I2C_HAL_DMADeInit(i2c_dev_t* i2c_dev); /*<!This function deinitialize the DMA Stream used 
                                                                                            by I2C Device (Configure them to their default
                                                                                            values). DMA clock is not disabled */
#endif /* I2C_DMA_PROGMODEL */
  
  void I2C_HAL_ITInit(i2c_dev_t* i2c_dev); /*<!This function configures NVIC and interrupts used 
                                                                                                                                                 by I2C Device according to enabled options */  
  
  void I2C_HAL_ITDeInit(i2c_dev_t* i2c_dev); /*<!This function deinitialize NVIC and interrupts used 
                                                                                                                                             by I2C Device  */     

  
#ifdef __cplusplus
}
#endif

#endif /*__I2C_HAL_STM32F4XX_H */




