#include "drv_i2c.h"


#define I2C_TIMEOUT_DETECT                (i2c_dev->timeout < rt_tick_get())

#define I2C_TIMEOUT(cmd, tout)         i2c_dev->timeout = rt_tick_get()+ (tout);\
                                                 while (((cmd) == 0) && (!I2C_TIMEOUT_DETECT));\
                                                 if (I2C_TIMEOUT_DETECT)\
                                                 {\
                                                   return I2C_Timeout (i2c_dev); \
                                                 }\
                                                 i2c_dev->timeout = I2C_TIMEOUT_DEFAULT
                                                   
extern const uint32_t I2C_DMA_TX_TC_FLAG[];
extern const uint32_t I2C_DMA_RX_TC_FLAG[];

extern const uint32_t I2C_DMA_TX_HT_FLAG[];
extern const uint32_t I2C_DMA_RX_HT_FLAG[];

extern const uint32_t I2C_DMA_TX_TE_FLAG[];
extern const uint32_t I2C_DMA_RX_TE_FLAG[];


I2C_InitTypeDef I2C_InitStructure;



  static uint32_t I2C_MASTER_START_Handle(i2c_dev_t* i2c_dev); /* Handle Master SB Interrupt event */  
  static uint32_t I2C_MASTER_ADDR_Handle(i2c_dev_t* i2c_dev);  /* Handle Master ADDR Interrupt event */
  static uint32_t I2C_MASTER_RXNE_Handle(i2c_dev_t* i2c_dev);  /* Handle Master RXNE Interrupt event */
 

static uint32_t I2C_Timeout (i2c_dev_t* i2c_dev);

/**
  * @brief  Initialize the peripheral and all related clocks, GPIOs, DMA and 
  *         Interrupts according to the specified parameters in the 
  *         I2CDev_InitTypeDef structure.
  * @param  i2c_dev : Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR 
  */
uint32_t I2CDev_Init(i2c_dev_t* i2c_dev) 
{
  I2C_LOG("\n\r\n\rLOG <I2CDev_Init> : I2C Device Init");
  
  /* If I2C_State is not BUSY */
  if ((i2c_dev->state == I2C_STATE_READY) 
     || (i2c_dev->state == I2C_STATE_ERROR) 
     || (i2c_dev->state == I2C_STATE_DISABLED))
  {
    /* 
    - If I2C_State is I2C_STATE_ERROR (an Error occurred in transaction):
      Perform the initialization routines (device will be deinitialized during initialization).
    - If I2C_State is I2C_STATE_READY:  
      Perform the initialization routines 
    - If I2C_State is I2C_STATE_DISABLED:  
      Perform the Initialization routines                                   */    
    
    /* Disable I2Cx Device */
    i2c_dev->I2C->CR1 &= ~I2C_CR1_PE;
		
    I2C_LOG("\n\rLOG : I2C Device Disabled"); 
    
    /* Deinitialize I2Cx GPIO */
    I2C_HAL_GPIODeInit(i2c_dev->dev);
    
    I2C_LOG("\n\rLOG : I2C Device IOs Deinit");
    
    /* Deinitialize I2Cx Clock */
    I2C_HAL_CLKDeInit(i2c_dev->dev);
    
    I2C_LOG("\n\rLOG : I2C Device Clock Deinit");
      
    /* Deinitialize DMA Channels */
    if (i2c_dev->mode == I2C_PROGMODEL_DMA)
    {
      I2C_HAL_DMADeInit(i2c_dev);
      
      I2C_LOG("\n\rLOG : I2C Device DMA Deinit");  
    } 
    
    /* Initialize I2Cx Clock */
    I2C_HAL_CLKInit(i2c_dev->dev);
    
    I2C_LOG("\n\rLOG : I2C Device Clock Init"); 
    
		
    /* Initialize I2Cx GPIO */
    I2C_HAL_GPIOInit(i2c_dev->dev);
    
    I2C_LOG("\n\rLOG : I2C Device IOs Init");     
	       
  
    /* Enable I2Cx Device */
     i2c_dev->I2C->CR1 |= I2C_CR1_PE ;
		 
    I2C_LOG("\n\rLOG : I2C Device Enabled"); 
    
    /* Initialize I2Cx device with parameters stored in pI2C_Struct */
    I2C_Init(i2c_dev->I2C, i2c_dev->I2C_InitStruct);
    
    I2C_LOG("\n\rLOG : I2C Device Config");   
    
    /* If General Call Mode Option Bit Selected */
    if ((i2c_dev->options & OPT_I2C_GENCALL) != 0)
    {
      /* Enable GENERAL CALL Address Mode */
      i2c_dev->I2C->CR1 |= I2C_CR1_ENGC;
			
      I2C_LOG("\n\rLOG : I2C Device GENCALL Mode Enabled"); 
    }
    
    /* If Dual Address Mode Option Bit Selected */
    if ((i2c_dev->options & OPT_I2C_DUALADDR) != 0)
    {
      /* Enable Dual Address Mode */
      i2c_dev->I2C->OAR2 |= I2C_OAR2_ENDUAL;
			
      /* Configure OAR2 */
       i2c_dev->I2C->OAR2 &= 0xFF01;
       i2c_dev->I2C->OAR2 |= (i2c_dev->options &0xFE); 
			
      I2C_LOG("\n\rLOG : I2C Device DUAL ADDR Mode Enabled"); 
    }    

    /* If NACK Slave Own Address option bit selected */
    if ((i2c_dev->options & OPT_I2C_NACK_ADD) != 0)
    {
      /* Disable Acknowledgement of own Address */
      	i2c_dev->I2C->CR1 &= ~I2C_CR1_ACK;

      I2C_LOG("\n\rLOG : I2C Device NACK Own Address Mode Enabled");
    }
    
    /* If DMA Programming model is selected*/
    if (i2c_dev->mode == I2C_PROGMODEL_DMA) 
    {
      /* Initialize I2Cx DMA Channels */
      I2C_HAL_DMAInit(i2c_dev);
      
      I2C_LOG("\n\rLOG : I2C Device DMA Init");  
    }
    
    /* Initialize I2Cx Interrupts */
    I2C_HAL_ITInit(i2c_dev);
    
    I2C_LOG("\n\rLOG : I2C Device IT Init");
    
    /* Update State to I2C_STATE_READY */
    i2c_dev->state = I2C_STATE_READY;
    
    I2C_LOG("\n\rLOG : I2C Device Ready"); 
    
    
    return RT_EOK;
  }    
  /* If State is BUSY (a transaction is still on going) Exit Init function */
  else 
  {
    I2C_LOG("\n\rERROR : I2C Device Busy"); 
    
    return RT_ERROR; 
  }
}


/**
  * @brief  Deinitialize the peripheral and all related clocks, GPIOs, DMA and NVIC 
  *         to their reset values.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR
  * @note   The Peripheral clock is disabled but the GPIO Ports clocks remains 
  *         enabled after this deinitialization. 
  */
uint32_t I2CDev_DeInit(i2c_dev_t* i2c_dev) 
{
  I2C_LOG("\n\r\n\rLOG <I2CDev_DeInit> : I2C Device Deinit");
  
  /* If State is not BUSY */
  if ((i2c_dev->state == I2C_STATE_READY) 
     || (i2c_dev->state == I2C_STATE_ERROR) 
     || (i2c_dev->state == I2C_STATE_DISABLED))
  {
    /* 
    - If State is I2C_STATE_ERROR (an Error occurred in transaction):
    Perform the deinitialization routines 
    - If State is I2C_STATE_READY:  
    Perform the deinitialization routines 
    - If State is I2C_STATE_DISABLED:  
    Perform the deinitialization routines                                   */
    
    /* Deinitialize I2Cx GPIO */
    I2C_HAL_GPIODeInit(i2c_dev->dev);
    
    I2C_LOG("\n\rLOG : I2C Device IOs Deinit");
     
    /* Disable I2Cx Device */
    i2c_dev->I2C->CR1 &= ~I2C_CR1_PE;
    
    I2C_LOG("\n\rLOG : I2C Device Disabled"); 
    
    /* Deinitialize I2Cx Clock */
    I2C_HAL_CLKDeInit(i2c_dev->dev);
    
    I2C_LOG("\n\rLOG : I2C Device Clock Deinit");       
    

    if (i2c_dev->mode == I2C_PROGMODEL_DMA)
    {
      I2C_HAL_DMADeInit(i2c_dev);
      
      I2C_LOG("\n\rLOG : I2C Device DMA Deinit");  
    } 
    
    I2C_HAL_ITDeInit(i2c_dev);
    
    I2C_LOG("\n\rLOG : I2C Device IT Deinit"); 
       
    /* Initialize i2c_dev state parameters to their default values */
    i2c_dev-> state     = I2C_STATE_DISABLED;     /* Device Disabled */
    i2c_dev-> error = I2C_ERR_NONE;       /* No Device Error */
    i2c_dev-> timeout  = ((uint32_t)I2C_TIMEOUT_DEFAULT);  /* Set timeout value to I2C_TIMEOUT_DEFAULT */  

    I2C_LOG("\n\rLOG :Set State fields to default"); 
    
    
    return RT_EOK;
  }  
  /* If State is BUSY (a transaction is still on going) Exit Init function */
  else 
  {
    I2C_LOG("\n\rERROR : I2C Device Busy"); 
    
    return RT_ERROR; 
  }
}


/**
  * @brief  Initialize the peripheral structure with default values according
  *         to the specified parameters in the I2CDevTypeDef structure.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
uint32_t I2CDev_StructInit(i2c_dev_t* i2c_dev) 
{   
  /* Initialize I2C_InitStructure to their default values */
  I2C_InitStructure.I2C_ClockSpeed          = 100000;                        /* Initialize the I2C_ClockSpeed member */
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;                  /* Initialize the I2C_Mode member */
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;               /* Initialize the I2C_DutyCycle member */
  I2C_InitStructure.I2C_OwnAddress1         = 0;                             /* Initialize the I2C_OwnAddress1 member */
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;                /* Initialize the I2C_Ack member */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* Initialize the I2C_AcknowledgedAddress member */
  
  /* Initialize i2c_dev parameter to their default values */
  i2c_dev-> direction     = I2C_DIRECTION_TXRX;                  /* Transmitter and Receiver direction selected */
  i2c_dev-> mode     =      I2C_PROGMODEL_DMA;                   /* DMA Programming Model selected */
  i2c_dev-> state         = I2C_STATE_DISABLED;                  /* Device Disabled */
  i2c_dev-> error     =   I2C_ERR_NONE;                    /* No Device Error */
  i2c_dev-> options      = ((uint32_t)0x00000000);               /* No Options selected */
  i2c_dev-> timeout      = ((uint32_t)I2C_TIMEOUT_DEFAULT); /* Set timeout value to I2C_TIMEOUT_DEFAULT */
  i2c_dev-> I2C_InitStruct   = &I2C_InitStructure;                   /* Point to I2C_InitStructure (with default values) */
  
  I2C_LOG("\n\r\n\rLOG <I2CDev_StructInit> : I2C Device Structure set to Default Value"); 
  
  return RT_EOK;
}

/**
  * @brief  Allows to send a data or a buffer of data through the peripheral to 
  *         a selected device in a selected location address.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR.
  */
uint32_t I2C_Write(i2c_dev_t* i2c_dev)
{      
  /* Check I2C State: 
  - If busy       --> exit Write operation  
  - If disabled   --> exit Write operation    
  - If error      --> exit Write operation
  - If ready      --> 
                      - If Master Mode selected with No memory Address Mode OR Slave Mode Selected. 
                           - Test the value of programming model
                              - If DMA Programming Model        --> enable TX DMA Channel.
                              - If Interrupt Programming Model  --> enable Buffer Interrupt.
                      - Update STATE to I2C_STATE_READY_TX.
                      - If Master mode selected --> generate start condition 
                      - Enable Event Interrupt                                                 */
  
  I2C_LOG("\n\r\n\rLOG <I2C_Write> : I2C Device Write OP");
  
  /* If Device is Busy (a transaction is still on going) Exit Write function */
  if (((i2c_dev->state & I2C_STATE_BUSY) != 0) 
     || (i2c_dev->state == I2C_STATE_READY_TX) 
     || (i2c_dev->state == I2C_STATE_READY_RX))
  {
    I2C_LOG("\n\rERROR : I2C Device Busy"); 
    
    return RT_ERROR;
  }  
  /* If State is I2C_STATE_DISABLED (device is not initialized) Exit Write function */  
  else if (i2c_dev->state == I2C_STATE_DISABLED)  
  {
    I2C_LOG("\n\rERROR : I2C Device Not Initialized"); 
    
    return RT_ERROR;
  }  
  /* If State is I2C_STATE_ERROR (Error occurred ) */
  else if (i2c_dev->state == I2C_STATE_ERROR)
  {
    I2C_LOG("\n\rERROR : I2C Device Error"); 
    
    return RT_ERROR;
  }  
  /* If State is I2C_STATE_READY ( Start Communication )*/
  else
  {   
    /* Update State to I2C_STATE_BUSY */
    i2c_dev->state = I2C_STATE_BUSY;
    
    /* No Stop Condition Generation option Mode bit not selected */   
    if ((i2c_dev->options & OPT_I2C_NOSTOP_MODE) == 0)
    {
      /* Wait until Busy flag is reset */ 
      I2C_TIMEOUT(!((uint16_t)(i2c_dev->I2C->SR2 & I2C_SR2_BUSY) ), I2C_TIMEOUT_BUSY);
    } 
       
      I2C_LOG("\n\rLOG : I2C Device Master");
      
      /* Generate Start */
		  i2c_dev->I2C->CR1 |= I2C_CR1_START;
      
      /* If No memory Address Option Bit is Selected  */   
      if ((i2c_dev->options & OPT_NO_MEM_ADDR) != 0)
        
      {             
        /* Switch Programing Mode Enable DMA or IT Buffer */
				i2c_dev->direction = I2C_DIRECTION_TX;  
      }   
      
      /* Update State to I2C_STATE_READY_TX */
      i2c_dev->state = I2C_STATE_READY_TX;
      
      I2C_LOG("\n\rLOG : I2C Device Ready TX");
      
      I2C_LOG("\n\rLOG : I2C Device Generates Start");
      
      /* Initialize timeout value */
      i2c_dev->timeout = rt_tick_get() + I2C_TIMEOUT_SB;
     
    /* Enable EVENT Interrupts*/
    I2C_LOG("\n\rLOG : I2C Device EVT IT Enabled"); 
    
     i2c_dev->I2C->CR2 |= I2C_CR2_ITEVTEN;
  }
  
   return RT_EOK;
}

/**
  * @brief  Allows to receive a data or a buffer of data through the peripheral 
  *         from a selected device in a selected location address.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
uint32_t I2C_Read(i2c_dev_t* i2c_dev)
{    
  /* Check I2C State: 
  - If busy       --> exit Read operation  
  - If disabled   --> exit Read operation  
  - If error      --> exit Read operation
  - If ready      --> 
          - If Master Mode with Memory Address Mode.
              - Generate start
              - Send Slave Address
              - Send Memory Address
          - Else Test the value of programming model
              - If DMA Programming Model        --> enable RX DMA Channel 
              - If Interrupt Programming Model  --> enable Buffer Interrupt 
          - Update STATE to I2C_STATE_READY_RX.
          - If Master mode selected --> generate start condition 
          - Enable Event Interrupt                                               */
  
  I2C_LOG("\n\r\n\rLOG <I2C_Read> : I2C Device Perform Read OP");
  
   /* If Device is Busy (a transaction is still on going) Exit Read function */
   if (((i2c_dev->state & I2C_STATE_BUSY) != 0)
      || (i2c_dev->state == I2C_STATE_READY_TX)
      || (i2c_dev->state == I2C_STATE_READY_RX))
  {
    I2C_LOG("\n\rERROR : I2C Device Busy"); 
    
    return RT_ERROR;
  }  
  /* If State is I2C_STATE_DISABLED (device is not initialized) Exit Read function */  
  else if (i2c_dev->state == I2C_STATE_DISABLED)  
  {
    I2C_LOG("\n\rERROR : I2C Device Not Initialized"); 
    
    return RT_ERROR;
  }  
  /* If State is I2C_STATE_ERROR (Error occurred ) */
  else if (i2c_dev->state == I2C_STATE_ERROR)
  {
    I2C_LOG("\n\rERROR : I2C Device Error"); 
    
    return RT_ERROR;
  }  
  /* If State is I2C_STATE_READY */
  else
  {
    /* Update State to I2C_STATE_BUSY */
    i2c_dev->state = I2C_STATE_BUSY;
   
    /* No Stop Condition Generation Mode option bit not selected */   
    if ((i2c_dev->options & OPT_I2C_NOSTOP_MODE) == 0)
    {
      /* Wait until Busy flag is reset */ 
      I2C_TIMEOUT(!((uint16_t)(i2c_dev->I2C->SR2 & I2C_SR2_BUSY) ), I2C_TIMEOUT_BUSY);
    }
           
    /* If One byte transfer with DMA programming model */
    if ((i2c_dev->mode == I2C_PROGMODEL_DMA) 
       && (i2c_dev->bytes_to_read == 1))
    {
      /* Affect 1Byte DMA option to wOptions */
      i2c_dev->options |= DMA_1BYTE_CASE;
      
      /* Change ProgModel to Interrupt */
      i2c_dev->mode = I2C_PROGMODEL_INTERRUPT;
    }
      
    /* If "No Memory Address" Option Bit is not selected and Master Mode selected */
    if ((i2c_dev->options & OPT_NO_MEM_ADDR) == 0)
    {       
      I2C_LOG("\n\rLOG : I2C Device Master No Addr Mem Mode");
      
      /* Generate Start */
     i2c_dev->I2C->CR1 |= I2C_CR1_START;
      
      /* Wait until SB flag is set */
      I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_SB), I2C_TIMEOUT_SB);
      
      /* Send Device Address */
      /* If 7 Bit Addressing Mode */
      if (i2c_dev->I2C_InitStruct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_7bit)
      {             
        /* Send Slave address with bit0 reset for write */
        i2c_dev->I2C->DR =(uint8_t)(i2c_dev->addr<<0x01);   
        
        /* Wait until ADDR flag is reset */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_ADDR), I2C_TIMEOUT_ADDR);        
      }          
      
      I2C_LOG("\n\rLOG : I2C Device Target Address Sent ");
      
      /* Clear ADDR flag: (Read SR1 followed by read of SR2), SR1 read operation is already done */
      i2c_dev->I2C->SR1;
      i2c_dev->I2C->SR2;
			
      /* Wait until TXE flag is set */ 
      I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_TXE), I2C_TIMEOUT_TXE); 
      
      /* If 8 Bit register mode */
      if ((i2c_dev->options & OPT_16BIT_REG) == 0)
      {
        /* Send Register Address */
        i2c_dev->I2C->DR = (uint8_t)(((i2c_dev->reg)& 0x00FF)); 
        
        /* Wait until TXE flag is set */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_TXE), I2C_TIMEOUT_TXE); 
      }      
  #ifdef I2C_16BIT_REG_OPTION
      /* If 16 Bit register mode */
      else
      {
        /* Send MSB Register Address */
        i2c_dev->I2C->DR = (uint8_t)(((i2c_dev->reg)& 0xFF00) >>8);  
        
        /* Wait until TXE flag is set */ 
        I2C_TIMEOUT(I2C_HAL_GET_TXE(i2c_dev->dev), I2C_TIMEOUT_TXE);
        
        /* Send LSB Register Address */
        i2c_dev->I2C->DR = (uint8_t)((i2c_dev->reg)& 0x00FF);  
        
        /* Initialize Timeout value */
        i2c_dev->timeout = I2C_TIMEOUT_MIN + I2C_TIMEOUT_TXE;
        
        /* Wait until TXE flag is set */ 
        I2C_TIMEOUT(I2C_HAL_GET_TXE(i2c_dev->dev), I2C_TIMEOUT_TXE); 
      }      
  #endif /* I2C_16BIT_REG_OPTION */
      
      I2C_LOG("\n\rLOG : I2C Device Target Memory Address Sent");      
    }        
    /* Update State to I2C_STATE_READY_RX */
    i2c_dev->state = I2C_STATE_READY_RX;
    
    I2C_LOG("\n\rLOG : I2C Device Ready RX"); 
        
      
      /* Generate Start */
     i2c_dev->I2C->CR1 |= I2C_CR1_START;
      
     I2C_LOG("\n\rLOG : I2C Device Generates Start"); 
         
      /* Initialize Timeout value */
      i2c_dev->timeout = rt_tick_get() + I2C_TIMEOUT_SB;          
         
    I2C_LOG("\n\rLOG : I2C Device EVT IT Enabled");   
    
    /* Enable EVENT Interrupts*/
     i2c_dev->I2C->CR2 |= I2C_CR2_ITEVTEN;
  }
  
  return RT_EOK;
}




/**
  * @brief  Wait until target device is ready for communication (This function is 
  *         used with Memory devices).           
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
uint32_t I2C_IsDeviceReady(i2c_dev_t* i2c_dev)
{ 
  __IO uint32_t Timeout = 0xFFFF;
   
  I2C_LOG("\n\r\n\rLOG <I2C_DEVICE IsReady> : Wait until I2C Device is Ready");
  
  /* Set  state to I2C_STATE_DISABLED */
  i2c_dev->state = I2C_STATE_BUSY;
  
  /* Disable ERROR Interrupt */
  i2c_dev->I2C->CR2 &= ~I2C_CR2_ITERREN;   
  
  /* Disable I2Cx Device */
  i2c_dev->I2C->CR1 &= ~I2C_CR1_PE;
  
  /* Enable I2Cx Device */
  i2c_dev->I2C->CR1 |= I2C_CR1_PE ;
  
  /* Generate Start */
 i2c_dev->I2C->CR1 |= I2C_CR1_START;
  
  /* Wait until SB flag is set */ 
  I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_SB), I2C_TIMEOUT_SB); 
  
  /* Send Slave address with bit0 reset for write */
  i2c_dev->I2C->DR = (uint8_t)(i2c_dev->addr<<0x01);   
  
  /* wait until timeout elapsed or target device acknowledge its address*/
  while (((i2c_dev->I2C->SR1 & I2C_SR1_ADDR) == 0) && (Timeout-- != 0));
  
  /* If Timeout occurred  */
  if (Timeout == 0) 
  {    
    return RT_ERROR;    
  }  
  /* If ADDR flag is set */
  else
  {      
    /* Clear AF flag */
    i2c_dev->I2C->SR1 = ~I2C_SR1_AF;
		
    /* Generate Stop */ 
		i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
    
    /* wait until Busy flag is reset */
    while(i2c_dev->I2C->SR2 & I2C_SR2_BUSY);
    
    /* Disable I2Cx Device */
    i2c_dev->I2C->CR1 &= ~I2C_CR1_PE;
    
    I2C_LOG("\n\rLOG : I2C Device Disabled"); 
    
    /* Enable I2Cx Device */
    i2c_dev->I2C->CR1 |= I2C_CR1_PE ;
    
    I2C_LOG("\n\rLOG : I2C Device Enabled"); 
    
    /* Enable ACK */
    i2c_dev->I2C->CR1 |= I2C_CR1_ACK;
    
    /* Enable ERROR Interrupt */
    i2c_dev->I2C->CR2 |= I2C_CR2_ITERREN ;
    
    /* Set state to ready */
    i2c_dev->state = I2C_STATE_READY;
   
    I2C_LOG("\n\rLOG : I2C Target device Ready");  
    
    return RT_EOK;
  }  
}

/**
  * @brief  This function handles I2C interrupt request for preparing communication
  *         and for transfer phase in case of using Interrupt Programming Model.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK. 
  */
uint32_t I2C_EV_IRQHandler(i2c_dev_t* i2c_dev)
{     
  __IO uint16_t I2CFlagStatus = 0x0000;
  
  /* Read I2C1 Status Registers 1 and 2 */
  I2CFlagStatus = (uint16_t)(i2c_dev->I2C->SR1 & I2C_STATUS1_EVT_MASK); 
 
    /* If SB event */
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_SB ) != 0)
    {       
      return I2C_MASTER_START_Handle(i2c_dev);        
    } 
    /* If ADDR event */
    if((I2CFlagStatus & (uint16_t)I2C_EVT_ADDR ) != 0)
    {  
      return I2C_MASTER_ADDR_Handle(i2c_dev);              
    }
    
    /* If RXNE event */
    if (((I2CFlagStatus & (uint16_t)I2C_EVT_RXNE) != 0) && (i2c_dev->state == I2C_STATE_BUSY_RX))
    { 
      return I2C_MASTER_RXNE_Handle(i2c_dev); 
    }      

  return RT_EOK;
}

/**
  * @brief  Allows to handle errors occurred during initialization or communication 
  *         in order to recover the correct communication status or call specific 
  *         user functions.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK. 
  */
uint32_t I2C_ER_IRQHandler(i2c_dev_t* i2c_dev)
{  

    /* Read Error Register and affect to error */
    i2c_dev->error = (uint16_t)(i2c_dev->I2C->SR1 & I2C_STATUS_ERR_MASK);
    
    /* Set Device state to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR;
    
    I2C_LOG("\n\r\n\rERROR <I2C_ErrorHandler> : I2C Device Error"); 
    
    /* Clear error flags that can be cleared by writing to SR register */
    i2c_dev->I2C->SR1 = ~I2C_STATUS_ERR_MASK;
    
    /* If Bus error occurred ---------------------------------------------------*/
    if ((i2c_dev->error & I2C_ERR_BERR) != 0)
    {      
      I2C_LOG("\n\rERROR : I2C Device BERR"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */
			i2c_dev->I2C->CR1 |= I2C_CR1_SWRST; 
      i2c_dev->I2C->CR1 &= ~I2C_CR1_SWRST;    
      
      I2C_LOG("\n\r I2C Device Software reset"); 
      
    }
    
    /* If Arbitration Loss error occurred --------------------------------------*/
    if ((i2c_dev->error & I2C_ERR_ARLO) != 0)
    {
      I2C_LOG("\n\rERROR : I2C Device ARLO"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */    
      i2c_dev->I2C->CR1 |= I2C_CR1_SWRST; 
      i2c_dev->I2C->CR1 &= ~I2C_CR1_SWRST;   
      
      I2C_LOG("\n\r I2C Device Software reset"); 
       
    }
    
    /* If Overrun error occurred */
    if ((i2c_dev->error & I2C_ERR_OVR) != 0)
    {
      I2C_LOG("\n\rERROR : I2C Device OVR");
      
    }
        
    /* If Acknowledge Failure error occurred */
    if ((i2c_dev->error & I2C_ERR_AF) != 0)
    {        
      I2C_LOG("\n\rERROR : I2C Device AF"); 
       
    }   
        
    /* USE_SINGLE_ERROR_CALLBACK is defined in conf.h file */
#if defined(USE_SINGLE_ERROR_CALLBACK)  
    /* Call Error UserCallback */  
    I2C_ERR_UserCallback(i2c_dev, i2c_dev->error);
#endif /* USE_SINGLE_ERROR_CALLBACK */
  
  return RT_EOK;
}


/**
  * @brief  Handle I2C DMA TX interrupt request when DMA programming Model is 
  *         used for data transmission. 
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK. 
  */
uint32_t I2C_DMA_TX_IRQHandler(i2c_dev_t* i2c_dev)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  i2c_dev->timeout = I2C_TIMEOUT_DEFAULT; 
  
  I2C_LOG("\n\r\n\rLOG <I2C_DMA_TX_IRQHandler> : I2C Device TX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if((I2C_HAL_GET_DMATX_TCIT(i2c_dev)) != 0)
  {  
    I2C_LOG("\n\rLOG : I2C Device TX Complete");
    
    /* Update remaining number of data */
    i2c_dev->bytes_to_write = 0;
    
    /* Call DMA TX TC UserCallback */
    I2C_DMATXTC_UserCallback(i2c_dev);
    
    /* If DMA Normal mode */
    if ((i2c_dev->options & OPT_DMATX_CIRCULAR) == 0)
    {                   
        /* Disable DMA Request */
        i2c_dev->I2C->CR2 &= ~I2C_CR2_DMAEN ;
			
        /* Wait until BTF flag is set */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_BTF), I2C_TIMEOUT_BTF);
        
        /* No Stop Condition Generation option bit not selected */   
        if ((i2c_dev->options & OPT_I2C_NOSTOP) == 0)
        {          
          /* Generate Stop Condition */
          i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
          
          /* Wait until Busy flag is reset */         
          I2C_TIMEOUT(!((uint16_t)(i2c_dev->I2C->SR2 & I2C_SR2_BUSY) ), I2C_TIMEOUT_BUSY);
        }
        
        /* Disable DMA Channel */                 
        i2c_dev->DMA_TX_Stream->CR &= ~DMA_CR_EN ;        
        
        /* Disable EVENT Interrupt */
				i2c_dev->I2C->CR2 &= ~I2C_CR2_ITEVTEN ;
   
        
        I2C_LOG("\n\rLOG : I2C Device Master TX DMA Disabled");
        
        /* Update State to I2C_STATE_READY */
        i2c_dev->state = I2C_STATE_READY; 
      } 
   
    /* Call TX TC UserCallback */
    I2C_TXTC_UserCallback(i2c_dev);
  }
  /*------------- If HT interrupt ------------*/
  else if ((I2C_HAL_GET_DMATX_HTIT(i2c_dev)) != 0)
  {         
    I2C_LOG("\n\rLOG : I2C Device TX DMA Half Transfer ");
    
    /* Call DMA TX HT UserCallback */
    I2C_DMATXHT_UserCallback(i2c_dev);
  }  
  /* If TE interrupt */
  else if ((I2C_HAL_GET_DMATX_TEIT(i2c_dev)) != 0)
  { 
    I2C_LOG("\n\rERROR : I2C Device TX DMA Transfer Error ");
    
    /* Update State to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR; 
    
    /* Update remaining number of data */
    i2c_dev->bytes_to_write = i2c_dev->DMA_TX_Stream->NDTR;
    
    /* Call DMA TX TE UserCallback */
    I2C_DMATXTE_UserCallback(i2c_dev); 
  }  
  
   /* Clear DMA Interrupt Flag */
    I2C_HAL_CLEAR_DMATX_IT(i2c_dev);
  
  return RT_EOK;
}


/**
  * @brief  Handle I2C DMA RX interrupt request when DMA programming Model is 
  *         used for data reception.  
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK. 
  */
uint32_t I2C_DMA_RX_IRQHandler(i2c_dev_t* i2c_dev)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  i2c_dev->timeout = I2C_TIMEOUT_DEFAULT; 
  
  I2C_LOG("\n\r\n\rLOG <I2C_DMA_RX_IRQHandler> : I2C Device RX DMA ");
  
  /* If TC interrupt */
  if ((I2C_HAL_GET_DMARX_TCIT(i2c_dev)) != 0)
  {   
    I2C_LOG("\n\rLOG : I2C Device RX Complete");
    
    /* Update remaining number of data */
    i2c_dev->bytes_to_read = 0;
       
    /* Call DMA RX TC UserCallback */
    I2C_DMARXTC_UserCallback(i2c_dev);
    
    /* If DMA Normal model */
    if ((i2c_dev->options & OPT_DMARX_CIRCULAR) == 0)
    {      
        /* Generate Stop Condition */
        i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
        
        /* Disable DMA Request and Channel */          
         i2c_dev->I2C->CR2 &= ~I2C_CR2_DMAEN ;
        
        /* Wait until Busy flag is reset */ 
        I2C_TIMEOUT(!((uint16_t)(i2c_dev->I2C->SR2 & I2C_SR2_BUSY)), I2C_TIMEOUT_BUSY);
        
        /* Disable DMA Channel */
        i2c_dev->DMA_RX_Stream->CR &= ~DMA_CR_EN;
        
        /* Disable EVENT Interrupt */
			  i2c_dev->I2C->CR2 &= ~I2C_CR2_ITEVTEN ;
        
        /* Disable DMA automatic NACK generation */
        i2c_dev->I2C->CR2 &= ~I2C_CR2_LAST ;  
			
        I2C_LOG("\n\rLOG : I2C Device Master RX DMA Disabled");
        
        /* Update State to I2C_STATE_READY */
        i2c_dev->state = I2C_STATE_READY; 
        
        /* Call RX TC UserCallback */
        I2C_RXTC_UserCallback(i2c_dev);
    }
    else
    {
      /* Call RX TC UserCallback */
      I2C_RXTC_UserCallback(i2c_dev);
    }
  }  
  /* If HT interrupt */
  else if ((I2C_HAL_GET_DMARX_HTIT(i2c_dev)) != 0)
  {   
    I2C_LOG("\n\rLOG : I2C Device RX DMA Half Transfer");
    
    /* Call DMA RX HT UserCallback */
    I2C_DMARXHT_UserCallback(i2c_dev);
  }  
  /* If TE interrupt */
  else if ((I2C_HAL_GET_DMARX_TEIT(i2c_dev)) != 0)
  {   
    I2C_LOG("\n\rERROR : I2C Device RX DMA Transfer Error ");
    
    /* Update State to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR; 
    
    /* Update remaining number of data */
    i2c_dev->bytes_to_read = i2c_dev->DMA_RX_Stream->NDTR;
    
    /* Call DMA RX TE UserCallback */
    I2C_DMARXTE_UserCallback(i2c_dev); 
  }
  
  /* Clear DMA Interrupt Flag */
  I2C_HAL_CLEAR_DMARX_IT(i2c_dev);
  
  return RT_EOK;
}

/**
  * @brief  This function Manages I2C Timeouts when Timeout occurred.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
uint32_t I2C_Timeout (i2c_dev_t* i2c_dev)
{
  /* Reinitialize Timeout Value */
  i2c_dev->timeout = I2C_TIMEOUT_DEFAULT;
  
  /* update State to I2C_STATE_ERROR */
  i2c_dev->state = I2C_STATE_ERROR;
  
  /* update wDevError to I2C_ERR_TIMEOUT */
  i2c_dev->error = I2C_ERR_TIMEOUT;
  
  /* Call Timeout Callback and quit current function */
  return (I2C_TIMEOUT_UserCallback(i2c_dev));
}

/**
  * @brief  Handles Master Start condition (SB) interrupt event.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
static uint32_t I2C_MASTER_START_Handle(i2c_dev_t* i2c_dev)
{
  /* Reinitialize Timeout Value */
  i2c_dev->timeout = I2C_TIMEOUT_DEFAULT;
  
  I2C_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT"); 
  
  I2C_LOG("\n\rLOG : I2C Device Start Acknowledged"); 
  
    I2C_LOG("\n\rLOG : I2C Device 7bit Address");
    /* Send Address */
    /* If Master run as receiver */
    if (i2c_dev->state == I2C_STATE_READY_RX)
    {
      /* Send Slave address with bit0 set for read */
     i2c_dev->I2C->DR = (uint8_t)((i2c_dev->addr<<0x01) | I2C_OAR1_ADD0);  
      
      /* Update State to I2C_STATE_BUSY */
      i2c_dev->state = I2C_STATE_BUSY_RX; 
      
      I2C_LOG("\n\rLOG : I2C Device Busy RX");
    }    
    /* If Master run as Transmitter */
    else
    {
      /* Send Slave address with bit0 reset for write */
      i2c_dev->I2C->DR = (uint8_t)((i2c_dev->addr<<0x01));        
      
      /* Update State to I2C_STATE_BUSY */
      i2c_dev->state = I2C_STATE_BUSY_TX; 
      
      I2C_LOG("\n\rLOG : I2C Device Busy TX");
    }
    
    I2C_LOG("\n\rLOG : I2C Device Target Address Sent");
    
    /* Initialize Timeout value */
    i2c_dev->timeout = rt_tick_get() + I2C_TIMEOUT_ADDR;             

  return RT_EOK;
}


/**
  * @brief  Handles Master address matched (ADDR) interrupt event. 
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
static uint32_t I2C_MASTER_ADDR_Handle(i2c_dev_t* i2c_dev)
{     
  /* Initialize Timeout value (1 ms for each data to be sent/received) */
  if (i2c_dev->mode != I2C_PROGMODEL_DMA)
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    i2c_dev->timeout = I2C_TIMEOUT_DEFAULT;                
  }  
  else if (i2c_dev->state == I2C_STATE_BUSY_TX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Tx mode */
    i2c_dev->timeout = rt_tick_get() + i2c_dev->bytes_to_write;
  }  
  else if (i2c_dev->state == I2C_STATE_BUSY_RX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Rx mode */ 
    i2c_dev->timeout = rt_tick_get() + i2c_dev->bytes_to_read;
  }  
  else
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    i2c_dev->timeout = I2C_TIMEOUT_DEFAULT;        
  }
  
  if ((i2c_dev->state == I2C_STATE_BUSY_RX) && (i2c_dev->mode == I2C_PROGMODEL_INTERRUPT) && (i2c_dev->bytes_to_read == 0))
  {    
    /* Program STOP bit then clear ADDR flag */
    i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
    i2c_dev->I2C->SR1;
    i2c_dev->I2C->SR2;
    
    /* Update State to I2C_STATE_READY */
    i2c_dev->state = I2C_STATE_READY; 
  }
  else
  {
    if ((i2c_dev->state == I2C_STATE_BUSY_RX) && (i2c_dev->mode == I2C_PROGMODEL_INTERRUPT))
    {       
      /* Switch Programing Mode Enable DMA or IT Buffer */
			i2c_dev->direction = I2C_DIRECTION_RX;
      I2C_Enable_DMA_IT(i2c_dev);
    }
  
    /* If State is I2C_STATE_BUSY_RX and receiving one byte */  
    if ((i2c_dev->state == I2C_STATE_BUSY_RX) && (i2c_dev->bytes_to_read == 1))
    { 
      /* Disable Acknowledge */
			i2c_dev->I2C->CR1 &= ~I2C_CR1_ACK;
      
      /* Clear ADDR Flag by reading SR1 then SR2 */
      i2c_dev->I2C->SR1;
      i2c_dev->I2C->SR2; 
      
      /* Program Generation of Stop Condition */
      i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
    }  
    else if ((i2c_dev->mode == I2C_PROGMODEL_INTERRUPT) &&(i2c_dev->state == I2C_STATE_BUSY_RX) && (i2c_dev->bytes_to_read== 2))
    {
      /* Disable Acknowledge */
     	i2c_dev->I2C->CR1 &= ~I2C_CR1_ACK;
      
      /* Enable Pos */
      i2c_dev->I2C->CR1 |= I2C_CR1_POS;
      
      /* Clear ADDR Flag by reading SR1 then SR2 */
      i2c_dev->I2C->SR1;
      i2c_dev->I2C->SR2;     
    }
    else
    {
      /* Clear ADDR Flag by reading SR1 then SR2 */
      i2c_dev->I2C->SR1;
      i2c_dev->I2C->SR2;
    }  
      /* If OPT_NO_MEM_ADDR is not enabled */
      if ((i2c_dev->options & OPT_NO_MEM_ADDR) == 0)
      {
        /* If State is I2C_STATE_BUSY_TX */  
        if (i2c_dev->state == I2C_STATE_BUSY_TX)
        {         
          /* If 8 Bit register mode */
          if ((i2c_dev->options & OPT_16BIT_REG) == 0)
          {
            /* Send Register Address */
            i2c_dev->I2C->DR = (uint8_t)((i2c_dev->reg)& 0x00FF); 
            
            /* Wait until TXE flag is set */ 
            I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_TXE), I2C_TIMEOUT_TXE);              
          }          
#ifdef I2C_16BIT_REG_OPTION
          /* If 16 Bit register mode */
          else
          {
            /* Send MSB Register Address */
            i2c_dev->I2C->DR = (uint8_t)(((i2c_dev->reg)& 0xFF00) >>8);  
            
            /* Wait until TXE flag is set */ 
            I2C_TIMEOUT(I2C_HAL_GET_TXE(i2c_dev->dev), I2C_TIMEOUT_TXE); 
            
            /* Send LSB Register Address */
            i2c_dev->I2C->DR = (uint8_t)((i2c_dev->reg)& 0x00FF);  
            
            /* Wait until TXE flag is set */ 
            I2C_TIMEOUT(I2C_HAL_GET_TXE(i2c_dev->dev), I2C_TIMEOUT_TXE); 
          }     
#endif /* I2C_16BIT_REG_OPTION */
        }  
        
        /* Switch Programing Mode Enable DMA or IT Buffer */
				i2c_dev->direction = I2C_DIRECTION_TXRX;
        I2C_Enable_DMA_IT(i2c_dev);   
      }      
  }
  return RT_EOK;
}

/**
  * @brief  Handles Master reception (RXNE flag) interrupt event.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR. 
  */
static uint32_t I2C_MASTER_RXNE_Handle(i2c_dev_t* i2c_dev)
{  
  /* If Interrupt Programming Model selected */
  if (i2c_dev->mode == I2C_PROGMODEL_INTERRUPT)
  {  
    /* if less than 3 bytes remaining for reception */ 
    if (i2c_dev->bytes_to_read <= 3)
    {  
      /* One byte */
      if (i2c_dev->bytes_to_read == 1)
      {              
        /* Read Byte */
        *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
        
        /* Point to next data and Decrement remaining number of data */
        i2c_dev->bytes_to_read--;   
        
        I2C_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        I2C_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        I2C_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* Two bytes */
      if (i2c_dev->bytes_to_read == 2)
      {           
        /* Disable Buffer interrupt */
        i2c_dev->I2C->CR2 &= ~I2C_CR2_ITBUFEN;
        
        /* Wait until BTF flag is set */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_BTF), I2C_TIMEOUT_BTF); 
        
        /* Generate Stop Condition */
        i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
        
        /* Read Byte */
        *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
        
        /* Point to next data and Decrement remaining number of data */
        i2c_dev->buffer++;
        
        i2c_dev->bytes_to_read--; 
        
        /* Read Byte */
        *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
        
        /*Decrement remaining number of data */
        i2c_dev->bytes_to_read--;           
        
        I2C_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        I2C_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        I2C_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* 3 Last bytes */
      if (i2c_dev->bytes_to_read == 3)
      {
        /* Disable Buffer interrupt */
         i2c_dev->I2C->CR2 &= ~I2C_CR2_ITBUFEN;
        
        /* Wait until BTF flag is set */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_BTF), I2C_TIMEOUT_BTF); 
        
        /* Program NACK Generation */
        	i2c_dev->I2C->CR1 &= ~I2C_CR1_ACK;
        
        /* Read Byte */
        *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
        /* Point to next data and Decrement remaining number of data */
        i2c_dev->buffer++;
        
        i2c_dev->bytes_to_read--; 
        
         /* Wait until BTF flag is set */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_BTF), I2C_TIMEOUT_BTF); 
        
        /* Generate Stop Condition */
        i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;        
        
        /* Read Byte */
        *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
        
        /* Point to next data and Decrement remaining number of data */
        i2c_dev->buffer++;
        
        i2c_dev->bytes_to_read--; 
          
        /* Read Byte */
        *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
        
        /* Decrement remaining number of data */
        i2c_dev->bytes_to_read--;   
        
        I2C_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        I2C_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        I2C_LOG("\n\rLOG : I2C Device RX Stop Programmed");
      }          
    }     
    /* if bytes remaining for reception */ 
    else
    {
      /* Read Byte */
      *(i2c_dev->buffer) = (uint8_t)i2c_dev->I2C->DR;
      
      /* Point to next data and Decrement remaining number of data */
      i2c_dev->buffer++;
      
      i2c_dev->bytes_to_read--; 
      
      /* Call RX UserCallback */
      I2C_RX_UserCallback(i2c_dev);
    }
    
    /* If All data are received */
    if (i2c_dev->bytes_to_read == 0)
    {      
      I2C_LOG("\n\rLOG : I2C Device Nack and Stop Generated ");
      
      I2C_LOG("\n\rLOG : I2C Device RX Complete"); 
      
      /* Disable EVENT Interrupt */
			i2c_dev->I2C->CR2 &= ~I2C_CR2_ITEVTEN ;
      
      I2C_LOG("\n\rLOG : I2C Device RX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
       i2c_dev->I2C->CR2 &= ~I2C_CR2_ITBUFEN;
      
      I2C_LOG("\n\rLOG : I2C Device RX BUFF IT Disabled");
      
      /* Clear BTF Flag */ 
      i2c_dev->I2C->SR1;
      i2c_dev->I2C->DR ;
			
      /* If 1Byte DMA option is selected */
      if ((i2c_dev->options & DMA_1BYTE_CASE) != 0)
      {
        /* Clear 1Byte DMA option from wOptions */
        i2c_dev->options &= ~DMA_1BYTE_CASE;
        
        /* Change ProgModel to DMA */
        i2c_dev->mode = I2C_PROGMODEL_DMA;
      }
      
      /* Wait until Busy flag is reset */ 
      I2C_TIMEOUT(!((uint16_t)(i2c_dev->I2C->SR2 & I2C_SR2_BUSY)), I2C_TIMEOUT_BUSY);
      
      /* Enable ACK generation and disable POS */
      i2c_dev->I2C->CR1 |= I2C_CR1_ACK;      
       i2c_dev->I2C->CR1 &= ~I2C_CR1_POS ;
      
      /* Update State to I2C_STATE_READY */
      i2c_dev->state = I2C_STATE_READY;
      
      /* Call RX Transfer complete Callback */
      I2C_RXTC_UserCallback(i2c_dev);
    }
  }  
  return RT_EOK;
}


/**
  * @brief  This function Configure I2C DMA and Interrupts before starting transfer phase.
  * @param  i2c_dev: Pointer to the peripheral configuration structure.
  * @param  Direction : Transfer direction.
  * @retval RT_EOK or RT_ERROR. 
  */
uint32_t I2C_Enable_DMA_IT (i2c_dev_t* i2c_dev)
{
  /* Switch the value of ProgModel */
  switch (i2c_dev->mode)
  { 
           
  case I2C_PROGMODEL_INTERRUPT:
   
    /* Enable BUFFER Interrupt*/
    i2c_dev->I2C->CR2 |= I2C_CR2_ITBUFEN;
    
    I2C_LOG("\n\rLOG : I2C Device BUFF IT Enabled"); 
    
    return RT_EOK;
    

    case I2C_PROGMODEL_DMA:
    
     /* Disable EVENT Interrupt */
     i2c_dev->I2C->CR2 &= ~I2C_CR2_ITEVTEN ;
    
     /* Enable DMA request */
		i2c_dev->I2C->CR2 |= I2C_CR2_DMAEN;
    
    /* If a data transmission will be performed */
    if ((i2c_dev->state == I2C_STATE_BUSY_TX) || (i2c_dev->direction == I2C_DIRECTION_TX))
    {
      /* Configure TX DMA Channels */
      I2C_HAL_DMATXConfig(i2c_dev);
      
      /* Disable DMA automatic NACK generation */
      i2c_dev->I2C->CR2 &= ~I2C_CR2_LAST ; 
    
      /* Enable TX DMA Channels */
			i2c_dev->DMA_TX_Stream->CR |= DMA_CR_EN ; 
      
      I2C_LOG("\n\rLOG : I2C Device DMA TX Enabled");       
    }    
     /* If a data reception will be performed */
    else if ((i2c_dev->state == I2C_STATE_BUSY_RX) || (i2c_dev->direction == I2C_DIRECTION_RX))
    {
      /* Configure RX DMA Channels */
      I2C_HAL_DMARXConfig(i2c_dev);
      
        /* Enable DMA automatic NACK generation */
			i2c_dev->I2C->CR2 |= I2C_CR2_LAST;
    
      /* Enable RX DMA Channels */
			i2c_dev->DMA_RX_Stream->CR |= DMA_CR_EN ;                  
    }
    
    return RT_EOK; 
      
  default:
    
    /* Update State to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR;
    
    I2C_LOG("\n\rERROR : I2C Device Error"); 
    
    /* exit function */
    return RT_ERROR;
  }  
}

