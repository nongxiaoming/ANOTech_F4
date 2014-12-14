#include "drv_i2c.h"



#ifdef I2C_DEBUG
#define i2c_debug(fmt, ...)  rt_kprintf(fmt, ##__VA_ARGS__)
#else
#define i2c_debug(fmt, ...)
#endif																								 
                                                   
i2c_dev_t i2c1_dev = {                  I2C1,
	                                      I2C1_DMA,
	                                      I2C1_DMA_CHANNEL,
	                                      I2C1_DMA_TX_Stream,
	                                      I2C1_DMA_RX_Stream,
	                                      I2C_DIRECTION_TXRX,
                                        I2C_PROGMODEL_DMA,               
                                        RT_NULL,                         
                                        0,       
	                                      0,
	                                      0,
	                                      0,
                                        I2C_STATE_DISABLED,                  /* Device Disabled */
                                        I2C_ERR_NONE,                        /* No Device Error */
                                        ((uint32_t)0x00000000),              /* No Options selected */ 
                                        ((uint32_t)I2C_TIMEOUT_DEFAULT)      /* Set timeout value to I2C_TIMEOUT_DEFAULT */ 
                                        };       


I2C_InitTypeDef I2C_InitStructure;
DMA_InitTypeDef I2C_DMA_InitStructure;


static uint32_t I2C_MASTER_START_Handle(i2c_dev_t* i2c_dev); /* Handle Master SB Interrupt event */  
static uint32_t I2C_MASTER_ADDR_Handle(i2c_dev_t* i2c_dev);  /* Handle Master ADDR Interrupt event */
static uint32_t I2C_MASTER_RXNE_Handle(i2c_dev_t* i2c_dev);  /* Handle Master RXNE Interrupt event */
 

static uint32_t I2C_Timeout (i2c_dev_t* i2c_dev);


static void I2C_Configuration(i2c_dev_t *i2c_dev)
{  
	if(i2c_dev->I2C == I2C1)
	{
   /* Reset I2C1 device clock in order to avoid non-cleared error flags */
  RCC_APB1PeriphResetCmd(I2C1_CLK,ENABLE);
  RCC_APB1PeriphResetCmd(I2C1_CLK,DISABLE);  
  /* Enable I2C1 device clock */
  RCC_APB1PeriphClockCmd(I2C1_CLK, ENABLE); 
	}	
  /* Initialize I2C_InitStructure to their default values */
  I2C_InitStructure.I2C_ClockSpeed          = 400000;                        /* Initialize the I2C_ClockSpeed member */
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;                  /* Initialize the I2C_Mode member */
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;               /* Initialize the I2C_DutyCycle member */
  I2C_InitStructure.I2C_OwnAddress1         = 0;                             /* Initialize the I2C_OwnAddress1 member */
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;                /* Initialize the I2C_Ack member */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  /* Initialize the I2C_AcknowledgedAddress member */	
	I2C_Init(i2c_dev->I2C, &I2C_InitStructure);
}


static void GPIO_Configuration(i2c_dev_t *i2c_dev)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  
	  /* Set GPIO frequency to 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  /* Select Alternate function mode */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  /* Select output Open Drain type */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  /* Disable internal Pull-up */
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	if(i2c_dev->I2C==I2C1)
	{
  /* Enable I2Cx SCL and SDA Pin Clock */
   RCC_AHB1PeriphClockCmd(I2C1_SCL_GPIO_CLK | I2C1_SDA_GPIO_CLK, ENABLE); 
  /* Connect PXx to I2C_SCL */
  GPIO_PinAFConfig((GPIO_TypeDef*)I2C1_SCL_GPIO_PORT,I2C1_SCL_GPIO_PINSOURCE,I2C1_AF);
  /* Connect PXx to I2C_SDA */
  GPIO_PinAFConfig((GPIO_TypeDef*)I2C1_SDA_GPIO_PORT,I2C1_SDA_GPIO_PINSOURCE,I2C1_AF); 
  /* Initialize I2Cx SCL Pin */ 
  GPIO_InitStructure.GPIO_Pin = I2C1_SCL_GPIO_PIN;
  GPIO_Init((GPIO_TypeDef*)I2C1_SCL_GPIO_PORT, &GPIO_InitStructure);
  /* Initialize I2Cx SDA Pin */
  GPIO_InitStructure.GPIO_Pin = I2C1_SDA_GPIO_PIN;
  GPIO_Init((GPIO_TypeDef*)I2C1_SDA_GPIO_PORT, &GPIO_InitStructure);  
  }		
}
static void NVIC_Configuration(i2c_dev_t* i2c_dev)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  /* Enable the IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  /* Configure NVIC for I2Cx EVT Interrupt */
	if(i2c_dev->I2C == I2C1)
	{
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_IT_EVT_IRQn ;
	}
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_EVT_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_EVT_SUBPRIO;
  NVIC_Init(&NVIC_InitStructure);
  
  /* If I2C ERR Interrupt Option Bit not selected */ 
  if ((i2c_dev->options & OPT_I2C_ERRIT_DISABLE) == 0)    
  {
    /* Configure NVIC for I2Cx ERR Interrupt */
	if(i2c_dev->I2C == I2C1)
	   {
      NVIC_InitStructure.NVIC_IRQChannel = I2C1_IT_ERR_IRQn;
     }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_ERR_PREPRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_ERR_SUBPRIO;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable I2C Error Interrupts */
    i2c_dev->I2C->CR2 |= I2C_CR2_ITERREN ;
  }
  
  if (i2c_dev->mode == I2C_PROGMODEL_DMA)
  {
 
      /* Configure NVIC for DMA TX channel interrupt */
			if(i2c_dev->I2C == I2C1)
	   {
      NVIC_InitStructure.NVIC_IRQChannel = I2C1_DMA_TX_IRQn;
     }
      
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMATX_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMATX_SUBPRIO;
      NVIC_Init(&NVIC_InitStructure);
      
      /* Enable DMA TX Channel TCIT  */
      i2c_dev->DMA_TX_Stream->CR |= DMA_IT_TC;
      
      /* Enable DMA TX Channel TEIT  */    
      i2c_dev->DMA_TX_Stream->CR |= DMA_IT_TE;
      
      /* If DMA TX HT interrupt Option Bits Selected */
      if ((i2c_dev->options & OPT_DMATX_HTIT) != 0)
      {
        /* Enable DMA TX Channel HTIT  */ 
				i2c_dev->DMA_TX_Stream->CR |= DMA_IT_HT;
      }

      /* Configure NVIC for DMA RX channel interrupt */
		if(i2c_dev->I2C == I2C1)
	   {
      NVIC_InitStructure.NVIC_IRQChannel = I2C1_DMA_RX_IRQn;
     }
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMARX_PREPRIO;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMARX_SUBPRIO;
      NVIC_Init(&NVIC_InitStructure);
      
      /* Enable DMA RX Channel TCIT  */
      i2c_dev->DMA_RX_Stream->CR |= DMA_IT_TC; 
      
      /* Enable DMA RX Channel TEIT  */
			i2c_dev->DMA_RX_Stream->CR |= DMA_IT_TE; 
      
      /* If DMA RX HT interrupt Option Bits Selected */
      if ((i2c_dev->options & OPT_DMARX_HTIT) != 0)
      {
        /* Enable DMA RX Channel HTIT  */    
        i2c_dev->DMA_RX_Stream->CR |= DMA_IT_HT;				
      }
  }
}    
static void DMA_Configuration(i2c_dev_t* i2c_dev)
{  
  
		if(i2c_dev->I2C == I2C1)
	{
	  /* Enable I2Cx DMA */
  RCC_AHB1PeriphClockCmd(I2C1_DMA_CLK, ENABLE);
	}
  /* I2Cx Common Stream Configuration */
  I2C_DMA_InitStructure.DMA_Channel = i2c_dev->DMA_Channel;
  I2C_DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  I2C_DMA_InitStructure.DMA_BufferSize = 0xFFFF;
  I2C_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  I2C_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  I2C_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  I2C_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  I2C_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  I2C_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  I2C_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  I2C_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  I2C_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  I2C_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    /* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
    I2C_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&i2c_dev->I2C->DR;
    /* Select Memory to Peripheral transfer direction */
    I2C_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    /* Initialize I2Cx DMA Tx Stream */
    DMA_Init(i2c_dev->DMA_TX_Stream, &I2C_DMA_InitStructure);   
  
    /* Select Peripheral to Memory transfer direction */
    I2C_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    /* Initialize I2Cx DMA Rx Stream */
    DMA_Init(i2c_dev->DMA_RX_Stream, &I2C_DMA_InitStructure);   
}
    
static void I2C_DMATXConfig(i2c_dev_t* i2c_dev)
{
  /* Set Channel */
  I2C_DMA_InitStructure.DMA_Channel = i2c_dev->DMA_Channel;
  
  /* Set Memory Base Address */
  I2C_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(i2c_dev->buffer);
  
  /* Set number of data */
  I2C_DMA_InitStructure.DMA_BufferSize = i2c_dev->bytes_to_write;
  
  /* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
  I2C_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&i2c_dev->I2C->DR;
  
  /* If TX DMA Circular Mode Option Bit Selected */
  if ((i2c_dev->options & OPT_DMATX_CIRCULAR) != 0)
  {
    /* Select DMA Circular Mode */  
    I2C_DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  }
  /* If TX DMA Circular Mode Option Bit not selected */
  else 
  {
    /* Select DMA Normal Mode */
    I2C_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  } 
  
  /* Select Peripheral to Memory transfer direction */
  I2C_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  
  /* Initialize I2Cx DMA Tx Stream */
  DMA_Init(i2c_dev->DMA_TX_Stream, &I2C_DMA_InitStructure);   
}

void I2C_DMARXConfig(i2c_dev_t* i2c_dev)
{
  /* Set Channel */
  I2C_DMA_InitStructure.DMA_Channel = i2c_dev->DMA_Channel;
  
  /* Set Memory Base Address */
  I2C_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(i2c_dev->buffer);
  
  /* Set number of data */
  I2C_DMA_InitStructure.DMA_BufferSize = i2c_dev->bytes_to_read;

  /* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
  I2C_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&i2c_dev->I2C->DR;
  
  /* If RX DMA Circular Mode Option Bit Selected */
  if ((i2c_dev->options & OPT_DMARX_CIRCULAR) != 0)
  {
    /* Select DMA Circular Mode */  
    I2C_DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  }  
  /* If RX DMA Circular Mode Option Bit not selected */
  else 
  {
    /* Select DMA Normal Mode */
    I2C_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  } 
  
  /* Select Peripheral to Memory transfer direction */
  I2C_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  
  /* Initialize I2Cx DMA Rx Stream */
  DMA_Init(i2c_dev->DMA_RX_Stream, &I2C_DMA_InitStructure);   
}

/**
  * @brief  Initialize the peripheral and all related clocks, GPIOs, DMA and 
  *         Interrupts according to the specified parameters in the 
  *         I2CDev_InitTypeDef structure.
  * @param  i2c_dev : Pointer to the peripheral configuration structure.
  * @retval RT_EOK or RT_ERROR 
  */
uint32_t I2CDev_Init(i2c_dev_t* i2c_dev) 
{
  i2c_debug("\n\r\n\rLOG <I2CDev_Init> : I2C Device Init");
  
  /* If I2C_State is not BUSY */
  if ((i2c_dev->state == I2C_STATE_READY) 
     || (i2c_dev->state == I2C_STATE_ERROR) 
     || (i2c_dev->state == I2C_STATE_DISABLED))
  { 
    
    /* Disable I2Cx Device */
    i2c_dev->I2C->CR1 &= ~I2C_CR1_PE;
		
    /* Initialize I2Cx GPIO */
    GPIO_Configuration(i2c_dev);
    
    i2c_debug("\n\rLOG : I2C Device IOs Init");     
	       
    /* Enable I2Cx Device */
    i2c_dev->I2C->CR1 |= I2C_CR1_PE ;
		 
    i2c_debug("\n\rLOG : I2C Device Enabled"); 
    
    /* Initialize I2Cx device */
    I2C_Configuration(i2c_dev);
    
    i2c_debug("\n\rLOG : I2C Device Config");   
    

    /* If NACK Slave Own Address option bit selected */
    if ((i2c_dev->options & OPT_I2C_NACK_ADD) != 0)
    {
      /* Disable Acknowledgement of own Address */
      	i2c_dev->I2C->CR1 &= ~I2C_CR1_ACK;

      i2c_debug("\n\rLOG : I2C Device NACK Own Address Mode Enabled");
    }
    
    /* If DMA Programming model is selected*/
    if (i2c_dev->mode == I2C_PROGMODEL_DMA) 
    {
      /* Initialize I2Cx DMA Channels */
      DMA_Configuration(i2c_dev);
      
      i2c_debug("\n\rLOG : I2C Device DMA Init");  
    }
    
    /* Initialize I2Cx Interrupts */
    NVIC_Configuration(i2c_dev);
    
    i2c_debug("\n\rLOG : I2C Device IT Init");
    
    /* Update State to I2C_STATE_READY */
    i2c_dev->state = I2C_STATE_READY;
    
    i2c_debug("\n\rLOG : I2C Device Ready"); 
    
    return RT_EOK;
  }    
  /* If State is BUSY (a transaction is still on going) Exit Init function */
  else 
  {
    i2c_debug("\n\rERROR : I2C Device Busy"); 
    
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
  
  /* Initialize i2c_dev parameter to their default values */
  i2c_dev-> direction     = I2C_DIRECTION_TXRX;          /* Transmitter and Receiver direction selected */  
	i2c_dev-> mode     =      I2C_PROGMODEL_DMA;           /* DMA Programming Model selected */
  i2c_dev-> state    = I2C_STATE_DISABLED;               /* Device Disabled */
  i2c_dev-> error    =   I2C_ERR_NONE;                   /* No Device Error */
  i2c_dev-> options  = ((uint32_t)0x00000000);           /* No Options selected */
  i2c_dev-> timeout  = ((uint32_t)I2C_TIMEOUT_DEFAULT);  /* Set timeout value to I2C_TIMEOUT_DEFAULT */
  
  i2c_debug("\n\r\n\rLOG <I2CDev_StructInit> : I2C Device Structure set to Default Value"); 
  
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
  /* If State is not I2C_STATE_READY  */
 if (i2c_dev->state != I2C_STATE_READY)
  {
    i2c_debug("error : I2C Device Error\n"); 
    return RT_ERROR;
  }    
 
    /* Update State to I2C_STATE_BUSY */
    i2c_dev->state = I2C_STATE_BUSY;
    
    /* No Stop Condition Generation option Mode bit not selected */   
    if ((i2c_dev->options & OPT_I2C_NOSTOP_MODE) == 0)
    {
      /* Wait until Busy flag is reset */ 
      I2C_TIMEOUT(!((uint16_t)(i2c_dev->I2C->SR2 & I2C_SR2_BUSY) ), I2C_TIMEOUT_BUSY);
    } 
       
      i2c_debug("LOG : I2C Device Master\n");
      
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
      
      i2c_debug("LOG : I2C Device Ready TX\n");
      
      i2c_debug("LOG : I2C Device Generates Start\n");
      
      /* Initialize timeout value */
      i2c_dev->timeout = rt_tick_get() + I2C_TIMEOUT_SB;
     
    /* Enable EVENT Interrupts*/
    i2c_debug("LOG : I2C Device EVT IT Enabled\n"); 
    
     i2c_dev->I2C->CR2 |= I2C_CR2_ITEVTEN;
  
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
  
  /* If State is not I2C_STATE_READY  */
 if (i2c_dev->state != I2C_STATE_READY)
  {
    i2c_debug("error : I2C Device Error\n"); 
    return RT_ERROR;
  }  

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
      i2c_debug("LOG : I2C Device Master No Addr Mem Mode\n");
      
      /* Generate Start */
     i2c_dev->I2C->CR1 |= I2C_CR1_START;
      
      /* Wait until SB flag is set */
      I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_SB), I2C_TIMEOUT_SB);
              
        /* Send Slave address with bit0 reset for write */
        i2c_dev->I2C->DR =(uint8_t)(i2c_dev->addr<<0x01);   
        
        /* Wait until ADDR flag is reset */ 
        I2C_TIMEOUT((uint16_t)(i2c_dev->I2C->SR1 & I2C_SR1_ADDR), I2C_TIMEOUT_ADDR);              
      
      i2c_debug("LOG : I2C Device Target Address Sent\n");
      
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
      
      i2c_debug("LOG : I2C Device Target Memory Address Sent\n");      
    }        
    /* Update State to I2C_STATE_READY_RX */
    i2c_dev->state = I2C_STATE_READY_RX;
    
    i2c_debug("LOG : I2C Device Ready RX\n"); 
        
      
      /* Generate Start */
     i2c_dev->I2C->CR1 |= I2C_CR1_START;
      
     i2c_debug("LOG : I2C Device Generates Start\n"); 
         
      /* Initialize Timeout value */
      i2c_dev->timeout = rt_tick_get() + I2C_TIMEOUT_SB;          
         
    i2c_debug("LOG : I2C Device EVT IT Enabled\n");   
    
    /* Enable EVENT Interrupts*/
     i2c_dev->I2C->CR2 |= I2C_CR2_ITEVTEN;
  
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
   
  i2c_debug("LOG <I2C_DEVICE IsReady> : Wait until I2C Device is Ready\n");
  
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
    
    i2c_debug("LOG : I2C Device Disabled\n"); 
    
    /* Enable I2Cx Device */
    i2c_dev->I2C->CR1 |= I2C_CR1_PE ;
    
    i2c_debug("LOG : I2C Device Enabled\n"); 
    
    /* Enable ACK */
    i2c_dev->I2C->CR1 |= I2C_CR1_ACK;
    
    /* Enable ERROR Interrupt */
    i2c_dev->I2C->CR2 |= I2C_CR2_ITERREN ;
    
    /* Set state to ready */
    i2c_dev->state = I2C_STATE_READY;
   
    i2c_debug("LOG : I2C Target device Ready\n");  
    
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
    if ((I2CFlagStatus & I2C_SR1_SB ) != 0)
    {       
      return I2C_MASTER_START_Handle(i2c_dev);        
    } 
    /* If ADDR event */
    if((I2CFlagStatus & I2C_SR1_ADDR ) != 0)
    {  
      return I2C_MASTER_ADDR_Handle(i2c_dev);              
    }
    /* If RXNE event */
    if (((I2CFlagStatus & (uint16_t)I2C_SR1_RXNE) != 0) && (i2c_dev->state == I2C_STATE_BUSY_RX))
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
    i2c_debug("ERROR <I2C_ErrorHandler> : I2C Device Error\n"); 
    /* Clear error flags that can be cleared by writing to SR register */
    i2c_dev->I2C->SR1 = ~I2C_STATUS_ERR_MASK;
    /* If Bus error occurred */
    if ((i2c_dev->error & I2C_ERR_BERR) != 0)
    {      
      i2c_debug("ERROR : I2C Device BERR\n"); 
      /* Generate I2C software reset in order to release SDA and SCL lines */
			i2c_dev->I2C->CR1 |= I2C_CR1_SWRST; 
      i2c_dev->I2C->CR1 &= ~I2C_CR1_SWRST;    
      i2c_debug("I2C Device Software reset\n");    
    }
    /* If Arbitration Loss error occurred */
    if ((i2c_dev->error & I2C_ERR_ARLO) != 0)
    {
      i2c_debug("ERROR : I2C Device ARLO\n"); 
      /* Generate I2C software reset in order to release SDA and SCL lines */    
      i2c_dev->I2C->CR1 |= I2C_CR1_SWRST; 
      i2c_dev->I2C->CR1 &= ~I2C_CR1_SWRST;   
      i2c_debug("I2C Device Software reset\n"); 
    }
    /* If Overrun error occurred */
    if ((i2c_dev->error & I2C_ERR_OVR) != 0)
    {
      i2c_debug("ERROR : I2C Device OVR\n");
    }
    /* If Acknowledge Failure error occurred */
    if ((i2c_dev->error & I2C_ERR_AF) != 0)
    {        
      i2c_debug("ERROR : I2C Device AF\n");   
    }   
   /* Generate STOP */
  i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
  I2CDev_Init(i2c_dev);
  
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
  i2c_debug("LOG <I2C_DMA_TX_IRQHandler> : I2C Device TX DMA \n");
  /* If TC interrupt */
  if((I2C_GET_DMATX_TCIT(i2c_dev)) != 0)
  {  
    i2c_debug("LOG : I2C Device TX Complete\n");
    /* Update remaining number of data */
    i2c_dev->bytes_to_write = 0;
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
        i2c_debug("LOG : I2C Device Master TX DMA Disabled\n");
        /* Update State to I2C_STATE_READY */
        i2c_dev->state = I2C_STATE_READY; 
      } 
  }
  /* If HT interrupt */
  else if ((I2C_GET_DMATX_HTIT(i2c_dev)) != 0)
  {         
    i2c_debug("LOG : I2C Device TX DMA Half Transfer \n");
  }  
  /* If TE interrupt */
  else if ((I2C_GET_DMATX_TEIT(i2c_dev)) != 0)
  { 
    i2c_debug("ERROR : I2C Device TX DMA Transfer Error \n");
    /* Update State to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR; 
    /* Update remaining number of data */
    i2c_dev->bytes_to_write = i2c_dev->DMA_TX_Stream->NDTR; 
  }  
   /* Clear DMA Interrupt Flag */
    I2C_CLEAR_DMATX_IT(i2c_dev);
  
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
  
  i2c_debug("LOG <I2C_DMA_RX_IRQHandler> : I2C Device RX DMA \n");
  /* If TC interrupt */
  if ((I2C_GET_DMARX_TCIT(i2c_dev)) != 0)
  {   
    i2c_debug("\n\rLOG : I2C Device RX Complete");
    /* Update remaining number of data */
    i2c_dev->bytes_to_read = 0; 
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
        i2c_debug("LOG : I2C Device Master RX DMA Disabled\n");
        /* Update State to I2C_STATE_READY */
        i2c_dev->state = I2C_STATE_READY; 
    }
  }  
  /* If HT interrupt */
  else if ((I2C_GET_DMARX_HTIT(i2c_dev)) != 0)
  {   
    i2c_debug("LOG : I2C Device RX DMA Half Transfer\n");
  }  
  /* If TE interrupt */
  else if ((I2C_GET_DMARX_TEIT(i2c_dev)) != 0)
  {   
    i2c_debug("ERROR : I2C Device RX DMA Transfer Error \n");
    /* Update State to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR; 
    /* Update remaining number of data */
    i2c_dev->bytes_to_read = i2c_dev->DMA_RX_Stream->NDTR;

  }
  /* Clear DMA Interrupt Flag */
  I2C_CLEAR_DMARX_IT(i2c_dev);
  
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
  
	  /* Generate STOP */
  i2c_dev->I2C->CR1 |= I2C_CR1_STOP ;
	
  I2CDev_Init(i2c_dev);
	
	return 0;
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
  
  i2c_debug("LOG : I2C Device Start Acknowledged\n"); 
  
    i2c_debug("LOG : I2C Device 7bit Address\n");
    /* Send Address */
    /* If Master run as receiver */
    if (i2c_dev->state == I2C_STATE_READY_RX)
    {
      /* Send Slave address with bit0 set for read */
     i2c_dev->I2C->DR = (uint8_t)((i2c_dev->addr<<0x01) | I2C_OAR1_ADD0);  
      
      /* Update State to I2C_STATE_BUSY */
      i2c_dev->state = I2C_STATE_BUSY_RX; 
      
      i2c_debug("\n\rLOG : I2C Device Busy RX");
    }    
    /* If Master run as Transmitter */
    else
    {
      /* Send Slave address with bit0 reset for write */
      i2c_dev->I2C->DR = (uint8_t)((i2c_dev->addr<<0x01));        
      
      /* Update State to I2C_STATE_BUSY */
      i2c_dev->state = I2C_STATE_BUSY_TX; 
      
      i2c_debug("LOG : I2C Device Busy TX\n");
    }
    
    i2c_debug("LOG : I2C Device Target Address Sent\n");
    
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
    /* Set 1 tick timeout for each data transfer in case of DMA Tx mode */
    i2c_dev->timeout = rt_tick_get() + i2c_dev->bytes_to_write;
  }  
  else if (i2c_dev->state == I2C_STATE_BUSY_RX)
  {
    /* Set 1 tick timeout for each data transfer in case of DMA Rx mode */ 
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
        
        i2c_debug("LOG <I2C_EV_IRQHandler> : I2C Device Master IT\n");
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
        
        i2c_debug("LOG <I2C_EV_IRQHandler> : I2C Device Master IT\n");
        
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
        
        i2c_debug("LOG <I2C_EV_IRQHandler> : I2C Device Master IT\n");
        
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
      
    }
    
    /* If All data are received */
    if (i2c_dev->bytes_to_read == 0)
    {      
      
      i2c_debug("LOG : I2C Device RX Complete\n"); 
      
      /* Disable EVENT Interrupt */
			i2c_dev->I2C->CR2 &= ~I2C_CR2_ITEVTEN ;
      
      /* Disable Buffer interrupt */
       i2c_dev->I2C->CR2 &= ~I2C_CR2_ITBUFEN;
      
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
    
    i2c_debug("LOG : I2C Device BUFF IT Enabled\n"); 
    
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
      I2C_DMATXConfig(i2c_dev);
      
      /* Disable DMA automatic NACK generation */
      i2c_dev->I2C->CR2 &= ~I2C_CR2_LAST ; 
    
      /* Enable TX DMA Channels */
			i2c_dev->DMA_TX_Stream->CR |= DMA_CR_EN ; 
      
      i2c_debug("LOG : I2C Device DMA TX Enabled\n");       
    }    
     /* If a data reception will be performed */
    else if ((i2c_dev->state == I2C_STATE_BUSY_RX) || (i2c_dev->direction == I2C_DIRECTION_RX))
    {
      /* Configure RX DMA Channels */
      I2C_DMARXConfig(i2c_dev);
      
        /* Enable DMA automatic NACK generation */
			i2c_dev->I2C->CR2 |= I2C_CR2_LAST;
    
      /* Enable RX DMA Channels */
			i2c_dev->DMA_RX_Stream->CR |= DMA_CR_EN ;                  
    }
    
    return RT_EOK; 
      
  default:
    
    /* Update State to I2C_STATE_ERROR */
    i2c_dev->state = I2C_STATE_ERROR;
    
    i2c_debug("ERROR : I2C Device Error\n"); 
    
    /* exit function */
    return RT_ERROR;
  }  
}

