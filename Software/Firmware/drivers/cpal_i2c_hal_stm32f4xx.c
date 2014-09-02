
#include "cpal_i2c_hal_stm32f4xx.h"
#include "drv_i2c.h"


i2c_dev_t i2c1_dev = { I2C1_Dev,                          /* I2C1 device number */
	                                       I2C1,
	                                      #ifdef I2C_DMA_PROGMODEL
	                                      I2C1_DMA,
	                                      I2C1_DMA_CHANNEL,
	                                      I2C1_DMA_TX_Stream,
	                                      I2C1_DMA_RX_Stream,
	                                      #endif
                                        I2C_DIRECTION_TXRX,                /* Transmitter and Receiver direction selected */
                                        #ifdef I2C_DMA_PROGMODEL
                                        I2C_PROGMODEL_DMA,                 /* DMA Programming Model selected */
                                        #else
                                        I2C_PROGMODEL_INTERRUPT,           /* IT Programming Model selected */
                                        #endif /* I2C_DMA_PROGMODEL */
                                        RT_NULL,       /* Point pCPAL_TransferTx to a Null pointer */ 
                                        0,       
	                                      0,
	                                      0,
	                                      0,
                                        I2C_STATE_DISABLED,                 /* Device Disabled */
                                        I2C_ERR_NONE,                   /* No Device Error */
                                        ((uint32_t)0x00000000),              /* No Options selected */ 
                                        ((uint32_t)I2C_TIMEOUT_DEFAULT)/* Set timeout value to I2C_TIMEOUT_DEFAULT */ 
                                        };          
  



DMA_InitTypeDef I2C_DMA_InitStructure;

I2C_TypeDef* I2C_DEVICE[1] = {I2C1};

const uint32_t I2C_CLK[1] = {I2C1_CLK};
const uint32_t I2C_DR[1] = {I2C1_DR};
const uint32_t I2C_AF[1] = {I2C1_AF};

const GPIO_TypeDef* I2C_SCL_GPIO_PORT[3] = {I2C1_SCL_GPIO_PORT, I2C2_SCL_GPIO_PORT, I2C3_SCL_GPIO_PORT};
const uint16_t I2C_SCL_GPIO_PIN[3] = {I2C1_SCL_GPIO_PIN, I2C2_SCL_GPIO_PIN, I2C3_SCL_GPIO_PIN};
const uint32_t I2C_SCL_GPIO_CLK[3] = {I2C1_SCL_GPIO_CLK, I2C2_SCL_GPIO_CLK, I2C3_SCL_GPIO_CLK};
const uint16_t I2C_SCL_GPIO_PINSOURCE[3] = {I2C1_SCL_GPIO_PINSOURCE, I2C2_SCL_GPIO_PINSOURCE, I2C3_SCL_GPIO_PINSOURCE};

const GPIO_TypeDef* I2C_SDA_GPIO_PORT[3] = {I2C1_SDA_GPIO_PORT,I2C2_SDA_GPIO_PORT,I2C3_SDA_GPIO_PORT};
const uint16_t I2C_SDA_GPIO_PIN[3] = {I2C1_SDA_GPIO_PIN,I2C2_SDA_GPIO_PIN,I2C3_SDA_GPIO_PIN};
const uint32_t I2C_SDA_GPIO_CLK[3] = {I2C1_SDA_GPIO_CLK,I2C2_SDA_GPIO_CLK,I2C3_SDA_GPIO_CLK};
const uint16_t I2C_SDA_GPIO_PINSOURCE[3] = {I2C1_SDA_GPIO_PINSOURCE,I2C2_SDA_GPIO_PINSOURCE,I2C3_SDA_GPIO_PINSOURCE};

const uint32_t I2C_DMA_CLK[1] = {I2C1_DMA_CLK};


const IRQn_Type I2C_DMA_TX_IRQn[1] = {I2C1_DMA_TX_IRQn};
const IRQn_Type I2C_DMA_RX_IRQn[1] = {I2C1_DMA_RX_IRQn};

const IRQn_Type I2C_IT_EVT_IRQn[1] = {I2C1_IT_EVT_IRQn};
const IRQn_Type I2C_IT_ERR_IRQn [1] = {I2C1_IT_ERR_IRQn};


DMA_TypeDef* I2C_DMA[1] = {I2C1_DMA}; 

const uint32_t I2C_DMA_TX_TC_FLAG[1] = {I2C1_DMA_TX_TC_FLAG};
const uint32_t I2C_DMA_TX_HT_FLAG[1] = {I2C1_DMA_TX_HT_FLAG};
const uint32_t I2C_DMA_TX_TE_FLAG[1] = {I2C1_DMA_TX_TE_FLAG};

const uint32_t I2C_DMA_RX_TC_FLAG[1] = {I2C1_DMA_RX_TC_FLAG};
const uint32_t I2C_DMA_RX_HT_FLAG[1] = {I2C1_DMA_RX_HT_FLAG};
const uint32_t I2C_DMA_RX_TE_FLAG[1] = {I2C1_DMA_RX_TE_FLAG};


/*================== CPAL_I2C_HAL_Config ==================*/

/**
  * @brief  Reset then enable the I2C device clock.
  * @param  Device : I2C Device instance. 
  * @retval None
  */
void I2C_HAL_CLKInit(I2C_DevTypeDef Device)
{    
  /* Reset I2Cx device clock in order to avoid non-cleared error flags */
  RCC_APB1PeriphResetCmd(I2C_CLK [Device],ENABLE);
  RCC_APB1PeriphResetCmd(I2C_CLK [Device],DISABLE);  

  /* Enable I2Cx device clock */
 RCC_APB1PeriphClockCmd(I2C_CLK [Device], ENABLE);  
}


/**
  * @brief  Reset then disable the I2C device clock.
  * @param  Device : I2C Device instance 
  * @retval None. 
  */
void I2C_HAL_CLKDeInit(I2C_DevTypeDef Device)
{   
  /* Reset I2Cx device clock in order to avoid non-cleared error flags */
  RCC_APB1PeriphResetCmd(I2C_CLK[Device],ENABLE);
  RCC_APB1PeriphResetCmd(I2C_CLK[Device],DISABLE);
	
  /* Disable I2Cx device clock */
  RCC_APB1PeriphClockCmd(I2C_CLK[Device], DISABLE);   
}


/**
  * @brief  Configure the IO pins used by the I2C device.
  * @param  Device : I2C Device instance. 
  * @retval None. 
  */
void I2C_HAL_GPIOInit(I2C_DevTypeDef Device)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable I2Cx SCL and SDA Pin Clock */
   RCC_AHB1PeriphClockCmd(I2C_SCL_GPIO_CLK[Device] | I2C_SDA_GPIO_CLK[Device], ENABLE); 
  
  /* Connect PXx to I2C_SCL */
  GPIO_PinAFConfig((GPIO_TypeDef*)I2C_SCL_GPIO_PORT[Device],I2C_SCL_GPIO_PINSOURCE[Device],I2C_AF[Device]);
  
  /* Connect PXx to I2C_SDA */
  GPIO_PinAFConfig((GPIO_TypeDef*)I2C_SDA_GPIO_PORT[Device],I2C_SDA_GPIO_PINSOURCE[Device],I2C_AF[Device]); 
  
  /* Set GPIO frequency to 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* Select Alternate function mode */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  
  /* Select output Open Drain type */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  
  /* Disable internal Pull-up */
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  
  /* Initialize I2Cx SCL Pin */ 
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN[Device];
  
  GPIO_Init((GPIO_TypeDef*)I2C_SCL_GPIO_PORT[Device], &GPIO_InitStructure);
  
  /* Initialize I2Cx SDA Pin */
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN[Device];
  
  GPIO_Init((GPIO_TypeDef*)I2C_SDA_GPIO_PORT[Device], &GPIO_InitStructure);     
}


/**
  * @brief  Deinitialize the IO pins used by the I2C device 
  *         (configured to their default state).
  * @param  Device : I2C Device instance. 
  * @retval None. 
  */
void I2C_HAL_GPIODeInit(I2C_DevTypeDef Device)
{      
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Set GPIO frequency to 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* Select Input floating mode */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  
  /* Deinitialize I2Cx SCL Pin */ 
  GPIO_InitStructure.GPIO_Pin = I2C_SCL_GPIO_PIN[Device];
  
  GPIO_Init((GPIO_TypeDef*)I2C_SCL_GPIO_PORT[Device], &GPIO_InitStructure);
  
  /* Deinitialize I2Cx SDA Pin */
  GPIO_InitStructure.GPIO_Pin = I2C_SDA_GPIO_PIN[Device];
  
  GPIO_Init((GPIO_TypeDef*)I2C_SDA_GPIO_PORT[Device], &GPIO_InitStructure); 
}



#ifdef I2C_DMA_PROGMODEL
/**
  * @brief  Enable the DMA clock and initialize needed DMA Streams 
  *         used by the I2C device.
  * @param  Device : I2C Device instance.
  * @param  Direction : Transfer direction.
  * @param  Options :  Transfer Options.
  * @retval None. 
  */             
void I2C_HAL_DMAInit(i2c_dev_t* i2c_dev)
{  
  /* Enable I2Cx DMA */
  RCC_AHB1PeriphClockCmd(I2C_DMA_CLK[i2c_dev->dev], ENABLE);
  
  /* I2Cx Common Stream Configuration */
  I2C_DMA_InitStructure.DMA_Channel = i2c_dev->DMA_Channel;
  I2C_DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  I2C_DMA_InitStructure.DMA_BufferSize = 0;
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
  I2C_DMA_InitStructure.DMA_PeripheralBaseAddr = I2C_DR [i2c_dev->dev];
  
  /* If TX Direction (Transmission) selected */
  if ((i2c_dev->direction & I2C_DIRECTION_TX) != 0)
  {         
    /* Select Memory to Peripheral transfer direction */
    I2C_DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    
    /* Initialize I2Cx DMA Tx Stream */
    DMA_Init(i2c_dev->DMA_TX_Stream, &I2C_DMA_InitStructure);   
  }
  
  /* If RX Direction (Reception) selected */
  if ((i2c_dev->direction & I2C_DIRECTION_RX ) != 0)
  {  
    /* Select Peripheral to Memory transfer direction */
    I2C_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    
    /* Initialize I2Cx DMA Rx Stream */
    DMA_Init(i2c_dev->DMA_RX_Stream, &I2C_DMA_InitStructure);   
  }
}
    

/**
  * @brief  Configure the DMA Stream specific for TX transfer.
  * @param  Device : I2C Device instance.
  * @param  TXferStruct : DMA TX Transfer Parameters.
  * @param  Options :  Transfer Options.
  * @retval None. 
  */
void I2C_HAL_DMATXConfig(i2c_dev_t* i2c_dev)
{
  /* Set Channel */
  I2C_DMA_InitStructure.DMA_Channel = i2c_dev->DMA_Channel;
  
  /* Set Memory Base Address */
  I2C_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(i2c_dev->buffer);
  
  /* Set number of data */
  I2C_DMA_InitStructure.DMA_BufferSize = i2c_dev->bytes_to_write;
  
  /* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
  I2C_DMA_InitStructure.DMA_PeripheralBaseAddr = I2C_DR [i2c_dev->dev];
  
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


/**
  * @brief  Configure the DMA Stream specific for RX transfer.
  * @param  Device : I2C Device instance.
  * @param  RXferStruct : DMA RX Transfer Parameters.
  * @param  Options :  Transfer Options.
  * @retval None. 
  */
void I2C_HAL_DMARXConfig(i2c_dev_t* i2c_dev)
{
  /* Set Channel */
  I2C_DMA_InitStructure.DMA_Channel = i2c_dev->DMA_Channel;
  
  /* Set Memory Base Address */
  I2C_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(i2c_dev->buffer);
  
  /* Set number of data */
  I2C_DMA_InitStructure.DMA_BufferSize = i2c_dev->bytes_to_read;
  
  /* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
  I2C_DMA_InitStructure.DMA_PeripheralBaseAddr = I2C_DR [i2c_dev->dev];
  
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
  * @brief  Deinitialize the DMA Stream used by I2C Device(configured to their default state).
  *         DMA clock is not disabled. 
  * @param  Device : I2C Device instance.
  * @param  Direction : Transfer direction.
  * @retval None. 
  */
void I2C_HAL_DMADeInit(i2c_dev_t* i2c_dev)
{
  /* If TX Direction (Transmission) selected */
  if ((i2c_dev->direction & I2C_DIRECTION_TX) != 0)
  {
    /* Deinitialize I2Cx DMA Tx Stream */
    DMA_DeInit(i2c_dev->DMA_TX_Stream);  
  }
  
  /* If RX Direction (Reception) selected */
  if ((i2c_dev->direction & I2C_DIRECTION_RX) != 0)
  {
    /* Deinitialize I2Cx DMA Rx Stream */
    DMA_DeInit(i2c_dev->DMA_RX_Stream);  
  }  
}  
#endif /* I2C_DMA_PROGMODEL */


/**
  * @brief  Configure NVIC and interrupts used by I2C Device according to 
  *         enabled options
  * @param  Device : I2C Device instance.
  * @param  Options : I2C Transfer Options.
  * @retval None. 
  */
void I2C_HAL_ITInit(i2c_dev_t* i2c_dev)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  /* Configure NVIC priority Group */ 
  NVIC_PriorityGroupConfig (I2C_NVIC_PRIOGROUP);
  
  /* Enable the IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  /* Configure NVIC for I2Cx EVT Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = I2C_IT_EVT_IRQn [i2c_dev->dev] ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_EVT_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_EVT_SUBPRIO;
  NVIC_Init(&NVIC_InitStructure);
  
  /* If I2C ERR Interrupt Option Bit not selected */ 
  if ((i2c_dev->options & OPT_I2C_ERRIT_DISABLE) == 0)    
  {
    /* Configure NVIC for I2Cx ERR Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_IT_ERR_IRQn  [i2c_dev->dev] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_ERR_PREPRIO;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_ERR_SUBPRIO;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable I2C Error Interrupts */
    i2c_dev->I2C->CR2 |= I2C_CR2_ITERREN ;
  }
  
#ifdef I2C_DMA_PROGMODEL
  if (i2c_dev->mode == I2C_PROGMODEL_DMA)
  {
    if ( (i2c_dev->direction & I2C_DIRECTION_TX) != 0)
    {   
      /* Configure NVIC for DMA TX channel interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn [i2c_dev->dev] ;
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
    }  
    if ((i2c_dev->direction & I2C_DIRECTION_RX) != 0)

    {
      /* Configure NVIC for DMA RX channel interrupt */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn [i2c_dev->dev] ;
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
#endif /* I2C_DMA_PROGMODEL */ 
  
}



/**
  * @brief  Deinitialize NVIC and interrupts used by I2C Device in 
  *         the current Configuration.
  * @param  Device : I2C Device instance.
  * @param  Options : I2C Transfer Options.
  * @retval None. 
  */
void I2C_HAL_ITDeInit(i2c_dev_t* i2c_dev)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  
  /* Disable the IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
   
  /* Disable I2Cx EVT IRQn */
  NVIC_InitStructure.NVIC_IRQChannel = I2C_IT_EVT_IRQn [i2c_dev->dev] ;
  NVIC_Init(&NVIC_InitStructure);
  
  /* If I2C ERR Interrupt Option Bit Deselected */ 
  if ((i2c_dev->options & OPT_I2C_ERRIT_DISABLE) == 0)    
  {
    /* Disable I2Cx ERR IRQn */ 
    NVIC_InitStructure.NVIC_IRQChannel = I2C_IT_ERR_IRQn  [i2c_dev->dev] ;
    NVIC_Init(&NVIC_InitStructure);
  }
  
#ifdef I2C_DMA_PROGMODEL
  if (i2c_dev->mode == I2C_PROGMODEL_DMA)

  {
    if ( (i2c_dev->direction & I2C_DIRECTION_TX) != 0)
    {      
      /* Disable I2Cx DMA TX IRQn */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn [i2c_dev->dev] ;
      NVIC_Init(&NVIC_InitStructure);
    }
    
    if ( (i2c_dev->direction & I2C_DIRECTION_RX) != 0)
    { 
      /* Disable I2Cx DMA RX IRQn */
      NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn [i2c_dev->dev] ;
      NVIC_Init(&NVIC_InitStructure);
    }  
  }
#endif /* I2C_DMA_PROGMODEL */
}

/**
  * @brief  This function handles I2C1 interrupt request.
  * @param  None. 
  * @retval RT_EOK. 
  */
uint32_t I2C1_EV_IRQHandler(void)
{  
  /* Call the Common Event handler function */
  return I2C_EV_IRQHandler(&i2c1_dev);
}


/**
  * @brief  This function handles I2C1 Errors interrupt.
  * @param  None. 
  * @retval RT_EOK. 
  */
uint32_t I2C1_ER_IRQHandler(void)
{
  I2C_LOG("\n\r\n\rLOG <I2C1_ER_IRQHandler> : I2C1 Device Error IT ");
  
  /* Call the Common Error handler function */
  I2C_ER_IRQHandler(&i2c1_dev);
  
  return RT_EOK;  
}

 #ifdef I2C_DMA_PROGMODEL
/**
  * @brief  This function handles I2C1 TX DMA interrupt request.
  * @param  None. 
  * @retval RT_EOK. 
  */
uint32_t I2C1_DMA_TX_IRQHandler(void)
{
  /* Call the Common DMA TX handler function */
  return I2C_DMA_TX_IRQHandler(&i2c1_dev);
}


/**
  * @brief  This function handles I2C1 RX DMA interrupt request.
  * @param  None. 
  * @retval RT_EOK. 
  */
uint32_t I2C1_DMA_RX_IRQHandler(void)
{
  /* Call the Common DMA RX handler function */
  return I2C_DMA_RX_IRQHandler(&i2c1_dev);
}
 #endif /* I2C_DMA_PROGMODEL */






