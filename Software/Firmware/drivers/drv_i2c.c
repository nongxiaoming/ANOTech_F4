#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#ifdef RT_USING_I2C_BITOPS

/* GPIO define
SCL: I2C1_SCL PB6
SDA: I2C1_SDA PB7
*/
#define GPIO_PORT_I2C_SCL   GPIOB
#define RCC_I2C_SCL         RCC_AHB1Periph_GPIOB
#define PIN_I2C_SCL		    GPIO_Pin_6

#define GPIO_PORT_I2C_SDA   GPIOB
#define RCC_I2C_SDA         RCC_AHB1Periph_GPIOB
#define PIN_I2C_SDA		    GPIO_Pin_7

static struct rt_i2c_bus_device i2c_device;


static rt_err_t stm32_i2c_configure(struct rt_i2c_bus_device * device,
                                    struct rt_i2c_configuration* configuration)
{
    return RT_EOK;
}



static rt_size_t stm32_i2c_read(struct rt_i2c_bus_device * device,
                                struct rt_i2c_message * message,
                                rt_uint8_t * read_buffer,
                                rt_size_t size)
{
    uint16_t temp;

    /* Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C1, message->device_addr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C1, *message->device_offset);

    /* Test on EV8 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for read */
    I2C_Send7bitAddress(I2C1, message->device_addr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));


    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    I2C_AcknowledgeConfig(I2C1,DISABLE);

    *read_buffer++ = I2C_ReceiveData(I2C1);

    /* Test on EV7 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

    *read_buffer++ |= I2C_ReceiveData(I2C1);

    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C1, ENABLE);

    I2C_AcknowledgeConfig(I2C1,ENABLE);

    return size;
}

static rt_size_t stm32_i2c_write(struct rt_i2c_bus_device * device,
                                 struct rt_i2c_message * message,
                                 const rt_uint8_t * write_buffer,
                                 rt_size_t size)
{
    /* Send START condition */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C1, message->device_addr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the EEPROM's internal address to write to */
    I2C_SendData(I2C1, message->device_offset);

    /* Test on EV8 and clear it */
    while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the current byte */
    I2C_SendData(I2C1, (uint8_t)*write_buffer++);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the current byte */
    I2C_SendData(I2C1, (uint8_t)*write_buffer++);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(I2C1, ENABLE);

    return 0;
}

static rt_size_t stm32_i2c_recv_bytes(I2C_TypeDef* I2Cx, struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
    rt_size_t len = msg->len;

    while (len--)
    {
        while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

        msg->buf[bytes++] = (rt_uint8_t)I2Cx->DR;
    }

    return bytes;
}


static rt_size_t stm32_i2c_send_bytes(I2C_TypeDef* I2Cx, struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
    rt_size_t len = msg->len;

    while (len--)
    {
        I2Cx->DR = msg->buf[bytes++];
        while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    return bytes;
}


static void I2C_SendAddress(I2C_TypeDef* I2Cx, struct rt_i2c_msg *msg)
{
    rt_uint16_t addr;
    rt_uint16_t flags = msg->flags;
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    /* Test on the direction to set/reset the read/write bit */
    addr = msg->addr << 1;
    if (flags & RT_I2C_RD)
    {
        /* Set the address bit0 for read */
        addr |= 1;
    }
    /* Send the address */
    I2Cx->DR = addr;
}


static rt_size_t stm32_i2c_xfer(struct rt_i2c_bus_device *bus,
                                struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_int32_t i, ret;
    rt_uint16_t ignore_nack;

    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on EV5 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        ignore_nack = msg->flags & RT_I2C_IGNORE_NACK;
        if (!(msg->flags & RT_I2C_NO_START))
        {
            if (i)
            {
                I2C_GenerateSTART(I2C1, ENABLE);
            }
            I2C_SendAddress(I2C1, msg);
            if (msg->flags & RT_I2C_RD)
                while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
            else
                while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

        }
        if (msg->flags & RT_I2C_RD)
        {
            if (!(msg->flags & RT_I2C_NO_READ_ACK))
                I2C_AcknowledgeConfig(I2C1,ENABLE);
            else
                I2C_AcknowledgeConfig(I2C1,DISABLE);
            ret = stm32_i2c_recv_bytes(I2C1, msg);
            if (ret >= 1)
                i2c_dbg("read %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_EIO;
                goto out;
            }
        }
        else
        {
            ret = stm32_i2c_send_bytes(I2C1, msg);
            if (ret >= 1)
                i2c_dbg("write %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_ERROR;
                goto out;
            }
        }
    }
    ret = i;

out:
    i2c_dbg("send stop condition\n");
    I2C_GenerateSTOP(I2C1, ENABLE);

    return ret;
}


static const struct rt_i2c_bus_device_ops i2c1_ops =
{
    stm32_i2c_xfer,
    RT_NULL,
    RT_NULL
};

static struct rt_i2c_bus_device stm32_i2c1;

#endif

//void rt_hw_i2c_init(void)
//{
//    GPIO_InitTypeDef  GPIO_InitStructure;
//
//#ifdef RT_USING_I2C_BITOPS
//    RCC_AHB1PeriphClockCmd(RCC_I2C_SCL | RCC_I2C_SDA, ENABLE);
//
//    /* config SCL PIN */
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//
//    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL;
//    GPIO_SetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
//    GPIO_Init(GPIO_PORT_I2C_SCL, &GPIO_InitStructure);
//
//    /* config SDA PIN */
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//
//    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
//    GPIO_SetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
//    GPIO_Init(GPIO_PORT_I2C_SDA, &GPIO_InitStructure);
//
//    rt_memset((void *)&i2c_device, 0, sizeof(struct rt_i2c_bus_device));
//    i2c_device.priv = (void *)&bit_ops;
//    rt_i2c_bit_add_bus(&i2c_device, "i2c1");
//
//#else
//    I2C_InitTypeDef  I2C_InitStructure;
//
//    /*!< sEE_I2C Periph clock enable */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//
//    /*!< sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//
//    /* Reset sEE_I2C IP */
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
//
//    /* Release reset signal of sEE_I2C IP */
//    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
//
//    /*!< GPIO configuration */
//    /*!< Configure sEE_I2C pins: SCL */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//    /*!< Configure sEE_I2C pins: SDA */
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//    GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//    /* Connect PXx to I2C_SCL*/
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
//
//    /* Connect PXx to I2C_SDA*/
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
//
//
//    /*!< I2C configuration */
//    /* sEE_I2C configuration */
//    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
////	I2C_InitStructure.I2C_OwnAddress1 = SLA_ADDRESS;
//    I2C_InitStructure.I2C_OwnAddress1 = 0x18;
//    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//    I2C_InitStructure.I2C_ClockSpeed = 400000;
//
//    /* Apply sEE_I2C configuration after enabling it */
//    I2C_Init(I2C1, &I2C_InitStructure);
//    /* sEE_I2C Peripheral Enable */
//    I2C_Cmd(I2C1, ENABLE);
//
//    rt_memset((void *)&stm32_i2c1, 0, sizeof(struct rt_i2c_bus_device));
//    stm32_i2c1.ops = &i2c1_ops;
//
//    rt_i2c_bus_device_register(&stm32_i2c1, "i2c1");
//
//#endif
//}
//
///*******************************************************************************
//* Function Name  : I2C_EE_WaitOperationIsCompleted
//* Description    : wait operation is completed
//* Input          : None 
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void I2C_EE_WaitOperationIsCompleted(void)
//{
//	while(i2c_comm_state != COMM_DONE);
//}	
//
///**********************  Interrupt Service Routines	 **************************/
//
//void i2c1_evt_isr()
//{  
//  	uint32_t lastevent=  I2C_GetLastEvent(I2C1);
//    switch (lastevent)
//    {
///************************** Master Invoke**************************************/
//          case I2C_EVENT_MASTER_MODE_SELECT:        /* EV5 */
//            // MSL SB BUSY 30001
//            if(!check_begin)
//              i2c_comm_state = COMM_IN_PROCESS;
//            
//              if (MasterDirection == Receiver)
//              {
//                if (!OffsetDone)
//				#if (EE_ADDRESS_NUM>1)
//				I2C_Send7bitAddress(I2C1,SlaveADDR,I2C_Direction_Transmitter);
//				#else
//				I2C_Send7bitAddress(I2C1,((DeviceOffset & 0x0700) >>7) | SlaveADDR,
//									 I2C_Direction_Transmitter);
//				#endif
//                      
//                else
//                {
//                  /* Send slave Address for read */
//				  #if (EE_ADDRESS_NUM>1)
//				  I2C_Send7bitAddress(I2C1, SlaveADDR, I2C_Direction_Receiver); 
//				  #else
//                  I2C_Send7bitAddress(I2C1, ((DeviceOffset & 0x0700) >>7) | SlaveADDR, I2C_Direction_Receiver);      
//                  #endif
//				  OffsetDone = FALSE;
//
//                }
//              }
//              else
//              {
//                  /* Send slave Address for write */
//				  #if (EE_ADDRESS_NUM>1)
//				  I2C_Send7bitAddress(I2C1, SlaveADDR, I2C_Direction_Transmitter);
//				  #else
//				  I2C_Send7bitAddress(I2C1, ((DeviceOffset & 0x0700) >>7) | SlaveADDR, I2C_Direction_Transmitter);
//				  #endif
//                  
//              }
//              I2C_ITConfig(I2C1, I2C_IT_BUF , ENABLE);//also TxE int allowed
//              break;
//              
///********************** Master Receiver events ********************************/
//          case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:  /* EV6 */
//            // MSL BUSY ADDR 0x30002
//              I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
//              
//              //without it, no NAK signal on bus after last Data
//              //I2C data line would be hold low 
//              I2C_DMALastTransferCmd(I2C1, ENABLE);
//              
//              I2C_DMACmd(I2C1, ENABLE);
//              DMA_Cmd(DMA1_Channel7, ENABLE);            
//              break;
//      
//          case I2C_EVENT_MASTER_BYTE_RECEIVED:    /* EV7 */
//            // MSL BUSY RXNE 0x30040
//              break;
//      
///************************* Master Transmitter events **************************/
//          case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:     /* EV8 just after EV6 */
//            //BUSY, MSL, ADDR, TXE and TRA 0x70082
//            if (check_begin)
//			{
//				check_begin = FALSE;
//				I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF |I2C_IT_ERR, DISABLE);
//				I2C_GenerateSTOP(I2C1, ENABLE);
//				//!! write over
//				I2C_EE_WriteOnePageCompleted();
//				if(I2C_NumByteToWrite>0)
//				{
//					 I2C_EE_WriteOnePage(I2C_pBuffer,I2C_WriteAddr,I2C_NumByteToWrite);
//				}
//				else
//				{
//				    WriteComplete = 1;
//				    i2c_comm_state = COMM_DONE;
//					PV_flag_1 = 0;
//				}
//				
//				break;
//			}
//
//             #if (EE_ADDRESS_NUM>1)
//				EE_Addr_Count++;
//			  	if (EE_Addr_Count < (EE_ADDRESS_NUM))
//		        {
//					I2C_SendData(I2C1, DeviceOffset>>8);
//		        }
////				else
////				{
////					I2C_SendData(I2C1, DeviceOffset);
////				}
//			#else
//				I2C_SendData(I2C1, DeviceOffset);
//				OffsetDone = TRUE;
//			#endif
//          break;
//              
//          case I2C_EVENT_MASTER_BYTE_TRANSMITTING:       /* EV8 I2C_EVENT_MASTER_BYTE_TRANSMITTING*/
//              #if (EE_ADDRESS_NUM>1)
//			  if(!OffsetDone)
//              {
//                
//	            if (EE_Addr_Count < (EE_ADDRESS_NUM))
//	            {
//					//while ((I2C1->CR1 & 0x200) == 0x200); 
//	            	//I2C_GenerateSTART(I2C1, ENABLE);
//					I2C_SendData(I2C1, DeviceOffset);
//					EE_Addr_Count++;	
//	            }
//				else
//				{
//					EE_Addr_Count = 0;
//					OffsetDone = TRUE;
//				}
//				break;
//              }
//			  #endif
//			  if (MasterDirection == Receiver)
//              {
//                I2C_ITConfig(I2C1, I2C_IT_BUF , DISABLE);
//                while ((I2C1->CR1 & 0x200) == 0x200); 
//                I2C_GenerateSTART(I2C1, ENABLE);
//                break;
//              }
//              else
//              {
//                I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
//                I2C_DMACmd(I2C1, ENABLE);
//                DMA_Cmd(DMA1_Channel6, ENABLE);
//                break;
//              }
//      
//          case I2C_EVENT_MASTER_BYTE_TRANSMITTED:       /* EV8-2 */
//            //TRA, BUSY, MSL, TXE and BTF 0x70084
//              break;
//    }
//}
//
//
//void i2c1_err_isr()
//{
//    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_AF))
//    {
//      if (check_begin)
//        I2C_GenerateSTART(I2C1, ENABLE);
//      else if (I2C1->SR2 &0x01)
//      {	
//	  	//!! receive over
//        I2C_GenerateSTOP(I2C1, ENABLE);
//        i2c_comm_state = COMM_EXIT;
//        PV_flag_1 = 0;
//      }
//      
//      I2C_ClearFlag(I2C1, I2C_FLAG_AF);
//    }
//
//    if (I2C_GetFlagStatus(I2C1, I2C_FLAG_BERR))
//    {
//      if (I2C1->SR2 &0x01)
//      {
//        I2C_GenerateSTOP(I2C1, ENABLE);
//        i2c_comm_state = COMM_EXIT;
//        PV_flag_1 = 0;
//      }
//      
//      I2C_ClearFlag(I2C1, I2C_FLAG_BERR);
//    }
//}
//
//
//
//void i2c1_send_dma_isr()
//{
//
//  if (DMA_GetFlagStatus(DMA1_FLAG_TC6))
//  {
//    if (I2C1->SR2 & 0x01) // master send DMA finish, check process later
//    {
//      // DMA1-6 (I2C1 Tx DMA)transfer complete ISR
//      I2C_DMACmd(I2C1, DISABLE);
//      DMA_Cmd(DMA1_Channel6, DISABLE);
//      // wait until BTF
//      while (!(I2C1->SR1 & 0x04));
//      I2C_GenerateSTOP(I2C1, ENABLE);
//      // wait until BUSY clear
//      while (I2C1->SR2 & 0x02);
//    
//      MasterTransitionComplete=1;
//      i2c_comm_state = COMM_IN_PROCESS;
//      I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE); // use interrupt to handle check process
//      check_begin = TRUE;
//      if(I2C1->CR1 & 0x200)
//        I2C1->CR1 &= 0xFDFF;
//      I2C_GenerateSTART(I2C1, ENABLE);
//    }
//    else // slave send DMA finish
//    {
//
//    }
//
//    
//    DMA_ClearFlag(DMA1_FLAG_TC6);
//  }
//  if (DMA_GetFlagStatus(DMA1_FLAG_GL6))
//  {
//    DMA_ClearFlag( DMA1_FLAG_GL6);
//  }
//    if (DMA_GetFlagStatus(DMA1_FLAG_HT6))
//  {
//    DMA_ClearFlag( DMA1_FLAG_HT6);
//  }
//}
//
//void i2c1_receive_dma_isr()
//{
//  if (DMA_GetFlagStatus(DMA1_FLAG_TC7))
//  {
//    if (I2C1->SR2 & 0x01) // master receive DMA finish
//    {
//      I2C_DMACmd(I2C1, DISABLE);
//      I2C_GenerateSTOP(I2C1, ENABLE);
//      i2c_comm_state = COMM_DONE;
//      MasterReceptionComplete = 1;
//      PV_flag_1 =0;
//    }
//    else // slave receive DMA finish
//    {
//
//    }
//    DMA_ClearFlag(DMA1_FLAG_TC7);
//  }
//  if (DMA_GetFlagStatus(DMA1_FLAG_GL7))
//  {
//    DMA_ClearFlag( DMA1_FLAG_GL7);
//  }
//    if (DMA_GetFlagStatus(DMA1_FLAG_HT7))
//  {
//    DMA_ClearFlag( DMA1_FLAG_HT7);
//  }
//}
//
//
///*******************************************************************************
//* Function Name  : DMA1_Channel6_IRQHandler
//* Description    : This function handles DMA1 Channel 6 interrupt request.
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void DMA1_Channel6_IRQHandler(void)
//{
//  i2c1_send_dma_isr();
//}
//
///*******************************************************************************
//* Function Name  : DMA1_Channel7_IRQHandler
//* Description    : This function handles DMA1 Channel 7 interrupt request.
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void DMA1_Channel7_IRQHandler(void)
//{
//  i2c1_receive_dma_isr();
//}
//
///*******************************************************************************
//* Function Name  : I2C1_EV_IRQHandler
//* Description    : This function handles I2C1 Event interrupt request.
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void I2C1_EV_IRQHandler(void)
//{
//  i2c1_evt_isr();
//}
//
///*******************************************************************************
//* Function Name  : I2C1_ER_IRQHandler
//* Description    : This function handles I2C1 Error interrupt request.
//* Input          : None
//* Output         : None
//* Return         : None
//*******************************************************************************/
//void I2C1_ER_IRQHandler(void)
//{
//  i2c1_err_isr();
//}
