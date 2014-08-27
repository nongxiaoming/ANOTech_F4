
#ifndef __I2C_CONF_H
#define __I2C_CONF_H

#include <rtthread.h>
        

/* Enable the use of Master Mode */
#define I2C_MASTER_MODE  

/* Enable the use of Slave Mode */
//#define I2C_SLAVE_MODE

/* Enable Listen mode for slave device */
//#define I2C_LISTEN_MODE


/* Enable the use of DMA Programming Model */
#define I2C_DMA_PROGMODEL


/* Enable 1 Byte reception with DMA Programming Model */
#define I2C_DMA_1BYTE_CASE

/* Enable the use of IT Programming Model */
//#define I2C_IT_PROGMODEL

/* Enable the use of 10Bit Addressing Mode */
//#define I2C_10BIT_ADDR_MODE

/* Enable the use of 16Bit Address memory register option   */
//#define I2C_16BIT_REG_OPTION

/* Method1 used for closing communication with master receiver */
#define I2C_CLOSECOM_METHOD1 

/* Method2 used for closing communication with master receiver */
//#define I2C_CLOSECOM_METHOD2 

/* Critical section CallBack can be used only when Method2 of Closing communication for master receiver is enabled.
   Uncomment this line to use the I2C critical section callback (it disables then enables all interrupts) */
//#define USE_CRITICAL_CALLBACK
#define USE_SINGLE_ERROR_CALLBACK   /*<! select single UserCallbacks type */  

/* Single Error Callback */
//#define I2C_ERR_UserCallback       (void)
/* In this example I2C_ERR_UserCallback is used to handle communication error. 
     To enable this Error UserCallback USE_SINGLE_ERROR_CALLBACK define must be uncommented 
     and comment I2C_ERR_UserCallback define */

/* Multiple Error Callback */
#define I2C_BERR_UserCallback      (void)
#define I2C_ARLO_UserCallback      (void)
#define I2C_OVR_UserCallback       (void)
#define I2C_AF_UserCallback        (void)


/* Transfer UserCallbacks : To use a Transfer callback comment the relative define */
#define I2C_TX_UserCallback        (void)    
#define I2C_RX_UserCallback        (void)
//#define I2C_TXTC_UserCallback      (void)    
//#define I2C_RXTC_UserCallback      (void)
/* Communication with EEPROM is handled by I2C_TXTC_UserCallback and I2C_RXTC_UserCallback.
   To enable these UserCallback related define must be commented */

/* DMA Transfer UserCallbacks : To use a DMA Transfer UserCallbacks comment the relative define */
#define I2C_DMATXTC_UserCallback   (void)  
#define I2C_DMATXHT_UserCallback   (void)
#define I2C_DMATXTE_UserCallback   (void) 
#define I2C_DMARXTC_UserCallback   (void)  
#define I2C_DMARXHT_UserCallback   (void)
#define I2C_DMARXTE_UserCallback   (void)

/* Address Mode UserCallbacks : To use an Address Mode UserCallbacks comment the relative define */
#define I2C_GENCALL_UserCallback   (void)
#define I2C_DUALF_UserCallback     (void)

/* CriticalSectionCallback : Call User callback for critical section (should typically disable interrupts) */
#define EnterCriticalSection_UserCallback        __disable_irq
#define ExitCriticalSection_UserCallback         __enable_irq

/* Listen mode Callback : Used to handle communication in listen mode */
#define I2C_SLAVE_READ_UserCallback        (void)    
#define I2C_SLAVE_WRITE_UserCallback       (void)


void i2c_timeout(void);

/*#define I2C_TIMEOUT_UserCallback        (void)      */            /*<! Comment this line and implement the callback body in your 
                                                                      application in order to use the Timeout Callback. 
                                                                      It is strongly advised to implement this callback, since it
                                                                      is the only way to manage timeout errors. */

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



/*-----------NVIC Group Priority-------------*/

  
#define I2C_NVIC_PRIOGROUP      NVIC_PriorityGroup_2  /*!< 2 bits for preemption priority
                                                                       2 bits for subpriority */
  
/*-----------Interrupt Priority Offset-------------*/

#define I2C_IT_OFFSET_SUBPRIO          0      /* I2C SUB-PRIORITY Offset */ 
#define I2C_IT_OFFSET_PREPRIO          0      /* I2C PREEMPTION PRIORITY Offset */ 




//#define I2C_DEBUG

#ifdef I2C_DEBUG
#define I2C_LOG(Str)                   rt_kprintf(Str)
#include <stdio.h>                     /* This header file must be included when using I2C_DEBUG option   */
#else
#define I2C_LOG(Str)                   ((void)0)
#endif /* I2C_DEBUG */   



#endif /* __I2C_CONF_H */

