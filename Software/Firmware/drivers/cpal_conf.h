/**
  ******************************************************************************
  * @file    Project/STM32_I2C_Examples/STM32_I2C/Advanced_Example/i2c_conf.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_CONF_H
#define __I2C_CONF_H


/*=======================================================================================================================================
                                       I2C Firmware Functionality Configuration
=========================================================================================================================================*/

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*   -- Section 1 :                   **** I2Cx Device Selection ****

    Description: This section provide an easy way to select I2Cx devices in user application. 
                 Choosing device allows to save memory resources.
                 If you need I2C1 device, uncomment relative define: #define USE_I2C1.
                 All available I2Cx device can be used at the same time.
                 At least one I2C device should be selected. */
        
#if defined (USE_STM3210C_EVAL) || defined (USE_STM322xG_EVAL) || defined (USE_STM324xG_EVAL)
  #define USE_I2C1          /*<! Uncomment to use I2C1 device */
#endif
/* This define must be uncommented for this example to allow I2C library the use of I2C1 device when using
   STM3210C-EVAL, STM322xG-EVAL and STM324xG-EVAL evaluation boards */

#ifdef USE_STM32100E_EVAL
  #define USE_I2C2        /*<! Uncomment to use I2C2 device */
#endif
/* This define must be uncommented for this example to allow I2C library the use of I2C2 device when using
   STM32100E-EVAL (High-Density Value line) evaluation boards */

//#define USE_I2C3        /*<! Uncomment to use I2C3 device */

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 2 :                **** Transfer Options Configuration ****

    Description: This section allows user to enable/disable some Transfer Options. The benefits of these 
                 defines is to minimize the size of the source code  */                             


/* Enable the use of Master Mode */
#define I2C_MASTER_MODE  
/* This define must be uncommented for this example. In this example I2C1 device 
   is used as master to communicate with Slave components (IOE, EEPROM)*/


/* Enable the use of Slave Mode */
//#define I2C_SLAVE_MODE
/* Commenting or uncommenting this define will not affect software functionality 
   but code size will increase */

/* Enable Listen mode for slave device */
//#define I2C_LISTEN_MODE
/* This define must be commented for this example. */


/* Enable the use of DMA Programming Model */
#define I2C_DMA_PROGMODEL
/* This define must be uncommented for this example. Communication between I2C1 
   Device and IOE expander and EEPROM is handled by DMA */


/* Enable 1 Byte reception with DMA Programming Model */
/* NOTE : This option must be set if user will use DMA for receiving 1 Byte */
#define I2C_DMA_1BYTE_CASE
/* This define must be uncommented for this example. DMA can't be used to receive 
   one byte. This define enable switch from DMA programming Model to Interrupt Model 
   automatically for reading one byte */ 


/* Enable the use of IT Programming Model */
//#define I2C_IT_PROGMODEL
/* Commenting or uncommenting this define will not affect software functionality 
   but code size will increase */

/* Enable the use of 10Bit Addressing Mode */
//#define I2C_10BIT_ADDR_MODE
/* Commenting or uncommenting this define will not affect software functionality 
   but code size will increase */

/* Enable the use of 16Bit Address memory register option 
      !! This define is available only when I2C_MASTER_MODE is enabled !!  */
#define I2C_16BIT_REG_OPTION
/* This define must be uncommented for this example. the Addressing of the EEPROM
   Memory is performed with 16Bit memory address */


/* Select which Closing communication Method is used for master receiver */
/* !! WARNING: These two defines are EXCLUSIVE, only one define should be uncommented !*/

/* Method1 used for closing communication with master receiver: This method is for the case when
   the I2C interrupts have the highest priority in the application */
#define I2C_CLOSECOM_METHOD1 

/* Method2 used for closing communication with master receiver: This method is for the case when 
   the I2C interrupts do not have the highest priority in the application */
//#define I2C_CLOSECOM_METHOD2 

/* Critical section CallBack can be used only when Method2 of Closing communication for master receiver is enabled.
   Uncomment this line to use the I2C critical section callback (it disables then enables all interrupts) */
//#define USE_CRITICAL_CALLBACK

/* In this Example User can select one of this Method of closing communication
   for master receiver */
/*------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 3 :           **** UserCallbacks Selection and Configuration ****

    Description: This section provides an easy way to enable UserCallbacks and select type of Error UserCallbacks.
                 By default, All UserCallbacks are disabled (UserCallbacks are defined as void functions).
                 To implement a UserCallbacks in your application, comment the relative define and 
                 implement the callback body in your application file.*/
          
/* Error UserCallbacks Type : Uncomment to select UserCallbacks type. One type must be selected */
/* Note : if Error UserCallbacks are not used the two following defines must be commented 
 
   WARNING: These two defines are EXCLUSIVE, only one define should be uncommented ! 
 */
#define USE_SINGLE_ERROR_CALLBACK   /*<! select single UserCallbacks type */  
//#define USE_MULTIPLE_ERROR_CALLBACK /*<! select multiple UserCallbacks type */

/* Error UserCallbacks : To use an Error UserCallback comment the relative define */

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

/*------------------------------------------------------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 4 :         **** Configure Timeout method, TimeoutCallback ****

    Description: This section allows you to implement your own Timeout Procedure .
                 By default Timeout procedure is implemented with Systick timer and 
                 I2C_TIMEOUT_Manager is defined as SysTick_Handler.
                 */


//#define TIMEOUT_INIT()           SysTick_Config((SystemCoreClock / 1000));\
//                                       NVIC_SetPriority (SysTick_IRQn, 0) 
//                                       /*<! Configure and enable the systick timer
//                                       to generate an interrupt when counter value
//                                       reaches 0. In the Systick interrupt handler 
//                                       the Timeout Error function is called. Time base is 1 ms */

#define TIMEOUT_DEINIT()         SysTick->CTRL = 0        /*<! Disable the systick timer */ 


#define I2C_TIMEOUT_Manager       SysTick_Handler         /*<! This callback is used to handle Timeout error.
                                                                     When a timeout occurs I2C_TIMEOUT_UserCallback
                                                                     is called to handle this error */
#ifndef I2C_TIMEOUT_Manager
   void I2C_TIMEOUT_Manager(void);
#else   
   void SysTick_Handler(void);  
#endif /* I2C_TIMEOUT_Manager */ 

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
#define I2C_TIMEOUT_MIN            ((uint32_t)0x00000001)
#define I2C_TIMEOUT_DETECTED       ((uint32_t)0x00000000)

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*   -- Section 5 :             **** NVIC Priority Group Selection and Interrupt Priority Offset ****
  
  Description: This section allows user to select NVIC Priority Group and configure Interrupt Priority Offset. 
               To change I2C_NVIC_PRIOGROUP uncomment wanted Priority Group and comment others. Only one 
               define is possible. 
               By default Priority Offset of I2Cx device (ERR, EVT, DMA) are set to 0 */
                     
  
/*-----------NVIC Group Priority-------------*/
  
/* #define I2C_NVIC_PRIOGROUP      NVIC_PriorityGroup_0 */ /*!< 0 bits for preemption priority
                                                                       4 bits for subpriority */
  
/* #define I2C_NVIC_PRIOGROUP      NVIC_PriorityGroup_1 */ /*!< 1 bits for preemption priority
                                                                       3 bits for subpriority */
  
#define I2C_NVIC_PRIOGROUP      NVIC_PriorityGroup_2  /*!< 2 bits for preemption priority
                                                                       2 bits for subpriority */
  
/* #define I2C_NVIC_PRIOGROUP       NVIC_PriorityGroup_3 */ /*!< 3 bits for preemption priority
                                                                       1 bits for subpriority */
  
/* #define I2C_NVIC_PRIOGROUP       NVIC_PriorityGroup_4 */ /*!< 4 bits for preemption priority */
  
/*-----------Interrupt Priority Offset-------------*/

/* This defines can be used to decrease the Level of Interrupt Priority for I2Cx Device (ERR, EVT, DMA_TX, DMA_RX).
   The value of I2Cx_IT_OFFSET_SUBPRIO is added to I2Cx_IT_XXX_SUBPRIO and the value of I2Cx_IT_OFFSET_PREPRIO 
   is added to I2Cx_IT_XXX_PREPRIO (XXX: ERR, EVT, DMATX, DMARX). 
   I2Cx Interrupt Priority are defined in i2c_hal_stm32f10x.h file in Section 3  */

#define I2C1_IT_OFFSET_SUBPRIO          0      /* I2C1 SUB-PRIORITY Offset */ 
#define I2C1_IT_OFFSET_PREPRIO          0      /* I2C1 PREEMPTION PRIORITY Offset */ 

#define I2C2_IT_OFFSET_SUBPRIO          0      /* I2C2 SUB-PRIORITY Offset */ 
#define I2C2_IT_OFFSET_PREPRIO          0      /* I2C2 PREEMPTION PRIORITY Offset */

#define I2C3_IT_OFFSET_SUBPRIO          0      /* I2C3 SUB-PRIORITY Offset */ 
#define I2C3_IT_OFFSET_PREPRIO          0      /* I2C3 PREEMPTION PRIORITY Offset */  

/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*  -- Section 6 :                  **** I2C DEBUG Configuration ****

    Description: This section allow user to enable or disable I2C Debug option. Enabling this option provide 
                 to user an easy way to debug the application code. This option use I2C_LOG Macro that integrate 
                 printf function. User can retarget printf function to USART ( use hyperterminal), LCD Screen 
                 on ST Eval Board or development toolchain debugger.
                 In this example, the log is managed through printf function routed to USART peripheral and allowing
                 to display messages on Hyperterminal-like terminals. This is performed through redefining the 
                 function PUTCHAR_PROTOTYPE (depending on the compiler) as follows:
 
                   #ifdef __GNUC__
                // With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
                // set to 'Yes') calls __io_putchar() 
                    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
                   #else
                    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
                   #endif 
    
    WARNING      Be aware that enabling this feature may slow down the communication process, increase the code size
                 significantly, and may in some cases cause communication errors (when print/display mechanism is too slow)*/
                 

/* To enable I2C_DEBUG option uncomment the define below */
//#define I2C_DEBUG

#ifdef I2C_DEBUG
#define I2C_LOG(Str)                   printf(Str)
#include <stdio.h>                     /* This header file must be included when using I2C_DEBUG option   */
#else
#define I2C_LOG(Str)                   ((void)0)
#endif /* I2C_DEBUG */   


/*-----------------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------------------*/

/*********END OF I2C Firmware Functionality Configuration****************************************************************/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __I2C_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
