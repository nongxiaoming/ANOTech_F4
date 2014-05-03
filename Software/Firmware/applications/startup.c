/*
 * File      : startup.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://openlab.rt-thread.com/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2011-06-05     Bernard      modify for STM32F107 version
 * 2012-02-28     Bernard      modify for RTArduino
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "board.h"

/**
 * @addtogroup STM32
 */

/*@{*/

extern int  rt_application_init(void);

#ifdef __CC_ARM
extern int Image$$RW_SRAM$$ZI$$Limit;
#define HEAP_BEGIN    (&Image$$RW_SRAM$$ZI$$Limit)
#elif __ICCARM__
#error IAR not support yet.
#else
extern int __sram_available;
#define HEAP_BEGIN    (&__sram_available)
#endif

#define HEAP_END        STM32_SRAM_END

/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
	rt_kprintf("\n\r Wrong parameter value detected on\r\n");
	rt_kprintf("       file  %s\r\n", file);
	rt_kprintf("       line  %d\r\n", line);

	while (1) ;
}

/**
 * This function will startup RT-Thread RTOS.
 */
void rtthread_startup(void)
{
	/* initialize board */
	rt_hw_board_init();

	/* show version */
	rt_show_version();

#ifdef RT_USING_HEAP
    rt_system_heap_init((void*)HEAP_BEGIN, (void*)HEAP_END);
#endif

	/* initialize scheduler system */
	rt_system_scheduler_init();

	/* initialize application */
	rt_application_init();

    /* initialize timer thread */
    rt_system_timer_thread_init();
	/* initialize idle thread */
	rt_thread_idle_init();

	/* start scheduler */
	rt_system_scheduler_start();

	/* never reach here */
	return ;
}

int main(void)
{
	/* disable interrupt first */
	rt_hw_interrupt_disable();

	/* startup RT-Thread RTOS */
	rtthread_startup();

	return 0;
}

/*@}*/
