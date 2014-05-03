/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-02-28     Bernard      Add Configurator tag
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtthread.h>
#include <stm32f4xx.h>

/* board configuration */

// <o> Internal SRAM memory size[Kbytes] <8-64>
//	<i>Default: 64
#define STM32_SRAM_SIZE         128
#define STM32_SRAM_END          (0x20000000 + STM32_SRAM_SIZE * 1024)

#define RT_USING_UART1
//#define RT_USING_UART2
//#define RT_USING_UART3
#define RT_USING_UART6


void rt_hw_board_init(void);


void rt_hw_usart_init(void);

rt_uint32_t rt_hw_tick_get_microsecond(void);
rt_uint32_t rt_hw_tick_get_millisecond(void);

#ifdef __cplusplus
}
#endif

#endif
