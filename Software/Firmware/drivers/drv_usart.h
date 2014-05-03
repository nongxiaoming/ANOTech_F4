/*
 * File      : usart.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-11-05     xiaonong      the first version
 */

#ifndef __DRV_USART_H
#define __DRV_USART_H

#include <rthw.h>
#include <rtthread.h>

/* UART GPIO define. */
#define UART1_GPIO_TX		GPIO_Pin_9
#define UART1_TX_PIN_SOURCE GPIO_PinSource9
#define UART1_GPIO_RX		GPIO_Pin_10
#define UART1_RX_PIN_SOURCE GPIO_PinSource10
#define UART1_GPIO			GPIOA
#define UART1_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART1	RCC_APB2Periph_USART1
#define UART1_TX_DMA		DMA1_Channel4
#define UART1_RX_DMA		DMA1_Channel5

#define UART2_GPIO_TX	    GPIO_Pin_5
#define UART2_TX_PIN_SOURCE GPIO_PinSource5
#define UART2_GPIO_RX	    GPIO_Pin_6
#define UART2_RX_PIN_SOURCE GPIO_PinSource6
#define UART2_GPIO	    	GPIOD
#define UART2_GPIO_RCC   	RCC_AHB1Periph_GPIOD
#define RCC_APBPeriph_UART2	RCC_APB1Periph_USART2

#define UART3_GPIO_TX		GPIO_Pin_8
#define UART3_TX_PIN_SOURCE GPIO_PinSource8
#define UART3_GPIO_RX		GPIO_Pin_9
#define UART3_RX_PIN_SOURCE GPIO_PinSource9
#define UART3_GPIO			GPIOD
#define UART3_GPIO_RCC   	RCC_AHB1Periph_GPIOD
#define RCC_APBPeriph_UART3	RCC_APB1Periph_USART3
#define UART3_TX_DMA		DMA1_Stream1
#define UART3_RX_DMA		DMA1_Stream3

#define UART6_GPIO_TX		GPIO_Pin_6
#define UART6_TX_PIN_SOURCE GPIO_PinSource6
#define UART6_GPIO_RX		GPIO_Pin_7
#define UART6_RX_PIN_SOURCE GPIO_PinSource7
#define UART6_GPIO			GPIOC
#define UART6_GPIO_RCC   	RCC_AHB1Periph_GPIOC
#define RCC_APBPeriph_UART6	RCC_APB2Periph_USART6
//#define UART6_TX_DMA		DMA1_Stream1
//#define UART6_RX_DMA		DMA1_Stream3

void rt_hw_usart_init(void);

#endif
