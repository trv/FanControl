/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "diag/Trace.h"

#include "stm32f0xx_exti.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"


// ----------------------------------------------------------------------------
//
// Standalone STM32F0 empty sample (trace via NONE).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the NONE output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

void Button_Init(void);
void Output_Init(void);
void Timer_Init(void);

static bool isRunning = false;

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{
	// At this stage the system clock should have already been configured
	// at high speed.

	Button_Init();
	Output_Init();
	Timer_Init();

	// Infinite loop
	while (1)
	{
		// Add your code here.
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

void Output_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitTypeDef    initGPIO = {
    	.GPIO_Pin   = GPIO_Pin_0,
        .GPIO_Mode  = GPIO_Mode_OUT,
        .GPIO_Speed = GPIO_Speed_Level_1,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd  = GPIO_PuPd_NOPULL,
    };

    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	GPIO_Init(GPIOB, &initGPIO);

	initGPIO.GPIO_Pin = GPIO_Pin_9;
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);
	GPIO_Init(GPIOC, &initGPIO);

}

void Button_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitTypeDef    initGPIO = {
    	.GPIO_Pin   = GPIO_Pin_0,
        .GPIO_Mode  = GPIO_Mode_IN,
        .GPIO_Speed = GPIO_Speed_Level_1,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd  = GPIO_PuPd_NOPULL
    };

	GPIO_Init(GPIOA, &initGPIO);

    EXTI_InitTypeDef initEXTI = {
    	.EXTI_Line = EXTI_Line0,
        .EXTI_Mode = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Rising,
        .EXTI_LineCmd = ENABLE,
    };

	EXTI_Init(&initEXTI);

    NVIC_InitTypeDef initNVIC = {
        .NVIC_IRQChannel = EXTI0_1_IRQn,
        .NVIC_IRQChannelPriority = 2,
        .NVIC_IRQChannelCmd = ENABLE,
    };

    NVIC_Init(&initNVIC);
}

void Timer_Init(void)
{
    // PWM timer config
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    TIM_TimeBaseInitTypeDef initTIM;
    TIM_TimeBaseStructInit(&initTIM);
    initTIM.TIM_Prescaler = (SystemCoreClock/1000)-1;  // 1kHz clock
    initTIM.TIM_Period = 5000;      // 5000 / 1kHz clock = 5 seconds
    TIM_TimeBaseInit(TIM14, &initTIM);
    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef initNVIC = {
		.NVIC_IRQChannel = TIM14_IRQn,
        .NVIC_IRQChannelPriority = 1,
        .NVIC_IRQChannelCmd = ENABLE,
    };

    NVIC_Init(&initNVIC);
}

void TIM14_IRQHandler(void)
{
	if (TIM_GetFlagStatus(TIM14, TIM_FLAG_Update)) {
        TIM_ClearFlag(TIM14, TIM_FLAG_Update);
		TIM_Cmd(TIM14, DISABLE);

		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		isRunning = false;
	}
}

void EXTI0_1_IRQHandler(void)
{
	uint16_t source = EXTI->PR;
	if (source & EXTI_Line0) {
		EXTI->PR = EXTI_Line0;	// clear flag
		if (isRunning) {
			TIM_Cmd(TIM14, DISABLE);
			GPIO_ResetBits(GPIOB, GPIO_Pin_0);
			GPIO_ResetBits(GPIOC, GPIO_Pin_9);
			isRunning = false;
		} else {
            TIM_SetCounter(TIM14, 0);
		    TIM_Cmd(TIM14, ENABLE);
			GPIO_SetBits(GPIOB, GPIO_Pin_0);
			GPIO_SetBits(GPIOC, GPIO_Pin_9);
			isRunning = true;
		}
	}
}



