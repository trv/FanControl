/*
RPM calcualtion

data:
	can be lower-resolution timer increments (16 bit per half-second = 10µs period = 100kHz)
	6000 RPM = 400Hz (at 4 edges per revolution) = 200 sample buffer maximum for half-second
	300 RPM = 20Hz = 10 samples in half-second window (50ms period)

	FIFO: store head/tail pointer, tail may be unnecessary if timestamps expire naturally
	Windowing: add fade-in and fade-out time
		aim to cover ~2 samples minimum?  100ms fade-in, 300ms full-on, 100ms fade-out
		straight trapezoidal ramp vs something fancier?
		need to calculate equivalent divisor for windowed samples - trapezoid would be 400ms

*/
#include "RPM.h"

#include <string.h>
#include <stdio.h>
#include "Fan.h"

#include "stm32f0xx_rcc.h"

#define MAX_LENGTH		256
#define TICKS_PER_REV	4

static volatile uint8_t event_index[FAN_NUM_CHANNELS];
static volatile uint16_t events[FAN_NUM_CHANNELS][MAX_LENGTH];
static volatile uint16_t rpm[FAN_NUM_CHANNELS] = {0};

static const uint32_t g_timeWindow_ticks = 25000;
static const uint32_t g_tickRate_us = 20;

static volatile uint16_t lastCount[FAN_NUM_CHANNELS] = {0};
static volatile uint8_t valid[FAN_NUM_CHANNELS] = {0};

const uint32_t rpmPins[FAN_NUM_CHANNELS] = {GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_8, GPIO_Pin_9}; // config alternate EXTI handler for GPIO 0-3
const uint8_t rpm_exti_pinSource[FAN_NUM_CHANNELS] = {EXTI_PinSource13, EXTI_PinSource14, EXTI_PinSource8, EXTI_PinSource9};
GPIO_TypeDef *rpmPort = GPIOB;

void RPM_Init()
{
	memset(event_index, 0, sizeof(event_index));
	memset(events, 0, sizeof(events));
	memset(rpm, 0, sizeof(rpm));
	memset(valid, 0, sizeof(valid));

	// GPIO Config ------------------------
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// Configure pin in output push/pull mode
	GPIO_InitTypeDef GPIO_InitStructure = {
			.GPIO_Speed = GPIO_Speed_Level_2,
			.GPIO_Mode = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_UP,
	};

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		GPIO_InitStructure.GPIO_Pin = rpmPins[i];
		GPIO_Init(rpmPort, &GPIO_InitStructure);
	}

	// EXTI Config
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, rpm_exti_pinSource[i]);
	}

	EXTI_InitTypeDef EXTI_InitStruct = {
			.EXTI_Mode = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising_Falling,
			.EXTI_LineCmd = ENABLE
	};

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		EXTI_InitStruct.EXTI_Line = rpmPins[i];
		EXTI_Init(&EXTI_InitStruct);
	}

	NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
	NVIC_SetPriority(EXTI4_15_IRQn, 3);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// TIM2 config - input capture
	// RPM up to 4000 = 133Hz pulses @ 2 per revolution (~7.5ms)
	// min RPM ~500 = 16 Hz (~62.5 ms)
	// resolution of +/-1% ~= 1ms

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
			.TIM_Prescaler = 960,	// 50 kHz clock (20us)
			.TIM_CounterMode = TIM_CounterMode_Up,
			.TIM_Period = 0xFFFF,	// 65.536 ms period
			.TIM_ClockDivision = TIM_CKD_DIV1,
			.TIM_RepetitionCounter = 0,
	};
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	TIM_SetCounter(TIM2, 0x8000);
	TIM_Cmd(TIM2, ENABLE);
}

#define FADE_DIV		3
#define FADE_COMP		(FADE_DIV - 1)
void RPM_getAllMeasured_RPM(uint16_t *measured_RPM)
{
	uint16_t now_ticks = TIM2->CNT;
	uint16_t fade_time = g_timeWindow_ticks / FADE_DIV;
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		if (valid[i]) {
			uint32_t count = 0;
			uint8_t index = event_index[i] - 1;
			while (index != event_index[i]) {
				uint16_t delta_ticks = now_ticks - events[i][index];
				if (delta_ticks < fade_time) {
					count += (1000 * delta_ticks + fade_time/2) / fade_time;
				} else if (delta_ticks < g_timeWindow_ticks - fade_time) {
					count += 1000;
				} else if (delta_ticks < g_timeWindow_ticks) {
					count += (1000 * (g_timeWindow_ticks - delta_ticks) + fade_time/2) / fade_time;
				} else if (count != 0) {
					break;
				}
				index--;
			}
			printf("ch%d: index=%d, count=%lu, now=%u\r\n", i, event_index[i], count, now_ticks);
			uint32_t divisor = (FADE_COMP*g_timeWindow_ticks*g_tickRate_us)/FADE_DIV;
			measured_RPM[i] = (count*15000 + divisor/2)/divisor;
		} else {
			measured_RPM[i] = 0;
		}
	}
}

void RPM_event(uint8_t channel, uint16_t timestamp)
{
	events[channel][event_index[channel]++] = timestamp;
}

void EXTI4_15_IRQHandler(void);
void EXTI4_15_IRQHandler(void)
{
	GPIOB->BSRR = GPIO_Pin_11;

	uint16_t count = TIM2->CNT;

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		if (EXTI_GetFlagStatus(rpmPins[i])) {
			if (!valid[i] || (uint16_t)(count - lastCount[i]) > (2500 / g_tickRate_us)) {
				events[i][event_index[i]++] = count;
				valid[i] = 1;
			}
			lastCount[i] = count;
			EXTI_ClearFlag(rpmPins[i]);
		}
	}
	GPIOB->BRR = GPIO_Pin_11;
}


#ifdef RPM_TEST
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
int main(int argc, char *argv[])
{
	uint16_t sim_RPM = 300;

	if (argc == 2) {
		sim_RPM = atoi(argv[1]);
	}

	uint16_t rpm[FAN_NUM_CHANNELS];
	RPM_Init(20, 25000);		// 20us * 25000 ticks = 0.5 seconds

	uint32_t now_ticks = 30000 * 65536;	// init current time at least a window past the start


	uint32_t period = (50000u * 65536u / sim_RPM) * 15;
	uint32_t jitter = period / 20;

	uint16_t meas_ticks = now_ticks >> 16;
	uint16_t meas_period = 100;

	// populate events
	bool even = true;
	uint32_t last_tick = 0;
	for (int i = 0; i < 500; i++) {
		uint16_t event_tick = 0;
		if (true) {
			event_tick = (now_ticks + rand() % jitter) >> 16;
		} else {
			event_tick = (now_ticks - rand() % jitter) >> 16;
		}
		even = !even;

		RPM_event(0, event_tick);
		printf("    event at %6d\n", event_tick);
		last_tick = now_ticks;
		now_ticks += period;

		// test reported RPM
		uint16_t delta = (now_ticks >> 16) - meas_ticks;
		while (delta > meas_period) {
			RPM_getAllMeasured_RPM(rpm, meas_ticks);
			printf("time: %8d\trpm: %4d\n", meas_ticks, rpm[0]);
			meas_ticks += meas_period;
			delta -= meas_period;
		}
	}

	printf("\nactual RPM: %4d\n", (65536u*50000u)/(period/15u));
	printf("actual period32: %u\n", period);
	printf("actual period16: %u\n", period>>16);

}
#endif
