#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#include "stm32f10x_dbgmcu.h"

#include "Timer.h"
#include "LCD.h"

#include "stm32f0xx_adc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"

void Sense_Init(void);
void RPM_Init(void);
void Control_Init(void);
void Display_Update(void);

#define NUM_CHANNELS 4

static volatile uint16_t rpmMax[NUM_CHANNELS] = {0};
static volatile uint16_t rpm[NUM_CHANNELS] = {0};
static volatile uint16_t lastCount[NUM_CHANNELS] = {0};
static volatile uint16_t delta[NUM_CHANNELS] = {0};
static volatile uint8_t valid[NUM_CHANNELS] = {0};

/*

ADC: sample CH10-13 @ 200kHz, DMA to memory (double-buffered, use half-full and complete flags)
TIM2-3: output PWM at 200kHz, update based on ADC or RPM values
TIMx?: timer for EXTI-triggered GPIO RPM sense pins (internal only)
TIM14: trigger display refresh with latest ADC/PWM out/RPM in values
TIM15: LCD DMA timing
 */

/*
TODO:
- pulse-skipping for 100% duty cycle? need to periodically output high to discharge cap
- better RPM algorithm (report 0 when pulses stop, nicer filtering, disable interrupts after each edge?)
- RPM feedback loop to control voltage based on desired RPM
- fan hotplug detection (add, remove, stalled, etc)

- UI?
- SMBUS?
 */

// start fans at full speed to measure max RPM for each channel
static volatile uint16_t target_ADC[NUM_CHANNELS] = {170, 170, 170, 170};
static int16_t errorP_ADC[NUM_CHANNELS] = {0};
static int16_t errorI_ADC[NUM_CHANNELS] = {0};
static uint16_t adcValue[2*NUM_CHANNELS] = {0};

static volatile uint32_t *PWM[NUM_CHANNELS] = {&(TIM3->CCR1), &(TIM3->CCR2), &(TIM3->CCR3), &(TIM3->CCR4)};

void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void)
{
	GPIOB->BSRR = GPIO_Pin_12;

	uint32_t flags = DMA1->ISR;
	size_t offset = (flags & DMA1_FLAG_TC1) << 1;
	DMA1->IFCR = flags;

	for (int i = 0; i < NUM_CHANNELS; i++) {

		errorP_ADC[i] = target_ADC[i] - adcValue[offset + i];
		errorI_ADC[i] += errorP_ADC[i];

		// prevent integral wind-up
		if (errorI_ADC[i] > 128*236) {
			errorI_ADC[i] = 128*236;
		} else if (errorI_ADC[i] < 0) {
			errorI_ADC[i] = 0;
		}

		int newPWM = (errorP_ADC[i]<<4) + (errorI_ADC[i]>>7);

		if (newPWM > 236) {
			newPWM = 236;
		} else if (newPWM < 0) {
			newPWM = 0;
		}

		*(PWM[i]) = newPWM;

	}
	GPIOB->BRR = GPIO_Pin_12;
}

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[])
{
	//DBGMCU_Config(DBGMCU_TIM15_STOP, ENABLE);

	timer_start();
	timer_sleep(100);

	LCD_Init();

	char *hello = " Hello World!";
	char *test = "Test \xF4 \010\011\012\013\014\015\016\017";

	LCD_Write(LCD_Line1, 0, hello, strlen(hello));
	LCD_Write(LCD_Line2, 0, test, strlen(test));
	LCD_Update();
	timer_sleep(250);

	size_t offset = 0;
	size_t i = 0;

	RPM_Init();
	Control_Init();
	Sense_Init();

	// Configure pin in output push/pull mode
	GPIO_InitTypeDef GPIO_InitStructure = {
			.GPIO_Pin = GPIO_Pin_12,
			.GPIO_Speed = GPIO_Speed_Level_2,
			.GPIO_Mode = GPIO_Mode_OUT,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
	};
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	for (;;) {
		if (i > 8) {
			if (i & 0x10) {
				target_ADC[0] = 60;
				target_ADC[1] = 113;
			} else {
				target_ADC[0] = 113;
				target_ADC[1] = 60;
			}
			target_ADC[2] = 85;
			target_ADC[3] = 85;
		}
		i++;
		timer_sleep(250);

		Display_Update();
		offset = (offset + 1) % 14;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

void Display_Update(void)
{
	char line1[17];
	char line2[17];

	uint8_t ADC_Max = 170;	// 12V / 6 = 2V, 2V / 3V * 255 = 170
	uint16_t PWM_Max = 240;

	uint8_t adc_target, adc_value, pwm_value, rpm_value;

	const char *lookup1 = "        \010\011\012\013\014\015\016\017";
	const char *lookup2 = "\010\011\012\013\014\015\016\017        ";

	for (int i = 0; i < NUM_CHANNELS; i++) {
		adc_target = 15 * target_ADC[i] / ADC_Max;
		adc_value = 15 * adcValue[i] / ADC_Max;
		pwm_value = 15 * *PWM[i] / PWM_Max;
		rpm_value = 15 * rpm[i] / rpmMax[i];
		snprintf(&line1[4*i], 5, "%c%c%c%c ", lookup1[adc_target], lookup1[adc_value], lookup1[pwm_value], lookup1[rpm_value]);
		snprintf(&line2[4*i], 5, "%c%c%c%c ", lookup2[adc_target], lookup2[adc_value], lookup2[pwm_value], lookup2[rpm_value]);
	}

	LCD_Write(LCD_Line1, 0, line1, 16);
	LCD_Write(LCD_Line2, 0, line2, 16);
	LCD_Update();
}


void Sense_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	// GPIO config
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	for (int i = 0; i < 4; i++) {
		GPIO_InitStructure.GPIO_Pin = (uint32_t)((0x01) << i);
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	}

	// DMA config

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStruct = {
			.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR,
			.DMA_MemoryBaseAddr = (uint32_t)adcValue,
			.DMA_DIR = DMA_DIR_PeripheralSRC,
			.DMA_BufferSize = 2*NUM_CHANNELS,
			.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
			.DMA_MemoryInc = DMA_MemoryInc_Enable,
			.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word,
			.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
			.DMA_Mode = DMA_Mode_Circular,
			.DMA_Priority = DMA_Priority_High,
			.DMA_M2M = DMA_M2M_Disable,
	};

	// ADC uses channel 1 if not remapped
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap_ADC1, DISABLE);
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);
	//DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_TE, ENABLE);

	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA_Cmd(DMA1_Channel1, ENABLE);

	// ADC config
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStructure = {
		.ADC_Resolution = ADC_Resolution_6b,
		.ADC_ContinuousConvMode = DISABLE,
		.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO,
		.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising,
		.ADC_DataAlign = ADC_DataAlign_Left,
		.ADC_ScanDirection = ADC_ScanDirection_Backward
    };

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_ClockModeConfig(ADC1, ADC_ClockMode_AsynClk);
    ADC_ChannelConfig(ADC1, ADC_Channel_13, ADC_SampleTime_7_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_12, ADC_SampleTime_7_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_11, ADC_SampleTime_7_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_10, ADC_SampleTime_7_5Cycles);

    ADC_GetCalibrationFactor(ADC1);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
	ADC_StartOfConversion(ADC1);
}

const uint32_t rpmPins[NUM_CHANNELS] = {GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_8, GPIO_Pin_9}; // config alternate EXTI handler for GPIO 0-3
const uint8_t rpm_exti_pinSource[NUM_CHANNELS] = {EXTI_PinSource13, EXTI_PinSource14, EXTI_PinSource8, EXTI_PinSource9};
GPIO_TypeDef *rpmPort = GPIOB;

void RPM_Init(void)
{
	// GPIO Config ------------------------
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// Configure pin in output push/pull mode
	GPIO_InitTypeDef GPIO_InitStructure = {
			.GPIO_Speed = GPIO_Speed_Level_2,
			.GPIO_Mode = GPIO_Mode_IN,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_UP,
	};

	for (int i = 0; i < NUM_CHANNELS; i++) {
		GPIO_InitStructure.GPIO_Pin = rpmPins[i];
		GPIO_Init(rpmPort, &GPIO_InitStructure);
	}

	// EXTI Config
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	for (int i = 0; i < NUM_CHANNELS; i++) {
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, rpm_exti_pinSource[i]);
	}

	EXTI_InitTypeDef EXTI_InitStruct = {
			.EXTI_Mode = EXTI_Mode_Interrupt,
			.EXTI_Trigger = EXTI_Trigger_Rising_Falling,
			.EXTI_LineCmd = ENABLE
	};

	for (int i = 0; i < NUM_CHANNELS; i++) {
		EXTI_InitStruct.EXTI_Line = rpmPins[i];
		EXTI_Init(&EXTI_InitStruct);
	}

	NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// TIM2 config - input capture
	// RPM up to 4000 = 133Hz pulses @ 2 per revolution (~7.5ms)
	// min RPM ~500 = 16 Hz (~62.5 ms)
	// resolution of +/-1% ~= 1ms

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
			.TIM_Prescaler = 48,	// 1000 kHz clock (1us)
			.TIM_CounterMode = TIM_CounterMode_Up,
			.TIM_Period = 0xFFFF,	// 65.536 ms period
			.TIM_ClockDivision = TIM_CKD_DIV1,
			.TIM_RepetitionCounter = 0,
	};
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	TIM_Cmd(TIM2, ENABLE);
}

void EXTI4_15_IRQHandler(void);
void EXTI4_15_IRQHandler(void)
{
	uint16_t count = TIM2->CNT;

	for (int i = 0; i < NUM_CHANNELS; i++) {
		if (EXTI_GetFlagStatus(rpmPins[i])) {
			if (valid[i] && (count - lastCount[i]) > 4000) {
				delta[i] = count - lastCount[i];
				uint16_t newRPM = 15000000/delta[i];
				rpm[i] = (newRPM + (rpm[i] * 3))/4;
				if (rpm[i] > rpmMax[i]) {
					rpmMax[i] = rpm[i];
				}
				lastCount[i] = count;
			} else {
				lastCount[i] = count;
				valid[i] = 1;
			}
			EXTI_ClearFlag(rpmPins[i]);
		}
	}
}

void Control_Init(void)
{
	// GPIO Config ------------------------

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	for (int i = 6; i < 10; i++) {
		GPIO_InitStructure.GPIO_Pin = (uint32_t)((0x01) << i);
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_PinAFConfig(GPIOC, i, GPIO_AF_0);
	}

	// Timer Config -----------------------

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
			.TIM_Prescaler = 0,	// 48 MHz clock
			.TIM_CounterMode = TIM_CounterMode_Up,
			.TIM_Period = (48 * 5) - 1,		// 200 kHz
			.TIM_ClockDivision = TIM_CKD_DIV1,
			.TIM_RepetitionCounter = 0,
	};
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct = {
			.TIM_OCMode = TIM_OCMode_PWM2,
			.TIM_OutputState = TIM_OutputState_Enable,
			.TIM_OutputNState = TIM_OutputNState_Disable,
			.TIM_Pulse = 0,
			.TIM_OCPolarity = TIM_OCPolarity_High,
			.TIM_OCNPolarity = TIM_OCNPolarity_High,
			.TIM_OCIdleState = TIM_OCIdleState_Set,
			.TIM_OCNIdleState = TIM_OCNIdleState_Set,
	};

	TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM_OCInitStruct);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM_OCInitStruct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Repetitive);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

	TIM_Cmd(TIM3, ENABLE);
}
