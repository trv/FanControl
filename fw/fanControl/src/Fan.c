
#include "Fan.h"
#include <stdlib.h>

#include "stm32f0xx_adc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"

static void Sense_Init(void);
static void RPM_Init(void);
static void Control_Init(void);
static uint16_t mV2ADC(uint16_t mV);
static uint16_t ADC2mV(uint16_t mV);

static volatile uint16_t rpmMax[FAN_NUM_CHANNELS] = {0};
static volatile uint16_t rpm[FAN_NUM_CHANNELS] = {0};
static volatile uint16_t lastCount[FAN_NUM_CHANNELS] = {0};
static volatile uint16_t delta[FAN_NUM_CHANNELS] = {0};
static volatile uint8_t valid[FAN_NUM_CHANNELS] = {0};

// start fans at full speed to measure max RPM for each channel
static volatile uint16_t target_ADC[FAN_NUM_CHANNELS] = {170, 170, 170, 170};
static int16_t errorP_ADC[FAN_NUM_CHANNELS] = {0};
static int16_t errorI_ADC[FAN_NUM_CHANNELS] = {0};
static uint16_t adcValue[2*FAN_NUM_CHANNELS] = {0};

static volatile uint32_t *PWM[FAN_NUM_CHANNELS] = {&(TIM3->CCR1), &(TIM3->CCR2), &(TIM3->CCR3), &(TIM3->CCR4)};

// Public API
void Fan_Init(void)
{
	RPM_Init();
	Control_Init();
	Sense_Init();\

	// for ADC timing debug
	// Configure pin in output push/pull mode
	GPIO_InitTypeDef GPIO_InitStructure = {
			.GPIO_Pin = GPIO_Pin_12,
			.GPIO_Speed = GPIO_Speed_Level_2,
			.GPIO_Mode = GPIO_Mode_OUT,
			.GPIO_OType = GPIO_OType_PP,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
	};
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET);
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Fan_setAllTarget_mV(uint16_t *target_mV)
{
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		target_ADC[i] = mV2ADC(target_mV[i]);
	}
}

void Fan_setAllTarget_RPM(uint16_t *target_RPM)
{
	// TODO!
	(void)target_RPM;
}

void Fan_getAllMeasured_mV(uint16_t *measured_mV)
{
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		measured_mV[i] = ADC2mV(adcValue[i]);
	}
}

void Fan_getAllMeasured_PWM(uint16_t *measured_PWM)
{
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		measured_PWM[i] = *PWM[i];
	}
}

void Fan_getAllMeasured_RPM(uint16_t *measured_RPM)
{
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		measured_RPM[i] = rpm[i];
	}
}

void Fan_getAllMax_RPM(uint16_t *max_RPM)
{
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		max_RPM[i] = rpmMax[i];
	}
}


#define R_HIGH		100		// in 100s of Ohms
#define R_LOW		33		// in 100s of Ohms
#define ADC_REF_mV	3197
#define ADC_COUNTS	255

static uint16_t mV2ADC(uint16_t mV)
{
	return ADC_COUNTS * (mV * R_LOW) / ((R_LOW + R_HIGH) * ADC_REF_mV);
}
static uint16_t ADC2mV(uint16_t adc)
{
	return adc * ADC_REF_mV * (R_LOW + R_HIGH) / (R_LOW * ADC_COUNTS);
}


void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void)
{
	GPIOB->BSRR = GPIO_Pin_12;

	uint32_t flags = DMA1->ISR;
	size_t offset = (flags & DMA1_FLAG_TC1) << 1;
	DMA1->IFCR = flags;

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {

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
			.DMA_BufferSize = 2*FAN_NUM_CHANNELS,
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
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE, ENABLE);

	NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA_Cmd(DMA1_Channel1, ENABLE);

	// ADC config
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStructure = {
		.ADC_Resolution = ADC_Resolution_8b,
		.ADC_ContinuousConvMode = DISABLE,
		.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO,
		.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising,
		.ADC_DataAlign = ADC_DataAlign_Right,
		.ADC_ScanDirection = ADC_ScanDirection_Backward
    };

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv2);
    ADC_ChannelConfig(ADC1, ADC_Channel_13, ADC_SampleTime_13_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_12, ADC_SampleTime_13_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_11, ADC_SampleTime_13_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_10, ADC_SampleTime_13_5Cycles);

    ADC_GetCalibrationFactor(ADC1);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
	ADC_StartOfConversion(ADC1);
}

const uint32_t rpmPins[FAN_NUM_CHANNELS] = {GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_8, GPIO_Pin_9}; // config alternate EXTI handler for GPIO 0-3
const uint8_t rpm_exti_pinSource[FAN_NUM_CHANNELS] = {EXTI_PinSource13, EXTI_PinSource14, EXTI_PinSource8, EXTI_PinSource9};
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
	GPIOB->BSRR = GPIO_Pin_11;

	uint16_t count = TIM2->CNT;

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
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
	GPIOB->BRR = GPIO_Pin_11;

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
