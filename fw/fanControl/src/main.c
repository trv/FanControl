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
void Control_Set(uint8_t channel, uint16_t value);

static uint16_t rpm[4] = {0};
static uint16_t lastCount[4] = {0};
static uint16_t delta[4] = {0};
static uint8_t valid[4] = {0};

static uint16_t target_ADC = 67;	// 4.5V
static int errorP_ADC = 0;
static int errorI_ADC = 0;
static int errorD_ADC = 0;

// ----- main() ---------------------------------------------------------------

/*

ADC: sample CH10-13 @ 200kHz, DMA to memory (double-buffered, flag for completion?)
TIM3: output PWM at 200kHz, update based on ADC or RPM values
TIM2: measure RPM, periodically report
TIM14: trigger display refresh with latest ADC/PWM out/RPM in values

 */
void ADC1_COMP_IRQHandler(void);
void ADC1_COMP_IRQHandler(void)
{
	// read ADC conversion result, compare against target value, update PWM output
	static uint32_t lastSample = 0;
	uint32_t sample = ADC1->DR;

	errorP_ADC = target_ADC - sample;
	errorI_ADC += errorP_ADC;
	errorD_ADC = sample - lastSample;

	lastSample = sample;

	// with 1uF output cap:
	// int newPWM = 210 + errorP_ADC/16 + errorI_ADC/4096 + 0*errorD_ADC/64;

	// with more output cap:
	int newPWM = 0 + errorP_ADC*2 + errorI_ADC/128 + 0*errorD_ADC/64;
	if (newPWM > 236) {
		newPWM = 236;
	} else if (newPWM < 0) {
		newPWM = 0;
	}
	TIM3->CCR2 = newPWM;

	// prevent integral wind-up
	if (errorI_ADC/128 > 236) {
		errorI_ADC = 236*128;
	} else if (errorI_ADC < 0) {
		errorI_ADC = 0;
	}
}


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
	char adc[17];
	char pwm[17];

	LCD_Write(LCD_Line1, 0, hello, strlen(hello));
	timer_sleep(10);
	LCD_Write(LCD_Line2, 0, test, strlen(test));

	char *loop = "\010\011\012\013\014\015\016\017\016\015\014\013\012\011\010\011\012\013\014\015\016\017";
	size_t offset = 0;

	RPM_Init();
	Control_Init();
	Sense_Init();

	for (;;) {
		if (offset & 0x01) {
			target_ADC = 67;
		} else {
			target_ADC = 59;
		}
		timer_sleep(1000);
		uint16_t adcValue = (ADC_GetConversionValue(ADC1) * 68);
		snprintf(adc, 17, "%5dmV %3d %3lu  ", adcValue, errorP_ADC, TIM3->CCR2);
		LCD_Write(LCD_Line1, 0, adc, 16);
		snprintf(pwm, 17, "%3lu%% %6d %s", (100*TIM3->CCR2)/240, rpm[1], &loop[offset]);
		LCD_Write(LCD_Line2, 0, pwm, 16);

		//LCD_Write(LCD_Line2, 12, &loop[offset], 4);
		offset = (offset + 1) % 14;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

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

	// ADC config
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitTypeDef ADC_InitStructure = {
		.ADC_Resolution = ADC_Resolution_6b,
		.ADC_ContinuousConvMode = DISABLE,
		.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO,
		.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising,
		.ADC_DataAlign = ADC_DataAlign_Left,
		.ADC_ScanDirection = ADC_ScanDirection_Upward
    };

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_ClockModeConfig(ADC1, ADC_ClockMode_AsynClk);
    ADC_ChannelConfig(ADC1, ADC_Channel_12, ADC_SampleTime_7_5Cycles);

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

	NVIC_ClearPendingIRQ(ADC1_COMP_IRQn);
	NVIC_EnableIRQ(ADC1_COMP_IRQn);

    ADC_GetCalibrationFactor(ADC1);
    ADC_Cmd(ADC1, ENABLE);
	ADC_StartOfConversion(ADC1);
}

const uint32_t rpmPins[4] = {GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_8, GPIO_Pin_9};
GPIO_TypeDef *rpmPort = GPIOB;

void RPM_Init(void)
{
	// GPIO Config ------------------------
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	for (int i = 0; i < 4; i++) {
		GPIO_InitStructure.GPIO_Pin = rpmPins[i];
		GPIO_Init(rpmPort, &GPIO_InitStructure);
	}

	// EXTI Config
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

	EXTI_InitTypeDef EXTI_InitStruct = {
			.EXTI_Line = 0,
			.EXTI_LineCmd = 0,
			.EXTI_Mode = 0,
			.EXTI_Trigger = 0	// TODO: configure these values
	};

	EXTI_Init(&EXTI_InitStruct);


	// TIM2 config - input capture
	// RPM up to 4000 = 133Hz pulses @ 2 per revolution (~7.5ms)
	// min RPM ~500 = 16 Hz (~62.5 ms)
	// resolution of +/-1% ~= 1ms

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
			.TIM_Prescaler = 480,	// 100 kHz clock
			.TIM_CounterMode = TIM_CounterMode_Up,
			.TIM_Period = 0xFFFF,
			.TIM_ClockDivision = TIM_CKD_DIV1,
			.TIM_RepetitionCounter = 0,
	};
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	TIM_ICInitTypeDef TIM_ICInitStruct = {
			.TIM_Channel = TIM_Channel_1,
			.TIM_ICPolarity = TIM_ICPolarity_BothEdge,
			.TIM_ICSelection = TIM_ICSelection_DirectTI,
			.TIM_ICPrescaler = TIM_ICPSC_DIV1,
			.TIM_ICFilter = 3,
	};
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2, &TIM_ICInitStruct);

	// interrupt config
	uint16_t flags = TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4; // | TIM_IT_Update;
	TIM_ITConfig(TIM2, flags, ENABLE);
	TIM_ClearFlag(TIM2, flags);

	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);

	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void)
{

	if (TIM2->SR & TIM_FLAG_CC1) {
		uint16_t count = TIM2->CCR1;
		uint8_t i = 0;
		if (valid[i]) {
			delta[i] = count - lastCount[i];
			rpm[i] = 30000/delta[i];
			lastCount[i] = count;
		} else {
			lastCount[i] = count;
			valid[i] = 1;
		}
	}

	if (TIM2->SR & TIM_FLAG_CC2) {
		uint16_t count = TIM2->CCR2;
		uint8_t i = 1;
		if (valid[i] && (count - lastCount[i]) > 400) {
			delta[i] = count - lastCount[i];
			uint16_t newRPM = 1500000/delta[i];
			rpm[i] = (newRPM + (rpm[i] * 3))/4;
			lastCount[i] = count;
		} else {
			lastCount[i] = count;
			valid[i] = 1;
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

	for (int i = 7; i < 8; i++) {
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
			.TIM_Pulse = (48*5)/4,
			.TIM_OCPolarity = TIM_OCPolarity_High,
			.TIM_OCNPolarity = TIM_OCNPolarity_High,
			.TIM_OCIdleState = TIM_OCIdleState_Set,
			.TIM_OCNIdleState = TIM_OCNIdleState_Set,
	};

	//TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	//TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_SelectOnePulseMode(TIM3, TIM_OPMode_Repetitive);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);

	TIM_Cmd(TIM3, ENABLE);
}

void Control_Set(uint8_t channel, uint16_t value)
{
	switch (channel) {
	case 1: TIM_SetCompare1(TIM3, value); break;
	case 2: TIM_SetCompare2(TIM3, value); break;
	case 3: TIM_SetCompare3(TIM3, value); break;
	case 4: TIM_SetCompare4(TIM3, value); break;
	}
}





