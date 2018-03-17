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

// ----- main() ---------------------------------------------------------------

/*

ADC: sample CH10-13 @ 200kHz, DMA to memory (double-buffered, flag for completion?)
TIM3: output PWM at 200kHz, update based on ADC or RPM values
TIM2: measure RPM, periodically report
TIM14: trigger display refresh with latest ADC/PWM out/RPM in values

 */

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
	char adc[16];
	char pwm[16];

	LCD_Write(LCD_Line1, 0, hello, strlen(hello));
	timer_sleep(10);
	LCD_Write(LCD_Line2, 0, test, strlen(test));

	char *loop = "\010\011\012\013\014\015\016\017\016\015\014\013\012\011\010\011\012\013\014\015\016\017";
	size_t offset = 0;

	RPM_Init();
	Control_Init();
	Sense_Init();

	uint16_t pwmValue = 180;

	for (;;) {
		pwmValue += 1;
		if (pwmValue >= 236) { pwmValue = 180; }
		Control_Set(2, pwmValue);
		timer_sleep(250);
		uint16_t adcValue = (ADC_GetConversionValue(ADC1) * 1125) / 4; // * 3V * 6x * 1000mV / (2^6);
		snprintf(adc, 16, "% 5dmV %d", adcValue, delta[1]);
		LCD_Write(LCD_Line1, 0, adc, 12);
		snprintf(pwm, 13, "% 3d%% % 6d", (100*pwmValue)/240, rpm[1]);
		LCD_Write(LCD_Line2, 0, pwm, 12);
		LCD_Write(LCD_Line2, 12, &loop[offset], 4);
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
		.ADC_DataAlign = ADC_DataAlign_Right,
		.ADC_ScanDirection = ADC_ScanDirection_Upward
    };

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_ClockModeConfig(ADC1, ADC_ClockMode_AsynClk);
    ADC_ChannelConfig(ADC1, ADC_Channel_11, ADC_SampleTime_7_5Cycles);

    ADC_GetCalibrationFactor(ADC1);
    ADC_Cmd(ADC1, ENABLE);
	ADC_StartOfConversion(ADC1);
}


void RPM_Init(void)
{
	// GPIO Config ------------------------

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// only for PA0

	for (int i = 0; i < 4; i++) {
		GPIO_InitStructure.GPIO_Pin = (uint32_t)((0x01) << i);
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	// for now to use PA0 button for testing
		GPIO_PinAFConfig(GPIOA, i, GPIO_AF_2);
	}

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





