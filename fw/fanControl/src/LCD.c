
#include "LCD.h"

#include "Timer.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_dma.h"


// LCD pin defs
#define LCD_RS_PIN		GPIO_Pin_8
#define LCD_CTRL_PINS	(LCD_RS_PIN)
#define LCD_CTRL_PORT	GPIOA

#define LCD_E_PIN		GPIO_Pin_15
#define LCD_E_PORT		GPIOB

#define LCD_DATA_PINS	(0xFF)
#define LCD_DATA_PORT	GPIOA

#define LINE_LENGTH		40
#define LINE1OFFSET		1
#define LINE2OFFSET		(LINE1OFFSET + LINE_LENGTH + 1)
#define BUFFER_LENGTH	(LINE2OFFSET + LINE_LENGTH)
static uint16_t buffer[BUFFER_LENGTH];


static void initGPIO(void);
static void initTimer(void);
static void initDMA(void);
static void initCGRAM(void);

void LCD_Init(void)
{
	initGPIO();
	initTimer();
	initDMA();
	initCGRAM();

	timer_sleep(10);	// replace with DMA complete interrupt

	for (int i=0; i < BUFFER_LENGTH; i++) {
		buffer[i] = LCD_RS_PIN | (uint16_t)' ';
	}

	buffer[LINE1OFFSET-1] = 0x0080;
	buffer[LINE2OFFSET-1] = 0x00C0;

	LCD_Update();
	timer_sleep(10);	// replace with DMA complete interrupt

}

void LCD_Write(enum LCD_Line line, size_t position, char *str, size_t len)
{
	size_t offset = (size_t)(position + 1 + line * (1 + LINE_LENGTH));
	for (unsigned i=0; i < len; i++) {
		buffer[offset + i] = LCD_RS_PIN | (uint8_t)str[i];
	}
}

void LCD_Update(void)
{
	TIM_Cmd(TIM15, DISABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);

	DMA_SetCurrDataCounter(DMA1_Channel5, BUFFER_LENGTH);
	DMA_Cmd(DMA1_Channel5, ENABLE);

	TIM15->RCR = BUFFER_LENGTH-1;
	TIM15->EGR = TIM_PSCReloadMode_Immediate;
	TIM_Cmd(TIM15, ENABLE);
}

// private functions

static void initGPIO(void)
{
	// enable GPIO peripheral clocks
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// reset all outputs
	GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_PINS);
	GPIO_ResetBits(LCD_E_PORT, LCD_E_PIN);
	GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_PINS);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
	GPIO_Init(LCD_CTRL_PORT, &GPIO_InitStructure);

	for (int i = 0; i < 8; i++) {
		GPIO_InitStructure.GPIO_Pin = (uint32_t)((0x01) << i);
		GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);
	}

	// configure TIM15 output
	GPIO_InitStructure.GPIO_Pin = LCD_E_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(LCD_E_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(LCD_E_PORT, GPIO_PinSource15, GPIO_AF_1);

}

static void initTimer(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {
			.TIM_Prescaler = 48,	// 48 MHz clock / 48 = 1MHz
			.TIM_CounterMode = TIM_CounterMode_Up,
			.TIM_Period = 49,		// 50us period
			.TIM_ClockDivision = TIM_CKD_DIV1,
			.TIM_RepetitionCounter = 0,
	};
	TIM_TimeBaseInit(TIM15, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct = {
			.TIM_OCMode = TIM_OCMode_Timing,
			.TIM_OutputState = TIM_OutputState_Disable,
			.TIM_OutputNState = TIM_OutputNState_Disable,
			.TIM_Pulse = 11,
			.TIM_OCPolarity = TIM_OCPolarity_High,
			.TIM_OCNPolarity = TIM_OCNPolarity_High,
			.TIM_OCIdleState = TIM_OCIdleState_Reset,
			.TIM_OCNIdleState = TIM_OCNIdleState_Reset,
	};

	TIM_OC1Init(TIM15, &TIM_OCInitStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;		// low until match, high until overflow
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_Pulse = 24;

	TIM_OC2Init(TIM15, &TIM_OCInitStruct);

	TIM_CtrlPWMOutputs(TIM15, ENABLE);
	TIM_SelectOnePulseMode(TIM15, TIM_OPMode_Single);

	TIM_DMACmd(TIM15, TIM_DMA_CC1, ENABLE);
}

static void initDMA(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_InitTypeDef DMA_InitStruct = {
			.DMA_PeripheralBaseAddr = (uint32_t)&LCD_DATA_PORT->ODR,
			.DMA_MemoryBaseAddr = (uint32_t)&buffer,
			.DMA_DIR = DMA_DIR_PeripheralDST,
			.DMA_BufferSize = sizeof(buffer),
			.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
			.DMA_MemoryInc = DMA_MemoryInc_Enable,
			.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word,
			.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
			.DMA_Mode = DMA_Mode_Normal,
			.DMA_Priority = DMA_Priority_Medium,
			.DMA_M2M = DMA_M2M_Disable,
	};

	DMA_Init(DMA1_Channel5, &DMA_InitStruct);
}

static void initCGRAM(void)
{
	size_t len = 0;
	buffer[len++] = 0x0038;
	buffer[len++] = 0x0038;
	buffer[len++] = 0x0038;
	buffer[len++] = 0x0006;
	buffer[len++] = 0x000C;
	buffer[len++] = 0x0040;
	for (int i=0; i < 8; i++) {
		for (int j=7; j >= 0 ; j--) {
			if (i == j) {
				buffer[len++] = LCD_RS_PIN | (0x0E);
			} else {
				buffer[len++] = LCD_RS_PIN | (0x00);
			}
		}
	}

	DMA_SetCurrDataCounter(DMA1_Channel5, len);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	TIM15->RCR = len-1;
	TIM15->EGR = TIM_PSCReloadMode_Immediate;
	TIM_Cmd(TIM15, ENABLE);
}
