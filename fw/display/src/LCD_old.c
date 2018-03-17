
#include "LCD.h"

#include "Timer.h"
#include "stm32f10x_gpio.h"

// LCD pin defs
#define LCD_RS_PIN		GPIO_Pin_1
#define LCD_RW_PIN		GPIO_Pin_2
#define LCD_E_PIN		GPIO_Pin_3
#define LCD_CTRL_PINS	(LCD_RS_PIN | LCD_RW_PIN | LCD_E_PIN)
#define LCD_CTRL_PORT	GPIOC

#define LCD_DATA_PINS	(0xFF)
#define LCD_DATA_PORT	GPIOA


void LCD_Init(void)
{
	// enable GPIO peripheral clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// reset all outputs
	GPIO_ResetBits(LCD_CTRL_PORT, LCD_CTRL_PINS);
	GPIO_ResetBits(LCD_DATA_PORT, LCD_DATA_PINS);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

	GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN;
	GPIO_Init(LCD_CTRL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_RW_PIN;
	GPIO_Init(LCD_CTRL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_E_PIN;
	GPIO_Init(LCD_CTRL_PORT, &GPIO_InitStructure);

	for (int i = 0; i < 8; i++) {
		GPIO_InitStructure.GPIO_Pin = (0x01) << i;
		GPIO_Init(LCD_DATA_PORT, &GPIO_InitStructure);
	}

	// init and timings from datasheet
	LCD_WriteCmd(0x38);
	timer_sleep(41);
	LCD_WriteCmd(0x38);
	timer_sleep(1);


	LCD_WriteCmd(0x38);	// 8-bit, 2-line, 5x8 font
	timer_sleep(1);

	LCD_WriteCmd(0x01);	// clear display
	timer_sleep(16);

	LCD_WriteCmd(0x06);	// increment, no shift
	timer_sleep(1);

	LCD_WriteCmd(0x0C);	// display on, no cursor, no cursor blink
	timer_sleep(1);

	// custom chars
	LCD_WriteCmd(0x40);
	timer_sleep(1);

	for (int i=0; i < 8; i++) {
		for (int j=7; j >= 0 ; j--) {
			if (i == j) {
				LCD_WriteChar(0x0E);
			} else {
				LCD_WriteChar(0x00);
			}
			timer_sleep(1);
		}
	}

	LCD_WriteCmd(0x80);	// set DDRAM address to zero
	timer_sleep(1);

	LCD_WriteChar('H');
	timer_sleep(1);
	LCD_WriteChar('e');
	timer_sleep(1);
	LCD_WriteChar('l');
	timer_sleep(1);
	LCD_WriteChar('l');
	timer_sleep(1);
	LCD_WriteChar('o');
	timer_sleep(1);
	LCD_WriteChar(' ');
	timer_sleep(1);
	LCD_WriteChar('W');
	timer_sleep(1);
	LCD_WriteChar('o');
	timer_sleep(1);
	LCD_WriteChar('r');
	timer_sleep(1);
	LCD_WriteChar('l');
	timer_sleep(1);
	LCD_WriteChar('d');
	timer_sleep(1);
	LCD_WriteChar('!');
	timer_sleep(1);


	LCD_WriteCmd(0xC0);	// 2nd row DDRAM address
	timer_sleep(1);

	LCD_WriteChar('T');
	timer_sleep(1);
	LCD_WriteChar('e');
	timer_sleep(1);
	LCD_WriteChar('s');
	timer_sleep(1);
	LCD_WriteChar('t');
	timer_sleep(1);

	LCD_WriteChar(' ');
	timer_sleep(1);

	for (char i=0; i < 8; i++) {
		LCD_WriteChar(i);
		timer_sleep(1);
	}
}

void LCD_WriteCmd(uint8_t data)
{
	// clear RS, RW
	GPIO_ResetBits(LCD_CTRL_PORT, LCD_RS_PIN | LCD_RW_PIN);

	// set E high
	GPIO_SetBits(LCD_CTRL_PORT, LCD_E_PIN);

	// write data
	GPIO_Write(LCD_DATA_PORT, data);

	// set E low
	GPIO_ResetBits(LCD_CTRL_PORT, LCD_E_PIN);
}

void LCD_WriteChar(char data)
{
	// clear RW, set RS
	GPIO_ResetBits(LCD_CTRL_PORT, LCD_RW_PIN);
	GPIO_SetBits(LCD_CTRL_PORT, LCD_RS_PIN);

	// set E high
	GPIO_SetBits(LCD_CTRL_PORT, LCD_E_PIN);

	// write data
	GPIO_Write(LCD_DATA_PORT, data);

	// set E low
	GPIO_ResetBits(LCD_CTRL_PORT, LCD_E_PIN);
}


uint8_t LCD_Read(void)
{
	return 0;
}
