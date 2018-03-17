#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x_dbgmcu.h"

#include "Timer.h"
#include "LCD.h"

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[])
{
	DBGMCU_Config(DBGMCU_TIM15_STOP, ENABLE);

	timer_start();

	timer_sleep(100);

	LCD_Init();

	char *hello = "Hello World!";
	char *test = "Test \xF4 \010\011\012\013\014\015\016\017";

	LCD_Write(LCD_Line1, 0, hello, strlen(hello));
	timer_sleep(10);
	LCD_Write(LCD_Line2, 0, test, strlen(test));

	char *loop = "\010\011\012\013\014\015\016\017\016\015\014\013\012\011\010\011\012\013\014\015\016\017";
	size_t offset = 0;

	for (;;) {
		timer_sleep(250);
		LCD_Write(LCD_Line2, 7, &loop[offset], 8);
		offset = (offset + 1) % 14;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

