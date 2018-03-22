
#include <stdint.h>
#include <stddef.h>

enum LCD_Line {
	LCD_Line1,
	LCD_Line2,
};

void LCD_Init(void);

void LCD_Write(enum LCD_Line line, size_t position, char *str, size_t len);

void LCD_Update(void);
