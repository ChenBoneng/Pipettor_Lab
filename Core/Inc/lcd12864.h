#ifndef LCD12864_H
#define LCD12864_H

#include "main.h"

void LCD12864_Init(void);
void LCD12864_Clear(void);
void LCD12864_Fill(uint8_t pattern);
void LCD12864_SetCursor(uint8_t row, uint8_t col);
void LCD12864_WriteString(uint8_t row, uint8_t col, const char *text);

#endif /* LCD12864_H */
