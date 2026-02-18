// I2C LCD device driver
#ifndef DD_LCD_H
#define DD_LCD_H

#include <stdint.h>

void ddLcdSetup(uint8_t address, uint8_t cols, uint8_t rows);
void ddLcdClear();
void ddLcdSetCursor(uint8_t col, uint8_t row);
void ddLcdPrintChar(char c);
void ddLcdPrint(const char* str);

#endif
