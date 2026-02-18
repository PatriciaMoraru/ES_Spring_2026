// I2C LCD device driver - wraps the LiquidCrystal_I2C library
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "dd_lcd.h"

static LiquidCrystal_I2C* lcd;
static uint8_t lcdCols;
static uint8_t lcdRows;
static uint8_t cursorCol;
static uint8_t cursorRow;

void ddLcdSetup(uint8_t address, uint8_t cols, uint8_t rows)
{
    lcdCols = cols;
    lcdRows = rows;
    cursorCol = 0;
    cursorRow = 0;

    static LiquidCrystal_I2C lcdInstance(address, cols, rows);
    lcd = &lcdInstance;

    lcd->init();
    lcd->backlight();
    lcd->clear();
}

void ddLcdClear()
{
    lcd->clear();
    cursorCol = 0;
    cursorRow = 0;
}

void ddLcdSetCursor(uint8_t col, uint8_t row)
{
    cursorCol = col;
    cursorRow = row;
    lcd->setCursor(col, row);
}

void ddLcdPrintChar(char c)
{
    if (c == '\n')
    {
        cursorCol = 0;
        cursorRow++;

        if (cursorRow >= lcdRows)
        {
            cursorRow = 0;
        }

        lcd->setCursor(cursorCol, cursorRow);
        return;
    }

    if (c == '\r')
    {
        cursorCol = 0;
        lcd->setCursor(cursorCol, cursorRow);
        return;
    }

    lcd->write(c);
    cursorCol++;

    if (cursorCol >= lcdCols)
    {
        cursorCol = 0;
        cursorRow++;

        if (cursorRow >= lcdRows)
        {
            cursorRow = 0;
        }

        lcd->setCursor(cursorCol, cursorRow);
    }
}

void ddLcdPrint(const char* str)
{
    while (*str)
    {
        ddLcdPrintChar(*str);
        str++;
    }
}
