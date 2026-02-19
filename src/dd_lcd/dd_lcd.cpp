// I2C LCD device driver - wraps the LiquidCrystal_I2C library
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "dd_lcd.h"

// LCD object pointer and display dimensions
static LiquidCrystal_I2C* lcd;
static uint8_t lcdCols;
static uint8_t lcdRows;

// Internal cursor position tracking
static uint8_t cursorCol;
static uint8_t cursorRow;

// Initialize LCD: create instance, turn on backlight, clear screen
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

// Clear display and reset cursor to top-left
void ddLcdClear()
{
    lcd->clear();
    cursorCol = 0;
    cursorRow = 0;
}

// Move cursor to a specific position
void ddLcdSetCursor(uint8_t col, uint8_t row)
{
    cursorCol = col;
    cursorRow = row;
    lcd->setCursor(col, row);
}

// Print a single character, handling newline and auto-wrap
void ddLcdPrintChar(char c)
{
    // Newline: move to start of next row
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

    // Carriage return: move to start of current row
    if (c == '\r')
    {
        cursorCol = 0;
        lcd->setCursor(cursorCol, cursorRow);
        return;
    }

    // Write character at current position
    lcd->write(c);
    cursorCol++;

    // Auto-wrap to next row if end of line reached
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

// Print a full string character by character
void ddLcdPrint(const char* str)
{
    while (*str)
    {
        ddLcdPrintChar(*str);
        str++;
    }
}
