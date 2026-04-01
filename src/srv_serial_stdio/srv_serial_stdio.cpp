// Serial STDIO service implementation
#include "srv_serial_stdio.h"
#include <stdio.h>
#include <Arduino.h>

// When built inside a FreeRTOS environment that uses scanf(),
// the busy-wait in srvSerialGetChar() would starve all lower-priority tasks.
#ifdef SERIAL_FREERTOS_YIELD
#include <Arduino_FreeRTOS.h>
#endif

static FILE *srvSerialStream;       // Pointer to the serial stream
static FILE srvSerialStreamStorage; // Storage for the FILE structure

// Initialize serial port and redirect stdin/stdout to it
void srvSerialSetup()
{
    Serial.begin(9600);

    // Configure the stream with put/get callbacks for read-write mode
    fdev_setup_stream(&srvSerialStreamStorage,
                      srvSerialPutChar,
                      srvSerialGetChar,
                      _FDEV_SETUP_RW);

    srvSerialStream = &srvSerialStreamStorage;

    // Redirect standard input and output to the serial stream
    stdin = srvSerialStream;
    stdout = srvSerialStream;
}

// Send a character to the serial port
int srvSerialPutChar(char c, FILE *stream)
{
    // Convert \n to \r\n for proper terminal display
    if (c == '\n')
    {
        srvSerialPutChar('\r', stream);
    }

    Serial.write(c);

    return 0;
}

// Read a character from the serial port (blocking)
int srvSerialGetChar(FILE *stream)
{
    // Wait until data is available.
    // With SERIAL_FREERTOS_YIELD: properly block and yield so the FreeRTOS
    // scheduler can run other tasks while no input is pending.
    // Without it: busy-spin (fine for bare-metal / non-FreeRTOS builds).
    while (!Serial.available())
    {
#ifdef SERIAL_FREERTOS_YIELD
        vTaskDelay(1);
#endif
    }

    int c = Serial.read();

    // Convert carriage return to newline
    if (c == '\r')
    {
        c = '\n';
    }

    return c;
}
