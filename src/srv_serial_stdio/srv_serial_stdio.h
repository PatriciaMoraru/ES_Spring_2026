// Serial STDIO service - redirects printf/getchar to the serial port
#ifndef SRV_SERIAL_STDIO_H
#define SRV_SERIAL_STDIO_H

#include <stdio.h>

void srvSerialSetup();                    // Initialize serial and bind to STDIO
int srvSerialPutChar(char c, FILE *stream); // Send a character to serial
int srvSerialGetChar(FILE *stream);         // Read a character from serial

#endif