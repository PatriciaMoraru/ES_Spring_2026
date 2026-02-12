#ifndef SRV_SERIAL_STDIO_H
#define SRV_SERIAL_STDIO_H

#include <stdio.h>

void srvSerialSetup();
int srvSerialPutChar(char c, FILE *stream);
int srvSerialGetChar(FILE *stream);

#endif