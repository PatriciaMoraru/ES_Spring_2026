#include "srv_serial_stdio.h"
#include <stdio.h>
#include <Arduino.h>

static FILE* srvSerialStream;
static FILE srvSerialStreamStorage;


void srvSerialSetup() 
{
    Serial.begin(9600);

    fdev_setup_stream(&srvSerialStreamStorage, 
                      srvSerialPutChar, 
                      srvSerialGetChar, 
                      _FDEV_SETUP_RW);

    srvSerialStream = &srvSerialStreamStorage;

    stdin = srvSerialStream;
    stdout = srvSerialStream;
}

int srvSerialPutChar(char c, FILE *stream)
{
    if (c == '\n')
    {
        srvSerialPutChar('\r', stream);
    }

    Serial.write(c);

    return 0;
}

int srvSerialGetChar(FILE *stream)
{
    while (!Serial.available())
    ;
    
    int c = Serial.read();

    if(c == '\r')
    {
        c = '\n';
    }

    return c;
}
