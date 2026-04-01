#include <Arduino.h>

#if defined(LAB_4_2)
#include "lab_4_2/app_lab_4_2.h"
#elif defined(LAB_4_1)
#include "lab_4_1/app_lab_4_1.h"
#endif

void setup()
{
#if defined(LAB_4_2)
    appLab42Setup();
#elif defined(LAB_4_1)
    appLab41Setup();
#endif
}

void loop()
{
#if defined(LAB_4_2)
    appLab42Loop();
#elif defined(LAB_4_1)
    appLab41Loop();
#endif
}
