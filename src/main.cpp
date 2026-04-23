#include <Arduino.h>

#if defined(LAB_5_2)
#include "lab_5_2/app_lab_5_2.h"
#elif defined(LAB_5_1)
#include "lab_5_1/app_lab_5_1.h"
#endif

void setup()
{
#if defined(LAB_5_2)
    appLab52Setup();
#elif defined(LAB_5_1)
    appLab51Setup();
#endif
}

void loop()
{
#if defined(LAB_5_2)
    appLab52Loop();
#elif defined(LAB_5_1)
    appLab51Loop();
#endif
}
