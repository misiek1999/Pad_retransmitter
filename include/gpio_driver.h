#pragma once

#include <Arduino.h>
#include "pinout.h"

namespace GPIO
{

void init_led()    {
    pinMode(kPowerLedPin, OUTPUT);
    pinMode(KControllerConnectedLedPin, OUTPUT);
    digitalWrite(kPowerLedPin, HIGH);   // Turn on power led
}

}   // namespace GPIO
