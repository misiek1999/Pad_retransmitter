#include "gpio_driver.h"

void GPIO_PIN::init_led() {
    pinMode(kPowerLedPin, OUTPUT);
    pinMode(KControllerConnectedLedPin, OUTPUT);
    digitalWrite(kPowerLedPin, HIGH);   // Turn on power led
}
