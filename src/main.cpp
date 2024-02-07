#include <Arduino.h>
#include <nRF24L01.h>
#include <log_debug.h>
#include "Bluepad32_driver.h"
#include "gpio_driver.h"

BP32Driver::Bluepad32Driver bp32_driver;
BP32Data::PackedControllerData data;

// Arduino setup function. Runs in CPU 1
void setup() {
    // start serial port with baud rate 115200
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    // Read current core ID
    LOG_D("Core ID Arduino: %d\n", xPortGetCoreID());
    pinMode(2, OUTPUT);
    // Init bluepad32
    bp32_driver.init();

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    bp32_driver.forgetBluetoothKeys();

    LOG_I("Initialization successful!");

    // Init GPIO
    GPIO::init_led();

}

// Arduino loop function. Runs in CPU 1
void loop() {
    // Update bluepad32
    bp32_driver.processGamepad(data);
    BP32Driver::dump_bluepad_driver_data(data);
    // Delay 150ms
    delay(150);
}
