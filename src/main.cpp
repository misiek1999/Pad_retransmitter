#include <Arduino.h>
#include <nRF24L01.h>
#include <log_debug.h>
#include "Bluepad32_driver.h"
#include "nrf24_driver.h"
#include "gpio_driver.h"

constexpr size_t kMainLoopDelay = 150;

BP32Driver::Bluepad32Driver bp32_driver;
BP32Data::PackedControllerData data;
RF24Driver::NRF24Controller nrf24_driver;

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

    // Init nrf24
    nrf24_driver.init();
    // Init GPIO
    GPIO_PIN::init_led();

    LOG_I("Initialization successful!");
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // Update bluepad32
    bp32_driver.processGamepad(data);
    BP32Driver::dump_bluepad_driver_data(data);

    // try to init if not initialized
    if (nrf24_driver.checkDriverIsInitialized() == false) {
        Serial.println("NRF24 not initialized, try to init");
        nrf24_driver.init();
    }
    // Send gamepad data to receiver
    nrf24_driver.sendGamepadData(data);

    // Delay 150ms
    delay(kMainLoopDelay);
}
