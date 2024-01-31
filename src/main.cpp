#include <Arduino.h>
#include <nRF24L01.h>
#include "Bluepad32_initialization.h"

using namespace BP32Controller;

// Arduino setup function. Runs in CPU 1
void setup() {
    // start serial port with baud rate 115200
    Serial.begin(115200);

    // esp_log_level_set("*", ESP_LOG_VERBOSE);

    // Read current core ID
    Serial.printf("Core ID Arduino: %d\n", xPortGetCoreID());

    // Init bluepad32
    init_bluepad32();

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    Serial.println("Initialization successful!");
}


void processGamepad(ControllerPtr gamepad) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

  if (gamepad->a()) {
    static int colorIdx = 0;
    // Some gamepads like DS4 and DualSense support changing the color LED.
    // It is possible to change it by calling:
    switch (colorIdx % 3) {
      case 0:
        // Red
        gamepad->setColorLED(255, 0, 0);
        break;
      case 1:
        // Green
        gamepad->setColorLED(0, 255, 0);
        break;
      case 2:
        // Blue
        gamepad->setColorLED(0, 0, 255);
        break;
    }
    colorIdx++;
  }

  if (gamepad->b()) {
    // Turn on the 4 LED. Each bit represents one LED.
    static int led = 0;
    led++;
    // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
    // support changing the "Player LEDs": those 4 LEDs that usually indicate
    // the "gamepad seat".
    // It is possible to change them by calling:
    gamepad->setPlayerLEDs(led & 0x0f);
  }

  if (gamepad->x()) {
    // Duration: 255 is ~2 seconds
    // force: intensity
    // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
    // rumble.
    // It is possible to set it by calling:
    gamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
  }

  // Another way to query the buttons, is by calling buttons(), or
  // miscButtons() which return a bitmask.
  // Some gamepads also have DPAD, axis and more.
  char buf[256];
  snprintf(buf, sizeof(buf) - 1,
           "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
           "axis L: %4li, %4li, axis R: %4li, %4li, "
           "brake: %4ld, throttle: %4li, misc: 0x%02x, "
           "gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, "
           "battery: %d",
           gamepad->index(),        // Gamepad Index
           gamepad->dpad(),         // DPAD
           gamepad->buttons(),      // bitmask of pressed buttons
           gamepad->axisX(),        // (-511 - 512) left X Axis
           gamepad->axisY(),        // (-511 - 512) left Y axis
           gamepad->axisRX(),       // (-511 - 512) right X axis
           gamepad->axisRY(),       // (-511 - 512) right Y axis
           gamepad->brake(),        // (0 - 1023): brake button
           gamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
           gamepad->miscButtons(),  // bitmak of pressed "misc" buttons
           gamepad->gyroX(),      // Gyro X
           gamepad->gyroY(),      // Gyro Y
           gamepad->gyroZ(),      // Gyro Z
           gamepad->accelX(),     // Accelerometer X
           gamepad->accelY(),     // Accelerometer Y
           gamepad->accelZ(),     // Accelerometer Z
           gamepad->battery()       // 0=Unknown, 1=empty, 255=full
  );
  Serial.println(buf);

  // You can query the axis and other properties as well. See
  // Controller.h For all the available functions.
}

void processMouse(ControllerPtr mouse) {
  char buf[160];
  sprintf(buf,
          "idx=%d, deltaX:%4li, deltaY:%4li, buttons: 0x%04x, misc: 0x%02x, "
          "scrollWheel: %d, battery=%d",
          mouse->index(),        // Controller Index
          mouse->deltaX(),       // Mouse delta X
          mouse->deltaY(),       // Mouse delta Y
          mouse->buttons(),      // bitmask of pressed buttons
          mouse->miscButtons(),  // bitmak of pressed "misc" buttons
          mouse->scrollWheel(),  // Direction: 1=up, -1=down, 0=no movement
          mouse->battery()       // 0=Unk, 1=Empty, 255=full
  );
  Serial.println(buf);
}

void processBalanceBoard(ControllerPtr balance) {
  char buf[160];
  sprintf(buf,
          "idx=%d, tl:%4i, tr:%4i, bl: %4i, br: %4i, temperature=%d, "
          "battery=%d",
          balance->index(),  // Controller Index
          balance->topLeft(), balance->topRight(), balance->bottomLeft(),
          balance->bottomRight(), balance->temperature(),
          balance->battery()  // 0=Unk, 1=Empty, 255=full
  );
  Serial.println(buf);
}

// Arduino loop function. Runs in CPU 1
void loop() {
  // This call fetches all the controller info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The controllers pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
      else if (myController->isMouse())
        processMouse(myController);
      else if (myController->isBalanceBoard())
        processBalanceBoard(myController);
    }
  }
  delay(150);
}
