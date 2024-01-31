#include "Bluepad32_initialization.h"

namespace BP32Controller
{
// global variable with all connected gamepads
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      Serial.println(buf);
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Controller disconnected, but not found in myGamepads");
  }
}

TaskHandle_t Bluepad32Task;
bool setup_complete = false;

void bluepad32_and_btstack_run_loop_execute( void * pvParameters ){
    Serial.printf("Btstack core ID: %d\n", xPortGetCoreID());

    // Init btstack
    printf("Init btstack\n");
    btstack_init();

    // Init bluepad32
    Serial.printf("Start initialization bluepad32\n");
    uni_property_init();
    uni_platform_init(0, NULL);
    uni_hid_device_setup();
    // Continue with bluetooth setup.
    uni_bt_setup();
    uni_bt_allowlist_init();
    uni_virtual_device_init();
    Serial.printf("Init bluepad32 done\n");
    setup_complete = true;
    btstack_run_loop_execute();
    Serial.printf("Illegal Bluepad32 exit from btstack run loop\n");
}


bool init_bluepad32()
{
    // Stop default arduino bluetooth stack
    btStop();

    Serial.printf("Start Btstack task\n");
    xTaskCreatePinnedToCore(
                    bluepad32_and_btstack_run_loop_execute,   /* Task function. */
                    "btstack_run_loop_execute",     /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Bluepad32Task,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */


   while (setup_complete == false) {
        vTaskDelay(1);
    }
    // Get bluetooth firmware and address
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                    addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    return true;
}

} // namespace BP32Controller
