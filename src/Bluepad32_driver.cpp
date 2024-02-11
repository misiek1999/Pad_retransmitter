#include "Bluepad32_driver.h"
#include <esp32-hal-log.h>
#include <btstack.h>
#include <Bluepad32.h>
#include <btstack_port_esp32.h>
extern "C" {
#include <uni_esp32.h>
#include <uni_property.h>
#include <uni_bt_setup.h>
#include <uni_bt_allowlist.h>
#include <uni_virtual_device.h>
#include "Bluepad32_driver.h"
}

namespace BP32Driver
{
constexpr size_t kBLuepad32TaskMaxStackSize = 20000;
static constexpr char kBp32Tag[] = "BP32";

std::atomic<bool> _driver_is_init(false);

TaskHandle_t Bluepad32Task;
std::atomic<bool> driver_setup_complete(false);

void bluepad32_and_btstack_run_loop_execute( void * pvParameters ){
    ESP_LOGD(kBp32Tag, "Btstack core ID: %d", xPortGetCoreID());

    // Init btstack
    ESP_LOGV(kBp32Tag, "Init btstack");
    btstack_init();

    // Init bluepad32
    ESP_LOGV(kBp32Tag, "Start initialization bluepad32");
    uni_property_init();
    uni_platform_init(0, nullptr);
    uni_hid_device_setup();
    // Continue with bluetooth setup.
    uni_bt_setup();
    uni_bt_allowlist_init();
    uni_virtual_device_init();
    ESP_LOGV(kBp32Tag, "Init bluepad32 done");

    driver_setup_complete = true;
    btstack_run_loop_execute();
    ESP_LOGE(kBp32Tag, "Illegal Bluepad32 exit from btstack run loop");
}

void init_driver() {
    // check it is first init of driver
    if (_driver_is_init == false) {
        _driver_is_init = true;
        // Stop default arduino bluetooth stack
        btStop();
        ESP_LOGD(kBp32Tag, "Start Btstack task");
        xTaskCreatePinnedToCore(
                        bluepad32_and_btstack_run_loop_execute, /* Task function. */
                        "btstack_run_loop_execute",             /* name of task. */
                        kBLuepad32TaskMaxStackSize,             /* Stack size of task */
                        nullptr,                                /* parameter of the task */
                        1,                                      /* priority of the task */
                        &Bluepad32Task,                         /* Task handle to keep track of created task */
                        0);                                     /* pin task to core 0 */
    }
}

BP32Driver::Bluepad32Driver::Bluepad32Driver():
        _is_initialized(false),
        _myControllers{} {

}

bool BP32Driver::Bluepad32Driver::init() {
    // try to init driver if was not initilized
    init_driver();
    // wait until driver is not init
    while (driver_setup_complete == false) {
        vTaskDelay(1);
    }
    // Get bluetooth firmware and address
    ESP_LOGD(kBp32Tag, "Firmware: %s", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    ESP_LOGD(kBp32Tag, "BD Addr: %2X:%2X:%2X:%2X:%2X:%2X", addr[0], addr[1], addr[2],
                    addr[3], addr[4], addr[5]);
    // Setup the Bluepad32 callbacks
    BP32.setup(std::bind(&Bluepad32Driver::onConnectedController, this, std::placeholders::_1),
               std::bind(&Bluepad32Driver::onDisconnectedController, this, std::placeholders::_1));

    _is_initialized = true;

    return true;
}

bool BP32Driver::Bluepad32Driver::checkDriverIsInitialized() const {
    return this->_is_initialized;
}

void BP32Driver::Bluepad32Driver::forgetBluetoothKeys() {
    if(_is_initialized) {
        BP32.forgetBluetoothKeys();
    }
}

// TODO: Tidy up this function
bool BP32Driver::Bluepad32Driver::processGamepad(BP32Data::PackedControllerData &updated_data) {
    bool status = false;
    if (_is_initialized) {
        BP32.update();
        for (size_t i = 0; i < BP32_MAX_GAMEPADS; ++i) {
            ControllerPtr myController = _myControllers[i];
            if (myController != nullptr && myController->isConnected()) {
                if (myController->isGamepad()) {
                    copyControllerData(updated_data, myController);
                    status = true;
                } else {
                    ESP_LOGW(kBp32Tag, "Connected device is not a gamepad!");
                }
            }
        }
    }
    if (!status) {
        updated_data.id = -1;
    }
    return status;
}

void BP32Driver::Bluepad32Driver::onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (_myControllers[i] == nullptr) {
      ESP_LOGI(kBp32Tag, "CALLBACK: Controller is connected, index=");
      Serial.println(i);
      _myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the
      // gamepad.
      const ControllerProperties properties = ctl->getProperties();
      ESP_LOGD(kBp32Tag, "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags);
      break;
    }
  }
  if (!foundEmptySlot) {
    ESP_LOGW(kBp32Tag, "CALLBACK: Controller connected, but could not found empty slot");
  }
}

void BP32Driver::Bluepad32Driver::onDisconnectedController(ControllerPtr ctl) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (_myControllers[i] == ctl) {
            ESP_LOGI(kBp32Tag, "CALLBACK: Controller is disconnected from index=");
            Serial.println(i);
            _myControllers[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        ESP_LOGW(kBp32Tag, "CALLBACK: Controller disconnected, but not found in myGamepads");
    }
}

bool BP32Driver::Bluepad32Driver::isIndexValid(size_t index) const {
    return (index < BP32_MAX_GAMEPADS) ? true : false;
}

void BP32Driver::Bluepad32Driver::copyControllerData(BP32Data::PackedControllerData & data, ControllerPtr controller) {
    data.id = controller->index();
    data.dpad = controller->dpad();
    data.axis_x = controller->axisX();
    data.axis_y = controller->axisY();
    data.axis_rx = controller->axisRX();
    data.axis_ry = controller->axisRY();
    data.brake = controller->brake();
    data.throttle = controller->throttle();
    data.buttons = controller->buttons();
    data.misc_buttons = static_cast<uint8_t>(controller->miscButtons());
    data.gyro[0] = controller->gyroX();
    data.gyro[1] = controller->gyroY();
    data.gyro[2] = controller->gyroZ();
    data.accel[0] = controller->accelX();
    data.accel[1] = controller->accelY();
    data.accel[2] = controller->accelZ();
}

void dump_bluepad_driver_data(const BP32Data::PackedControllerData & data) {
    if (data.id != -1) {
        ESP_LOGD(kBp32Tag,   "dpad: 0x%02x, buttons: 0x%04x, "
                "axis L: %4li, %4li, axis R: %4li, %4li, "
                "brake: %4ld, throttle: %4li, misc: 0x%02x, "
                "gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, ",
                data.dpad,          // DPAD
                data.buttons,       // bitmask of pressed buttons
                data.axis_rx,       // (-511 - 512) left X Axis
                data.axis_ry,       // (-511 - 512) left Y axis
                data.axis_x,        // (-511 - 512) right X axis
                data.axis_y,        // (-511 - 512) right Y axis
                data.brake,         // (0 - 1023): brake button
                data.throttle,      // (0 - 1023): throttle (AKA gas) button
                data.misc_buttons,  // bitmak of pressed "misc" buttons
                data.gyro[0],       // Gyro X
                data.gyro[1],       // Gyro Y
                data.gyro[2],       // Gyro Z
                data.accel[0],      // Accelerometer X
                data.accel[1],      // Accelerometer Y
                data.accel[2]      // Accelerometer Z
        );
    }
}

}   // namespace BP32Driver
