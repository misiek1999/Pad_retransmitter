#pragma once

#include <Arduino.h>
#include <btstack.h>
#include <Bluepad32.h>
#include <btstack_port_esp32.h>
extern "C" {
#include <uni_esp32.h>
#include <uni_property.h>
#include <uni_bt_setup.h>
#include <uni_bt_allowlist.h>
#include <uni_virtual_device.h>
}

namespace BP32Controller {
    extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];


    bool init_bluepad32();


}   // namespace BP32Controller
