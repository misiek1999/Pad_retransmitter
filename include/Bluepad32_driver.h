#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <Arduino.h>
#include <Bluepad32.h>
#include "Bluepad32_data_struct.h"

namespace BP32Driver {
// Bluepad32Driver class to wrap Bluepad32 library
class Bluepad32Driver {
public:
    Bluepad32Driver();
    ~Bluepad32Driver() = default;
    bool init();
    bool checkDriverIsInitialized() const;
    void forgetBluetoothKeys();
    bool processGamepad(BP32Data::PackedControllerData &updated_data);
private:
    // initialize flag, active when bluepad32 task is working
    std::atomic<bool> _is_initialized;
    // object with single bluepad32 instance
    ControllerPtr _myControllers[BP32_MAX_GAMEPADS];
    // callback for connected controller
    void onConnectedController(ControllerPtr ctl);
    void onDisconnectedController(ControllerPtr ctl);
    // check provaided index is valid
    bool isIndexValid(size_t index) const;
    // read data from controller and convert to PackedControllerData
    void copyControllerData(BP32Data::PackedControllerData &data, ControllerPtr controller);
};

// Debug function to dump bluepad32 data
void dump_bluepad_driver_data(const BP32Data::PackedControllerData &data);

}   // namespace BP32Driver
