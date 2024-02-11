#pragma once

#include <Arduino.h>
#include <RF24.h>
#include "log_debug.h"
#include "Bluepad32_data_struct.h"

namespace RF24Driver
{
constexpr byte address_rx[6] = {"PADRX"};
constexpr byte address_tx[6] = {"PADTX"};

class NRF24Controller
{
public:
    NRF24Controller();
    ~NRF24Controller();
    // initialize driver
    bool init();
    // check if driver is initialized
    bool checkDriverIsInitialized() const;
    // send gamepad data to receiver
    bool sendGamepadData(const BP32Data::PackedControllerData &data);

private:
    static int count;
    RF24 radio_;            // Object with single RF24 instance
    bool is_initialized_;   // flag to check if driver is initialized
};

}   // namespace NRF24Driver
