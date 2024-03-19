/*
    This file include universal data structures which contain the data collected from the controllers.
    The data is collected by the Bluepad32 library and is then passed to the user application.
    Struct ControllerData was copied from Bluepad32 library
*/
#pragma once

#include <cinttypes>

namespace BP32Data
{
// use 1 byte alignment to avoid padding
// struct __attribute__((packed)) PackedControllerData   // struct copy from bluepad32 library
struct PackedControllerData   // struct copy from bluepad32 library
{
    int8_t id;          // index of current controller, -1 means no controller is connected
    // Usage Page: 0x01 (Generic Desktop Controls)
    uint8_t dpad;
    int32_t axis_x;     // L joypad -512 +512
    int32_t axis_y;     // L joypad -512 +512
    int32_t axis_rx;    // R joypad -512 +512
    int32_t axis_ry;    // R joypad -512 +512

    // Usage Page: 0x02 (Sim controls)
    int32_t brake;      // 0 - 1023
    int32_t throttle;   // 0 - 1023

    // Usage Page: 0x09 (Button)
    uint16_t buttons;   // A, B, X, Y

    // Misc buttons (from 0x0c (Consumer) and others)
    uint8_t misc_buttons;   //

    int32_t gyro[3];        // deg/sec
    int32_t accel[3];       // G/s
};

namespace ControllerButtonConst
{
    static constexpr uint16_t kButtonA = 0x01;
    static constexpr uint16_t kButtonB = 0x02;
    static constexpr uint16_t kButtonX = 0x04;
    static constexpr uint16_t kButtonY = 0x08;
    static constexpr uint16_t kButtonShoulderL = 0x10;
    static constexpr uint16_t kButtonShoulderR = 0x20;
    static constexpr uint16_t kButtonTriggerL = 0x40;
    static constexpr uint16_t kButtonTriggerR = 0x80;
    static constexpr uint16_t kButtonThumbL = 0x100;
    static constexpr uint16_t kButtonThumbR = 0x200;
}

namespace ControllerMiscConst
{
    static constexpr uint8_t kButtonSystem = 0x01;
    static constexpr uint8_t kButtonSelect = 0x02;
    static constexpr uint8_t kButtonStart = 0x04;
    static constexpr uint8_t kButtonCapture = 0x08;
}

class ControllerDataManager {
public:
    ControllerDataManager();
    explicit ControllerDataManager(const PackedControllerData &data);
    ControllerDataManager& operator=(const PackedControllerData &data);
    ~ControllerDataManager() = default;

    void setControllerData(const PackedControllerData &data);
    PackedControllerData getControllerData() const;

    bool isControllerConnected() const {
        return _controller_data.id != -1;
    }

    uint8_t dpad() const { return _controller_data.dpad; }
    uint16_t buttons() const { return _controller_data.buttons; }
    uint8_t miscButtons() const { return _controller_data.misc_buttons; }
    // Axis
    int32_t axisX() const { return _controller_data.axis_x; }
    int32_t axisY() const { return _controller_data.axis_y; }
    int32_t axisRX() const { return _controller_data.axis_rx; }
    int32_t axisRY() const { return _controller_data.axis_ry; }

    // Brake & Throttle
    int32_t brake() const { return _controller_data.brake; }
    int32_t throttle() const { return _controller_data.throttle; }

    // Gyro / Accel
    int32_t gyroX() const { return _controller_data.gyro[0]; }
    int32_t gyroY() const { return _controller_data.gyro[1]; }
    int32_t gyroZ() const { return _controller_data.gyro[2]; }
    int32_t accelX() const { return _controller_data.accel[0]; }
    int32_t accelY() const { return _controller_data.accel[1]; }
    int32_t accelZ() const { return _controller_data.accel[2]; }

    bool a() const { return buttons() & ControllerButtonConst::kButtonA; }
    bool b() const { return buttons() & ControllerButtonConst::kButtonB; }
    bool x() const { return buttons() & ControllerButtonConst::kButtonX; }
    bool y() const { return buttons() & ControllerButtonConst::kButtonY; }
    bool l1() const { return buttons() & ControllerButtonConst::kButtonShoulderL; }
    bool l2() const { return buttons() & ControllerButtonConst::kButtonTriggerL; }
    bool r1() const { return buttons() & ControllerButtonConst::kButtonShoulderR; }
    bool r2() const { return buttons() & ControllerButtonConst::kButtonTriggerR; }
    bool thumbL() const { return buttons() & ControllerButtonConst::kButtonThumbL; }
    bool thumbR() const { return buttons() & ControllerButtonConst::kButtonThumbR; }

    // Misc buttons
    bool miscSystem() const { return miscButtons() & ControllerMiscConst::kButtonSystem; }
    bool miscSelect() const { return miscButtons() & ControllerMiscConst::kButtonSelect; }
    bool miscStart() const { return miscButtons() & ControllerMiscConst::kButtonStart; }
    bool miscCapture() const { return miscButtons() & ControllerMiscConst::kButtonCapture; }

private:
    PackedControllerData _controller_data;
};
} // namespace BP32Data
