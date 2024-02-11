#pragma once

#include <cstdint>

namespace GPIO_PIN
{

constexpr uint8_t kPowerLedPin = 2; // Power LED
constexpr uint8_t KControllerConnectedLedPin = 3; // Led indicating controller is connected

constexpr uint8_t kRF24L01CEPin = 4; // RF24 CE Pin
constexpr uint8_t kRF24L01CNSPin = 5; // RF24 CNS Pin
constexpr uint8_t kRF24L01MOSIPin = 23; // RF24 SPI MOSI Pin
constexpr uint8_t kRF24L01MISOPin = 19; // RF24 SPI MISO Pin
constexpr uint8_t kRF24L01SCKPin = 18; // RF24 SPI SCK Pin

} // namespace GPIO_PIN
