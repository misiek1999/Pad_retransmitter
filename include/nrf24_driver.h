#pragma once

#include <Arduino.h>
#include <RF24.h>
#include "log_debug.h"
#include "Bluepad32_data_struct.h"

namespace RF24Driver
{
constexpr byte address_rx[6] = {"PADRX"};
constexpr byte address_tx[6] = {"PADTX"};

constexpr size_t kMaxPayloadSize = 32;
constexpr size_t kPackageRequiedPerPayload = sizeof(BP32Data::PackedControllerData) / kMaxPayloadSize + 1;
constexpr size_t kPackageDataSize = 28;
struct Package {
    uint8_t packetID;
    uint8_t chunkIndex;
    uint8_t totalChunks;
    uint8_t dataBytes;
    uint8_t data[kPackageDataSize];  //  28-bytes payload - 6 bytes for metadata
};

struct PackageContainer {
    Package package;        // received data
    size_t package_size;    // total size of package without unused payload data
};

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
    // receive gamepad data from receiver
    bool receiveGamepadData(BP32Data::PackedControllerData &data);

private:
    static int count;
    RF24 radio_;            // Object with single RF24 instance
    bool is_initialized_;   // flag to check if driver is initialized
    Package packages_to_send_[kPackageRequiedPerPayload];
    BP32Data::PackedControllerData received_data_;
    uint16_t received_packet_id;
    uint16_t received_chunk;
    uint16_t received_total_chunks;
    inline static uint8_t packetIDCounter;
    void splitPayloadToPackages(const BP32Data::PackedControllerData &data);
    void convertPackageToPayload(BP32Data::PackedControllerData &data, const PackageContainer &packet);
    void resetReceivedPackages();
};

}   // namespace NRF24Driver
