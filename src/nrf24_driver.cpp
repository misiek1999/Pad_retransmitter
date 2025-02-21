#include "nrf24_driver.h"
#include "gpio_driver.h"

constexpr char kNrf24ControllerTag[] = "NRF24";

int RF24Driver::NRF24Controller::count = 0;

RF24Driver::NRF24Controller::NRF24Controller():
        radio_(GPIO_PIN::kRF24L01CEPin, GPIO_PIN::kRF24L01CNSPin),
        is_initialized_(false) {
    count++;
}

RF24Driver::NRF24Controller::~NRF24Controller()
{
    Serial.println("NRF24Controller destructor");
}

bool RF24Driver::NRF24Controller::init() {
    bool result = false;
    if (this->is_initialized_ == true) {
        ESP_LOGI(kNrf24ControllerTag, "NRF24Controller was already initialized");
        return result;
    }
    if (radio_.begin()) {
        ESP_LOGI(kNrf24ControllerTag, "NRF24Controller initialization begin.");
        // radio_.setPALevel(RF24_PA_HIGH);
        radio_.setPALevel(RF24_PA_LOW);
        // radio_.setDataRate(RF24_250KBPS);
        radio_.setPayloadSize(sizeof(BP32Data::PackedControllerData));
        ESP_LOGI(kNrf24ControllerTag, "Payload set to: %d.", sizeof(BP32Data::PackedControllerData));
        radio_.openWritingPipe(RF24Driver::address_tx);
        radio_.openReadingPipe(1, RF24Driver::address_rx);
        radio_.stopListening();
        this->is_initialized_ = true;
        result = true;
        radio_.printPrettyDetails();
        ESP_LOGI(kNrf24ControllerTag, "NRF24Controller initialization done");
    } else {
        ESP_LOGW(kNrf24ControllerTag, "NRF24Controller initialization failed");
    }
    return result;
}

bool RF24Driver::NRF24Controller::checkDriverIsInitialized() const {
    return this->is_initialized_;
}

bool RF24Driver::NRF24Controller::sendGamepadData(const BP32Data::PackedControllerData & data) {
    bool status = false;
    if (this->is_initialized_) {
        radio_.stopListening();
        splitPayloadToPackages(data);
        for (size_t i = 0; i < kPackageRequiedPerPayload; ++i) {
            status = radio_.write(&packages_to_send_[i], sizeof(packages_to_send_[i]));
            if (status) {
                LOG_I("Data sent successfully");
            } else {
                LOG_I("Data sending failed");
            }
        }
        radio_.startListening();
    } else {
        LOG_W("NRF24Controller is not initialized");
    }
    return status;
}

void dumpPacketToLog(uint8_t data_to_dump[28]) {
    char str[128];
    char *buf_ptr = str;
    for (size_t i = 0; i < 28; i++) {
        buf_ptr += sprintf(buf_ptr, " x%02X", (data_to_dump[i]));
    }
    LOG_V("Data: %s", str);
}

bool RF24Driver::NRF24Controller::receiveGamepadData(BP32Data::PackedControllerData & data) {
    bool status = false;
    if (this->is_initialized_) {
        uint8_t pipe;
        if (radio_.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
            const uint8_t bytes = radio_.getPayloadSize();  // get the size of the payload
            PackageContainer received_packet;
            received_packet.package_size = bytes;
            radio_.read(&received_packet.package, bytes);             // fetch payload from FIFO
            LOG_V("Received %d bytes on pipe %d", bytes, pipe);
            status = true;

            // If it's the first chunk of a new packet, update metadata
            if (received_packet.package.chunkIndex == 0) {
                received_chunk = 0;
                received_packet_id = received_packet.package.packetID;
                received_total_chunks = received_packet.package.totalChunks;
                LOG_D("Received packet ID: %d, total chunks: %d", received_packet_id, received_total_chunks);
            }
            dumpPacketToLog(received_packet.package.data);
            // merge received packages to payload
            convertPackageToPayload(data, received_packet);
        } else {
            LOG_V("No data available");
        }
    } else {
        LOG_W("NRF24Controller is not initialized");
    }
    return status;
}

void RF24Driver::NRF24Controller::splitPayloadToPackages(const BP32Data::PackedControllerData & data) {
    const auto packageCount = packetIDCounter++;
    for (size_t i = 0; i < kPackageRequiedPerPayload; ++i) {
        // update package metadata
        packages_to_send_[i].packetID = packageCount;
        packages_to_send_[i].chunkIndex = static_cast<uint8_t>(i);
        packages_to_send_[i].totalChunks = kPackageRequiedPerPayload;
        // covert payload to package data
        const auto* dataPtr = reinterpret_cast<const char*>(&data);
        const size_t dataSize = sizeof(BP32Data::PackedControllerData);
        const size_t offset = i * kPackageDataSize;
        // calculate chunk size
        size_t chunkSize = min(kPackageDataSize, dataSize - offset);
        packages_to_send_[i].dataBytes = static_cast<uint8_t>(chunkSize);
        // copy data to package
        memcpy(packages_to_send_[i].data, dataPtr + offset, chunkSize);
    }
}

void RF24Driver::NRF24Controller::convertPackageToPayload(BP32Data::PackedControllerData &data, const PackageContainer &packet_container) {
    // Check if the packet ID matches the expected ID
    if (packet_container.package.packetID != received_packet_id) {
        LOG_W("Unexpected Packet ID %d. Last received packet %d. Discarding packet",
               static_cast<int>(packet_container.package.packetID),
               static_cast<int>(received_packet_id));
        return;
    }

    // Check if the chunk index is within bounds
    if (packet_container.package.chunkIndex >= received_total_chunks) {
        LOG_W("Invalid Chunk Index %d. Total expected chunks: %d. Discarding packet",
               static_cast<int>(packet_container.package.chunkIndex),
               static_cast<int>(received_total_chunks));
        return;
    }

    // Check if the chunk data bytes is within bounds
    if (packet_container.package.dataBytes > sizeof(PackageContainer::package.data)) {
        LOG_W("Invalid Data Bytes %d. Package data size %d. Discarding packet",
               static_cast<int>(packet_container.package.dataBytes),
               static_cast<int>(sizeof(PackageContainer::package.data)));
        return;
    }

    // Copy the received data chunk into the appropriate position in the data buffer
    const uint16_t startByte = packet_container.package.chunkIndex * sizeof(Package::data);
    auto* dataPtr = reinterpret_cast<char*>(&received_data_);
    memcpy(&dataPtr[startByte], packet_container.package.data, packet_container.package.dataBytes);

    // Increment the received chunks counter
    ++received_chunk;

    // Check if all chunks have been received
    if (received_chunk == received_total_chunks) {
        // All data received, process or use the complete data
        LOG_D("All data received: %d", received_chunk);
        memcpy(&data, &received_data_, sizeof(BP32Data::PackedControllerData));
        // Reset receiver for next packet reception
        resetReceivedPackages();
    } else {
        LOG_D("Data received: %d, total %d ", received_chunk, received_total_chunks);
  }
}

void RF24Driver::NRF24Controller::resetReceivedPackages() {
    received_chunk = 0;
    received_packet_id = 0;
    received_total_chunks = 0;
}
