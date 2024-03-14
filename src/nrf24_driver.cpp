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
        // radio_.enableDynamicPayloads();
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
    if (this->is_initialized_ == true) {
        status = radio_.write(&data, sizeof(BP32Data::PackedControllerData));
        if (status) {
            ESP_LOGI(kNrf24ControllerTag, "Data sent successfully");
        } else {
            ESP_LOGI(kNrf24ControllerTag, "Data sending failed");
        }
    } else {
        ESP_LOGI(kNrf24ControllerTag, "NRF24Controller is not initialized");
    }
    return status;
}
