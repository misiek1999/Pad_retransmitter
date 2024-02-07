#include "Bluepad32_data_struct.h"

BP32Data::ControllerDataManager::ControllerDataManager():
    _controller_data({}) {  // set all values to zero

}

BP32Data::ControllerDataManager::ControllerDataManager(const PackedControllerData & data):
    _controller_data(data) {
}

BP32Data::ControllerDataManager & BP32Data::ControllerDataManager::operator=(const PackedControllerData & data) {
    setControllerData(data);
    return *this;
}

void BP32Data::ControllerDataManager::setControllerData(const PackedControllerData & data) {
    _controller_data = data;
}

BP32Data::PackedControllerData BP32Data::ControllerDataManager::getControllerData() const {
    return _controller_data;
}

