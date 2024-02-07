#pragma once

#include <esp32-hal-log.h>

#define LOG_TAG "Main"

#define LOG_E(format, ...) ESP_LOGE(LOG_TAG, format, ##__VA_ARGS__)
#define LOG_W(format, ...) ESP_LOGW(LOG_TAG, format, ##__VA_ARGS__)
#define LOG_I(format, ...) ESP_LOGI(LOG_TAG, format, ##__VA_ARGS__)
#define LOG_D(format, ...) ESP_LOGD(LOG_TAG, format, ##__VA_ARGS__)
#define LOG_V(format, ...) ESP_LOGV(LOG_TAG, format, ##__VA_ARGS__)