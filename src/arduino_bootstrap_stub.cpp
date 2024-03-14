#include "uni_platform_arduino_bootstrap.h"

// stub this file to avoid double execution of setup() and loop()
extern "C" {
void arduino_bootstrap() {
    // do nothing
}
}