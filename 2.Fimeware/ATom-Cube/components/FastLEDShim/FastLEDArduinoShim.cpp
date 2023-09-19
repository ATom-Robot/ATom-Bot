#include "esp32-hal.h"
#include "FastLEDArduinoShim.h"

// there are 1024 microseconds per overflow counter tick.
unsigned long ARDUINO_ISR_ATTR micros()
{
    return (unsigned long)(esp_timer_get_time());
}

unsigned long ARDUINO_ISR_ATTR millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}
