#ifndef TEENSY_SPOOF_H
#define TEENSY_SPOOF_H

#if ARDUINO_WIZNET_5500_EVB_PICO

#include "Arduino.h"

// Hacky way to spoof teensyduino on pi pico
#ifdef ARDUINO_WIZNET_5500_EVB_PICO
#define analogWriteFrequency(pin, freq) analogWriteFreq(freq)
class IntervalTimer
{
private:
    uint32_t nextTick = 0;
    uint32_t timeoutMicros = 1000000;
    void (*callback)() = nullptr;

public:
    void begin(void (*callback)(), uint32_t timeoutMicros)
    {
        this->callback = callback;
        this->timeoutMicros = timeoutMicros;
        nextTick = micros() + timeoutMicros;
    }
    void end()
    {
        callback = nullptr;
    }
    void update()
    {
        if (callback == nullptr)
            return;
        if (micros() >= nextTick)
        {
            callback();
            nextTick += timeoutMicros;
        }
    }
};
template <typename... Args>
void Serial_printf(const char *fmt, Args... args)
{
    char buffer[1024];
    snprintf(buffer, 1024, fmt, args...);
    Serial.print(buffer);
}
#endif

#endif

#endif // TEENSY_SPOOF_H