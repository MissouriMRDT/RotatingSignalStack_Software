#include <Arduino.h>
#include <StepperMC.h>
#include <RoveComm.h>
#include <Wire.h>
#include "AK8975.h"
#include <cmath>

// the number of steps on your motor
#define STEPS 200
#if defined(ARDUINO_TEENSY41)
#define DIR_PIN 11
#define STP_PIN 10
#define EN_PIN 16
#define COMPASS_SDA
#define COMPASS_SCL
#elif defined(ARDUINO_WIZNET_5500_EVB_PICO)
#define DIR_PIN 11
#define STP_PIN 10
#define EN_PIN 0
#define COMPASS_SDA 6
#define COMPASS_SCL 7
#endif

#define DEGTORAD(x) (x * (180 / M_PI))

AK8975 mag(0x0C);
int16_t mx, my, mz;
float heading;
const int outputPin = 10; // Use any PWM-capable pin

RoveCommPacket packet;
RoveCommEthernet RoveComm;

#ifdef ARDUINO_WIZNET_5500_EVB_PICO
// Hacky way to spoof teensyduino on pi pico
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

#define WATCHDOG_TIMEOUT 1000000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

int64_t step_us = 0;
bool stepping = false;
void estop();

IntervalTimer stepTimer;

volatile long stepsRemaining = 0;
const int stepPin = STP_PIN;
const int dirPin = DIR_PIN;

// StepperMC stepper(DIR_PIN, STP_PIN, STEPS);
void feedWatchdog() { Watchdog.begin(estop, WATCHDOG_TIMEOUT); }

void estop()
{
  if (!watchdogOverride)
  {
    analogWrite(STP_PIN, 0);
    Serial.println("E-STOP ACTIVATED");
  }
}

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  mag.initialize();
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");

  // pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(DIR_PIN, OUTPUT);
  // digitalWrite(DIR_PIN, HIGH);
  // pinMode(STP_PIN, OUTPUT);

  // analogWriteFrequency(STP_PIN, 100);
#if ARDUINO_WIZNET_5500_EVB_PICO
  analogWriteRange(255);
#endif
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  // RoveComm Initialization
  Serial.println("RoveComm Initializing...");
  RoveComm.begin(RC_SIGNALSTACKBOARD_IPADDRESS);
  Serial.println("Complete");

  pinMode(outputPin, OUTPUT);

  // Set frequency to 32 Hz
  analogWriteFrequency(outputPin, 8);

  // 50% duty cycle (out of 256)
  analogWrite(outputPin, 64);
}
// square wave 32 Hz 4.5V amplitude 2.25V offset 0V lowlevel and highlevel 50% duty cycle
int32_t position = 0;
float_t targetAngle = 0;
double_t roverLat = 0;
double_t roverLon = 0;
double_t baseLat = 0;
double_t baseLon = 0;
#define M_PI 3.14159265358979323846
bool closedLoopActive = false;

void loop()
{
}
#if ARDUINO_WIZNET_5500_EVB_PICO
// Watchdog.update();
#endif