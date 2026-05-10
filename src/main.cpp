#include <Arduino.h>
#include <StepperMC.h>
#include <RoveComm.h>
#include <Wire.h>
#include "AK8975.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

// the number of steps on your motor
#define STEPS 256
#if defined(ARDUINO_TEENSY41)
// alternative teensy pin assignments
#elif defined(ARDUINO_WIZNET_5500_EVB_PICO)
#define DIR_PIN 11
#define STP_PIN 10
#endif

// compass
AK8975 mag(0x0C);
MPU6050 accelgyro;
int16_t mx, my, mz;
float heading;
#define LED_PIN 25
bool blinkState = true;
float targetAzimuth = 0;
float currentAzimuth = 0;

float calculateTargetAzimuth(double lat1, double lon1, double lat2, double lon2)
{
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  double rLat1 = lat1 * M_PI / 180.0;
  double rLat2 = lat2 * M_PI / 180.0;

  double y = sin(dLon) * cos(rLat2);
  double x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon);
  float az = atan2(y, x) * 180.0 / M_PI;

  return (az < 0) ? (az + 360) : az; // Normalize to 0-360
}

float readCompassAzimuth()
{
  int16_t mx, my, mz;
  mag.setMode(AK8975_MODE_SINGLE); // Trigger a read
  delay(10);                       // Wait for conversion
  mag.getHeading(&mx, &my, &mz);

  // Calculate angle. Note: swap mx/my based on sensor orientation
  float angle = atan2((float)my, (float)mx) * 180.0 / M_PI;

  // Normalize to 0-360
  if (angle < 0)
    angle += 360;
  return angle;
}

RoveCommPacket packet;
RoveCommEthernet RoveComm;

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
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  mag.initialize();
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

#if ARDUINO_WIZNET_5500_EVB_PICO
  analogWriteRange(255);
#endif
  // RoveComm Initialization
  Serial.println("RoveComm Initializing...");
  RoveComm.begin(RC_SIGNALSTACKBOARD_IPADDRESS);
  Serial.println("Complete");
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
  RoveComm.read(packet);
  currentAzimuth = readCompassAzimuth();
  switch (packet.dataId)
  {
  case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID:
  {
    int16_t speed = packet.i16data[0];
    Serial.print("Open loop speed: ");
    Serial.println(speed);

    if (speed == 0)
    {
      analogWrite(STP_PIN, 0); // Stop pulses
    }
    else
    {
      // Set direction FIRST
      if (speed < 0)
      {
        digitalWrite(DIR_PIN, HIGH);
        speed = -speed; // Make positive for frequency setting
      }
      else
      {
        digitalWrite(DIR_PIN, LOW);
      }

      // Apply frequency and start pulses
      analogWriteFrequency(STP_PIN, speed);
      analogWrite(STP_PIN, 128); // 50% duty cycle is usually more stable
    }

    closedLoopActive = false;
    feedWatchdog();
    break;
  }
  case RC_SIGNALSTACKBOARD_SETANGLETARGET_DATA_ID:
  {
    // get rover heading
    closedLoopActive = true;
    feedWatchdog();
    break;
  }
  case RC_SIGNALSTACKBOARD_SETGPSTARGET_DATA_ID:
  {
    // get rover and basestation GPS
    roverLat = packet.ddata[0];
    Serial_printf("roverLat: %.10f\n", packet.ddata[0]);
    roverLon = packet.ddata[1];
    Serial_printf("roverLon: %.10f\n", packet.ddata[1]);
    baseLat = packet.ddata[2];
    Serial_printf("baseLat: %.10f\n", packet.ddata[2]);
    baseLon = packet.ddata[3];
    Serial_printf("baseLon: %.10f\n", packet.ddata[3]);

    targetAzimuth = calculateTargetAzimuth(baseLat, baseLon, roverLat, roverLon);
    closedLoopActive = true;
    feedWatchdog();
    break;
  }
    // do watchdogoverride and telemetry
  }
}
#if ARDUINO_WIZNET_5500_EVB_PICO
// Watchdog.update();
#endif