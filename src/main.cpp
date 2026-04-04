#include <Arduino.h>
#include <StepperMC.h>
#include <RoveComm.h>
#include <Wire.h>
#include "AK8975.h"
#include <cmath>

// the number of steps on your motor
#define STEPS 200
#if defined(ARDUINO_TEENSY41)
#define STP_PIN 15
#define DIR_PIN 14
#define EN_PIN 16
#define COMPASS_SDA
#define COMPASS_SCL
#elif defined(ARDUINO_WIZNET_5500_EVB_PICO)
#define DIR_PIN 2
#define STP_PIN 1
#define EN_PIN 0
#define COMPASS_SDA 6
#define COMPASS_SCL 7
#endif

#define DEGTORAD(x) (x * (180 / M_PI))

AK8975 mag(0x0C);
int16_t mx, my, mz;
float heading;

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

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  pinMode(STP_PIN, OUTPUT);

  analogWriteFrequency(STP_PIN, 100);
#if ARDUINO_WIZNET_5500_EVB_PICO
  analogWriteRange(255);
#endif
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  // RoveComm Initialization
  Serial.println("RoveComm Initializing...");
  RoveComm.begin(RC_SIGNALSTACKBOARD_IPADDRESS);
  Serial.println("Complete");
}

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
  switch (packet.dataId)
  {
  // speed
  case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID:
  {
    Serial.print("open loop ");
    Serial.println(packet.i16data[0]);
    if (packet.i16data[0] < 0)
    {
      analogWriteFrequency(STP_PIN, -packet.i16data[0]);
      analogWrite(STP_PIN, 128);
      digitalWrite(DIR_PIN, HIGH);
    }
    else if (packet.i16data[0] > 0)
    {
      analogWriteFrequency(STP_PIN, packet.i16data[0]);
      analogWrite(STP_PIN, 128);
      digitalWrite(DIR_PIN, LOW);
    }
    else
    {
      analogWrite(STP_PIN, 0);
    }
    closedLoopActive = false;
    feedWatchdog();
    break;
  }
  case RC_SIGNALSTACKBOARD_SETANGLETARGET_DATA_ID:
  {
    Serial.print("received: ");
    Serial.println(packet.fdata[0]);
    float targetAngle = packet.fdata[0]; // Assuming angle is sent as a float
    static float currentAngle = 0;

    // 1. Calculate the delta (change) needed
    float angleToMove = targetAngle - currentAngle;

    // 2. Convert angle to steps (Example: 1.8 degrees per step)
    // Formula: Steps = Angle / 1.8
    long stepsToMove = abs(angleToMove) / 1.40625;

    // 3. Set Direction
    digitalWrite(DIR_PIN, (angleToMove > 0) ? HIGH : LOW);

    // 4. Step execution (Blocking approach for simplicity)
    // Note: For smoother movement, use the AccelStepper library
    for (long i = 0; i < stepsToMove; i++)
    {
      Serial.print("moving");
      digitalWrite(STP_PIN, HIGH);
      delayMicroseconds(2000); // Controls speed
      digitalWrite(STP_PIN, LOW);
      delayMicroseconds(2000);
    }

    // 5. Update state
    currentAngle = targetAngle;
    closedLoopActive = true;
    feedWatchdog();
    break;
  }
  // heading
  // case RC_SIGNALSTACKBOARD_SETANGLETARGET_DATA_ID:
  // {
  //   targetAngle = packet.fdata[0];
  //   feedWatchdog();
  //   break;
  // }
  // gps
  case RC_SIGNALSTACKBOARD_SETGPSTARGET_DATA_ID:
  {
    // MY LOCATION: 37.951664, -91.777234
    // PRICE: 37.95388110046358, -91.751811233115

    // ROVER/CHANGING GPS: 37.95180969121836, -91.77626487233822
    // DELC/SIGNAL GPS: 37.951974771528725, -91.7772764007855
    roverLat = packet.ddata[0];
    Serial_printf("roverLat: %.10f\n", packet.ddata[0]);
    roverLon = packet.ddata[1];
    Serial_printf("roverLon: %.10f\n", packet.ddata[1]);
    baseLat = packet.ddata[2];
    Serial_printf("baseLat: %.10f\n", packet.ddata[2]);
    baseLon = packet.ddata[3];
    Serial_printf("baseLon: %.10f\n", packet.ddata[3]);

    mag.getHeading(&mx, &my, &mz);

    // display tab-separated magnetometer x/y/z values
    Serial.print("mag:\t");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\t\t");

    // angle from signalstack relative to north
    float azimuth = DEGTORAD(atan2(mx, my));
    if (azimuth < 0)
    {
      azimuth += 360.0;
    }

    Serial_printf("angle from signalstack relative to north: %.10f\n", azimuth);

    // angle from signalstack->rover vector relative to north
    float theta = DEGTORAD(atan2((roverLon - baseLon), (roverLat - baseLat)));

    Serial_printf("angle from signalstack->rover vector relative to north: %.10f\n", theta);

    float differenceAngle = azimuth - theta;

    //

    feedWatchdog();
    break;
  }
  case RC_SIGNALSTACKBOARD_WATCHDOGOVERRIDE_DATA_ID:
  {
    watchdogOverride = packet.i8data[0];
    break;
  }
  }
#if ARDUINO_WIZNET_5500_EVB_PICO
  Watchdog.update();
#endif
}