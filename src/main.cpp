#include <Arduino.h>
#include <StepperMC.h>
#include <RoveComm.h>
#include <Wire.h>
#include "AK8975.h"

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
#endif

AK8975 mag(0x0C);

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  mag.initialize();
  if (!mag.testConnection())
  {
    Serial.println("AK8975 connection failed!");
  }
}

void loop()
{
  int16_t mx, my, mz;

  mag.getHeading(&mx, &my, &mz);

  Serial.print("X:");
  Serial.println(mx);
  Serial.print("Y:");
  Serial.println(my);
  Serial.print("Z:");
  Serial.println(mz);

  delay(500);
}

// void setup()
// {
//   Wire.begin();

//   Serial.begin(9600);
//   while (!Serial);             // Leonardo: wait for serial monitor
//   Serial.println("\nI2C Scanner");
// }

// void loop()
// {
//   byte error, address;
//   int nDevices;

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for(address = 1; address < 127; address++ )
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.print(address,HEX);
//       Serial.println("  !");

//       nDevices++;
//     }
//     else if (error==4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address<16)
//         Serial.print("0");
//       Serial.println(address,HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");

//   delay(5000);           // wait 5 seconds for next scan
// }

// #define WATCHDOG_TIMEOUT 1000000
// IntervalTimer Watchdog;
// uint8_t watchdogStatus = 0;
// uint8_t watchdogOverride = 0;

// int64_t step_us = 0;
// bool stepping = false;
// void estop();

// // StepperMC stepper(DIR_PIN, STP_PIN, STEPS);
// void feedWatchdog() { Watchdog.begin(estop, WATCHDOG_TIMEOUT); }

// void estop()
// {
//   if (!watchdogOverride)
//   {
//     analogWrite(STP_PIN, 0);
//     Serial.println("E-STOP ACTIVATED");
//   }
// }

// void setup()
// {
//   Serial.begin(9600);
//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(DIR_PIN, OUTPUT);
//   digitalWrite(DIR_PIN, HIGH);
//   pinMode(STP_PIN, OUTPUT);
//   analogWriteFrequency(STP_PIN, 100);
// #if ARDUINO_WIZNET_5500_EVB_PICO
//   analogWriteRange(255);
// #endif
//   pinMode(EN_PIN, OUTPUT);
//   digitalWrite(EN_PIN, LOW);
//   // RoveComm Initialization
//   Serial.println("RoveComm Initializing...");
//   RoveComm.begin(RC_SIGNALSTACKBOARD_IPADDRESS);
//   Serial.println("Complete");
// }

// int32_t position = 0;

// void loop()
// {

//   RoveComm.read(packet);

//   switch (packet.dataId)
//   {
//   case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID:
//   {
//     Serial.print("open loop ");
//     Serial.println(packet.i16data[0]);
//     if (packet.i16data[0] < 0)
//     {
//       analogWriteFrequency(STP_PIN, -packet.i16data[0]);
//       analogWrite(STP_PIN, 128);
//       digitalWrite(DIR_PIN, HIGH);
//       // int16_t reverseSpeed = -packet.i16data[0];

//       // stepper.reverseDir(true);
//       // stepper.setSpeed(-5000, 0);
//       // stepper.setIncrementsRelative(movingIncrements);
//       // stepper.moveTarget();
//     }
//     else if (packet.i16data[0] > 0)
//     {
//       analogWriteFrequency(STP_PIN, packet.i16data[0]);
//       analogWrite(STP_PIN, 128);
//       digitalWrite(DIR_PIN, LOW);
//       //  stepper.reverseDir(false);
//       // stepper.setSpeed(5000, 0);
//       // stepper.setIncrementsRelative(movingIncrements);
//       // stepper.moveTarget();
//     }
//     else
//     {
//       analogWrite(STP_PIN, 0);
//       // stepper.setSpeed(0, 0);
//     }
//     feedWatchdog();
//     break;
//   }
//   case RC_SIGNALSTACKBOARD_SETANGLETARGET_DATA_ID:
//   {

//     float degreesToMove = packet.fdata[0];
//     float stepsPerDegree = STEPS / 360.0;
//     int32_t steps = (int32_t)roundf(degreesToMove * stepsPerDegree);
//     // analogWriteFrequency(STP_PIN, map(packet.i16data[0], 0, 1000, 0, 1000));
//     // analogWrite(STP_PIN, steps);
//     digitalWrite(DIR_PIN, HIGH);
//     for (int i = 0; i < steps; i++)
//     {
//       digitalWrite(DIR_PIN, HIGH);
//       delay(1);
//       digitalWrite(DIR_PIN, LOW);
//       delay(1);
//     }
//     // stepper.setIncrements(steps);
//     //  stepper.setIncrementsRelative(steps);
//     feedWatchdog();
//     break;
//   }
//   // case RC_SIGNALSTACKBOARD_SETGPSTARGET_DATA_ID: {

//   //   break;
//   // }
//   case RC_SIGNALSTACKBOARD_WATCHDOGOVERRIDE_DATA_ID:
//   {
//     watchdogOverride = packet.i8data[0];
//     break;
//   }
//     // case RC_SIGNALSTACKBOARD_COMPASSANGLE_DATA_ID: {

//     //   break;
//     // }
//   }
// #if ARDUINO_WIZNET_5500_EVB_PICO
//   Watchdog.update();
// #endif
// }