#include <Arduino.h>
#include <StepperMC.h>
#include <RoveComm.h>

// the number of steps on your motor
#define STEPS 200

#define STP_PIN 15
#define DIR_PIN 14
#define EN_PIN 16

RoveCommPacket packet;
RoveCommEthernet RoveComm;

#define WATCHDOG_TIMEOUT 100000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

int64_t step_us = 0;
bool stepping = false;
void estop();

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
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);
  pinMode(STP_PIN, OUTPUT);
  analogWriteFrequency(STP_PIN, 100);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  // RoveComm Initialization
  Serial.println("RoveComm Initializing...");
  RoveComm.begin(RC_SIGNALSTACKBOARD_IPADDRESS);
  Serial.println("Complete");
}

void loop()
{

  RoveComm.read(packet);

  switch (packet.dataId)
  {
  case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID:
  {
    Serial.printf("open loop %i\n", packet.i16data[0]);
    if (packet.i16data[0] < 0)
    {
      analogWriteFrequency(STP_PIN, map(packet.i16data[0], -1000, 0, 1000, 0));
      analogWrite(STP_PIN, 128);
      digitalWrite(DIR_PIN, HIGH);
      // int16_t reverseSpeed = -packet.i16data[0];

      // stepper.reverseDir(true);
      // stepper.setSpeed(-5000, 0);
      // stepper.setIncrementsRelative(movingIncrements);
      // stepper.moveTarget();
    }
    else if (packet.i16data[0] > 0)
    {
      analogWriteFrequency(STP_PIN, map(packet.i16data[0], 0, 1000, 0, 1000));
      analogWrite(STP_PIN, 128);
      digitalWrite(DIR_PIN, LOW);
      //  stepper.reverseDir(false);
      // stepper.setSpeed(5000, 0);
      // stepper.setIncrementsRelative(movingIncrements);
      // stepper.moveTarget();
    }
    else
    {
      analogWrite(STP_PIN, 0);
      // stepper.setSpeed(0, 0);
    }
    feedWatchdog();
    break;
  }
  case RC_SIGNALSTACKBOARD_SETANGLETARGET_DATA_ID:
  {

    float degreesToMove = packet.fdata[0];
    float stepsPerDegree = STEPS / 360.0;
    int32_t steps = (int32_t)roundf(degreesToMove * stepsPerDegree);
    // stepper.setIncrements(steps);
    //  stepper.setIncrementsRelative(steps);
    feedWatchdog();
    break;
  }
  // case RC_SIGNALSTACKBOARD_SETGPSTARGET_DATA_ID: {

  //   break;
  // }
  case RC_SIGNALSTACKBOARD_WATCHDOGOVERRIDE_DATA_ID:
  {
    watchdogOverride = packet.i8data[0];
    break;
  }
    // case RC_SIGNALSTACKBOARD_COMPASSANGLE_DATA_ID: {

    //   break;
    // }
  }
}