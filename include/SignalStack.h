#ifndef SIGNALSTACK_H
#define SIGNALSTACK_H

#include <Arduino.h>
#include <RoveComm.h>
#include <Wire.h>
#include <AK8975.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <math.h>

#include "TeensySpoof.h"

#define STEPS 256

// compass
AK8975 mag(0x0C);
MPU6050 accelgyro;
int16_t mx, my, mz;
// calibration offsets
float offX = 0;
float offY = 0;
float offZ = 0;
float heading;
#define LED_PIN 25
bool blinkState = true;
float targetAzimuth = 0;
float currentAzimuth = 0;

// square wave 32 Hz 4.5V amplitude 2.25V offset 0V lowlevel and highlevel 50% duty cycle
int32_t position = 0;
float_t targetAngle = 0;

bool closedLoopActive = false;

#define MAX_SPEED 350 // Define your hardware limit here

RoveCommPacket packet;
RoveCommEthernet RoveComm;

#define WATCHDOG_TIMEOUT 1000000
IntervalTimer Watchdog;
uint8_t watchdogStatus = 0;
uint8_t watchdogOverride = 0;

void feedWatchdog();

void estop();

float calculateTargetAzimuth(double baseLat, double baseLon, double roverLat, double roverLon);

void controlMotor(int16_t speed);

void calibrateMagnetometer();

float readCompassAzimuth();

#endif