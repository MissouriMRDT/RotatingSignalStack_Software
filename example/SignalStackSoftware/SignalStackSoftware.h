#pragma once

//////  RoveComm Initialization ////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RoveComm.h"

RoveCommEthernet RoveComm;
rovecomm_packet packet;

EthernetServer TCPServer(RC_ROVECOMM_SIGNALSTACKBOARD_PORT);

IntervalTimer telemetry;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





////// Pin Definitions /////////////////////////////////////////////////////////////////////////////////////////////////////////////
// motor pins
#define COIL1_FWD       A0
#define COIL1_RVS       A1
#define COIL2_FWD       A10
#define COIL2_RVS       A11

// compass I2C pins
#define COMPASS_SDA     A4
#define COMPASS_SCL     A5

// GPS serial pins
#define GPS_RX          0   // these aren't actually needed for the software to work, they are just left here to make it easier to find
#define GPS_TX          1
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





////// Variable Definitions ////////////////////////////////////////////////////////////////////////////////////////////////////////
// Position
double signalStackPosition[2];              // latitude and longitude
#define GPS_SERIAL_BAUD 9600
#define GPS_SERIAL      Serial1
TinyGPS gps;
bool newGPSData = false;

// Direction
float signalStackDirection;                 // Compass angle in degrees
#define COMPASS_ADDRESS         0x0D        // I2C address for Matek M8Q-5883
#define COMPASS_DATA_LENGTH     4           // number of bytes required to hold compass data
uint8_t compassBytes[COMPASS_DATA_LENGTH];  // total compass data as an array
uint8_t compassByte;                        // piece of compass data obtained over I2C

// Movement
const uint8_t stepDelay = 10;               // milliseconds
const uint16_t stepsPerRevolution = 3150;   // 360deg / 1.8deg step angle / 15.75 gear reduction
#define STEP_ANGLE  0.11428571428           // 1.8deg / 15.75 gear reduction
int16_t stepNumber;                         // number of steps to take, set by rovecomm command
float requestedAngle;                       // direction to point signal stack in requested over RoveComm
uint16_t requestedSteps;                    // steps needed to get to requested angle
float deltaAngle;                           // difference between current angle and desired angle
uint16_t closedLoopStepNumber = 200;        // number of steps to take before reevaluating number needed
float closedLoopMaxSSError = 0.5;           // Steady state error in degrees allowed by closed loop operation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





////// Function Definitions ////////////////////////////////////////////////////////////////////////////////////////////////////////
void Telemetry();               // sends position and direction data over RoveComm
void updateCompass();           // gets new compass data to send over RoveComm
void updateGPS();               // gets new GPS data to send over RoveComm
void GPSDump(TinyGPS &gps);     // gets new GPS data and writes it to Serial monitor
void goToAngle(float degrees);  // sends signal stack to direction from 0 to 360 degrees
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
