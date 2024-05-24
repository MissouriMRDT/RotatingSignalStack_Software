#ifndef SIGNALSTACKSOFTWARE_H
#define SIGNALSTACKSOFTWARE_H
#include "PinAssignments.h"
#include "RoveHBridge.h"
#include "QMC5883LCompass.h"
#include "RoveComm.h"

RoveHBridge Motor(HB_FWD, HB_RVS);

QMC5883LCompass Compass;

//RoveComm Initialization
RoveCommEthernet RoveComm;
rovecomm_packet packet;

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);

IntervalTimer Telemetry;

void telemetry();
void feedWatchdog();
void haultMotors();
bool softLimit(bool dir);

int16_t motorSpeed = 0;

float compassAngle = 0;

float targetAngle = 0;

bool closedLoopActive = false;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStarved = 0;
uint8_t watchdogOverride = 0;

#endif //SIGNALSTACKSOFTWARE_H