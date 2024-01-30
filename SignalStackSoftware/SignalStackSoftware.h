#ifndef SIGNALSTACKSOFTWARE_H
#define SIGNALSTACKSOFTWARE_H
#include "PinAssignments.h"
#include "RoveHBridge.h"
#include <QMC5883LCompass.h>

RoveHBridge Motor(HB_FWD, HB_RVS);

QMC5883LCompass Compass;

//RoveComm Initialization
#include "RoveComm.h"

RoveCommEthernet RoveComm;
rovecomm_packet packet;

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);

IntervalTimer Telemetry;

void telemetry();
void feedWatchdog();
void haultMotors();

int16_t motorSpeed = 0;

uint16_t compassAngle = 0;

// Watchdog
#define WATCHDOG_TIMEOUT 300000
IntervalTimer Watchdog;
uint8_t watchdogStarved = 0;
uint8_t watchdogOverride = 0;

#endif //SIGNALSTACKSOFTWARE_H