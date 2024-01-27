#ifndef SIGNALSTACKSOFTWARE_H
#define SIGNALSTACKSOFTWARE_H
#include "PinAssignments.h"
#include "RoveHBridge.h"
#include "HMC5883L.h"

RoveHBridge Motor(HB_FWD, HB_RVS);

HMC5883L Compass;

//RoveComm Initialization
#include "RoveComm.h"

RoveCommEthernet RoveComm;
rovecomm_packet packet;

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_TCP_PORT);

IntervalTimer Telemetry;

void telemetry();

int16_t motorSpeed = 0;

void watchdog() {
    
}

void feedWatchdog() {
    
}

#endif //SIGNALSTACKSOFTWARE_H