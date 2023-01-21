#pragma once

//  RoveComm initialization //
#include "RoveComm.h"

RoveCommEthernet RoveComm;
rovecomm_packet packet;

EthernetServer TCPServer(RC_ROVECOMM);


#DEFINE A0 COIL1_FWD;

