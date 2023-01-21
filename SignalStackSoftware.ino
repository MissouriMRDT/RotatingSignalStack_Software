// Signal Stack Board Software  /////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2023                    /////////////////////////////////////////////////////////////////////////////////////////////////////
// Grant Brinker                /////////////////////////////////////////////////////////////////////////////////////////////////////
// #RoveSoHard                  /////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SignalStackSoftware.h"

void setup()
{
    // motor pins
    pinMode(COIL1_FWD, OUTPUT);
    pinMode(COIL1_RVS, OUTPUT);
    pinMode(COIL2_FWD, OUTPUT);
    pinMode(COIL2_RVS, OUTPUT);

    // compass pins


    // GPS pins


    // Communication setup
    Serial.begin(115200);
    RoveComm.begin(RC_SIGNALSTACKBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_SIGNALSTACKBOARD_MAC);
    delay(100);
    Telemetry.begin(telemetry. 1500000);
    Serial.println("Started: ");
}

void loop()
{
    //code go brrr
}


void telemetry()
{
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_COUNT, SignalStackPosition);
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_COUNT, SignalStackDirection);
}



/*
REQUIREMENTS:
    -interface with RoveComm
        -RoveComm.begin()
    -receive motor commands
        -manual control in both directions
            -small value so no ramping is required and equipment doesn't break
            -digitalWrite high to one moco and low to other moco
        -possibly closed loop?
            -ramp speed up to small value
    -send GPS data
        -start serial communication
        -serial comm with GPS
        -RoveComm.write()
    -send compass data
        -I2C comm with GPS
        -RoveComm.write()
*/