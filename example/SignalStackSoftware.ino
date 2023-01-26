// Signal Stack Board Software  /////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2023                    /////////////////////////////////////////////////////////////////////////////////////////////////////
// Grant Brinker                /////////////////////////////////////////////////////////////////////////////////////////////////////
// #RoveSoHard                  /////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SignalStackSoftware.h"
#include <Wire.h>

void setup()
{
    // motor pins
    pinMode(COIL1_FWD, OUTPUT);
    pinMode(COIL1_RVS, OUTPUT);
    pinMode(COIL2_FWD, OUTPUT);
    pinMode(COIL2_RVS, OUTPUT);

    // compass pins
    Wire.setSDA(COMPASS_SDA);
    Wire.setSCL(COMPASS_SCL);

    // GPS pins
    pinMode(GPS_TX, OUTPUT);
    pinMode(GPS_RX, INPUT);

    // Communication setup
    Wire.begin();
    Serial.begin(115200);
    RoveComm.begin(RC_SIGNALSTACKBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_SIGNALSTACKBOARD_MAC);
    delay(100);
    Telemetry.begin(telemetry, 1500000);
    Serial.println("Started: ");
}

void loop()
{
    // Receive RoveComm commands
    packet = RoveComm.read();

    switch (packet.data_id)
    {
        case RC_SIGNALSTACKBOARD_SIGNALSROTATE_DATA_ID:
            motorSpeed = (int16_t)packet.data[0];
            stackRotate(motorSpeed);
            break;
    }


    
}


void telemetry()
{
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_COUNT, signalStackPosition);
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_COUNT, signalStackDirection);
}





void updateCompass();
{
    Wire.requestFrom(COMPASS_ADDRESS, COMPASS_DATA_LENGTH);
    compassByte = 0;
    while (Wire.available())
    {
        compassBytes[compassByte] = Wire.read();
        compassByte++;
    }
    
    for (i = 0; i < COMPASS_DATA_LENGTH; i++)
    {
        signalStackDirection += (compassBytes[i] << (8 * i));
    }
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