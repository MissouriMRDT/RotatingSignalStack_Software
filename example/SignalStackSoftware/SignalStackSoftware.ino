// Signal Stack Board Software  /////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2023                    /////////////////////////////////////////////////////////////////////////////////////////////////////
// Grant Brinker                /////////////////////////////////////////////////////////////////////////////////////////////////////
// #RoveSoHard                  /////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SignalStackSoftware.h"
#include <Wire.h>
#include <Stepper.cpp>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    // Communication setup
    Wire.begin();
    Serial.begin(9600);
    RoveComm.begin(RC_SIGNALSTACKBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_SIGNALSTACKBOARD_MAC);
    delay(100);
    telemetry.begin(Telemetry, 1500000);
    Serial.println("Started: ");
    Serial1.begin(115200);
    
    // motor pins
    pinMode(COIL1_FWD, OUTPUT);
    pinMode(COIL1_RVS, OUTPUT);
    pinMode(COIL2_FWD, OUTPUT);
    pinMode(COIL2_RVS, OUTPUT);
    Stepper stepperMotor(stepsPerRevolution, COIL1_FWD, COIL1_RVS, COIL2_FWD, COIL2_RVS);
    stepperMotor.setSpeed(stepDelay);

    // compass pins
    Wire.setSDA(COMPASS_SDA);
    Wire.setSCL(COMPASS_SCL);

    // GPS pins
    Serial1.setRX(GPS_RX);
    Serial1.setTX(GPS_TX);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
    // Receive RoveComm commands
    packet = RoveComm.read();
    
    switch (packet.data_id)
    {
        case RC_SIGNALSTACKBOARD_SIGNALSROTATE_DATA_ID:
            stepNumber = ((int16_t*) packet.data)[0];
            Stepper::step(stepNumber);
            break;
    }

    updateCompass();
    updateGPS();
    //Telemetry();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Telemetry()
{
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_COUNT, signalStackPosition);
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_COUNT, signalStackDirection);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateCompass()   // Obtains new compass data
{
    Wire.requestFrom(COMPASS_ADDRESS, COMPASS_DATA_LENGTH);
    compassByte = 0;
    while (Wire.available())
    {
        compassBytes[(COMPASS_DATA_LENGTH - 1) - compassByte] = Wire.read();    // Selects leftmost element in array,
        compassByte++;                                                          // (COMPASS_DATA_LENGTH - 1), and moves right
    }
    
    for (uint8_t i = 0; i < COMPASS_DATA_LENGTH; i++)
    {
        signalStackDirection += (compassBytes[(COMPASS_DATA_LENGTH - 1) - i] << (8 * ((COMPASS_DATA_LENGTH - 1) - i)));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateGPS()    // Obtains new GPS data
{
    signalStackPosition = Serial1.read();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void goToAngle(uint16_t deg)
{
    // code go brrr
}