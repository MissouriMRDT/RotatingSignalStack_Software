// Signal Stack Board Software  /////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2023                    /////////////////////////////////////////////////////////////////////////////////////////////////////
// Grant Brinker                /////////////////////////////////////////////////////////////////////////////////////////////////////
// #RoveSoHard                  /////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SignalStackSoftware.h"
#include <Wire.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    // Communication setup
    Wire.begin();
    Serial.begin(115200);
    RoveComm.begin(RC_SIGNALSTACKBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_SIGNALSTACKBOARD_MAC);
    delay(100);
    Telemetry.begin(telemetry, 1500000);
    Serial.println("Started: ");
    
    // motor pins
    pinMode(COIL1_FWD, OUTPUT);
    pinMode(COIL1_RVS, OUTPUT);
    pinMode(COIL2_FWD, OUTPUT);
    pinMode(COIL2_RVS, OUTPUT);

    // compass pins
    Wire.setSDA(COMPASS_SDA);
    Wire.setSCL(COMPASS_SCL);

    // GPS pins
    Serial.setRX(GPS_RX);
    Serial.setTX(GPS_TX);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

    updateCompass();
    updateGPS();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void telemetry()
{
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_COUNT, signalStackPosition);
    RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSDIRECTION_DATA_COUNT, signalStackDirection);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateCompass()   // This function looks like garbage, but I don't know how to make it look better
{
    Wire.requestFrom(COMPASS_ADDRESS, COMPASS_DATA_LENGTH);
    compassByte = 0;
    while (Wire.available())
    {
        compassBytes[(COMPASS_DATA_LENGTH - 1) - compassByte] = Wire.read();    // Selects leftmost element in array,
        compassByte++;                                                          // (COMPASS_DATA_LENGTH - 1), and moves right
    }
    
    for (i = 0; i < COMPASS_DATA_LENGTH; i++)
    {
        signalStackDirection += (compassBytes[(COMPASS_DATA_LENGTH - 1) - i] << (8 * ((COMPASS_DATA_LENGTH - 1) - i)));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateGPS()
{
    signalStackPosition = Serial.read(GPS_RX);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void stackRotate(int16_t decipercent)
{
    uint8_t fwdPWM = 0;
    uint8_t rvsPWM = 0;
    
    if      (decipercent > 0) fwdPWM = decipercent * 255 / 1000;
    else if (decipercent < 0) rvsPWM = decipercent * 255 / 1000;

    if      (fwdPWM) fwdRotate(fwdPWM);
    else if (rvsPWM) rvsRotate(rvsPWM);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void fwdRotate(uint8_t PWM)
{
    analogWrite(COIL1_FWD, PWM);

    analogWrite(COIL2_FWD, PWM);

    delay(10);

    analogWrite(COIL1_FWD, 0);
    analogWrite(COIL1_RVS, PWM);

    analogWrite(COIL2_FWD, 0);
    analogWrite(COIL2_RVS, PWM);

    delay(10);

    analogWrite(COIL1_RVS, 0);
    analogWrite(COIL2_RVS, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rvsRotate(uint8_t PWM)
{
    analogWrite(COIL1_RVS, PWM);

    analogWrite(COIL2_RVS, PWM);

    delay(10);

    analogWrite(COIL1_RVS, 0);
    analogWrite(COIL1_FWD, PWM);

    analogWrite(COIL2_RVS, 0);
    analogWrite(COIL2_FWD, PWM);

    delay(10);

    analogWrite(COIL1_FWD, 0);
    analogWrite(COIL2_FWD, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////