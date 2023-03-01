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
    Serial.begin(9600);
    //RoveComm.begin(RC_SIGNALSTACKBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_SIGNALSTACKBOARD_MAC);
    delay(100);
    telemetry.begin(Telemetry, 1500000);
    Serial.println("Started: ");
    Serial1.begin(115200);
    
    // motor pins
    pinMode(COIL1_FWD, OUTPUT);
    pinMode(COIL1_RVS, OUTPUT);
    pinMode(COIL2_FWD, OUTPUT);
    pinMode(COIL2_RVS, OUTPUT);

    // compass pins
    Wire.setSDA(COMPASS_SDA);
    Wire.setSCL(COMPASS_SCL);

    // GPS pins
    Serial1.setRX(GPS_RX);
    Serial1.setTX(GPS_TX);

    analogWrite(COIL1_FWD, 125);
    analogWrite(COIL2_FWD, 125);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
    // Receive RoveComm commands
    //packet = RoveComm.read();
    //
    //switch (packet.data_id)
    //{
    //    case RC_SIGNALSTACKBOARD_SIGNALSROTATE_DATA_ID:
    //        stepNumber = ((int16_t*) packet.data)[0];
    //        stackRotate(stepNumber);
    //        break;
    //}

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

void stackRotate(int16_t numSteps)  // Moves the motor a requested number of steps; positive = forward, negative = backward
{   
    if (numSteps>0)
    {
        for (uint8_t i=0; i<numSteps; i++)
        {
            motorStep(motorPWM);
            if (!(stepCount % 4)) stepCount = 1;    // if on the 4th step, go back to 1
            else stepCount++;                       // these 2 lines cycle the stepCount from 1 to 4 to know which step the motor is on
        }
    }

    if (numSteps<0)
    {
        for (uint8_t i=0; i>numSteps; i--)
        {
            motorStep(motorPWM);
            if (stepCount==1) stepCount = 4;        // if on the 1st step, go back to 4
            else stepCount--;                       // these 2 lines cycle the stepCount from 4 to 1
        }
    }

    analogWrite(COIL1_FWD, 0);
    analogWrite(COIL1_RVS, 0);
    analogWrite(COIL2_FWD, 0);
    analogWrite(COIL2_RVS, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motorStep(uint8_t PWM) // Writes to motor pins to perform appropriate step on the stepper motor
{
    switch(stepCount)
    {
        case 1:
            analogWrite(COIL1_FWD, PWM);
            analogWrite(COIL1_RVS, 0);
            analogWrite(COIL2_FWD, PWM);
            analogWrite(COIL2_RVS, 0);
            break;
        
        case 2:
            analogWrite(COIL1_FWD, 0);
            analogWrite(COIL1_RVS, PWM);
            analogWrite(COIL2_FWD, PWM);
            analogWrite(COIL2_RVS, 0);
            break;
        
        case 3:
            analogWrite(COIL1_FWD, 0);
            analogWrite(COIL1_RVS, PWM);
            analogWrite(COIL2_FWD, 0);
            analogWrite(COIL2_RVS, PWM);
            break;
        
        case 4:
            analogWrite(COIL1_FWD, PWM);
            analogWrite(COIL1_RVS, 0);
            analogWrite(COIL2_FWD, 0);
            analogWrite(COIL2_RVS, PWM);
            break;
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void goToAngle(uint16_t deg)
{
    // code go brrr
}