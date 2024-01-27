#include "SignalStackSoftware.h"
#include <Wire.h>

void setup()
{
    //Serial Debugger
    Serial.begin(115200);
    Serial.println("Rotating Signal Stack Setup");

    // Communication setup
    Serial.begin(9600);
    //RoveComm.begin(RC_SIGNALSTACKBOARD_FIRSTOCTET, RC_SIGNALSTACKBOARD_SECONDOCTET, RC_SIGNALSTACKBOARD_THIRDOCTET, RC_SIGNALSTACKBOARD_FOURTHOCTET, &TCPServer);
    Telemetry.begin(telemetry, 1500000);

    //IO PINS
    pinMode(JOG_A, INPUT);
    pinMode(JOG_B, INPUT);
    pinMode(SCL, OUTPUT);
    pinMode(SDA, INPUT);
    pinMode(MAG_ENABLE, OUTPUT);
    pinMode(HB_FWD, OUTPUT);
    pinMode(HB_RVS, OUTPUT);
    pinMode(COMP_PWR, OUTPUT);

    //Motor
    Motor.configInvert(false);
    Motor.configMaxOutputs(-1000, 1000);
    Motor.configMinOutputs(-50, 50);
    Motor.configRampRate(1000);

    //Compass
    Wire.begin();
    Compass.initCompass();
    Compass.setScale(0.88);
}

void loop()
{
    // Receive RoveComm commands
    //packet = RoveComm.read();
    
    Serial.println(Compass.getCompass());
    switch (packet.data_id)
    {
        //case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID: {
            //motorSpeed = *((int16_t*) packet.data);
        //}
    }

    Motor.drive(motorSpeed);
}

void telemetry() {

}