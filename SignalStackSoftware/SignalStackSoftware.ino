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
    Compass.init();
    //Compass.setScale(0.88);
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
            feedWatchdog();
        //}

        case RC_SIGNALSTACKBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            uint8_t watchdogOverride = ((uint8_t*) packet.data)[0];
            break;
        }
    }

    //Compass
    Compass.read();
    byte azimuth = Compass.getAzimuth();
    // Output here will be a value from 0 - 15 based on the direction of the bearing / azimuth.
    byte compassAngle = Compass.getBearing(azimuth);
    
    Motor.drive(motorSpeed);

    if (Watchdog.IntervalTimer = 0) {
        watchdogStarved = 1;
        haultMotors();
    }
}

void telemetry() {
    RoveComm.write(RC_SIGNALSTACKBOARD_WATCHDOGSTATUS_DATA_ID, RC_SIGNALSTACKBOARD_WATCHDOGSTATUS_DATA_COUNT, watchdogStarved);
    RoveComm.write(RC_SIGNALSTACKBOARD_COMPASSANGLE_DATA_ID, RC_SIGNALSTACKBOARD_COMPASSANGLE_DATA_COUNT, compassAngle);
}

void feedWatchdog() {
    watchdogStarved = 0;
    Watchdog.begin(haultMotors, WATCHDOG_TIMEOUT);
}

void haultMotors() {
    if (watchdogOverride == false) {
        //Turn off motors
    }
}