#include "SignalStackSoftware.h"

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
    pinMode(JOG_FWD, INPUT);
    pinMode(JOG_RVS, INPUT);
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
    Compass.init();
}

void loop()
{
    // Receive RoveComm commands
    //packet = RoveComm.read();
    
    Serial.println(Compass.getCompass());
    switch (packet.data_id)
    {
        case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID: {
            motorSpeed = *((int16_t*) packet.data);
            feedWatchdog();
        }

        case RC_SIGNALSTACKBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            watchdogOverride = ((uint8_t*) packet.data)[0];
            break;
        }
    }

    //Compass
    Compass.read();
    // Output here will be a value from 0 - 15 based on the direction of the bearing / azimuth.
    compassAngle = Compass.getAzimuth();
    
    //Buttons
    if (digitalRead(JOG_FWD)) {
        motorSpeed = 300;
    }
    
    else if (digitalRead(JOG_RVS)) {
        motorSpeed = -300;
    }

    Motor.drive(motorSpeed);
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
    if (watchdogOverride == 0) {
        motorSpeed = 0;
        Motor.drive(0);
    }
}