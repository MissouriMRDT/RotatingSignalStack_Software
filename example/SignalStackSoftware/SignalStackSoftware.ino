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
    GPS_SERIAL.begin(GPS_SERIAL_BAUD);
    
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

    // GPS Code
    unsigned long start = millis();
    // Every 5 seconds we print an update
    while (millis() - start < 5000) 
    {
        if (GPS_SERIAL.available()) 
        {
            char c = GPS_SERIAL.read();
            //Serial.print(c);  // uncomment to see raw GPS data
            if (gps.encode(c)) 
            {
                newGPSData = true;
                //break;  // uncomment to print new data immediately!
            }
        }
    }
    
    if (newGPSData)
    {
        Serial.println("Acquired Data");
        Serial.println("-------------");
        gpsDump();
        updateGPS();
        Serial.println("-------------");
        Serial.println();
    }
    
    updateCompass();
    Telemetry();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Telemetry()
{
    if (newGPSData) {
        RoveComm.write(RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_ID, RC_SIGNALSTACKBOARD_SIGNALSPOSITION_DATA_COUNT, signalStackPosition);
    }
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
    static float lat,lon;
    static unsigned long age;
    gps.f_get_position(&lat, &lon, &age);
    signalStackPosition[0] = lat;
    signalStackPosition[1] = lon; 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GPSDump()
{
    static long lat, lon;
    static unsigned long age, date, time, chars;
    static int year;
    byte month, day, hour, minute, second, hundredths;
    static unsigned short sentences, failed;

    gps.get_position(&lat, &lon, &age);
    Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
    
    // On Arduino, GPS characters may be lost during lengthy Serial.print()
    // On Teensy, Serial prints to USB, which has large output buffering and
    //   runs very fast, so it's not necessary to worry about missing 4800
    //   baud GPS characters.

    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

    gps.get_datetime(&date, &time, &age);
    Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
    Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
    Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths));
    Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

    Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
    gps.stats(&chars, &sentences, &failed);
    Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
    Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void goToAngle(uint16_t deg)
{
    // code go brrr
}