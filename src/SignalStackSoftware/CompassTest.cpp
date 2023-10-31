// Compass Module Test Script

#include "CompassTest.h"
#include <Wire.h>

void setup()
{
    Wire.begin();
    Serial.begin(9600);
}

void loop()
{
    Wire.beginTransmission(comp_address);
}