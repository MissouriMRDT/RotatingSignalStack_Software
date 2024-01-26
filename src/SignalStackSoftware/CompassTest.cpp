// Compass Module Test Script

#include "CompassTest.h"
#include <Wire.h>

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    while(!Serial); // wait for serial monitor
}

void loop()
{
    Wire.requestFrom(comp_address);
    while(Wire.available(comp_address))
    {
        data = Wire.read();
        Serial.print(data);
    }
    delay(1000);
}