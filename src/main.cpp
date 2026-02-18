#include <Arduino.h>
#include <StepperMC.h>

// the number of steps on your motor
#define STEPS 200

#define STP_PIN 15
#define DIR_PIN 14
#define EN_PIN 16


StepperMC stepper(DIR_PIN, STP_PIN, STEPS);

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
    
  pinMode(EN_PIN, OUTPUT);    //Set the enable signal port to output mode


  digitalWrite(EN_PIN, LOW);   //Set enable signal high level

  stepper.setIncrementsRelative(1000);
  stepper.setSpeed(5000, 0);               //delay 1000ms
}

void loop() {
    stepper.moveTarget();                //delay 1000ms
    stepper.setIncrementsRelative(1000);
}