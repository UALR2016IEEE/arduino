//
// Created by Autumn Rouse on 12/2/15.
//

#ifndef ARDUINO_MAIN_H
#define ARDUINO_MAIN_H

#include <AccelStepper.h>
#include <MultiStepper.h>


//200 steps per second, 4 stepper motors max speed

AccelStepper stepper1 (AccelStepper::DRIVER, 52, 53); //On pins 2,
AccelStepper stepper2 (AccelStepper::DRIVER, 50, 51);
AccelStepper stepper3 (AccelStepper::DRIVER, 48, 49);
AccelStepper stepper4 (AccelStepper::DRIVER, 46, 47);
//MultiStepper steppers;

void setup()
{
    //steppers.addStepper(stepper1);
    stepper1.setMaxSpeed(1000);
    stepper1.setSpeed(1000);
    stepper2.setMaxSpeed(1000);
    stepper2.setSpeed(1000);
    stepper3.setMaxSpeed(1000);
    stepper3.setSpeed(1000);
    stepper4.setMaxSpeed(1000);
    stepper4.setSpeed(1000);



}

void loop() {
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
    stepper4.runSpeed();
    //stepper1.run();
}

#endif //ARDUINO_MAIN_H
AccelStepper stepper1; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
AccelStepper stepper3(AccelStepper::FULL2WIRE, 10, 11);
void setup()
{
    stepper1.setMaxSpeed(200.0);
    stepper1.setAcceleration(100.0);
    stepper1.moveTo(24);

    stepper2.setMaxSpeed(300.0);
    stepper2.setAcceleration(100.0);
    stepper2.moveTo(1000000);

    stepper3.setMaxSpeed(300.0);
    stepper3.setAcceleration(100.0);
    stepper3.moveTo(1000000);
}
void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
        stepper1.moveTo(-stepper1.currentPosition());
    stepper1.run();
    stepper2.run();
    stepper3.run();
}
