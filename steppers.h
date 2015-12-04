//
// Created by Autumn Rouse on 12/2/15.
//

#ifndef steppers_h
#define steppers_h

#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper1 (AccelStepper::DRIVER, 52, 53);
AccelStepper stepper2 (AccelStepper::DRIVER, 50, 51);
AccelStepper stepper3 (AccelStepper::DRIVER, 48, 49);
AccelStepper stepper4 (AccelStepper::DRIVER, 46, 47);


void setup()
{
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(acc_value1);
    stepper1.moveTo(pos_value1);

    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(acc_value2);
    stepper2.moveTo(pos_value2);

    stepper3.setMaxSpeed(1000.0);
    stepper3.setAcceleration(acc_value3);
    stepper3.moveTo(pos_value3);

    stepper4.setMaxSpeed(1000.0);
    stepper4.setAcceleration(acc_value4);
    stepper4.moveTo(pos_value4);

}

void loop()
{
    //change direction at the limits
    if (stepper1.distanceToGo() == 0);
        stepper.moveTo(-stepper.currentPosition());
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
}

#endif //steppers_h

