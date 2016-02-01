//
// Created by Autumn Rouse on 12/7/15.
//

#include <AccelStepper.h>


AccelStepper stepper1 (AccelStepper::DRIVER, 46, 47);
AccelStepper stepper2(AccelStepper::DRIVER, 48, 49);
AccelStepper stepper3(AccelStepper::DRIVER, 50, 51);
AccelStepper stepper4(AccelStepper::DRIVER, 52, 53);

//Variable declarations - doubles may be able to change to float(take up less memory)

double wheel_diameter = 38.00;   // in millimeters,
double Circum = 3.14159 * wheel_diameter;    //in millimeters
int step_size = 4;   //by default 1/4 stepper
int stepsToTake1, stepsToTake2, stepsToTake3, stepsToTake4 = 0;
double distanceToTravel[4];  //needs to be in millimeters (array)
float acc1, acc2, acc3, acc4;     //may change to array
int r_degree;
long positions[4];
int step1, step2, step3, step4 = 0;  //may change to array
int dir1, dir2, dir3, dir4;    //may change to array

//funtion declaration
void rotate(r_degree);
void line(distanceToTravel);

void setup()
{
    Serial.begin(9600);

    //Configure steppers
    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(acc1);
    stepper1.moveTo(step1);

    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(acc2);
    stepper2.moveTo(step2);

    stepper3.setMaxSpeed(1000);
    stepper3.setAcceleration(acc3);
    stepper3.moveTo(step3);

    stepper4.setMaxSpeed(1000);
    stepper4.setAcceleration(acc4);
    stepper4.moveTo(step4);


}

void loop()
{

    //array of stepper positions
    long positions[4];
    positions[0] = step1;
    postions[1] = step2;
    postions[2] = step3;
    postions[3] = step4;
    stepper1.moveTo(positions[0]);
    stepper2.moveTo(positions[0]);
    stepper3.moveTo(positions[0]);
    stepper4.moveTo(positions[0]);

    for

}


void line(distanceToTravel)   //need to figure how to input from array
{
    //Need to bring in the move to
    step1 = (distanceToTravel / (Circum / step_size));
    step2 = step1;

}

/*void rotate(r_degree)
{
    if (r_degree = 0)
    {
        step1, step2 = 0;
    }
    if (r_degree = 90)
    {
        step1 = (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
        //need help with step2 ??????
        step2 = -(((wheel_diameter/2)*pi)/4) / (Circum / step_size);

    }
    if (r_degree = 180)
    {
        step1 = 2 * (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
        step2 = -2 * (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
    }
    if (r_degree = 270)
    {
        step1 = 3 * (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
        step2 = -3 * (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
    }
    if (r_degree = 360)
    {
        step1 = 4 * (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
        step2 = -4 * (((wheel_diameter/2)*pi)/4) / (Circum / step_size);
    }

}*/