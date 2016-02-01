//
// Created by Autumn Rouse on 2/1/16.
//

#include <AccelStepper.h>


AccelStepper stepper1 (AccelStepper::DRIVER, 46, 47);
AccelStepper stepper2(AccelStepper::DRIVER, 48, 49);
AccelStepper stepper3(AccelStepper::DRIVER, 50, 51);
AccelStepper stepper4(AccelStepper::DRIVER, 52, 53);

//Variable declarations - doubles may be able to change to float(take up less memory)

double wheel_diameter = 38.00;   // in millimeters,
double circumference = 3.14159 * wheel_diameter;    //in millimeters
int step_size = 4;   //by default 1/4 stepper
float acc1, acc2, acc3, acc4;     //may change to array
float dist1, dist2, dist3, dist4 = 0;
float inputDist1, inputDist2, inputDist3, inputDist4;
int step1, step2, step3, step4 = 0;  //may change to array
//int dir1, dir2, dir3, dir4 = 0;    //unnecessary ? (taken care of by negative or positive)

void setup()
{
    Serial.begin(9600);
    Serial.flush();
    //Configure steppers
    stepper1.setMaxSpeed(1000);
    stepper1.setAcceleration(acc1);

    stepper2.setMaxSpeed(1000);
    stepper2.setAcceleration(acc2);

    stepper3.setMaxSpeed(1000);
    stepper3.setAcceleration(acc3);

    stepper4.setMaxSpeed(1000);
    stepper4.setAcceleration(acc4);

    pinMode(44, OUTPUT);
    digitalWrite(44, HIGH);

    pinMode (42, INPUT);
    int input = digitalRead(42);

    acc1, acc2, acc3, acc4 = 200;
    inputDist1, inputDist4 = 10000;
    inputDist2, inputDist3 = -10000;

}

void loop()
{
    /*if (Serial.available > 0)
    {

        SerialData = Serial.read();
        Serial.println(serialData); */
    line(inputDist1, inputDist2, inputDist3, inputDist4);
    // }


}

void line(float inputDist1, float inputDist2, float inputDist3, float inputDist4)
{
    dist1 = inputDist1;
    dist2 = inputDist2;
    dist3 = inputDist3;
    dist4 = inputDist4;

    if (dist1 > 0) //distance in millimeters
    {
        step1  = dist1 / circumference;
        step1 = step1 * step_size;
        stepper1.moveTo(step1);
        stepper1.run();
    }
    else
    { }


    if (dist2 > 0) //distance in millimeters
    {
        step2  = dist2 / circumference;
        step2 = step2 * step_size;
        stepper2.moveTo(step2);
        stepper2.run();
    }
    else
    { }

    if (dist3 > 0) //distance in millimeters
    {
        step3  = dist3 / circumference;
        step3 = step3 * step_size;
        stepper3.moveTo(step3);
        stepper3.run();
    }
    else
    { }

    if (dist4 > 0) //distance in millimeters
    {
        step4  = dist4 / circumference;
        step4 = step4 * step_size;
        stepper4.moveTo(step4);
        stepper4.run();
        return;
    }
    else
    { }


}