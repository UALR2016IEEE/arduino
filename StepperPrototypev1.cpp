//
// Created by Autumn Rouse on 2/1/16.
//

#include <AccelStepper.h>
#include <stdlib.h>


AccelStepper stepper1 (AccelStepper::DRIVER, 46, 47);
AccelStepper stepper2(AccelStepper::DRIVER, 48, 49);
AccelStepper stepper3(AccelStepper::DRIVER, 50, 51);
AccelStepper stepper4(AccelStepper::DRIVER, 52, 53);
int enablePin = 32; //only 1 pin?

//define digital pins as drivers and digitalwrite out to them to enable motor controller, output 1 or 0, connecting
//to arduino, be able to enable individually (DONE)


//Variable declarations - doubles may be able to change to float(take up less memory)

double wheel_diameter = 38.00;   // in millimeters,
double circumference = 3.14159 * wheel_diameter;    //in millimeters
int step_size = 2;   //by default 1/2 stepper, has been changed
float dist1, dist2, dist3, dist4 = 0;
int inputDist1, inputDist2, inputDist3, inputDist4;
int step1, step2, step3, step4 = 0;  //may change to array
const long MAXSPEED = 15000;
const long ACCELERATION = 600;

void parseSerial();
void line(int length, double vector);
void mathHelper(int length, double vector);
long stepHelper (double len);

void setup()
{
    Serial.begin(115200);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);

    //Configure steppers
    stepper1.setMaxSpeed(MAXSPEED);
    stepper1.setAcceleration(ACCELERATION);

    stepper2.setMaxSpeed(MAXSPEED);
    stepper2.setAcceleration(ACCELERATION);

    stepper3.setMaxSpeed(MAXSPEED);
    stepper3.setAcceleration(ACCELERATION);

    stepper4.setMaxSpeed(MAXSPEED);
    stepper4.setAcceleration(ACCELERATION);

    inputDist1, inputDist4 = 10000;
    inputDist2, inputDist3 = -10000;

}

void loop()
{
    parseSerial();

}


void parseSerial ()
{
    char bufferInt[4];
    char bufferFloat [5];
    int length;
    double angle;

    //case statement for serial language
    //take in info and put it in right area - read each line - recast them to right type
    if (Serial.available() > 0) {
        int inByte = Serial.read();

        switch (inByte) {
            case 'L':
                Serial.readBytes(bufferInt, 4);
                length = atoi(bufferInt);
                Serial.readBytes(bufferFloat, 5);
                angle = atof(bufferFloat);
                line(length, angle);
                break;


        }
    }
}

void line(int length, double angle)
{

    int rawDistance = length;
    int rawRadians = angle;
    long x, y;

    x = length * cos(angle);
    y = length * sin(angle);

    long x_steps = stepHelper(x);
    long y_steps = stepHelper(y);

    stepper1.moveTo(x_steps);
    stepper2.moveTo(-y_steps);
    stepper3.moveTo(y_steps);
    stepper4.moveTo(-x_steps);

}

long stepHelper (double len)
{
    long step1  = len / circumference;
    step1 = step1 * step_size;
    return step1;
    //figure how far each wheel needs to go
    //can be another fuction - angle to arc length - to rotations - to steps (rotation)
    //want to get to arcs (non linear function)
}