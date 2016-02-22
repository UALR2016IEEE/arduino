//
// Created by Autumn Rouse on 2/1/16.
//

#include <AccelStepper.h>
#include <stdlib.h>


AccelStepper stepper1 (AccelStepper::DRIVER, 46, 47);
AccelStepper stepper2(AccelStepper::DRIVER, 48, 49);
AccelStepper stepper3(AccelStepper::DRIVER, 50, 51);
AccelStepper stepper4(AccelStepper::DRIVER, 52, 53);
int enablePin1 = 32;
int enablePin2 = 33;
int enablePin3 = 34;
int enablePin4 = 35;

//define digital pins as drivers and digitalwrite out to them to enable motor controller, output 1 or 0, connecting
//to arduino, be able to enable individually


//Variable declarations - doubles may be able to change to float(take up less memory)


int step_size = (360 / 1.8) * 2 * 5 * 1.05;   //1/2 stepper, has been changed
float dist1, dist2, dist3, dist4 = 0;
int inputDist1, inputDist2, inputDist3, inputDist4;
int step1, step2, step3, step4 = 0;  //may change to array

const double ROBOT_RADIUS = 136.91; // in millimeters
const double WHEEL_DIAMETER = 38.00;   // in millimeters,
const double CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;    //in millimeters
const long MAXSPEED = 300;
const long ACCELERATION = 60;

void parseSerial();
void line(int length, double angle);
void rotate(double angle);
void enableMotors(int enableInput);
long stepHelper (double len);

void setup()
{
    Serial.begin(115200);
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

    pinMode(enablePin1, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    pinMode(enablePin3, OUTPUT);
    pinMode(enablePin4, OUTPUT);


}

void loop()
{
    parseSerial();
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();


}


void parseSerial ()
{
    char bufferInt[4];
    char bufferFloat [6];
    char bufferEnable[1];
    int length;
    int enableInput;
    double angle;

    //case statement for serial language
    //take in info and put it in right area - read each line - recast them to right type
    if (Serial.available() > 0) {
        int inByte = Serial.read();

        switch (inByte) {
            case 'L':
                Serial.readBytes(bufferInt, 4);
                length = atoi(bufferInt);
                Serial.readBytes(bufferFloat, 6);
                angle = atof(bufferFloat);
                line(length, angle);
                break;
            case 'E':
                Serial.readBytes(bufferInt, 1);
                enableInput = atoi(bufferEnable);
                enableMotors(enableInput);
                break;

        }
    }
}

void line(int length, double angle)
{

    int rawDistance = length;
    int rawRadians = angle;
    long x, y;

    if (angle != 0)
    {
        rotate(rawRadians);
    }

    x = length * cos(angle); // i feel x and y are backwards x - sin, y - cos, check with them
    y = length * sin(angle);

    long x_steps = stepHelper(x);
    long y_steps = stepHelper(y);
   // Serial.println("steps calculated");
    //Serial.println(x_steps);
    //Serial.println(y_steps);
    stepper1.moveTo(x_steps);
    stepper2.moveTo(-y_steps);
    stepper3.moveTo(y_steps);
    stepper4.moveTo(-x_steps);

}

void rotate(double angle)
{
    double arcLength;
    double rawRadians = angle;
    long stepsRotate;

    arcLength = ROBOT_RADIUS * rawRadians;
    stepsRotate = stepHelper(arcLength);
    stepper1.moveTo(stepsRotate);
    stepper2.moveTo(stepsRotate);
    stepper3.moveTo(stepsRotate);
    stepper4.moveTo(stepsRotate);
    // s=r*theta, theta in radians

}

void enableMotors(int enableInput)
{
   int setEnable = enableInput;
    if (setEnable == 1)
    {
        digitalWrite(enablePin1, HIGH);
        delay(1000);
        digitalWrite(enablePin2, HIGH);
        delay(1000);
        digitalWrite(enablePin3, HIGH);
        delay(1000);
        digitalWrite(enablePin4, HIGH);
        delay(1000);
    }
    else
    {
        digitalWrite(enablePin1, LOW);
        delay(1000);
        digitalWrite(enablePin2, LOW);
        delay(1000);
        digitalWrite(enablePin3, LOW);
        delay(1000);
        digitalWrite(enablePin4, LOW);
        delay(1000);
    }
}


long stepHelper (double len)
{
    double step1  = len / CIRCUMFERENCE;
    step1 = step1 * step_size;
    return step1;
    //want to get to arcs (non linear function)
}