//
// Created by Autumn Rouse on 3/7/16.
//Set mode to 7 (packet serial address 0x80) and option to 4(38400) on first roboclaw
//Set mode to 8(address 0x81) on seconds roboclaw

#include <Arduino.h>
#include <stdlib.h>
#include <Servo.h>

//Roboclaw Address
#define address1 0x80
#define address2 0x81

#define ir_sensor1 1 //Sharp IR (4-30cm, analog)
#define ir_sensor2 2 //

int enablePin = 36; //digital pin - motor enable
int buttonPin = 38; //digital pin - button light up
int interruptPin = 3; //digital pin - enable and disable interrupt
int valueOn = 0; // used as toggle for on and off
int red = 11; //pwm of led
int green = 10; //pwm of led
int blue = 9; //pwm of led
int slideRail = 4;
int claw = 5;
int railLength = 100; // length in mm
//int railMaxHeight = 4092; // max value of height input, 5V
double liftFactor = 1;
volatile int buttonState = LOW;
Servo myClaw;
Servo myRail;

void getIR();
void serialEvent();
void serial1Check();
void serial2Check();
void serial1Write();
void serial2Write();
void ledLight();
void turnOn();
void controlEn(int buttonState);
void button();
void lift();
void lower();
void open();
void close();
void setup()
{
    //Communicate with roboclaw at 38400bps
    Serial.begin(115200);
    Serial1.begin(38400);
    Serial2.begin(38400);
    pinMode(enablePin, OUTPUT);
    pinMode(buttonPin, OUTPUT);
    pinMode(interruptPin, INPUT);
//    pinMode(red, OUTPUT);
//    pinMode(green, OUTPUT);
//    pinMode(blue, OUTPUT);
    attachInterrupt(1, turnOn, CHANGE);
    myClaw.attach(claw);
    myRail.attach(slideRail);


}

void loop()
{
    // ledLight();
    button();
    serialEvent();
    getIR();
    serial1Check();
    serial2Check();
    //put in status light function - RGB

}

void ledLight()
{
    /*have led red when not in use
    if (valueOn == 0)
    {
        analogWrite(red, 255);
        analogWrite(green, 3);
    }*/


}

void button()
{
    if (buttonState == LOW)
    {
        digitalWrite(buttonPin, HIGH);
        delay(1000);
        digitalWrite(buttonPin, LOW);
        delay(1000);
    }
    else
    {
        digitalWrite(buttonPin, HIGH);
    }
}
void turnOn()
{
    buttonState = !buttonState;
    controlEn(buttonState);
    //turn led on in the button, make it flash when it is turning on, 50mA button - figure resistor

}

void controlEn(int buttonState)
{
    digitalWrite(enablePin, buttonState);

}

void serialEvent()
{

    if (Serial.available() > 0)
    {
        int inByte = Serial.read();
        switch (inByte)
        {
            case 22:
                serial1Write();
                break;

            case 7:
                lift();
                break;

            case 6:
                lower();
                break;

            case 5:
                open();
                break;

            case 2:
                close();
                break;
        }
    }
}

void serial1Check()
{
    while (Serial1.available())
    {
        Serial.write(Serial1.read());
    }
}

void serial2Check()
{
    while (Serial2.available())
    {
        Serial.write(Serial2.read());
    }
}

void serial1Write()
{
    char indata[50];
    int numbytes = Serial.readBytesUntil(char(22), indata, 50);
    Serial1.write(indata, numbytes);
}

void serial2Write() //not needed
{
    //changed serial2Write to look like serial1Write
    char indata[50];
    int numbytes = Serial.readBytesUntil(char(22), indata, 50);
    Serial2.write(indata, numbytes);


    /* bool inputEvent2 = true;
    Serial2.write(81);
    while (inputEvent2) {
        int z = Serial.read();
        Serial2.write(z);
        if (z == 22) {
            inputEvent2 = false;
        }
    } */
}

void getIR ()
{
    float volts1 = analogRead(ir_sensor1) *0.0048828125; //value from sensor *(5/1024)
    float volts2 = analogRead(ir_sensor2) *0.0048828125;
    float distance1 = 13*pow(volts1, -1); //should i turn these into int?
    float distance2 = 13*pow(volts2, -1);

    if (distance1 <= 30)
    {
        //String dist1 = String(distance1);
        Serial.print('P');
        Serial.print(distance1, 6);
    }
    if (distance2 <= 30)
    {
        //String dist2 = String(distance2);
        Serial.print('P');
        Serial.print(distance2, 6);
    }
}

void lift()
{
    int val2 = 168; //all the way up
    //my not need the map effort since when testing I printed the mapped value
    //see what Kori says
    val2 = map(val2, 0, 1023, 0, 180);
    myRail.write(val2);
    delay(15); // is it necessary for it to get to position?
}

void lower()
{
    int val2 = 70; //all the way down - this needs need to be calibrated when installed on the robot to make sure
    val2 = map(val2, 0, 1023, 0, 180);
    myRail.write(val2);
    delay(15); // is it necessary for it to get to position?
}

void close()
{
    //can be changed at competition to make sure it is right, needs to be tested with actual victim peg
    int val1 = 110; // closed position, make sure %% maybe could put little rubber grips on tips to make more secure
    val1 = map(val1, 0, 1023, 0, 180);
    myClaw.write(val1);
    delay(15);
}

void open()
{
    //this valued is taken from its rating sheet, rated at 180 degree only hits 160
    int val1 = 160; // open position, make sure
    val1 = map(val1, 0, 1023, 0, 180); //may
    myClaw.write(val1);
    delay(15);
}


