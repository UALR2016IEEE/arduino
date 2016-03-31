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



//#define ir_sensor1 1 //Sharp IR (4-30cm, analog)
//#define ir_sensor2 2 //

//pin definitions
int enablePin = 46; //digital pin - motor enable
int buttonPin = 50; //digital pin - button light up
int interruptPin = 3; //digital pin - enable and disable interrupt
int red = 11; //pwm of led, pin
int green = 10; //pwm of led, pin
int blue = 9; //pwm of led, pin
int slideRail = 4; //pin
int claw = 5; //pin
int serialPin = 22; // pin
bool lightState = false;

int redVar, blueVar, greenVar;
volatile int buttonState = HIGH;
unsigned long lastIntTime = 0;
unsigned long lastBlinkTime = 0;
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
void returnState();
void setLight(int redVar, int greenVar, int blueVar);


void setup()
{
    //Communicate with roboclaw at 38400bps
    Serial.begin(115200);
    Serial1.begin(38400);
    //pinMode(ir_sensor1, INPUT);
    //pinMode(ir_sensor2, INPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(buttonPin, OUTPUT);
    pinMode(interruptPin, INPUT);
    pinMode(serialPin, INPUT);
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
    myClaw.attach(claw);
    myRail.attach(slideRail);

    attachInterrupt(1, turnOn, FALLING);

}

void loop()
{
    button();
    serialEvent();
    serial1Check();
    //getIR();
}

void setLight(int r, int g, int b)
{
    redVar = r;
    greenVar = g;
    blueVar = b;

    analogWrite(red, redVar);
    analogWrite(green, greenVar);
    analogWrite(blue, blueVar);

}
void ledLight()
{
    //have led light red when not in use, reversed for PWM, 0 is highest
    if (buttonState == LOW)
    {
        setLight(0, 252, 255);
    }
    else if (buttonState == HIGH) //green when in use
    {
        setLight(251, 0, 255);
    }
}

void button() //the button led light function
{
    if (buttonState == LOW)
    {
        if (millis() - lastBlinkTime > 1000)
        {
            lightState = !lightState;
            digitalWrite(buttonPin, lightState);
            lastBlinkTime = millis();
        }
    }
    else if (buttonState == HIGH)
    {
        digitalWrite(buttonPin, HIGH);
    }
}
void turnOn()
{
    if (millis() - lastIntTime > 200)
    {
        buttonState = !buttonState;
        controlEn(buttonState);
        lastIntTime = millis();
    }
}

void controlEn(int buttonState)
{
    digitalWrite(enablePin, buttonState);
}

void serialEvent()
{
    if (Serial.available() > 0)
    {
        if (digitalRead(serialPin) == HIGH)
        {
            Serial1.write(Serial.read());
            setLight(255, 0, 255);
            setLight(0, 255, 255);
        }
        else
        {
            int inByte = Serial.read();

            switch (inByte) {
                case '7':
                    lift();
                    break;

                case '6':
                    lower();
                    break;

                case '5':
                    open();
                    break;

                case '2':
                    close();
                    break;

                case 'b':
                    returnState();
                    break;
            }
        }
    }
}

void serial1Check()
{
    while (Serial1.available() > 0)
    {
        Serial.write(Serial1.read());
        setLight(210, 255, 0);
        //purple

    }
}

void serial1Write()
{
    int i = 0;
    Serial.write('1');
    int packet_len = Serial.read();
    byte buf[200];

    for(int x=0; x < packet_len; x++){
        while(!Serial.available()){}
        Serial1.write(Serial.read());
    }
    Serial.write(packet_len);
    Serial.write('1');

    //red
    setLight(0,255,255);

}

//method to be included if using IR sensors
/*void getIR ()
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
}*/

//look at defining all numbers at top for 4 functions
void lift()
{
    int val2 = 4092; //all the way up, was 168 in mapped number when tested
    val2 = map(val2, 0, 4092, 0, 180);
    myRail.write(val2);
    delay(15); // is it necessary for it to get to position?
    //blue
    setLight(255, 255, 0);
}

void lower()
{
    //was 70 in in mapped number when tested
    int val2 = 0; //all the way down - this needs need to be calibrated when installed on the robot to make sure
    val2 = map(val2, 0, 4092, 0, 180);
    myRail.write(val2);
    delay(15); // is it necessary for it to get to position?
    //blue
    setLight(255, 255, 0);
}

void close()
{
    //can be changed at competition to make sure it is right, needs to be tested with actual victim peg
    int val1 = 108; // closed position, make sure %% maybe could put little rubber grips on tips to make more secure
    myClaw.write(val1);
    delay(15);
    //yellow
    setLight(255, 0, 0);
}

void open()
{
    //this valued is taken from its rating sheet, rated at 180 degree only hits 160
    int val1 = 166; // open position, make sure
    myClaw.write(val1);
    delay(15);
    //yellow
    setLight(255, 0,0);
}

void returnState()
{
    Serial.write(buttonState);
}


