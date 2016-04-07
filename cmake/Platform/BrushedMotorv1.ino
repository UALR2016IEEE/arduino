//
// Created by Autumn Rouse on 3/7/16.
//Set mode to 7 (packet serial address 0x80) and option to 4(38400) on first roboclaw
//Set mode to 8(address 0x81) on seconds roboclaw
//all definitions need to be tuned at competition

#include <Arduino.h>
#include <stdlib.h>
#include <PWMServo.h>
#include <Servo.h>

//Roboclaw Address
#define address1 0x80
#define address2 0x81
//Rail Constants - will change with testing
#define allUp 180
#define halfWay 170
#define allDown 0
//Claw Constants - will change with testing
#define loosePos 102
#define openPos 159
#define closePos 98
//Hold Servo Constants - these values are not confirmed
#define engageHold 180
#define deEngageHold 0


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
int claw = SERVO_PIN_C; //pin
int hold_pin = 12;
int serialPin = 22; // pin
bool lightState = false;
bool railEngage = false;

int redVar, blueVar, greenVar;
volatile int buttonState = HIGH;
unsigned long lastIntTime = 0;
unsigned long lastBlinkTime = 0;
PWMServo myClaw;
Servo myRail;
Servo holdServo;

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
void lift(int height); //make these definitions: allUp, halfWay, allDown
void lower(int height, int endHeight);
void open();
void close();
void pickUp();
void letDown();
void engage();
void deEngage();
void loosen();
void returnState();
void railTransit();
void setLight(int redVar, int greenVar, int blueVar);


void setup()
{
    //Communicate with roboclaw at 38400bps
    Serial.begin(115200);
    Serial1.begin(38400);
    //pinMode(ir_sensor1, INPUT);
    //pinMode(ir_sensor2, INPUT);
    pinMode(enablePin, OUTPUT);
    controlEn(buttonState);
    pinMode(buttonPin, OUTPUT);
    pinMode(interruptPin, INPUT);
    pinMode(serialPin, INPUT);
    pinMode(red, OUTPUT);
    pinMode(green, OUTPUT);
    pinMode(blue, OUTPUT);
    myClaw.attach(claw);
    close();
    holdServo.attach(hold_pin);
    deEngageHold();
    myRail.attach(slideRail);
    railTransit();
    myRail.detach();

    attachInterrupt(1, turnOn, FALLING);

}

void loop()
{
    button();
    serialEvent();
    serial1Check();
    //getIR();
}

void setLight(int r, int g, int b, float bright)
{
    float brightness =  bright;
    redVar = (255 -r) * brightness;
    greenVar = (255 - g) * brightness;
    blueVar = (255 - b) * brightness;


    analogWrite(red, redVar);
    analogWrite(green, greenVar);
    analogWrite(blue, blueVar);

}
void ledLight()
{
    //have led light red when not in use, reversed for PWM, 0 is highest
    if (buttonState == LOW)
    {
        setLight(255, 0, 0, .5);
    }
    else if (buttonState == HIGH) //green when in use
    {
        setLight(0, 255, 0, 1);
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
            setLight(0, 255, 0, .5);
            setLight(255, 0, 0, .5);
        }
        else
        {
            int inByte = Serial.read();

            switch (inByte) {
                case '8':
                    preparePickUp();

                case '7':
                    pickUp();
                    break;

                case '6':
                    letDown();
                    break;

                case 'c':
                    turnOn();
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
        //purple
        setLight(255, 0, 255, .5);

    }
}

void serial1Write()
{
    int i = 0;
    Serial.write('1');
    int packet_len = Serial.read();
    byte buf[200];

    for(int x=0; x < packet_len; x++)
    {
        while(!Serial.available())
        {}

        Serial1.write(Serial.read());
    }
    Serial.write(packet_len);
    Serial.write('1');

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

void preparePickUp() {
    myRail.attach(slideRail);
    //open claw
    open();
    //lower rails - allDown
    lower(allDown);
    Serial.write('8');
}

void pickUp()
{
    //close claw
    close();
    //lift rails - allUp
    lift(allUp);
    //engage second servo to close position
    engage();
    //lower rails to half high
    lift(halfWay); //may have to change to gradual lower
    //release claw a couple of marks
    loosen();
    myRail.detach();
    Serial.write('7');

}

void letDown()
{
    myRail.attach(slideRail);
    //close claw completely
    close();
    //lift rails all the way
    lift(allUp);
    //de-engage second servo to open position
    deEngage();
    //lower rails all the way down (slowly)
    lower(allDown);
    //open claw
    open();
    //raise to halfway
    lift(halfWay);
    delay(500);
    railTransit();
    myRail.detach();
    Serial.write('6');
}

void lift(int height)
{
    //may just write the value instead of the if
    //blue
    setLight(0, 0, 255, 1);
    myRail.write(height);
}

void lower(int endHeight)
{
    //blue
    setLight(0, 0, 255, .4);
    int heightNow = myRail.read();

    for (int height = heightNow; height <= endHeight; height--)
    {
        //int val2 = map(i, 0, 4092, 0, 180); - dont need, may need to look at
        //run test and print without mapping to get highest and lowest number for the rail
        myRail.write(height);
        delay(2);
        //do I need minimal delay (will see in testing)
    }
}

void railTransit()
{
    myRail.write(halfWay);
}

void close()
{
    //can be changed at competition to make sure it is right, needs to be tested with actual victim peg
    myClaw.write(closePos);
    delay(15);
    //yellow
    setLight(0, 255, 255, 1);
}

void open()
{
    //yellow
    setLight(255, 0,0, 1);
    myClaw.write(openPos);
    delay(15);
}

void loosen()
{
    //to loosen the claw for travel - part to not kill servo
    myClaw.write(loosePos);
    //see if I need delay? - will see in testing
}

void engage()
{
    //engage to be under the victim (values will come from testing
    holdServo.write(engageHold);
}

void deEngage()
{
    holdServo.write(deEngageHold);
}

void returnState()
{
    Serial.write(buttonState);
}


