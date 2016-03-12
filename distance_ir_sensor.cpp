//
// Created by Autumn Rouse on 12/3/15.
//
//I think this is mostly right

#include <Arduino.h>
#include <stdlib.h>


#define ir_sensor1 1 //Sharp IR (4-30cm, analog)
#define ir_sensor2 2

void getIR();

void setup()
{
    Serial.begin(115200);

}

void loop()
{
    getIR();

}

void getIR ()
{
    float volts1 = analogRead(ir_sensor1) *0.0048828125; //value from sensor *(5/1024)
    float volts2 = analogRead(ir_sensor2) *0.0048828125;
    float distance1 = 13*pow(volts1, -1); //should i turn these into int?
    float distance2 = 13*pow(volts2, -1);

    if (distance1 <= 30)
    {
        String(distance1);
        Serial.write("P" + distance1);
    }
    if (distance2 <= 30)
    {
        String(distance2);
        Serial.write("P" + distance2);
    }
}