//
// Created by Autumn Rouse on 12/3/15.
//
//I think this is mostly right

#include <Arduino.h>
#include <Serial>;
//#include "distance_ir_sensor.h"

#define ir_sensor1 1 //Sharp IR (4-30cm, analog)
#define ir_sensor2 2

void setup()
{
    Serial.begin(9600);

}

void loop()
{
    float volts1 = analogRead(ir_sensor1) *0.0048828125; //value from sensor *(5/1024)
    float volts2 = analogRead(ir_sensor2) *0.0048828125;
    int distance1 = 13*pow(volts1, -1); //from datasheet
    int distance2 = 13*pow(volts2, -1);
    delay(100); //may not need delay

    if (distance1 <= 30)
    {
        Serial.println(distance1);
    }
    if (distance2 <= 30)
    {
        Serial.println(distance);
    }
}




/*
//variable declarations
distance_ir_sensor Dist_1;
distance_ir_sensor Dist_2;

//Constructor
distance_ir_sensor::distance_ir_sensor()
{
}
//getDistanceCentimeter(): Returns the distance in cetimeters: between 3-36cm (3 & 37 are boundary values)
int distance_ir_sensor::getDistanceCentimeter()
{

   float volts = analogRead(sensor1)*0.0048828125; //value from sensor * (5/1024)
    int adcValue=getDistanceRaw();
    if (adcValue > 600)             // lower boundary: 4 cm (3 cm means under the boundary)
    {
        return (3);
    }

    if (adcValue < 80)              // upper boundary: 36cm (returning 37 means over the boundary)
    {
        return (37);
    }

    else
    {
        return (1 / (0.000413153 * adcValue - 0.0055266887));
    }
}


void setup()
{
    Dist_1.begin(A0);   //may have to change pin look at arduino
    Dist_2.begin(A1);   //may have to change pin look at arduino
}

void loop()
{
    distance1 = Dist_1.getDistanceCentimeter();
    distance2 = Dist_2.getDistanceCentimeter();
    Serial.print("Sensor1: ");
    Serial.println(distance1);
    Serial.print("Sensor2: ");
    Serial.println(distance2);
    delay(500);                                 //make it readable to the RPi
}

*/