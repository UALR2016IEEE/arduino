//
// Created by Autumn Rouse on 12/3/15.
//

#include <Arduino.h>
#include "distance_ir_sensor.h"

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
    Serial.begin(9600);
    Dist_1.begin(A0);   //may have to change look at arduino
    Dist_2.begin(A1);   //may have to change look at arduino
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

