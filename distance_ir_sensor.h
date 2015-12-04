//
// Created by Autumn Rouse on 12/3/15.
//


#ifndef distance_ir_sensor_h
#define distance_ir_sensor_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
//may not need
#include"WProgram.h"
#include <pins_arduino.h>
#endif


#include "AnalogDistanceSensor.h"


class distance_ir_sensor : public AnalogDistanceSensor
{

    public:
        distance_ir_sensor();
        int getDistanceCentimeter();
};

#endif //distance_ir_sensor_h
