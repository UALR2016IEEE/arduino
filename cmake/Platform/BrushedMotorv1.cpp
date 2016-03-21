//
// Created by Autumn Rouse on 3/7/16.
//Set mode to 7 (packet serial address 0x80) and option to 4(38400) on first roboclaw
//Set mode to 8(address 0x81) on seconds roboclaw

#include <Arduino.h>
#include <stdlib.h>

//Roboclaw Address
#define address1 0x80
#define address2 0x81

#define ir_sensor1 1 //Sharp IR (4-30cm, analog)
#define ir_sensor2 2 //

//Setup communications with roboclaw. Use pins 19 - RX1, 18 - TX1,
// 17 - RX2, 16 - TX2

void getIR();
void serialEvent();
void serial1Check();
void serial2Check();
void serial1Write();
void serial2Write();

void setup()
{
    //Communicate with roboclaw at 38400bps
    Serial.begin(115200);
    Serial1.begin(38400);
    Serial2.begin(38400);
}

void loop()
{
    serialEvent();
//    //getIR();
    serial1Check();
//    serial2Check();

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
        }
    }
}

void serial1Check()
{
    while (Serial1.available()){
        Serial.write(Serial1.read());
    }
}

void serial2Check()
{
    while (Serial2.available()){
        Serial.write(Serial2.read());
    }
}

void serial1Write()
{
    char indata[50];
    int numbytes = Serial.readBytesUntil(char(22), indata, 50);
    Serial1.write(indata, numbytes);
}

void serial2Write()
{
    bool inputEvent2 = true;
    Serial2.write(81);
    while (inputEvent2) {
        int z = Serial.read();
        Serial2.write(z);
        if (z == 22) {
            inputEvent2 = false;
        }
    }
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

