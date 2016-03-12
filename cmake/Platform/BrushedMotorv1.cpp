//
// Created by Autumn Rouse on 3/7/16.
//Set mode to 7 (packet serial address 0x80) and option to 4(38400) on first roboclaw
//Set mode to 8(address 0x81) on seconds roboclaw

//roboclaw libraries
#include "BMSerial.h"
#include "RoboClaw.h"

//Roboclaw Address
#define address 0x80

//Setup communications with roboclaw. Use pins 19 - RX1, 18 - TX1,
// 17 - RX2, 16 - TX2
Roboclaw roboclaw1(19, 18, 10000); //serial1
Roboclaw roboclaw2(17, 16, 10000); //serial2

void setup()
{
    //Communicate with roboclaw at 38400bps
    Serial.begin(115200);
    roboclaw1.begin(38400);
    roboclaw2.begin(38400);

}

void loop()
{
    serialEvent();
    serial1Check();
    serial2Check();
}
void serialEvent()
{

    if (Serial.available() > 0)
    {
        int inByte = Serial.read();
        switch (inByte)
        {
            case '254':

                int x = Serial.read();
                if (x == '80')
                {
                    serial1Write();
                }
                else if (x == '81')
                {
                    serial2Write();
                }
                else
                {
                    Serial.print("Invalid Address");
                }
        }
    }
}

void serial1Check()
{
   for (i = 0; i < 6; i++)
   {
       int RR1 = Serial1.read();
       Serial.write(RR);
   }

}

void serial2Check()
{
    for (int j = 0; j < 6; j++)
    {
        int RR2 = Serial2.read();
        Serial.write(RR2);
    }
}

void serial1Write()
{
    bool inputEvent1 = true;

    while(inputEvent1)
    {
        int y = Serial.read();
        Serial1.write(y);
        if (y == '254')
        {
            inputEvent1 = false;
        }
    }
}

void serial2Write()
{
    bool inputEvent2 = true;

    while (inputEvent2) {
        int z = Serial.read();
        Serial2.write(y);
        if (z == '254') {
            inputEvent2 = false;
        }
    }
}


