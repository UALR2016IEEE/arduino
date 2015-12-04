//
// Created by Autumn Rouse on 12/2/15.
//

//Includes
#include <millis>;
#include <Serial>;
#include <steppers.h>
#include <distance_ir_sensor.h>

//function declarations
void exec_command();
int get_int( int chars = 1);
void show_sensors();
bool read_serial();

bool button = False;        //for what?


//variable declarations
byte readbyte;
byte writebyte;
byte dir;
byte move_blocks;
int distance1, distance2;
bool command_stat;



void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial.flush();
    Serial1.flush();
    Serial2.flush();
    set_speed(zero, zero);

    pinMode(44, OUTPUT);
    digitalWrite(44, HIGH);

//bring in stepper info from Pi



}

void loop()
{
 if (Serial.available() > 0)
 {
     serialData = Serial.read();
     Serial.println(serialData);
 }
}

// Need help, Is this where I bring in variable values???
void exec_command()
{
   /*
    set_speed(cruise, cruise);
    while(true)
    {
        time = millis();
        stablize();
        set_speed(target_left, target_right)
    }
    */
}

int get_int(int chars)
{
    while(!Serial.available());
    char encoded = Serial.read();
    return encoded - 48;
}

void show_sensors ()
{
    // Do I need this? If so how do I set it up
}

//Need explanation of this entire thing
boolean read_serial()
{
    if (Serial.available())
    {
        readbyte = Serial.read();
        Serial.println(readbyte);

        if (readbyte == 63)
        {
            Serial.println("get pos");
            //show_sensors(); - does it got here?
            return false;       //??? if 63 sending data over serial - need explanation
        }
        if (readbyte == 33)
        {
            move_blocks = 0;    //confused
            dir = 0;
                if (Serial.available())
                {
                    dir = get_int();
                }
                Serial.print("move: ");
                Serial.println(move_blocks);
                return true;

        }
        if (readbyte == 64)
        {
            if (Serial.available())
            {
                dir = get_int();
                Serial.print("dir: ");
                Serial.println(dir);
            }
            return true;
        }
        if (readbyte == 35)
        {
            Serial.print("button");
            Serial.print(button);
            return false;
        }

    }
}
