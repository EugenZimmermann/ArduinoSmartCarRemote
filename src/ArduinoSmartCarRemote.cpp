/***********************************************************
File name:  AdeeptRemoteControl.ino
Description:  

Website: www.adeept.com
E-mail: support@adeept.com
Author: Tom
Date: 2018/6/8 
***********************************************************/
#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <TaskScheduler.h>

/*
 * prototype functions
 ********************************************************/
void comRF();
void sendSerial();
void serialControl();

void readButtons();
void readJoysticks();
/*********************************************************/

/*
 * Task scheduler
 ********************************************************/
Scheduler taskManager;

#define updateInterval 25
#define updateIntervalSerial 500

// sensor and calculations
Task tButtons(updateInterval, TASK_FOREVER, &readButtons, &taskManager, true);
Task tJoysticks(updateInterval, TASK_FOREVER, &readJoysticks, &taskManager, true);

// communication
Task tRF(0, TASK_FOREVER, &comRF, &taskManager, true);
Task tSerial(updateIntervalSerial, TASK_FOREVER, &sendSerial, &taskManager, true);
Task tSerialControl(updateIntervalSerial / 2, TASK_FOREVER, &serialControl, &taskManager, true); // control/debug through serial connection
/*********************************************************/

/*
 * Drive mode and general variables
 ********************************************************/
#define debug 0                 //#ToDo
int mode = 1;
bool automatic = false;
/*********************************************************/

/*
 * RF24 transmission
 ********************************************************/
RF24 radio(9, 10);              // define the object to control NRF24L01
byte addresses[][6] = {"00001","00007"};
// new structure
// data[apMode] = mode (1 manual joystick, 2 manual gyroscope, 3 automatic ultrasonic sensors, 4 automatic line follower)
// data[apJ1x] = direction X of joystick J1 (not assigned yet) (used as return channel for xy)
// data[apSpeed] = direction Y of joystick J1 (front = 1023/back = 0 speed) (used as return channel for speed)
// data[apJ1fine] = fine tuning joystick J1 (used as return channel xy)
// data[apDirection4] = direction X of joystick J2 (left = 1023/right = 0) (used as return channel for direction)
// data[apJ2y] = direction Y of joystick J2 (front buzzer, back not assigned yet) (used as return channel for xy)
// data[apJ2fine] = fine tuning joystick J2 (used as return channel for xy)
// data[apUL] = data return channel to remote (used as return channel for left distance)
// data[apUR] = data return channel to remote (used as return channel for right distance)
// data[apLFS] = data return channel to remote (used as return channel for line follower sensor value)
#define apMode 0
#define apJ1x 1
#define apSpeed 2
#define apJ1fine 3
#define apDirection 4
#define apJ2y 5
#define apJ2fine 6
#define apUL 7
#define apUR 8
#define apLFS 9
int data[10] = {1, 512, 512, 0, 512, 512, 0, 0, 0, 0}; // define array used to save the communication data
/*********************************************************/

/*
 * define variables for serial communication/commands
 ********************************************************/
unsigned char cComIn;
char cCommand;
unsigned long iComandParamter;
bool bIsParameter;
/*********************************************************/

/*
 * Potentiometers and buttons
 ********************************************************/
const int pinJ1ySpeed = A1;           // define pin for accelerator and brake pedal J1 (speed)
const int pinJ2xDirection = A2;       // define pin for stearing wheel J2 (direction)
const int pinGearshiftLeft = A6;      // define pin for gearshift left (unused)
const int pinGearshiftRight = A7;     // define pin for gearshift right (horn)

// Buttons
const int APin = 2;             // define pin for D2
const int BPin = 3;             // define pin for D3
const int CPin = 4;             // define pin for D4
const int DPin = 5;             // define pin for D5
/*********************************************************/

/*
 * LED control
 ********************************************************/
const int led1Pin = 6;          // define pin for LED1 which is close to NRF24L01 and used to indicate the state of NRF24L01
const int led2Pin = 7;          // define pin for LED2 which is the mode is displayed in the car remote control mode  
const int led3Pin = 8;          // define pin for LED3 which is the mode is displayed in the car auto mode
/*********************************************************/

void readButtons()
{
    if (digitalRead(APin) == LOW)
    {
        mode = 1;
        digitalWrite(led2Pin, HIGH);
        digitalWrite(led3Pin, LOW);
        tJoysticks.enableIfNot();
    }
    if (digitalRead(BPin) == LOW)
    {
        mode = 2;
        digitalWrite(led2Pin, LOW);
        digitalWrite(led3Pin, HIGH);
        tJoysticks.enableIfNot();
    }
    if (digitalRead(CPin) == LOW)
    {
        mode = 3;
        digitalWrite(led2Pin, HIGH);
        digitalWrite(led3Pin, HIGH);
        tJoysticks.disable();
    }
    if (digitalRead(DPin) == LOW)
    {
        mode = 4;
        digitalWrite(led2Pin, LOW);
        digitalWrite(led3Pin, LOW);
        tJoysticks.disable();
    }
    data[apMode] = mode;
}

void readJoysticks()
{
    // only write values for direction and speed in data structure if in joystick mode
    if (mode == 1)
    {
        // put the values of the pedals and the steering wheel into the array
        int speedTemp = analogRead(pinJ1ySpeed);
        if (speedTemp > 460)
        {   // accelerate
            data[apSpeed] = (int)constrain(map(speedTemp, 465, 890, 512, 1023), 512, 1023);
        }
        else
        {   // brake
            data[apSpeed] = (int)constrain(map(speedTemp, 320, 455, 0, 511), 0, 511);
        }

        int directionTemp = analogRead(pinJ2xDirection);
        if (directionTemp > 450)
        {   // turn left
            data[apDirection] = (int)constrain(map(directionTemp, 460, 830, 512, 1023), 512, 1023);
        }
        else
        {   // turn right
            data[apDirection] = (int)constrain(map(directionTemp, 140, 440, 0, 511), 0, 511);
        }

        // data[apJ2xDirection] = directionTemp;
    }

    data[apJ1x] = (int)constrain(map(analogRead(pinGearshiftLeft), 380, 765, 0, 1023), 0, 1023);
    data[apJ2y] = (int)constrain(map(analogRead(pinGearshiftRight), 345, 710, 0, 1023), 0, 1023); // buzzer
}

void comRF()
{
    if (mode < 3 || automatic)
    {
        // if manual mode, stop listening to incoming signals
        radio.stopListening();
        // send array data. If the sending succeeds
        if (radio.write(data, sizeof(data)))
            digitalWrite(led1Pin, HIGH);

        // delay for a period of time, then turn off the signal LED for next sending
        delay(2);
        digitalWrite(led1Pin, LOW);

        // flag for sending the change to automatic mode at least once, before stopping to sending data
        automatic = mode < 3;
    }
    else if (mode > 2)
    {
        // if automatic mode, start listening
        radio.startListening(); // start monitoring
        if (radio.available())
        { // if receive the data
            while (radio.available())
            {                                   // read all the data
                radio.read(data, sizeof(data)); // read data
            }
            digitalWrite(led1Pin, HIGH);
        }
        else
        {
            digitalWrite(led1Pin, LOW);
        }
    }
}

void sendSerial()
{
    Serial.print("Mode: ");
    Serial.print(data[apMode]);
    Serial.print("; J1x: ");
    Serial.print(data[apJ1x]);
    Serial.print("; J1ySpeed: ");
    Serial.print(data[apSpeed]);
    Serial.print("; J1fine: ");
    Serial.print(data[apJ1fine]);
    Serial.print("; J2xDirection: ");
    Serial.print(data[apDirection]);
    Serial.print("; J2y: ");
    Serial.print(data[apJ2y]);
    Serial.print("; J2fine: ");
    Serial.print(data[apJ2fine]);
    Serial.print("; UL: ");
    Serial.print(data[apUL]);
    Serial.print("; UR: ");
    Serial.print(data[apUR]);
    Serial.print("; LSF: ");
    Serial.println(data[apLFS]);
}

/*
 * control car by serial commands (mainly for debug)
 * capital letter is for setting a property/variable
 * small letter is for getting a property/variable
 ********************************************************/
void serialControl()
{
    while (Serial.available())
    {
        cComIn = Serial.read();
        if (cComIn >= '0' && cComIn <= '9')
        {
            iComandParamter = 10 * iComandParamter + (cComIn - '0');
            bIsParameter = true;
        }
        else if (cComIn == 13)
        {
            switch (cCommand)
            {
            case 'P':
            case 'p':
                break;
            case 'S':
                // set speed in manual mode
                if (bIsParameter)
                {
                    if (iComandParamter >= 0 && iComandParamter < 1024)
                    {
                        data[apMode] = 0;
                        data[apSpeed] = round(iComandParamter);
                    }
                }
                break;
            case 's':
                // read current speed
                Serial.println("Speed: " + String(data[apSpeed]));
                break;
            case 'm': // get current mode
                Serial.println("Mode: " + String(mode));
                break;
            case 'M': // set new mode
                if (bIsParameter)
                {
                    switch (iComandParamter)
                    {
                    case 0:
                        tJoysticks.disable();
                        break;
                    case 1:
                        tJoysticks.enableIfNot();
                        break;
                    case 2:
                        tJoysticks.enableIfNot();
                        break;
                    case 3:
                    case 4:
                        tJoysticks.disable();
                        break;
                    }

                    mode = iComandParamter;
                    Serial.println("Mode: " + String(mode));
                }
                break;
            case 'D':
            case 'd':
                // get distance values of ultrasonic sensor(s)
                Serial.println("Left: " + String(data[apUL]) + "; Right: " + String(data[apUR]));
                break;
            case 'T':
                break;
            case 't':
                // get position from line follower sensor (0-5000)
                Serial.println("LineFollowerSensorDeviation: " + String(data[apLFS]));
                break;
            case 'R':
                // set direction in manual mode
                if (bIsParameter)
                {
                    if (iComandParamter >= 0 && iComandParamter < 1024)
                    {
                        data[apMode] = 0;
                        data[apDirection] = round(iComandParamter);
                    }
                }
                break;
            case 'r':
                // read current direction
                Serial.println("Direction: " + String(data[apDirection]));
                break;
            case 'L':
            case 'l':
                // set LED state
                if (bIsParameter)
                {
                    // switch (iComandParamter)
                    // {
                    // case 1:
                    //     tDrive.setCallback(&handleManualMode);
                    //     Serial.println("LED: red");
                    //     analogWrite(RPin, 255);
                    //     analogWrite(GPin, 0);
                    //     analogWrite(BPin, 0);
                    //     break;
                    // case 2:
                    //     tDrive.setCallback(&handleManualMode);
                    //     Serial.println("LED: green");
                    //     analogWrite(RPin, 0);
                    //     analogWrite(GPin, 255);
                    //     analogWrite(BPin, 0);
                    //     break;
                    // case 3:
                    //     tDrive.setCallback(&handleMode3);
                    //     Serial.println("LED: blue");
                    //     analogWrite(RPin, 0);
                    //     analogWrite(GPin, 0);
                    //     analogWrite(BPin, 255);
                    //     break;
                    // case 4:
                    //     tDrive.setCallback(&handleMode4);
                    //     Serial.println("LED: orange");
                    //     analogWrite(RPin, 255);
                    //     analogWrite(GPin, 165);
                    //     analogWrite(BPin, 0);
                    //     break;
                    // case 0:
                    //     Serial.println("LED: off");
                    //     analogWrite(RPin, 0);
                    //     analogWrite(GPin, 0);
                    //     analogWrite(BPin, 0);
                    //     break;
                    // default:
                    //     Serial.println("LED: white");
                    //     analogWrite(RPin, 255);
                    //     analogWrite(GPin, 255);
                    //     analogWrite(BPin, 255);
                    //     break;
                    // }
                }
            }
        }
        else
        {
            cCommand = cComIn;
            iComandParamter = 0;
            bIsParameter = false;
        }
    }
}

void setup()
{
    Serial.begin(9600);
    delay(500);

    Wire.begin();                        // join i2c bus (address optional for master)
    radio.begin();                       // initialize RF24
    radio.setRetries(0, 5);              // set retries times
    radio.setPALevel(RF24_PA_LOW);       // set power
    radio.openWritingPipe(addresses[1]); // open delivery channel
    radio.openReadingPipe(1, addresses[0]);
    radio.stopListening(); // stop monitoring

    pinMode(led1Pin, OUTPUT); // set led1Pin to output mode
    pinMode(led2Pin, OUTPUT); // set led2Pin to output mode
    pinMode(led3Pin, OUTPUT); // set led3Pin to output mode

    pinMode(APin, INPUT_PULLUP); // set APin to output mode
    pinMode(BPin, INPUT_PULLUP); // set BPin to output mode
    pinMode(CPin, INPUT_PULLUP); // set CPin to output mode
    pinMode(DPin, INPUT_PULLUP); // set DPin to output mode

    Serial.println("Steering wheel booted");
}

void loop()
{
    taskManager.execute();
}