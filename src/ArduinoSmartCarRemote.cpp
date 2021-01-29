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
void readGyroscopeSensor();
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
Task tGyroscope(updateInterval, TASK_FOREVER, &readGyroscopeSensor, &taskManager, false);

// communication
Task tRF(0, TASK_FOREVER, &comRF, &taskManager, true);
Task tSerial(updateIntervalSerial, TASK_FOREVER, &sendSerial, &taskManager, true);
Task tSerialControl(updateIntervalSerial / 2, TASK_FOREVER, &serialControl, &taskManager, true); // control/debug through serial connection
/*********************************************************/

/*
 * Drive mode and general variables
 ********************************************************/
#define debug 0
int mode = 1;
bool automatic = false;
/*********************************************************/

/*
 * RF24 transmission
 ********************************************************/
RF24 radio(9, 10);              // define the object to control NRF24L01
byte addresses[][6] = {"00001","00007"};
// new structure
// data[0] = mode (1 manual joystick, 2 manual gyroscope, 3 automatic ultrasonic sensors, 4 automatic line follower)
// data[1] = direction X of joystick J1 (not assigned yet) (used as return channel for xy)
// data[2] = direction Y of joystick J1 (front/back speed) (used as return channel for speed)
// data[3] = fine tuning joystick J1 (used as return channel xy)
// data[4] = direction X of joystick J2 (left/right) (used as return channel for direction)
// data[5] = direction Y of joystick J2 (front buzzer, back not assigned yet) (used as return channel for xy)
// data[6] = fine tuning joystick J2 (used as return channel for xy)
// data[7] = data return channel to remote (used as return channel for left distance)
// data[8] = data return channel to remote (used as return channel for right distance)
// data[9] = data return channel to remote (used as return channel for line follower sensor value)
#define apMode 0
#define apJ1x 1
#define apJ1ySpeed 2
#define apJ1fine 3
#define apJ2xDirection 4
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
 * Joysticks and buttons
 ********************************************************/
const int pinJ1x = 5;                // define pin for direction X of joystick J1 (unused)
const int pinJ1ySpeed = 1;           // define pin for direction Y of joystick J1 (speed)
const int pinJ2xDirection = 2;       // define pin for direction X of joystick J2 (direction)
const int pinJ2y = 3;                // define pin for direction Y of joystick J2 (horn)

// Potentiometers for fine tuning the joysticks
const int R1Pin = 6;            // define R1
const int R6Pin = 7;            // define R6

// Buttons
const int APin = 2;             // define pin for D2
const int BPin = 3;             // define pin for D3
const int CPin = 4;             // define pin for D4
const int DPin = 5;             // define pin for D5

/*********************************************************/


int radar[2];
char onlyOne=0;

const int led1Pin = 6;          // define pin for LED1 which is close to NRF24L01 and used to indicate the state of NRF24L01
const int led2Pin = 7;          // define pin for LED2 which is the mode is displayed in the car remote control mode  
const int led3Pin = 8;          // define pin for LED3 which is the mode is displayed in the car auto mode

/*
 * Gyroscope
 ********************************************************/
#define DEVICE (0x53) //ADXL345 device address
#define TO_READ (6)   //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ]; //6 bytes buffer for saving data read from the device
char str[512];      //string buffer to transform data before sending it to the serial port

int regAddress = 0x32;            //first axis-acceleration-data register on the ADXL345
int x, y, z;                      //three axis acceleration data
double roll = 0.00, pitch = 0.00; //Roll & Pitch are the angles which rotate by the axis X and y
                                  //in the sequence of R(x-y-z)
/*********************************************************/

// ADXL345 Gyroscope Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
    Wire.beginTransmission(device);     // start transmission to device 
    Wire.write(address);                // send register address
    Wire.write(val);                    // send value to write
    Wire.endTransmission();             // end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
    Wire.beginTransmission(device);     // start transmission to device 
    Wire.write(address);                // sends address to read from
    Wire.endTransmission();             // end transmission
    Wire.beginTransmission(device);     // start transmission to device
    Wire.requestFrom(device, num);      // request 6 bytes from device
    int i = 0;
    while(Wire.available())             // device may send less than requested (abnormal)
    { 
        buff[i] = Wire.read();          // receive a byte
        i++;
    }
    Wire.endTransmission();             // end transmission
}

//calculate the Roll&Pitch
void RP_calculate(){
    double x_Buff = float(x);
    double y_Buff = float(y);
    double z_Buff = float(z);
    roll = atan2(y_Buff, z_Buff)*57.3;
    pitch = atan2(-x_Buff, sqrt(y_Buff*y_Buff+z_Buff*z_Buff))*57.3;
}

void readGyroscopeSensor()
{
    readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
                                                 //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
                                                 //thus we are converting both bytes in to one int
    x = (((int)buff[1]) << 8) | buff[0];
    y = (((int)buff[3]) << 8) | buff[2];
    z = (((int)buff[5]) << 8) | buff[4];

    RP_calculate();

    data[apJ2xDirection] = map(x, -300, 300, 1023, 0);
    data[apJ1ySpeed] = map(y, -300, 300, 0, 1023);
}

void readButtons()
{
    if (digitalRead(APin) == LOW)
    {
        mode = 1;
        digitalWrite(led2Pin, HIGH);
        digitalWrite(led3Pin, LOW);
        tJoysticks.enableIfNot();
        tGyroscope.disable();
    }
    if (digitalRead(BPin) == LOW)
    {
        mode = 2;
        digitalWrite(led2Pin, LOW);
        digitalWrite(led3Pin, HIGH);
        tJoysticks.enableIfNot();
        tGyroscope.enableIfNot();
    }
    if (digitalRead(CPin) == LOW)
    {
        mode = 3;
        digitalWrite(led2Pin, HIGH);
        digitalWrite(led3Pin, HIGH);
        tJoysticks.disable();
        tGyroscope.disable();
    }
    if (digitalRead(DPin) == LOW)
    {
        mode = 4;
        digitalWrite(led2Pin, LOW);
        digitalWrite(led3Pin, LOW);
        tJoysticks.disable();
        tGyroscope.disable();
    }

    data[apMode] = mode;
}

void readJoysticks()
{
    // only write values for direction and speed in data structure if in joystick mode
    if (mode == 1)
    {
        // put the values of rocker, switch and potentiometer into the array
        data[apJ1ySpeed] = analogRead(pinJ1ySpeed);
        data[apJ2xDirection] = 1023 - analogRead(pinJ2xDirection);

        // calibration values of the resistors
        data[apJ1fine] = analogRead(R1Pin);
        data[apJ2fine] = analogRead(R6Pin);
    }

    data[apJ1x] = analogRead(pinJ1x);
    data[apJ2y] = analogRead(pinJ2y); // buzzer
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
    Serial.print(data[apJ1ySpeed]);
    Serial.print("; J1fine: ");
    Serial.print(data[apJ1fine]);
    Serial.print("; J2xDirection: ");
    Serial.print(data[apJ2xDirection]);
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
                        data[apJ1ySpeed] = round(iComandParamter);
                    }
                }
                break;
            case 's':
                // read current speed
                Serial.println("Speed: " + String(data[apJ1ySpeed]));
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
                        tGyroscope.disable();
                        break;
                    case 1:
                        tJoysticks.enableIfNot();
                        tGyroscope.disable();
                        break;
                    case 2:
                        tJoysticks.enableIfNot();
                        tGyroscope.enableIfNot();
                        break;
                    case 3:
                    case 4:
                        tJoysticks.disable();
                        tGyroscope.disable();
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
                        data[apJ2xDirection] = round(iComandParamter);
                    }
                }
                break;
            case 'r':
                // read current direction
                Serial.println("Direction: " + String(data[apJ2xDirection]));
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

    //Turning on the ADXL345
    writeTo(DEVICE, 0x2D, 0);
    writeTo(DEVICE, 0x2D, 16);
    writeTo(DEVICE, 0x2D, 8);

    Serial.println("Remote booted");
}

void loop()
{
    taskManager.execute();
}