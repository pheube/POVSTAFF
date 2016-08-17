

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


// LED DECLARATION

int redPin1 =  5;
int greenPin1 =  4;
int bluePin1 =  3;
int redPin2 =  12;
int greenPin2 =  11;
int bluePin2 =  10;
//COMMONS
//LEFT
int LEDL1 =  8;
int LEDL2 =  7;
int LEDL3 =  6;
//RIGHT
int LEDR1 =  A0;
int LEDR2 =  A1;
int LEDR3 =  A2;


// I2Cdev and mpu6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "Wire.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2   // use pin 2 on Arduino Uno & most boards

// mpu1 control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpu1IntStatus;   // holds actual interrupt status byte from mpu1
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpu1Interrupt = false;     // indicates whether mpu1 interrupt pin has gone high
void dmpDataReady() {
    mpu1Interrupt = true;
}


void setup() {
//LED SETUP
  pinMode(redPin1, OUTPUT);
  pinMode(greenPin1, OUTPUT);
  pinMode(bluePin1, OUTPUT);
  pinMode(redPin2, OUTPUT);
  pinMode(greenPin2, OUTPUT);
  pinMode(bluePin2, OUTPUT);
  pinMode(LEDL1, OUTPUT);
  pinMode(LEDL2, OUTPUT);
  pinMode(LEDL3, OUTPUT);
  pinMode(LEDR1, OUTPUT);
  pinMode(LEDR2, OUTPUT);
  pinMode(LEDR3, OUTPUT);

   // set all leds to 0 (off)
  digitalWrite (LEDL1, HIGH);
  digitalWrite (LEDL2, HIGH);
  digitalWrite (LEDL3, HIGH);
  digitalWrite (LEDR1, HIGH);
  digitalWrite (LEDR2, HIGH);
  digitalWrite (LEDR3, HIGH);

  
    // I2C CONNECT
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. 

    // initialize serial communication
    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(mpu1.testConnection() ? F("mpu1 connection successful") : F("mpu1 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu1.dmpInitialize();
    delay (5000); 
    // MPU1 Gryo and Accel Offset
    mpu1.setXGyroOffset(-2640);
    mpu1.setYGyroOffset(-1013);
    mpu1.setZGyroOffset(1507);
    mpu1.setXAccelOffset(21);
    mpu1.setYAccelOffset(-257);
    mpu1.setZAccelOffset(-5);

    // MPU2 Gryo and Accel Offset
    mpu2.setXGyroOffset(-669);
    mpu2.setYGyroOffset(2739);
    mpu2.setZGyroOffset(1539);
    mpu2.setXAccelOffset(-335);
    mpu2.setYAccelOffset(53);
    mpu2.setZAccelOffset(-47);

    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu1.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpu1IntStatus = mpu1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu1.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {


  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for mpu1 interrupt or extra packet(s) available
    while (!mpu1Interrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpu1Interrupt is true, and if so, "break;" from the
        // while() loop to immediately process the mpu1 data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpu1Interrupt = false;
    mpu1IntStatus = mpu1.getIntStatus();

    // get current FIFO count
    fifoCount = mpu1.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpu1IntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu1.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpu1IntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu1.getFIFOCount();

        // read a packet from FIFO
        mpu1.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


                #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu1.dmpGetQuaternion(&q, fifoBuffer);
            mpu1.dmpGetGravity(&gravity, &q);
            mpu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
    }

        //Test Yaw value and adjust LED color based on result
          if ((ypr[0] * 180/M_PI) > 0 ){
            digitalWrite (LEDR1, LOW);
            digitalWrite (redPin1, LOW);
            digitalWrite (greenPin1, HIGH);
            digitalWrite (bluePin1, LOW);
          }
            
//       //Test Pitch value and adjust LED color based on result
//          if ((ypr[1] * 180/M_PI) > 0 ){
//            digitalWrite (LEDL2, LOW);
//            digitalWrite (redPin1, LOW);
//            digitalWrite (greenPin1, LOW);
//            digitalWrite (bluePin1, HIGH);
//              }
}       

            
        #endif     

