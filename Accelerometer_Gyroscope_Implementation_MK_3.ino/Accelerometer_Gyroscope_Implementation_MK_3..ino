/*----------------------------------------
 * Accelerometer_Gyroscope_Implementation_Mk_3.ino
 * Thrust Vectoring Rocket Project
 * by Tom De Vries 
 * Version 1 (07/22/2020) Arduino 1.8.12
 *      Combined accelerometer code with (Servo_diamond_move.ino) to make the test stand adjust angle depending on the angle of the MPU6050 accelerometer
 *      The Stand follows the movement of the accelerometer
 *      https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 * 
 * Version 2 (07/23/2020) Arduino 1.8.12
 *      Implemented PID Controller raw code to make the stand attempt to correct the rockets movement back to center assuming the MPU6050 is attached to the rocket
 *      The Stand follows the movement of the accelerometer
 *      https://www.teachmemicro.com/arduino-pid-control-tutorial/
 *      https://playground.arduino.cc/Code/PIDLibrary/
 *      
 * Version 3 (07/30/2020) Arduino 1.8.12     
 *      Changed the code talking to the MPU 6050 to use I2C dev kit to talk to the onboard computer to do some calculations to stabalise the angle data coming form the chip
 *      Added the interupt wire onto the breadboard to support the I2C communication
 *      Used the second method of talking with the chip that involves the I2C dev library, This is Detailed on the (how to mechatronics link)
 *      https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/ 
 * ---------------------------------------
 */


// ================================================================================================================================================================================================================================================================
// ===                     SERVO SETUP                            ===
// ================================================================================================================================================================================================================================================================

//library imports
#include <Wire.h>
#include <Servo.h>

#define SERVO_PIN_LR 4
#define SERVO_PIN_FB 5

Servo servo_LR;
Servo servo_FB;





//Angles
float roll, pitch, yaw;

//PID 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
//---------------------------------------------------------------------------------------
//                                    Constants
//---------------------------------------------------------------------------------------

//PID constants
double kp = 10;
double ki = .001;
double kd = 1;

//double kp = 10;
//double ki = .1;
//double kd = .000001;

 




// ================================================================================================================================================================================================================================================================
// ===                     I2C DEV                            ===
// ================================================================================================================================================================================================================================================================



// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high




// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL




#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}





void setup() 
{
// ================================================================
// ===                 INITIAL MPU 6050 SETUP                   ===
// ================================================================
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(19200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
// ================================================================
// ===                       OTHER SETUP                        ===
// ================================================================



  //Attach servo to pins
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);


  //Set point for the PID  Controller
  setPoint = 0;                          //set point at zero degrees

    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
    { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            yaw = ypr[0] * 180/M_PI;
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            pitch = ypr[1] * 180/M_PI;
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            roll = ypr[2] * 180/M_PI;
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }




    
    input = roll;                //read from rotary encoder connected to A0
    output = computePID(input);
    
    Serial.print("\tOutput(before limit):");
    Serial.print(output);
    //analogWrite(3, output);                //control the motor based on PID value
    //Serial.print(output);
    //servo_FB.write(output);
    if(output >= 10)
    {
      output = 10;
    }
    else if(output <= -10)
    {
      output = -10;
    }
    delay(100);
    Serial.print("\tOutput(After limit):");
    Serial.println(output); 
    servo_LR.write(90 + pitch);
    servo_FB.write(90 - output);
    
    //delay(100);

  
}

//---------------------------------------------------------------------------------------
//                                    PID Controller
//---------------------------------------------------------------------------------------
double computePID(double inp)
{     
  currentTime = millis();                                    //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - inp;                                    // determine error
  cumError += error * elapsedTime;                           // compute integral
  rateError = (error - lastError)/elapsedTime;               // compute derivative

  double out = kp*error + ki*cumError + kd*rateError;        //PID output               

  lastError = error;                                         //remember current error
  previousTime = currentTime;                                //remember current time

  return out;                                                //have function return the PID output
}   




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
