/*----------------------------------------
 * Accelerometer_Gyroscope_Implementation_Mk_2.ino
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
 * ---------------------------------------
 */



//library imports
#include <Wire.h>
#include <Servo.h>

#define SERVO_PIN_LR 4
#define SERVO_PIN_FB 5

Servo servo_LR;
Servo servo_FB;


//---------------------------------------------------------------------------------------
//                                    Constants
//---------------------------------------------------------------------------------------
const int MPU = 0x68; // MPU6050 I2C address

//error calculation constants
int c = 0;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

//Angle measuring constants
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

//PID constants
double kp = 10;
double ki = .01;
double kd = 1;

//double kp = 10;
//double ki = .1;
//double kd = .000001;

 
unsigned long currentTime2, previousTime2;
double elapsedTime2;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;


//---------------------------------------------------------------------------------------
//                                        SETUP
//---------------------------------------------------------------------------------------

void setup() 
{
  Serial.begin(19200);               // send and receive at 19200 baud rate
  Wire.begin();                      // Initialize comunication to chip
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission 


  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);



  //Attach servo to pins
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);


  //Set point for the PID  Controller
  setPoint = 0;                          //set point at zero degrees
}
//---------------------------------------------------------------------------------------
//                                      Main Loop
//---------------------------------------------------------------------------------------
void loop() 
{
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.66; // AccErrorX ~(0.96) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 2.28; // AccErrorY ~(1.00)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - 0.28; // GyroErrorX ~(0.24)
  GyroY = GyroY - 0.51; // GyroErrorY ~(0.49)
  GyroZ = GyroZ - 0.48; // GyroErrorZ ~ (0.41)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.25 * gyroAngleX + 0.75 * accAngleX;
  pitch = 0.25 * gyroAngleY + 0.75 * accAngleY;


  Serial.print(AccX);
  Serial.print("/");
  Serial.print(AccY);
  Serial.print("/");
  Serial.print(AccZ);
  Serial.print("/    /");
  Serial.print(GyroX);
  Serial.print("/");
  Serial.print(GyroY);
  Serial.print("/");
  Serial.print(GyroZ);

  Serial.print("\t\t\t");
  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.print(yaw);

  //
  input = roll;                //read from rotary encoder connected to A0
  output = computePID(input);
  
  Serial.print("\t\t");
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
  Serial.print("\t\t");
  Serial.println(output); 
  servo_LR.write(90 + pitch);
  servo_FB.write(90 - output);
  
  //delay(100);

  
}

//---------------------------------------------------------------------------------------
//                                    Chip Error
//---------------------------------------------------------------------------------------
void calculate_IMU_error() 
{
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) 
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) 
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
//---------------------------------------------------------------------------------------
//                                    PID Controller
//---------------------------------------------------------------------------------------
double computePID(double inp)
{     
  currentTime2 = millis();                                    //get current time
  elapsedTime2 = (double)(currentTime2 - previousTime2);        //compute time elapsed from previous computation
  
  error = setPoint - inp;                                    // determine error
  cumError += error * elapsedTime2;                           // compute integral
  rateError = (error - lastError)/elapsedTime2;               // compute derivative

  double out = kp*error + ki*cumError + kd*rateError;        //PID output               

  lastError = error;                                         //remember current error
  previousTime2 = currentTime2;                                //remember current time

  return out;                                                //have function return the PID output
}   
