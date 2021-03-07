#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVO_PIN_PITCH 0
#define SERVO_PIN_YAW 1

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo servo_pitch;
Servo servo_yaw;

float pitch;
float yaw;

// Declare orientation array: [check, degX, degY, degZ] 
float orient[] = {0,0,0,0};
float orient_cur[3] = {0,0,0};
float orient_last[3] = {0,0,0};

void setup(void) 
{
  servo_pitch.attach(SERVO_PIN_PITCH);
  servo_yaw.attach(SERVO_PIN_YAW);
  servo_pitch.writeMicroseconds(1500);
  servo_yaw.writeMicroseconds(1500);
  
  Serial.begin(9600);
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } 
  orient[0] = 1;
  delay(1000);   
  bno.setExtCrystalUse(true);

  
  uint8_t cal, gyro, accel, mag = 0;
  bno.getCalibration(&cal, &gyro, &accel, &mag);

  Serial.print("Calibrating BNO055  ");
  Serial.print(cal);
  Serial.print("  ");
  Serial.print(gyro);
  Serial.print("  ");
  Serial.print(accel);
  Serial.print("  ");
  Serial.println(mag);

  while(cal != 3)
  {
    bno.getCalibration(&cal, &gyro, &accel, &mag);
    Serial.print("Calibrating BNO055  ");
    Serial.print(cal);
    Serial.print("  ");
    Serial.print(gyro);
    Serial.print("  ");
    Serial.print(accel);
    Serial.print("  ");
    Serial.println(mag);
    delay(1000);
  }
  
}

imu::Quaternion getInverse(imu::Quaternion q) {
  imu::Quaternion conj;
  conj.w() = q.w();
  conj.x() = -q.x();
  conj.y() = -q.y();
  conj.z() = -q.z();

  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());

  imu::Quaternion inv;

  inv.w() = conj.w()/norm;
  inv.x() = conj.x()/norm;
  inv.y() = conj.y()/norm;
  inv.z() = conj.z()/norm;

  return inv;
}

void loop(void) 
{
  /* Angle data */
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion qInv = getInverse(quat);
//  Serial.printf("q = [%5f %5f %5f %5f]         ", quat.w(),quat.x(),quat.y(),quat.z());

  //original i
  imu::Quaternion pt;
  pt.w() = 0;
  pt.x() = 1;
  pt.y() = 0;
  pt.z() = 0;

  //original k
  imu::Quaternion pt2;
  pt.w() = 0;
  pt.x() = 0;
  pt.y() = 0;
  pt.z() = 1;

  imu::Quaternion i = quat * pt * qInv;
  imu::Quaternion k = quat * pt2 * qInv;
  
  Serial.printf("   %f   %f   %f   %f   %f   %f\n", i.x(),i.y(),i.z(),k.x(),k.y(),k.z());

  delay(50);

  //x,y,z define axis of rotation
    //To rotate about a specified axis, only change that axis and w or only the other two
      //Ex: Rotate about x - w & x can change OR y & z can change
  //quaternion also described as q = cos(theta) + sin(theta)*(x,y,z)
    //theta = angle/amount of rotation, (x,y,z) described above
    
   // 1: Find original axis of orientation
   // 2: Find desired axis to base new quaternion on
   // 3: Determine how much to move by

  /*
   * /total = local_rotation * total //multiplication order matters on this line
   * //axis is a unit vector
   * local_rotation.w = cosf( fAngle/2)
   * local_rotation.x = axis.x * sinf( fAngle/2 )
   * local_rotation.y = axis.y * sinf( fAngle/2 )
   * local_rotation.z = axis.z * sinf( fAngle/2 )
   */
}
