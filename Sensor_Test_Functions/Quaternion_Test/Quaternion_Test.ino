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
  //Serial.printf("q = [%5f %5f %5f %5f]         ", quat.w(),quat.x(),quat.y(),quat.z());

  //original i vector (towards nosecone due to how bno055 is oriented)
  imu::Quaternion pt1; //vector to this point on a unit sphere from origin
  pt1.w() = 0;
  pt1.x() = 1;
  pt1.y() = 0;
  pt1.z() = 0;

  //original k
  imu::Quaternion pt2;
  pt2.w() = 0;
  pt2.x() = 0;
  pt2.y() = 0;
  pt2.z() = 1;

  imu::Quaternion dir = quat * pt1 * qInv;
  imu::Quaternion rollVec = quat * pt2 * qInv;
  
  float angles[2];
  Serial.printf("[%7.5f %7.5f %7.5f]      ", dir.x(), dir.y(), dir.z());
  angles[0] = asin(dir.x())*180/PI; //use 90 - acos(dot(dir, i)) and convert to degrees
  angles[1] = asin(dir.y())*180/PI; 
  Serial.printf("%7.2f %7.2f     ", angles[0], angles[1]);

  //Serial.printf("   %f   %f   %f   %f   %f   %f   %f   %f\n", dir.x(), dir.y(), dir.z(), rollVec.x(), rollVec.y(), rollVec.z(),angles[0], angles[1]);

  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //float roll = eul.x();
  //angles[0] -= roll;
  //angles[1] -= roll;
  
  //Serial.printf("Roll: %7.2f    modified: %7.2f %7.2f  \n", roll, angles[0], angles[1]);

  float pitchMicroSeconds = map(0.4285*angles[0], -45, 45 ,1000, 2000);
  float yawMicroSeconds = map(0.4285*angles[1], -45, 45 ,1000, 2000);
  
  servo_pitch.write(pitchMicroSeconds);
  servo_yaw.write(yawMicroSeconds);
  
  //delay(50);

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
