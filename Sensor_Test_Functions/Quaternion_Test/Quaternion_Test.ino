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

double pathDir[3];

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

bool firstRun = true;
double DCM[3][3];
double DCM2[3][3];

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

  //original j
  imu::Quaternion pt3;
  pt3.w() = 0;
  pt3.x() = 0;
  pt3.y() = 0;
  pt3.z() = 1;

  imu::Quaternion dir = quat * pt1 * qInv;
  imu::Quaternion rollVec1 = quat * pt2 * qInv;
  imu::Quaternion rollVec2 = quat * pt3 * qInv;

  if (firstRun){
    firstRun = false;
    
      DCM[0][0] = quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z();
      DCM[0][1] = 2 * (quat.x()*quat.y() + quat.z()*quat.w());
      DCM[0][2] = 2 * (quat.x()*quat.z() - quat.y()*quat.w());
      DCM[1][0] = 2 * (quat.x()*quat.y() - quat.z()*quat.w());
      DCM[1][1] = quat.w()*quat.w() - quat.x()*quat.x() + quat.y()*quat.y() - quat.z()*quat.z();
      DCM[1][2] = 2 * (quat.y()*quat.z() + quat.x()*quat.w());
      DCM[2][0] = 2 * (quat.x()*quat.z() + quat.y()*quat.w());
      DCM[2][1] = 2 * (quat.y()*quat.z() - quat.x()*quat.w());
      DCM[2][2] = quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z();
    
  }

  /* direction according to set up direction */
  pathDir[0] = DCM[0][0] * dir.x() + DCM[0][1] * dir.y() + DCM[0][2] * dir.z();
  pathDir[1] = DCM[1][0] * dir.x() + DCM[1][1] * dir.y() + DCM[1][2] * dir.z();
  pathDir[2] = DCM[2][0] * dir.x() + DCM[2][1] * dir.y() + DCM[2][2] * dir.z();
  //Serial.printf("Dir: %f   %f   %f   ", pathDir[0], pathDir[1], pathDir[2]);

  /* Constantly updating DCM according to the rocket's frame of reference */
  DCM2[0][0] = quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z();
  DCM2[0][1] = 2 * (quat.x()*quat.y() + quat.z()*quat.w());
  DCM2[0][2] = 2 * (quat.x()*quat.z() - quat.y()*quat.w());
  DCM2[1][0] = 2 * (quat.x()*quat.y() - quat.z()*quat.w());
  DCM2[1][1] = quat.w()*quat.w() - quat.x()*quat.x() + quat.y()*quat.y() - quat.z()*quat.z();
  DCM2[1][2] = 2 * (quat.y()*quat.z() + quat.x()*quat.w());
  DCM2[2][0] = 2 * (quat.x()*quat.z() + quat.y()*quat.w());
  DCM2[2][1] = 2 * (quat.y()*quat.z() - quat.x()*quat.w());
  DCM2[2][2] = quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z();

  /* Find the error in y and z directions according to the inertial frame of reference */
  double iError[4]; 
  iError[0] = 0;
  iError[1] = pathDir[1];
  iError[2] = pathDir[2];

  Serial.printf("error: %f   %f   ", iError[1], iError[2]);
  /* Apply the error to the constantly updating DCM to find the error components according to rocket FoR */
  double fError[4];
 
  fError[0] = DCM2[0][0] * iError[0] + DCM2[0][1] * iError[1] + DCM2[0][2] * iError[2];
  fError[1] = DCM2[1][0] * iError[0] + DCM2[1][1] * iError[1] + DCM2[1][2] * iError[2];
  fError[2] = DCM2[2][0] * iError[0] + DCM2[2][1] * iError[1] + DCM2[2][2] * iError[2];
  fError[3] = sqrt(fError[0]*fError[0] + fError[1]*fError[1] + fError[2]*fError[2]);
  
  Serial.printf("DCM error: %f   %f   %f   ", fError[0], fError[1], fError[2]);

  /* Add these error components to find direction in pitch and yaw */
  double errorPitch = fError[1];  
  double errorYaw = fError[2];

  Serial.printf("ePitch: %f   eYaw:%f   ", errorPitch, errorYaw);
  
  /* Find gimbal angles based on those new errors and convert to degrees*/
  double angles[2];
  angles[0] = asin(errorPitch) * 180/PI;
  angles[1] = asin(errorYaw) * 180/PI;
  Serial.printf("Pitch: %f   Yaw:%f\n",angles[0], angles[1]);

  //using pythagorean theorem to ensure a max of 10deg
  if (sin(adj[0])*sin(adj[0]) + sin(adj[1])*sin(adj[1]) <= sin(10*PI/180)*sin(10*PI/180)) {
    adj[1] = asin(sqrt(sin(10*PI/180)*sin(10*PI/180) - sin(adj[0])*sin(adj[0]))); //biasing to pitch
  }

  /* Convert gimbal angles to servo angles */ 
  angles[0] *= .4285;
  angles[1] *= .4285;

  /* Write angles to servos */
  float pitchMicroSeconds = map(angles[0], -60, 60 ,900, 2100);
  float yawMicroSeconds = map(angles[1], -60, 60 ,900, 2100);
  
  servo_pitch.write(pitchMicroSeconds);
  servo_yaw.write(yawMicroSeconds);

  /*
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
  
  //Serial.printf("Roll: %7.2f    modified: %7.2f %7.2f    ", roll, angles[0], angles[1]);

  angles[0] /= 0.4285;
  angles[1] /= 0.4285;
  Serial.printf("%7.2f %7.2f     \n", angles[0], angles[1]);
  float pitchMicroSeconds = map(angles[0], -45, 45 ,1000, 2000); //0.4285 linear relationship
  float yawMicroSeconds = map(angles[1], -45, 45 ,1000, 2000);
  
  servo_pitch.write(pitchMicroSeconds);
  servo_yaw.write(yawMicroSeconds);
  */

  //float pitch = map(pitchMicroSeconds, 1000, 2000 , -45, 45);
  //float yaw = map(yawMicroSeconds, 1000, 2000, -45, 45);
  //Serial.printf("%7.2f %7.2f     \n", pitch, yaw);
  
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
