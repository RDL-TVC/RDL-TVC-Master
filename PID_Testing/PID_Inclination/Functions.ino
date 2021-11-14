
/*
 * Returns Adjusted absolute angle for servo.
 * Input i is current inclination (Rad)
 */

double PID(double e, unsigned long timeNow)
{
  unsigned long dt = timeNow-timeLast;
  
  double out = P * e + D * ((e - errorLast)/dt);
  
  errorLast = e;
  timeLast = timeNow;

  return out;
}

/*
 * Given wanted absolute angle (Rad) and roll (Rad), 
 * calculates needed angle for servos and transmit it to them
 */

void angle2Servo(double a, double xy[2])
{
  if (a == 0)
  { // To protect against div by 0 errors
    // Center Servos
    servoX.writeMicroseconds(1500);
    servoY.writeMicroseconds(1500);
    return;
  }
  
  double angle = min(a, MAX_GIMBAL_ANGLE / RAD_TO_DEG);

  double z = 1/tan(angle);
  
  double servoXRad = atan2(xy[0], z) / SERVO_TO_GIMBAL;
  double servoYRad = atan2(xy[1], z) / SERVO_TO_GIMBAL;

  double servoXMicro = map(servoXRad, -PI/3, PI/3, 900, 2100);
  double servoYMicro = map(servoYRad, -PI/3, PI/3, 900, 2100);

  if (abs(servoXMicro - lastServoWrite[0]) > 100)
  {
    servoXMicro = lastServoWrite[0];
  }
  if (abs(servoYMicro - lastServoWrite[1]) > 100)
  {
    servoYMicro = lastServoWrite[1];
  }
  
  servoX.writeMicroseconds(servoXMicro);
  servoY.writeMicroseconds(servoYMicro);

  lastServoWrite[0] = servoXMicro;
  lastServoWrite[1] = servoYMicro;
}

int LEDBlink(int LED, unsigned int dutyCycle, float ratio)
{
  
  if (millis() % (dutyCycle) <= (dutyCycle) * ratio)
  { // If at time during cycle where LED should be on.
    digitalWrite(LED, HIGH);
    return 1;
  } else
  {
    digitalWrite(LED, LOW);
    return 0;
  }
  
}

bool logBuff()
{
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) 
  {
    Serial.println("File full - quiting.");
    file.truncate();
    file.close();
    return true;
  } if (n > maxUsed) 
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) 
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) 
    {
      Serial.println("writeOut failed"); 
      // Probably not the best call to give up whole program if writeout fails
      file.truncate();
      file.close();
      return true;
    }
  }
  if (millis() > MAX_TIME) 
  {
    Serial.println("Times Up - quitting.");
    file.truncate();
    file.close();
    return true;
  }
  return false;
}

void tick()
{
  unsigned long t = millis();
  // Run PID on call
  // interrupts();
  imu::Quaternion quat = bno.getQuat(); // Interrupts need to be on here to get the serial communication hopefully they are.
  
  double q[4] = {quat.w(), quat.x(), quat.y(), quat.z()};

  // Inclination: angle from upwards x-axis
  // we want this to be 0, Use PID to do so
  double i = acos(2 * (q[0]*q[0] + q[3]*q[3]) - 1);

  double w = -atan2(2 * (q[0] * q[2] - q[1] * q[3]), 2 * (q[0] * q[1] + q[2] * q[3]));

  double xy[2] = {-sin(w), cos(w)};

  double a = PID(i - targetInclination, t);

  angle2Servo(a, xy);

  char str[100];
  sprintf(str, "%10lu,% 1.8f,% 1.8f,% 1.8f,% 1.8f,% 4d,% 4d\n", t, q[0], q[1], q[2], q[3], lastServoWrite[0], lastServoWrite[1]);

  rb.memcpyIn(str, 72);
}
