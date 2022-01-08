#define Serial ThreadClone(SerialXtra)

/*
 * Given wanted absolute angle (Rad) and roll (Rad), 
 * calculates needed angle for servos and transmit it to them
 */

void writeServos(int xmicro, int ymicro)
{
  uint32_t xduty = (int)((1<<SERVO_RESOLUTION) * xmicro * SERVO_FREQUENCY / 1000000.0);
  uint32_t yduty = (int)((1<<SERVO_RESOLUTION) * ymicro * SERVO_FREQUENCY / 1000000.0);
  analogWriteResolution(SERVO_RESOLUTION);
  analogWrite(SERVO_PIN_X, xduty);
  analogWrite(SERVO_PIN_Y, yduty);
}

void angle2Servo(double a, double xy[2])
{
  if (a == 0)
  { // To protect against div by 0 errors
    writeServos(1500, 1500);
    /*
    Serial.print("XDiff: ");
    Serial.println(0);
    Serial.print("YDiff: ");
    Serial.println(0);
    Serial.print("Servo X: ");
    Serial.println(1500);
    Serial.print("Servo Y: ");
    Serial.println(1500);
    */
    return;
  }
  
  double angle = min(a, MAX_GIMBAL_ANGLE / RAD_TO_DEG);

  double z = 1/tan(angle);
  
  double servoXRad = atan2(xy[0], z) / SERVO_TO_GIMBAL;
  double servoYRad = atan2(xy[1], z) / SERVO_TO_GIMBAL;

  double servoXMicro = min(max(map(servoXRad, -PI/3, PI/3, 900, 2100), 900), 2100);
  double servoYMicro = min(max(map(servoYRad, -PI/3, PI/3, 900, 2100), 900), 2100);

  double Xdiff = servoXMicro - lastServoWrite[0];
  double Ydiff = servoYMicro - lastServoWrite[1];
  if (Xdiff > SERVO_LIMITING or Xdiff < -SERVO_LIMITING)
  {
    servoXMicro = lastServoWrite[0] + SERVO_LIMITING * Xdiff/abs(Xdiff);
  }
  if (Ydiff > SERVO_LIMITING or Ydiff < -SERVO_LIMITING)
  {
    servoYMicro = lastServoWrite[1] + SERVO_LIMITING * Ydiff/abs(Ydiff);
  }
/*
  Serial.print("XDiff: ");
  Serial.println(Xdiff);
  Serial.print("YDiff: ");
  Serial.println(Ydiff);
  
  Serial.print("Servo X: ");
  Serial.println(servoXMicro);
  Serial.print("Servo Y: ");
  Serial.println(servoYMicro);
*/
  writeServos(servoXMicro, servoYMicro);

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

void PIDThread()
{
  while (1){
    // Run PID on call
    imu::Quaternion quat = bno.getQuat(); // Interrupts need to be on here to get the serial communication hopefully they are.
  
    double q[4] = {quat.w(), quat.x(), quat.y(), quat.z()};

    // Inclination: angle from upwards x-axis
    // we want this to be 0, Use PID to do so
    double i = acos(2 * (q[0]*q[0] + q[3]*q[3]) - 1);

    double w = -atan2(2 * (q[0] * q[2] - q[1] * q[3]), 2 * (q[0] * q[1] + q[2] * q[3]));
    
    unsigned long dt = millis() - timeLast;
    double iOut = I_P * i + I_D * ((i - iLast)/(dt));
    iLast = i;
    
    double wOut = w;
    
    if (w - wLast < MAX_DW){
      wOut = w + W_D * (w - wLast);
    }
    
    wLast = w;
    
    double xy[2] = {-sin(wOut), cos(wOut)};
    
    if (dt > 12){
      //Serial.print("Last PID Completion: ");
      //Serial.println(dt);
    }
    
    if (iOut < 0)
    {
      iOut = abs(iOut);
      xy[0] = - xy[0];
      xy[1] = - xy[1];
    }

    angle2Servo(iOut, xy);
    
    timeLast = millis();
    
    char str[100];
    sprintf(str, "%10lu,% 1.8f,% 1.8f,% 1.8f,% 1.8f,% 4d,% 4d\n", timeLast, q[0], q[1], q[2], q[3], lastServoWrite[0], lastServoWrite[1]);
    rb.memcpyIn(str, 72);
    threads.delay(10);
  }
  
}
