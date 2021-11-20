
bool IOInit()
{
  // Set Pin mode for status LED
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_RED,OUTPUT);

  // Connect to Serial
  Serial.begin(9600);

  return true;
}

bool BNOInit()
{
  Serial.print("Initializing BNO055...");
  
  if (!bno.begin(bno.OPERATION_MODE_NDOF, B00001001))
  { // BNO055 was not able to initialize
    Serial.println("BNO055 failed to initialize.");
    return false;
  }

  Serial.println("BNO055 Initialized. Now Calibrating.");

  delay(1000);
  bno.setExtCrystalUse(true);

  // Calibrating BNO055. Do not Disturb!
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

  while (cal != 3)
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

  return true;
}

bool logInit()
{
  Serial.print("Initializing SD card...");

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    return false;
  }

  Serial.println("Card initialized.");

  Serial.print("Creating and Opening Logfile...");
  
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) 
  {
    Serial.println("File opening failed\n");
    return false;
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) 
  {
    Serial.println("Preallocate failed\n");
    file.close();
    return false;
  }
  
  // initialize the RingBuf.
  rb.begin(&file);

  Serial.println("Ringbuffer and Logfile initialized.");

  logHeader();

  return true;
}

bool servoInit()
{
  // Initialise Servos
  analogWriteResolution(SERVO_RESOLUTION);
  analogWriteFrequency(SERVO_PIN_X, SERVO_FREQUENCY);
  analogWriteFrequency(SERVO_PIN_Y, SERVO_FREQUENCY);

  testGimbal();
  return true;
}

/*
 * Test routine to make sure gimbal is functioning within range
 */
void testGimbal()
{
  // center gimbal
  double xy[2] = {0, 0};
  
  angle2Servo(0, xy);

  double wMax = PI * 2;
  double aMax = MAX_GIMBAL_ANGLE / RAD_TO_DEG;
  double w_to_a = aMax / wMax;

  // Spiral Outwards
  for (float w = 0; w < wMax; w += .01)
  {
    xy[0] = cos(w);
    xy[1] = sin(w);
    angle2Servo(w * w_to_a, xy);
    delay(1);
  }

  // One full rotation at max angle
  for (float w = 0; w < wMax; w += .01)
  {
    xy[0] = cos(w);
    xy[1] = sin(w);
    angle2Servo(aMax, xy);
    delay(1);
  }

  // Spiral Inwards
  for (float w = 0; w < wMax; w += .01)
  {
    xy[0] = cos(w);
    xy[1] = sin(w);
    angle2Servo(aMax - (w * w_to_a), xy);
    delay(1);
  }

  // Re-Center gimbal
  angle2Servo(0, xy);
}

int logHeader()
{
  char header[512]; 
  strcpy(header, "(unsigned long) Time [ms],(double) Quaternion W,(double) Quaternion X,(double) Quaternion Y,(double) Quaternion Z,(int) X Servo [us],(int) Y Servo [us]\n");

  rb.memcpyIn(header, 153);
  
  return 1;
}
