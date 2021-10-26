#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include <SdFat.h>
#include <RingBuf.h>

const int BUZZER = 4;

const int TONE_SUCCESS = 523; // In Hz, Currently C5
const int TONE_FAILURE = 261; // In Hz, currently C4
const int TONE_VICTORY = 1046; // In Hz, Currently C6

const int LED_GREEN = 7;
const int LED_RED = 8;

const int BNO_INT = 32;

volatile bool intFlag = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  
  // Set Pin mode for status LED
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_RED,OUTPUT);

  // Connect to Serial
  Serial.begin(9600);
  
  Serial.print("Initializing BNO055...");
  
  if (!bno.begin())
  { // BNO055 was not able to initialize
    Serial.println("BNO055 failed to initialize.");
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
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

  Serial.println("Calibration of BNO055 finished. Enabling Interrupts.");

  // Testing Gyro any motion interrupt
  bno.enableInt(0b00000100);

  // Interrupt on any motion about any axis
  bno.enableGyrAxisInt(0b00000111);

  // Setting interrupt sensitivity
  bno.configGyrAM(0b00001000, 0b00000010, 0b00000001);
/*
  // Testing Gyro High Rate interrupt (it is unlcear if this is sensitive enough for our purposes)
  bno.enableInt(0b00001000);

  // Interrupt on high rate about any axis
  bno.enableGyrAxisInt(0b00111000);

  // Setting interrupt sensitivity
  bno.configGyrHR(0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001);
*/
  // Sensors Initialized. Celebrate!
  digitalWrite(LED_GREEN, HIGH);
  tone(BUZZER, TONE_SUCCESS, 500);
  delay(500);
  tone(BUZZER, TONE_VICTORY, 250);
  delay(250);

//  Serial.println("Starting Interrupt Pulse.");
//  pulse.begin(BNO055ISR,10000);

  pinMode(BNO_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(BNO_INT), BNO055ISR, RISING);
  
}

void loop() 
{  
  if (intFlag)
  {
    intFlag = false;
    Serial.println("Interrupted");
  }
}

void BNO055ISR()
{

  uint32_t logTime = millis();
  
  intFlag = true;

  bno.clearInt();
}
