/*
 * The following code is taken directly from the Adafruit tutorial for the INA260 sensor tutorial:
 * https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout/arduino
 * It should output the current, voltage, and power to the serial log when ran from an arduino, viewable through the editor
 * or through an external program like Putty
 * 
 * File added by: Rita Tagarao
 * 02-11-2020
 */
 
#include <Adafruit_INA260.h>
 
Adafruit_INA260 ina260 = Adafruit_INA260();
 
void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(10); 
}
 
  Serial.println("Adafruit INA260 Test");
 
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
}
 
void loop() {
  Serial.print("Current: ");
  Serial.print(ina260.readCurrent());
  Serial.println(" mA");
 
  Serial.print("Bus Voltage: ");
  Serial.print(ina260.readBusVoltage());
  Serial.println(" mV");
 
  Serial.print("Power: ");
  Serial.print(ina260.readPower());
  Serial.println(" mW");
 
  Serial.println();
  delay(1000);
}
