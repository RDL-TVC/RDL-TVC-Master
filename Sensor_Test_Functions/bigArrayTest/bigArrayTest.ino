#include <Wire.h>

#define SIZE 10000
DMAMEM int bigArray[SIZE];

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  while (!Serial);
  for (int i = 0; i < SIZE; i++){
      bigArray[i] = i;
      Serial.println(bigArray[i]);
    }
  Serial.println(sizeof(bigArray));
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
