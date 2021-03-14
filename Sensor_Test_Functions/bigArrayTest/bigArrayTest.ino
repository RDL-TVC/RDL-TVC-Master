#include <Wire.h>

//120000 * 4 ints max on flash1 or flash2
#define SIZE 500000
static EXTMEM uint32_t bigArray[SIZE];

/*int incomingByte = 0; // for incoming serial data
int digit[7];
int place = 0;
uint32_t outputIndex;*/

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);
  while (!Serial);
  
  for (uint32_t i = 0; i < (sizeof(bigArray)/sizeof(int)); i++){
      bigArray[i] = i;
      //Serial.println(bigArray[i]);
      //delay(1);
    }

  /*for (uint32_t i = 0; i < (sizeof(bigArray)/sizeof(int)); i++){
      //bigArray[i] = i;
      //Serial.println(bigArray[i]);
      //delay(1);
    }*/
  
  /*Serial.print("Size: ");
  Serial.println(sizeof(bigArray));
  Serial.print("Last Value: ");
  Serial.println(bigArray[49]);*/
}

void loop() {
  // put your main code here, to run repeatedly:

  /*if (place == 6){
    outputIndex = digit[0] * 1000000 + digit[1] * 100000 + digit[2] * 10000 + digit[3] * 1000 + digit[4] * 100 + digit[5] * 10 + digit[6];
    Serial.print("Value at location: ");
    Serial.println(bigArray[outputIndex]);
    place = 0;
  }
  
  if (Serial.available() > 0 && place<7) {
    // read the incoming byte:
    incomingByte = Serial.read();
    digit[place] = incomingByte;
    place++;

    Serial.print("I recieved: ");
    Serial.println(incomingByte);
  }
  */
  
}
