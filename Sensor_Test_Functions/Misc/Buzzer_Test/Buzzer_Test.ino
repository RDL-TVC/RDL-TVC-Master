int piezoPin = 4;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  tone(piezoPin,3000,1000);
  delay(1000);
  tone(piezoPin,5000,1000);
  delay(1000);
 
}
