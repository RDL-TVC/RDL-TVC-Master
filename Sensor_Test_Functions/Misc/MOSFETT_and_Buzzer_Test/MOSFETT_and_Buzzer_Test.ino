#define Mosfet1 4
#define Mosfet2 5

int piezoPin = 14;


void setup() {
  // put your setup code here, to run once:
  pinMode(Mosfet1, OUTPUT);
  pinMode(Mosfet2, OUTPUT);
  pinMode(piezoPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(Mosfet1,HIGH);
  tone(piezoPin,2000,5000);
  delay(5000);
  digitalWrite(Mosfet1,LOW);
  delay(5000);
  
  digitalWrite(Mosfet2,HIGH);
  tone(piezoPin,2000,5000);
  delay(5000);
  digitalWrite(Mosfet2,LOW);
  delay(5000);
}  
