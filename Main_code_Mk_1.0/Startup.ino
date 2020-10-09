
int startup() {
  int nextState = 1;
  digitalWrite(chuteChargeContOut, HIGH;
  int charge1 = digitalRead(chuteCharge1);
  int charge2 = digitalRead(chuteCharge2); 
  digitalWrite(chuteChargeContOut, LOW);
  int armed = digitalRead(armingPin);

  if ((not charge1) or (not charge2)){
    nextState - 0; // Placeholder for failure state at least one of the chute charges is not connected
  } else if (armed) {
    nextState = 2;
  }
  return nextState;
}
