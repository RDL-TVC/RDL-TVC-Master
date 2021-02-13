
void failure() {
  float alts = altSensor.getAlt();
  int orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);
}
