
int burnout(){
  int nextState = 4;

  array alt = altitued();
  array orient = orientation();
  
  datalog(alt, orient, millis());

  if (alt < lastAlt){
    nextState = 5;
  }

  lastAlt = alt;
  
  return nextState;
}
