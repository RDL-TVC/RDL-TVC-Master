
int burnout(){
  int nextState = 4;

  array alt = altitude();
  array orient = orientation();
  
  datalog(alt, orient);

  if (alt < lastAlt){
    nextState = 5;
  }

  lastAlt = alt;
  
  return nextState;
}
