
int chute(){
  int nextState = 6;

  array alt = altitude();
  array orient = orientation();
  
  datalog(alt, orient);

  if (orient[2] >= lastAccel){
    nextState = 0; //Placeholder for failure state if parachutes do not deploy
  }else if (alt >= lastAlt){
    nextState = 7;
  }

  lastAccel = orient[2];

  return nextState;
}
