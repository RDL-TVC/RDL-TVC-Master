
int freefall(){
  int nextState = 5;

  chuteDeployAltitude = 1; //TODO: Determine threshold altitude for deploying parachutes
  
  array alt = altitued();
  array orient = orientation();
  
  datalog(alt, orient, millis());

  if (alt <= chuteDeployAltitude){
    nextState = 6;
    deployChutes()
  }

  return nextState;
}
