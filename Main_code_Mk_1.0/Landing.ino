
int landing(){
  int nextState = 7;

  writeToSD(); //Under current conditions, this will run in a loop indefitely. Either the main loop should stop after landing or the write function call should be called at the end of chute().

  return nextState;
}
