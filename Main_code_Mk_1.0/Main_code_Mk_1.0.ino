
const int armingPin = 2; // Place holder pin for the arming button
const int chuteChargeContOut = 5; // Placeholder Not sure how the continuity of the chute charge will be tested.
const int chuteCharge1 = 3; // Placeholder
const int chuteCharge2 = 4; // Placeholder
const float accelThreshold = 10; // Placeholder
int currentState = 0; // State of the state machine to know which flight function to call. Starts at startup.

void setup() {
  // Initializing all Pins

  pinMode(armingPin, INPUT);
  pinMode(chuteCharge1, INPUT);
  pinMode(chuteCharge2, INPUT);

  pinMode(chuteChargeContOut, OUTPUT);

  // Initializing sensors and center equipment
  initializeSensors();
  initializeServos();

}

void loop() {

  currentState = callFLightFunc(currentState); // added a function rapper to make more modular, however unlikely to be needed

}

int callFLightFunc(int state) {
  /* 
  Intakes the current state of the state machine and runs the appropriate function for that state.
  Returns the next state of the state machine
  */
  int nextState = state;
  if (state == 1) {
    nextState = startup();
  } else if (state == 2) {
    nextState = groundidle();
  } else if (state == 3) {
    nextState = liftoff();
  } else if (state == 4) {
    nextState = burnout();
  } else if (state == 5) {
    nextState = freefall();
   }else if (state == 6) {
    nextState = chute();
  } else if (state == 7) {
    nextState = landing();
  } else {
    nextState = failure(); // General failure state might need to specify different failure states.
  }
  return nextState;
  }
