int failure(int typeOfFailure) {
  orientation(orient);
  Serial.printf("Failure\n");

  switch (typeOfFailure) {
    case 7:
      failure(altsDataLoss());
      break;

    case 8:
      failure(orientDataLoss());
      break;

    case 9:
      failure(voltageDrop);
      break;

    case 10:
      failure(groundSensorLoss());
      break;

    case 11:
      failure(excessiveTilt());
      break;

    case 12:
      failure(noChute());
      break;

    default:
      failure(20);
      Serial.println("Unknown error");
      break;
  }
  return typeOfFailure; //state number for failure
}

int altsDataLoss() {
  //Lock gimbal, deploy chute, save data to SD​
  servoPitch.write(90);
  servoYaw.write(90);
}

int orientDataLoss() {
  //If in Boost state lock gimbal, deploy chute, save data to SD​
  servoPitch.write(90);
  servoYaw.write(90);
}

int voltageDrop() {
  //Lock gimbal, deploy chute, save data to SD 
  servoPitch.write(90);
  servoYaw.write(90);
}

int groundSensorLoss() {
  //Flash LED, sound piezo buzzer, save data to SD, stop code execution, set gimbal to max pitch downrange​
}

int excessiveTilt() {
  //Lock gimbal, deploy chute, save data to SD​
  servoPitch.write(90);
  servoYaw.write(90);
}

int noChute() {
  //Attempt to deploy chute again, save data to SD
}
