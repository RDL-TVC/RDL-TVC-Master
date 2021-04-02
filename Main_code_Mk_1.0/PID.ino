//error[] = desiredAngle - currentOrientation
float* PID(float error[]){
  static float adj[2] = {0, 0}; 

  
  const float P = 0;
  const float I = 0;
  const float D = 0;
  
  elapsedMillis dTime;
  float lastErrorPitch;
  float lastErrorYaw;

  float sumPitch;
  float sumYaw;

  sumPitch += error[0] * (dTime);
  sumYaw += error[1] * (dTime);

  adj[0] = P * error[0] + I * sumPitch + D * (error[0] - lastErrorPitch)/(dTime);
  adj[1] = P * error[1] + I * sumYaw + D * (error[0] - lastErrorYaw)/(dTime);

  float lastError = error[0];
  dTime = 0;

  //using pythagorean theorem to ensure a max of 10deg
  if (sin(adj[0])*sin(adj[0]) + sin(adj[1])*sin(adj[1]) <= sin(10*PI/180)*sin(10*PI/180)) {
    adj[1] = asin(sqrt(sin(10*PI/180)*sin(10*PI/180) - sin(adj[0])*sin(adj[0]))); //biasing to pitch
  }

  //Finds required servo angle using the linear relationship gimbalAngle = 0.4285*servoAngle and converts from radians to degress
  adj[0] *= 180/(PI*0.4285);
  adj[1] *= 180/(PI*0.4285);

  //Converts angles to micro seconds - what the BlueBird BMS-127WV servo reads
  adj[0] = map(adj[0], -45, 45 ,1000, 2000); 
  adj[1] = map(adj[1], -45, 45 ,1000, 2000); 
  return adj;
}
