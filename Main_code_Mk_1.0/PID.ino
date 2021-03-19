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
  return adj;
}
