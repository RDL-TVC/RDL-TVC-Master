
double PID(double error[]){
  int nowTime = millis();
  sumPitch += error[0] * (nowTime - lastTime);
  sumYaw += error[1] * (nowTime - lastTime);
  double adj[2] = {0, 0};
  adj[0] = P * error[0] + I * sumPitch + D * (error[0] - lastErrorPitch)/(nowTime - lastTime);
  adj[1] = P * error[1] + I * sumYaw + D * (error[0] - lastErrorYaw)/(nowTime - lastTime);
  lastError = error;
  lastTime = nowTime;
  return adj;
}
