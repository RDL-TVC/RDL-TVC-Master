
double PID(double error){
  int nowTime = millis();
  sum += error * (nowTime - lastTime);
  double adj = P * error + I * sum + D * (error - lastError)/(nowTime - lastTime);
  lastError = error;
  lastTime = nowTime;
  return adj;
}
