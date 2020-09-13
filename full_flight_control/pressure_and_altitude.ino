
////////////////////PRESSURE AND ALTITUDE////////////////////

void setupPressure()
{
  if (pressure.begin()) // initialize BMP180 (check if it's working)
  {
    data.println("Pressure initialization success");
    data.println();
  }
  else
  {
    data.println("Pressure initialization failed");
    data.println("F in the chat");
    failTone();
    while(1); // Pause
  }
  
  P_prelaunch = //collect initial pressure
  (getPressure() + getPressure() + getPressure() + getPressure() + getPressure() +
  getPressure() + getPressure() + getPressure() + getPressure() + getPressure() +
  getPressure() + getPressure() + getPressure() + getPressure() + getPressure() +
  getPressure() + getPressure() + getPressure() + getPressure() + getPressure()) / 20; // initial measurements
  tone(0,500);
  delay(500);
  tone(0,1000);
  delay(500);
  noTone(0);
}

double getPressure()
{
  char status;
  double T,Pr_temp;

  status = pressure.startTemperature();
  if (status != 0) // confirm temperature start
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) // confirm temperature check
    {
      status = pressure.startPressure(3);
      if (status != 0) // confirm pressure start
      {
        delay(status);
        status = pressure.getPressure(Pr_temp,T);
        if (status != 0)
        {
          error_occured = false;
          return Pr_temp;
        }
        else data.println("Error taking pressure");
      }
      else data.println("Error starting pressure");
    }
    else data.println("Error taking temperature");
  }
  else data.println("Error starting temperature");
  return failsafe();
}
