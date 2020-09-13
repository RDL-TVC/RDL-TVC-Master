
////////////////////FLIGHT STAGES////////////////////

void liftoffDetection()
{
  if ((not liftoff_detected) and (A >= liftoff_threshold)) // liftoff detection; prevents chute deployments on the ground
  {
    liftoff_detected = true;
    data.println();
    data.print("Liftoff detected: ");
    data.print(millis() / 1000.0);
    data.println(" s");
    data.println();
    tone(0,250);
    time_liftoff = millis();
    time_last = time_liftoff;
  }
}

void descentDetection()
{
  if (not drogue_deployed) // ascent phase of flight
  {
    if (A >= A_max) // test if above highest recorded alt
    {
      if(A_count >= 0) // reset count
      {
        A_count = 0;
      } 
    }
    else // if below or at apogee
    {
      A_count = A_count + 1; // add to count
    }
  }
}

void landingDetection()
{
  if (drogue_deployed and main_deployed and ((dA <= landing_threshold) or (dA * -1.0 <= landing_threshold))) // landing detection
  {
    data.println();
    data.println("Landing detected");
    data.print("Total flight time: ");
    data.print((millis() - time_liftoff) / 1000.0);
    data.println(" s");
    data.print("Max altitude : ");
    data.print(A_max);
    data.println(" m");
    data.print("Max velocity : ");
    data.print(V_max);
    data.println(" m/s");
    tone(0,1000);
    delay(1000);
    noTone(0);
    data.close();
    while(1);
  }
}

void deployDrogue() // deployed at apogee
{
  data.println();
  data.print("Drogue chute deployment initiated: ");
  data.print(millis() / 1000.0);
  data.println(" s");
  data.println();
  tone(0,500);

  s.write(135);

  drogue_deployed = true;
}

void deployMain() // deployed at specified altitude
{
  data.println();
  data.print("Main chute deployment initiated: ");
  data.print(millis() / 1000.0);
  data.println(" s");
  data.println();
  tone(0,750);

  // deployment code

  main_deployed = true;
}

void failTone()
{
  tone(0,300,2000);
  noTone(0);
}

double failsafe() // must be activated twice to trigger an abort
{
  data.println();
  data.println("Failsafe activated");
  data.println();
  
  if(error_occured){
    if(drogue_deployed and (not main_deployed)) // in descent before main is deployed
    {
      deployMain();
    }
    else if (liftoff_detected and (not drogue_deployed)) //in ascent
    {
      delay(1000);
      deployDrogue;
      delay(4000);
      deployMain;
    }
    data.println();
    data.print("Abort at ");
    data.print(millis() / 1000.0);
    data.println(" s");
    data.close();
    failTone();
    while(1);
  }
  else
  {
    error_occured = true;
    return getPressure();
  }    
}
