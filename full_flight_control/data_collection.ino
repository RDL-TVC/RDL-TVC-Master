
////////////////////DATA COLLECTION////////////////////

void setupSD()
{
  Serial.println("SD initialization start"); // data collection initialization
  if(!SD.begin(10)){
    Serial.println("Failed SD initialization");
    failTone();
    while(1); // pause
  }
  Serial.println("Completed SD initialization");

  while(SD.exists(file_Full_Name)){ // data file creation
    Serial.println(file_Full_Name + " already exists");
    file_Number = file_Number + 1;
    file_Full_Name = file_Name + (String)file_Number + ".txt";
  }
  data = SD.open(file_Full_Name, FILE_WRITE);
  Serial.println(file_Full_Name + " created");

  data.print("Data start: ");
  data.print(millis() / 1000.0);
  data.println(" s");
}

void recordData()
{
  data.print(pitch);
  data.print(" deg pitch    ");
  data.print(roll);
  data.print(" deg roll    ");
  data.print(yaw);
  data.print(" deg yaw    ");
  
  data.print("A = ");
  data.print(A);
  data.print(" m    dA = ");
  data.print(dA);
  data.print(" m    V = ");
  data.print(V);
  data.print(" m/s    A_count = ");
  data.print(A_count);
  data.print(" / ");
  data.println(deployDrogue_count);
}
