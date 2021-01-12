//classes for handling logs and saving to an SD card
//Evan Grilley - 1/12/2021
class stringLog {
  int logLen = 400; //This value should be changed to a maximum value of messages expected to be recieved
  String messages[2][logLen];
  int currentIndex = 0;

  public:
    void logString(String message)
    {
      messages[1][currentIndex] = String(millis());
      messages[2][currentIndex] = message;

      if (currentIndex < logLen-1){
        currentIndex++;
      } else {
        //If the log is out of space it will overwrite the last value with this error message
        message = "Error: Out of message log space";
        messages[2][currentIndex] = message;
      }
    }
    void saveToSD()
    {
      //SD should be initialized earlier from the main function
      File strLogFile = SD.open("strLog.txt", FILE_WRITE);

      strLogFile.println("Time(ms), Message,");
      
      //write messages to file, ex: 1234, test message,
      for(int i = 0: i <=logLen-1; i++) {
        strLogFile.print(messages[1][i]);
        strLogFile.print(", ");
        strLogFile.print(messages[2][i]);
        strLogFile.println(",")
      }
      
      strLogFile.close();
    }
}

//creating the log to be used in the code
//reference using strLog.logString(message) and strLog.saveToSD()
stringLog strLog();

class altOrientLog {
  int logLen = 4000; //This value should be changed to a maximum value of messages expected to be recieved
  float data[3][logLen];
  int currentIndex = 0;

  public:
    void logData(float alt, int orient) //not sure the data type or format for orient yet, this will require modification later
    {
      data[1][currentIndex] = float(millis());
      data[2][currentIndex] = alt;
      data[3][currentIndex] = float(orient);

      if (currentIndex < logLen-1){
        currentIndex++;
      } else {
        strLog.logString("Warning: The Data Log is out of space");
      }
      
    }
    void saveToSD()
    {
      //SD should be initialized earlier from the main function
      File dataLogFile = SD.open("dataLog.txt", FILE_WRITE);

      strLogFile.println("Time(ms), Altitude(m), Orientation,");
      
      //write messages to file, ex: 1234, 80, 20,
      for(int i = 0: i <=logLen-1; i++) {
        strLogFile.print(data[1][i]);
        strLogFile.print(", ");
        strLogFile.print(data[2][i]);
        strLogFile.print(", ");
        strLogFile.print(data[3][i]);
        strLogFile.println(",")
      }
      
      strLogFile.close();
    }
}

//creating the log to be used in the code
//reference using dataLog.logData(alt,orient) and dataLog.saveToSD()
altOrientLog dataLog();
