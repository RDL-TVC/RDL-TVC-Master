


String response = "";
int i = 0;


void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); // send and receive at 9600 baud
}

void loop() 
{
  // put your main code here, to run repeatedly:
  Serial.println("would you like to run the code? (y or n) ");
  while (Serial.available() == 0)   
  {}  
  response = Serial.readString();
  Serial.println("You entered: " + response);
  if(response.equals("y"))
  {
    for(i = 0; i <= 10; i++)
    {
      Serial.println("It Worked");
      delay(10);
    }
  }
  else
  {
    Serial.println("It didn't Work");
  }

}
