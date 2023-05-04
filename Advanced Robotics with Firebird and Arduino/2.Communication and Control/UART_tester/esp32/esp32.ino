/********************************************************************************
 Code to test Serial communication: ESP32
 Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)
 
 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Microcontroller: ESP32
*********************************************************************************/
#define ledpin 2             //Led control pin

void setup() 
{
  Serial.begin(9600);
  delay(1000);  
  digitalWrite(ledpin, HIGH);
  delay(1000);
  digitalWrite(ledpin, LOW);
  delay(1000);
}
void loop() 
{
  Serial.write("ESP32");
  Serial.println(Serial.readString());
  delay(1500);
}
