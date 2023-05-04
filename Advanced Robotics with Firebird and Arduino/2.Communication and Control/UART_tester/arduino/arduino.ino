/********************************************************************************
 Code to test Serial communication: ARDUINO
 Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)
 
 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Note: 
  >Microcontroller: atmega2560
  >Frequency: 14745600s
  >Optimization: -O0 (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)
*********************************************************************************/
#define ledpin 13             //Led control pin

void setup() 
{
  Serial.begin(9600);
  digitalWrite(ledpin, HIGH);
  delay(1000);
  digitalWrite(ledpin, LOW);
  delay(1000);
}
void loop() {
    Serial.write("ARDUINO");
    Serial.println(Serial.readString());
    delay(1500);
}
