/********************************************************************************
 Buzzer code for Firebird V
 Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)
 
 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Note: 
  >Microcontroller: atmega2560
  >Frequency: 14745600s
  >Optimization: -O0 (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)
*********************************************************************************/
#define buzzer 34              //buzzer control pinvoid setup()
void setup() 
{
  pinMode(buzzer, OUTPUT);
  //boot();                    //Booting indicator
}

void loop() 
{
  digitalWrite(buzzer, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                  // wait for a second
  digitalWrite(buzzer, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                  // wait for a second
}
//Application
void boot()                     //Buzzer theme
  {
    digitalWrite(buzzer, HIGH); // buzzer ON
    delay(100); 
    digitalWrite(buzzer, LOW);  // buzzer OFF
    delay(100);
    digitalWrite(buzzer, HIGH); // buzzer ON
    delay(200); 
    digitalWrite(buzzer, LOW);  // buzzer OFF
    delay(200);
    digitalWrite(buzzer, HIGH); // buzzer ON
    delay(100); 
    digitalWrite(buzzer, LOW);  // buzzer OFF
    delay(100);
    digitalWrite(buzzer, HIGH); // buzzer ON
    delay(300); 
    digitalWrite(buzzer, LOW);  // buzzer OFF
    delay(300);
  }