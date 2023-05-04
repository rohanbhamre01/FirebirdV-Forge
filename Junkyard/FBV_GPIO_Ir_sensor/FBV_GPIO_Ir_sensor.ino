/********************************************************************************
  In this experiment Gpio pins connect the analog sensor values and displays it on the Serial monitor
  
  Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)


 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
     To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
     
 Note: 
  Microcontroller: atmega2560
  Frequency: 14745600s
  Optimization: -O0 (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)
*********************************************************************************/
// IR sensor connections
#define IRproxyEN 6
#define IR1 A4
#define IR2 A5
#define IR3 A6
#define IR4 A7
#define IR5 A8

void setup() 
{
  Serial.begin(9600);
  pinMode(IRproxyEN,OUTPUT);
  digitalWrite(IRproxyEN,LOW);
}

void loop() {
  // Read the sensor values
  int ir1 = analogRead(IR1);
  int ir2 = analogRead(IR2);
  int ir3 = analogRead(IR3);
  int ir4 = analogRead(IR4);
  int ir5 = analogRead(IR5);
  int max_distance = 100; // Maximum distance in millimeters (100 cm)
  
  ir1 = min(ir1, max_distance);
  ir2 = min(ir2, max_distance);
  ir3 = min(ir3, max_distance);
  ir4 = min(ir4, max_distance);
  ir5 = min(ir5, max_distance);
  // Check if there is an obstacle
  bool obstacle = (ir1 < 100) || (ir2 < 100) || (ir3 < 100) || (ir4 < 100) || (ir5 < 100);

  if(obstacle)
  {   
    Serial.println("Obstacle Detected");// Print the sensor values to the Serial Monitor
    }
  else
  {
    Serial.print("IR1: ");
    Serial.print(ir1);
    Serial.print(" IR2: ");
    Serial.print(ir2);
    Serial.print(" IR3: ");
    Serial.print(ir3);
    Serial.print(" IR4: ");
    Serial.print(ir4);
    Serial.print(" IR5: ");
    Serial.println(ir5);
    }
}
