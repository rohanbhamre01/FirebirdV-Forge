/********************************************************************************
 Code for Waitron Robot Obstacle Detection
 Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)
 
 ADC Connection:
         ACD CH. PORT  Sensor
          1    ADC9    Sharp IR range sensor 1 left
          2    ADC10   Sharp IR range sensor 2
          3    ADC11   Sharp IR range sensor 3 front
          4    ADC12   Sharp IR range sensor 4
          5    ADC13   Sharp IR range sensor 5 right
       
 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
     To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
     
 Note: 
  >Microcontroller: atmega2560
  >Frequency: 14745600s
  >Optimization: -O0 (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)
  >Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor
  >"Serial3.println("");                        // acknowledgement" if this line is removed
  The code wont work.[Reasons unknown]

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.     -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
   For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<math.h>

//*************************Declarations****************************************//
#define buzzer 34                      //buzzer control pin
unsigned long int ShaftCountLeft = 0;  // to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; // to keep track of right position encoder
char Serial3Data;                      //Temp
int CurrTable;                         //Temp
unsigned char distance, ADC_read;      //Temp
int lanewidth = 35;                    //Lane Distance in Centimeters
int Degree = 80;                       //Turning Angle **True angle = degree+10**
unsigned char Sharp_Front =11;         //obstacle detection channel
unsigned char Sharp_Left = 9;          //obstacle detection channel
unsigned char Sharp_Right = 13;        //obstacle detection channel
int Range_Front =30;                   //obstacle detection trigger in cm
int Range_Left = 20;                   //obstacle detection trigger in cm
int Range_Right = 20;                  //obstacle detection trigger in cm

//*************************Declarations****************************************//

//********************Interrupt Service Routine********************************//
void left_position_encoder_interrupt_init() //Interrupt 4 enable
{
 cli();                // Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();                // Enables the global interrupt 
}
void right_position_encoder_interrupt_init() //Interrupt 5 enable
{
 cli();                // Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();                // Enables the global interrupt 
}
ISR(INT5_vect)         //ISR for right position encoder
  {        
    ShaftCountRight++; //increment right shaft position count
  }
ISR(INT4_vect)         //ISR for left position encoder
  {        
    ShaftCountLeft++;  //increment left shaft position count
  }
//********************Interrupt Service Routine********************************//
//*************************Motion control**************************************//
void motion_pin_config () 
{
 DDRA = DDRA | 0x0F;   // set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   // Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; // PL3 and PL4 pins are for velocity control using PWM.
}
//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config ()
{
 DDRE  = DDRE & 0xEF;  // Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; // Enable internal pullup for PORTE 4 pin
}
//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config ()
{
 DDRE  = DDRE & 0xDF;  // Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; // Enable internal pullup for PORTE 4 pin
}
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;     /// removing upper nibbel for the protection
 PortARestore = PORTA;    /// reading the PORTA original status
 PortARestore &= 0xF0;    /// making lower direction nibbel to 0
 PortARestore |= Direction; /// adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore;    /// executing the command
}
//*************************Motion control**************************************//
//************************Velocity control*************************************//
void timer5_init()
{
  TCCR5B = 0x00;  //Stop
  TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00;  //Output compare register high value for Left Motor
  OCR5AL = 0xFF;  //Output compare register low value for Left Motor
  OCR5BH = 0x00;  //Output compare register high value for Right Motor
  OCR5BL = 0xFF;  //Output compare register low value for Right Motor
  TCCR5A = 0xA9;  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
                    For Overriding normal port functionality to OCRnA outputs.
                    {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting
                    FAST PWM 8-bit Mode*/
  TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
void velocity (unsigned char left_motor, unsigned char right_motor) // min=0 and max= 255
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
}
//************************Velocity control*************************************//

//*************************Distance in cm**************************************//
void Move(unsigned int DistanceInCM, int direction) 
{
  velocity (200, 200);                            // Moving velocity
  float ReqdShaftCount = (DistanceInCM*10)/5.338; // Shaftcount =(Distance/resolution)
  unsigned long int ReqdShaftCountInt = (unsigned long int)ReqdShaftCount;//temp
  ShaftCountRight = 0;                            //Initialize
  ShaftCountLeft = 0;                             //Initialize
  while (ShaftCountLeft < ReqdShaftCountInt)
  {
    if (direction == 1) 
    {
      forward();                                  // Move forward
      obstacle_detector(Sharp_Left,Range_Left);   // Obstacle (left(9),10cm)
      obstacle_detector(Sharp_Front,Range_Front); // Obstacle (front(11),10cm)
      obstacle_detector(Sharp_Right,Range_Right); // Obstacle (right(13),10cm)
      Serial3.println("");                        // acknowledgement
    } 
    else
    {
      reverse();                                  // Move backward
      Serial3.println("");                        // acknowledgement
    }
    brake();
  }
  delay(100);
}
//*************************Distance in cm**************************************//

//************************Rotate in Degree*************************************//
void rotate(unsigned int Degrees, int direction) 
{
  velocity (180, 180);                      //Turning Velocity
  if (direction == 1) Degrees = Degree +2;
  float ReqdShaftCount = 0;                 //Initialize
  unsigned long int ReqdShaftCountInt = 0;
  ReqdShaftCount = (float) Degrees / 4.090; // Shaftcount =(Distance/resolution)
  ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
  ShaftCountRight = 0;
  ShaftCountLeft = 0;
  while ((ShaftCountLeft < ReqdShaftCountInt) || (ShaftCountRight < ReqdShaftCountInt)) {
    if(direction == 1) 
    {
      right();                              //Turn right
      Serial3.println("");                  // acknowledgement
      } 
    else
    {
      left();                               //Turn left
      Serial3.println("");                  // acknowledgement
      }
    brake();
  }
  delay(100);
}
//************************Rotate in Degree*************************************//
//***********************Differential Drive************************************//
void forward (void)           //both wheels forward
    {motion_set(0x06);}
void reverse (void)           //both wheels backward
    {motion_set(0x09);}
void left (void)              //Left wheel backward, Right wheel forward
    {motion_set(0x05);}
void right (void)             //Left wheel forward, Right wheel backward
    {motion_set(0x0A);}
void brake (void)
    {motion_set(0x00);}       //both wheels Stop
//***********************Differential Drive************************************//
//***********************Obstacle Detection************************************//
void adc_pin_config()       //ADC pin configuration
{
 DDRF = 0x00;               //set PORTF direction as input
 PORTF = 0x00;              //set PORTF pins floating
 DDRK = 0x00;               //set PORTK direction as input
 PORTK = 0x00;              //set PORTK pins floating
}
void adc_init()             //Function to Initialize ADC
{
  ADCSRA = 0x00;
  ADCSRB = 0x00;            //MUX5 = 0
  ADMUX = 0x20;             //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
  ACSR = 0x80;
  ADCSRA = 0x86;            //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}
unsigned char SharpSense(unsigned char Channel) 
{                           //Accepts Channel/Sensor number returns Distance
  /*ADC converter begin*/
  unsigned char ADC_Value;
  if(Channel>7)
    {
      ADCSRB = 0x08;
    }
  Channel = Channel & 0x07;       
  ADMUX= 0x20| Channel;        
  ADCSRA = ADCSRA | 0x40;   //Set start conversion bit
  while((ADCSRA&0x10)==0);  //Wait for ADC conversion to complete
  ADC_Value=ADCH;
  /*ADC converter end*/
  ADC_read = ADC_Value;     //temp
  /*Sharp sensor GP2D12 value estimation*/
  float distance = (float)(2799.6*(1.00/(pow(ADC_read,1.1546)))); //in cm
  unsigned int SharpDist = (int)distance; //temp
  if(SharpDist>80)
    {
      SharpDist=80;
    }
  /*Sharp sensor GP2D12 value estimation*/
  return SharpDist;
  ADCSRA = ADCSRA|0x10;     //clear ADIF (ADC Interrupt Flag) by writing 1 to it
  ADCSRB = 0x00;
}
void obstacle_detector(unsigned char Channel, int Range)
{                           //Accepts Channel/Sensor Distance and executes condition
  int value = SharpSense(Channel);
  if(value<Range)
  {
    brake();
    boot();                 // Acts as a alert
    delay(100);
  }
}
//***********************Obstacle Detection************************************//

//*************************Initialization**************************************//
void init_system()
{
 cli(); //Clears the global interrupt

 motion_pin_config(); //robot motion pins config
 timer5_init();       //robot veloctiy timer config
 
 left_encoder_pin_config(); //left encoder pin config
 left_position_encoder_interrupt_init();
 
 right_encoder_pin_config(); //right encoder pin config  
 right_position_encoder_interrupt_init();
 
 adc_pin_config(); 
 adc_init();
 
 sei();   // Enables the global interrupt 
}
void boot() //Buzzer theme
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
//*************************Initialization**************************************//

//***************************Main Code*****************************************//
void setup()
{
  Serial3.begin(9600);
  pinMode(buzzer, OUTPUT);
  boot(); //Booting indicator
  init_system();
}
void loop() 
{
  if(Serial3.available()>0)
  Serial3Data=Serial3.read();    
  //Manual Controls:
  if(Serial3Data=='v')    brake();
  if(Serial3Data=='w')    forward(); 
  if(Serial3Data=='s')    reverse();
  if(Serial3Data=='a')    left();
  if(Serial3Data=='d')    right();
  //test Controls:
  if(Serial3Data=='8')    Move(40,1);       //forward cm
  if(Serial3Data=='2')    Move(40,0);       //reverse cm
  if(Serial3Data=='4')    rotate(Degree,0); //left  90
  if(Serial3Data=='6')    rotate(Degree,1); //right 90
  //table Controls:
  if(Serial3Data=='t')    MoveToTable(1);
  if(Serial3Data=='y')    MoveToTable(2);
  if(Serial3Data=='u')    MoveToTable(3);
  if(Serial3Data=='i')    MoveToTable(4);
  if(Serial3Data=='o')    MoveToTable(5);
  if(Serial3Data=='p')    MoveToTable(6);
  if(Serial3Data=='k')    MoveToKitchen();
  if(Serial3Data=='r')    DDReset();
  if(Serial3Data=='z')    CurrTable = 0;
}
//***************************Main Code*****************************************//

/*****************************WaitronMains**************************************/

//***************************LaneSelect****************************************//
void LaneSelect(unsigned int direction)
{
  if(direction == 1)      //Right Fwd Left (even lane)
    {
      rotate(Degree,1);
      Move(lanewidth,1);
      rotate(Degree,0);
      } 
    else                  //Left Fwd Right (odd lane)
    {
      rotate(Degree,0);
      Move(lanewidth,1);
      rotate(Degree,1);
      }
}
//********************Differential Drive Reset*********************************//
void DDReset()
{
  Move(20,0); //Reverse
  delay(100);
  Move(10,1); //Forward
}

//**************************MoveToTable****************************************//
void MoveToTable(int tableNumber)//Destination table
{
  if(CurrTable == 0) MoveOriginToTable(tableNumber);
  else MoveBetweenTables(tableNumber); 
}
//*********************MoveFromKitchenToTable**********************************//
void MoveOriginToTable(int tableNumber)
{
  unsigned int initialDirection = (tableNumber % 2 == 0) ? 1 : 0;
  LaneSelect(initialDirection);
  delay(200);
  if(tableNumber == 1 || tableNumber == 2) Move(lanewidth,1);
  if(tableNumber == 3 || tableNumber == 4) Move(lanewidth*2,1);
  if(tableNumber == 5 || tableNumber == 6) Move(lanewidth*3,1);
  delay(50);
  // Determine rotation direction and degree
  unsigned int rotateDirection = (tableNumber % 2 == 0) ? 1 : 0;
  unsigned int rotateDegree = (initialDirection == rotateDirection) ? Degree : 270;
  // Rotate towards table
  rotate(rotateDegree, rotateDirection); // (degree, direction)
  delay(50);
  CurrTable = tableNumber;
}
//*************************MoveToKitchen***************************************//
void MoveToKitchen()
{
  // Determine initial direction
  unsigned int initialDirection = (CurrTable % 2 == 0) ? 1 : 0;
  unsigned int rotateDirection = (CurrTable % 2 == 0) ? 1 : 0;
  unsigned int rotateDegree = (initialDirection == rotateDirection) ? Degree : 270;
  delay(50);
  // Rotate towards lane
  rotate(rotateDegree, rotateDirection); // (degree, direction)
  delay(50);
  // move towards lane
  if(CurrTable == 1 || CurrTable == 2) Move(lanewidth,1);
  if(CurrTable == 3 || CurrTable == 4) Move(lanewidth*2,1);
  if(CurrTable == 5 || CurrTable == 6) Move(lanewidth*3,1);
  delay(50);
  // Rotate towards origin
  rotate(rotateDegree, rotateDirection); // (degree, direction)
  delay(50);
  // move towards origin
  Move(lanewidth,1);
  delay(50);
  //default 
  rotate(rotateDegree, rotateDirection); // (degree, direction)
  delay(50);
  //update current table to kitchen
  CurrTable = 0;
  delay(200);
  DDReset();
}

//*************************MoveBetweenTables***********************************//
void MoveBetweenTables(int tableNumber) //startTable and EndTable are jusst parameters
{ 
    int StartTable;
    int EndTable;
    
    if((StartTable == (CurrTable % 2 == 0) && EndTable == (tableNumber % 2 == 1)) ||
       (StartTable == (CurrTable % 2 == 1) && EndTable == (tableNumber % 2 == 0)))
      {
        MoveToKitchen();
        delay(50);
        MoveOriginToTable(tableNumber);
        delay(50);
        }
    else
      {
        MoveLaneTables(tableNumber);
        }
    //************Update the Current and Destination Table****************************
    CurrTable = tableNumber;
  }
  
//***********************MoveTablesOfSameLane**********************************//
void MoveLaneTables(int tableNumber) {
    int StartTable = CurrTable;
    int EndTable = tableNumber;
    int tablediff = abs(EndTable - StartTable);
    delay(50);
    if((StartTable % 2 == 1))
      {
        rotate(Degree, StartTable > EndTable ? 0 : 1);
        delay(50);
        Move(lanewidth * (tablediff == 4 ? 2 : 1), 1);
        delay(50);
        rotate(Degree, StartTable > EndTable ? 1 : 0);
        delay(50);
        }
    else
      {
        rotate(Degree, StartTable > EndTable ? 1 : 0);
        delay(50);
        Move(lanewidth * (tablediff == 4 ? 2 : 1), 1);
        delay(50);
        rotate(Degree, StartTable > EndTable ? 0: 1);
        delay(50);
        }
    // Update the current table number
    CurrTable = tableNumber;
  }