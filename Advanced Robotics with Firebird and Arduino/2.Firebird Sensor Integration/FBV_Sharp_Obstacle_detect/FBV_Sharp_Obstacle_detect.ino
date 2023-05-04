/********************************************************************************
  In this experiment ADC captures the Sharp sensor values and displays it on serial monitor
  
  Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)

 ADC Connection:
         ACD CH. PORT  Sensor
          1    ADC9    Sharp IR range sensor 1 left
          2    ADC10   Sharp IR range sensor 2
          3    ADC11   Sharp IR range sensor 3 front
          4    ADC12   Sharp IR range sensor 4
          5    ADC13   Sharp IR range sensor 5 right
       
 ***** For using Sharp (1, 2, 3 and 4) sensors short the jumper J2. 
     To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
     
 Note: 
  >Microcontroller: atmega2560
  >Frequency: 14745600s
  >Optimization: -O0 (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)
  >Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> 

//*************************Declarations****************************************//
unsigned char distance, ADC_Value, ADC_read;
unsigned char Sharp_Left = 9;                   //obstacle detection channel
unsigned char Sharp_front_left =10;             //obstacle detection channel
unsigned char Sharp_Front =11;                  //obstacle detection channel
unsigned char Sharp_front_right =12;            //obstacle detection channel
unsigned char Sharp_Right = 13;                 //obstacle detection channel
int Range_Left = 20;                            //obstacle detection trigger in cm
int Range_front_left = 20;                      //obstacle detection trigger in cm
int Range_Front =30;                            //obstacle detection trigger in cm
int Range_front_right = 20;                     //obstacle detection trigger in cm
int Range_Right = 20;                           //obstacle detection trigger in cm

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
    Serial.println("Obstacle Detected");
    delay(100);
  }
}
//*************************Initialization**************************************//
void init_system (void)
{
 cli(); //Clears the global interrupt
 adc_pin_config();
 adc_init();
 sei(); //Enables the global interrupts
}

void setup()
{
  Serial2.begin(9600);
  init_system();
}


//***************************Main Code*****************************************//
void loop()
{
  //Sharp sensors
  obstacle_detector(Sharp_Left,Range_Left);   // Obstacle (left(9),10cm)
  Serial2.print("  ");                        //Tab
  obstacle_detector(Sharp_front_left,Range_front_left); // Obstacle (front_left(10),10cm)
  Serial2.print("  ");                        //Tab
  obstacle_detector(Sharp_Front,Range_Front); // Obstacle (front(11),10cm)
  Serial2.print("  ");                        //Tab
  obstacle_detector(Sharp_front_right,Range_front_right); // Obstacle (front_right(12),10cm)
  Serial2.print("  ");                        //Tab
  obstacle_detector(Sharp_Right,Range_Right); // Obstacle (right(10),10cm)
  Serial2.println("");                        //new line
}
