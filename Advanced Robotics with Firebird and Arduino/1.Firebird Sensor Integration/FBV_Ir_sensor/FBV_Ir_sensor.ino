/********************************************************************************
  In this experiment ADC captures the IR sensor values and displays it on Serial monitor
  
  Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)

 ADC Connection:
 			  ACD CH.	PORT	  Sensor
          1			ADC4		IR Proximity analog sensor 1*****
          2			ADC5		IR Proximity analog sensor 2*****
          3			ADC6		IR Proximity analog sensor 3*****
          4			ADC7		IR Proximity analog sensor 4*****
          5			ADC8		IR Proximity analog sensor 5*****

       
 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
     
 Note: 
  >Microcontroller: atmega2560
  >Frequency: 14745600s
  >Optimization: -O0 (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)
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

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char distance, adc_reading;

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

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Channel)
{
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
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return ADC_Value;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void printIR(unsigned int channel) 
{
	int ADC_print = ADC_Conversion(channel);
  Serial2.print("Channel(");
  Serial2.print(channel);
  Serial2.print(") = ");
  Serial2.print(ADC_print);
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
  //IR sensors
  printIR(4);							//Prints value of Analog IR Proximity Sensor 1 (Left)
  Serial2.print("  ");    //Tab
  printIR(5);							//Prints value of Analog IR Proximity Sensor 2
  Serial2.print("  ");    //Tab
  printIR(6);							//Prints value of Analog IR Proximity Sensor 3 (Front)
  Serial2.print("  ");    //Tab
  printIR(7);							//Prints value of Analog IR Proximity Sensor 4
  Serial2.print("  ");    //Tab
  printIR(8);							//Prints value of Analog IR Proximity Sensor 5 (Right)
  Serial2.println("");    //new line
}
