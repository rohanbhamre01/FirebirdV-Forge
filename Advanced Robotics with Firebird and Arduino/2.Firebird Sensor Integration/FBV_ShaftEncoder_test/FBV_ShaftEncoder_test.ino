/********************************************************************************
 Code to test the shaft encoder of Firebird Robot
 Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)
 
 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Microcontroller: atmega2560
 Frequency: 14745600
 Optimization: -O0 (For more information read section: Selecting proper optimization 
           options below figure 2.22 in the Software Manual)
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
unsigned char data;                     //to store received data from UDR1
unsigned long int ShaftCountLeft = 0;   //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0;  //to keep track of right position encoder
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
 
 sei();   // Enables the global interrupt 
}

//***************************Main Code*****************************************//
void setup()
{
  Serial2.begin(9600);
  init_system(); 
  velocity (255, 255);  //Velocity control

  /*test one at a time.*/
//  forward();
//  reverse();
//  left();
//  right();
//  brake();
}

void loop() 
{
  //old arduino version do not show current time
  unsigned long currentMillis = millis(); 
  Serial2.print("Time: ");
  Serial2.print(currentMillis);

  Serial2.print("   Left wheel : ");
  Serial2.print(ShaftCountLeft);
  Serial2.print("   Right wheel : ");
  Serial2.println(ShaftCountRight);
  delay(100);
}