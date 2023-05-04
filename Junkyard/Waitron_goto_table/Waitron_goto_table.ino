/********************************************************************************
 Waitron can turn 90 using Bluetooth and analog write
 Written by: Rohan Bhamre and Team Technocrats,VESIT(2023)
 
 Note: Make sure that in the configuration options following settings are 
 done for proper operation of the code

 Microcontroller: atmega2560
 Frequency: 14745600
 Optimization: -O0 (For more information read section: Selecting proper optimization 
           options below figure 2.22 in the Software Manual)
*********************************************************************************/
#define Position_encoder_resolution 5.44;  //mm per pulse
//buzzer control pin
#define buzzer 34
// motor control pin
// Right motor is connected to digital pin 24 & 25
#define Rmotorfwd 24  // Initialize Pin 24 as rightmotorPin1
#define Rmotorrev 25 // Initialize Pin 25 as rightmotorPin2
#define RPWM 45     // Initialize Pin 45 as pwmR
// Left motor is connected to digital pin 22 & 23
#define Lmotorfwd 23  // Initialize Pin 23 as leftmotorPin1
#define Lmotorrev 22 // Initialize Pin 22 as leftmotorPin2
#define LPWM 46     // Initialize Pin 46 as pwmL
char Serial2Data;
unsigned char data; ///to store received data from UDR1
//Postion Encoders
unsigned long int ShaftCountLeft = 0; ///to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; ///to keep track of right position encoder
unsigned int Degrees; ///to accept angle in degrees for turning
unsigned long int LeftDistance = 0;
unsigned long int RightDistance = 0;

void setup()
{
  Serial2.begin(9600);
  pinMode(buzzer, OUTPUT);
  //Booting indicator
  digitalWrite(buzzer, HIGH); // turning 0n buzzer
  delay(1000); // delay of 1 sec.
  digitalWrite(buzzer, LOW); // turning off buzzer
  delay(1000);
  /* Configure direction control pins & PWM pins as output */
  pinMode(Rmotorfwd, OUTPUT);
  pinMode(Rmotorrev, OUTPUT);
  pinMode(Lmotorfwd, OUTPUT);
  pinMode(Lmotorrev, OUTPUT);
  analogWrite(RPWM, 255);
  analogWrite(LPWM, 255);
  attachInterrupt(0,Lencoder,RISING);
  attachInterrupt(1,Rencoder,RISING);
  Serial2.println("Enter table no. :"); 
}



//Postion Encoders
void Lencoder(void) ///Interrupt 4 enable
{
 ShaftCountLeft++;
 LeftDistance = ShaftCountLeft * Position_encoder_resolution;
}

void Rencoder(void) ///Interrupt 5 enable
{
 ShaftCountRight++;
 RightDistance = ShaftCountRight * Position_encoder_resolution;
}




//Distance function
void MoveDistance(int a)
{
  a = LeftDistance + a;
  while(LeftDistance <= a)
  {
    Serial2.print("Left Encoder count= ");
    Serial2.print(ShaftCountLeft);
    Serial2.print("Right Encoder count= ");
    Serial2.print(ShaftCountRight);
    forward(255);
  }
  brake();
  delay(50);
}


//Motion Functions
void forward(int a)
{
  analogWrite(RPWM,a);
  analogWrite(LPWM,a);
  left_Forward();
  right_Forward();
}
void reverse(int a)
{
 analogWrite(RPWM,a);
 analogWrite(LPWM,a);
 left_Reverse();
 right_Reverse();
 delay(400);
}
void left(int a)
{
  analogWrite(RPWM,a);
  analogWrite(LPWM,a);
  left_Reverse();
  right_Forward();
}
void right(int a)
{
  analogWrite(RPWM,a);
  analogWrite(LPWM,a);
  right_Reverse();
  left_Forward();
}
void brake()
{
 analogWrite(LPWM,0);
 analogWrite(RPWM,0);
 left_Stop();
 right_Stop();
}
void left_Forward()
{
 digitalWrite(Lmotorrev,LOW);
 digitalWrite(Lmotorfwd,HIGH);
}

void right_Forward()
{
 digitalWrite(Rmotorrev,LOW);
 digitalWrite(Rmotorfwd,HIGH);
}

void left_Stop()
{
 digitalWrite(Lmotorrev,LOW);
 digitalWrite(Lmotorfwd,LOW);
}

void right_Stop()
{
 digitalWrite(Rmotorrev,LOW);
 digitalWrite(Rmotorfwd,LOW);
}

void left_Reverse()
{
 digitalWrite(Lmotorrev,HIGH);
 digitalWrite(Lmotorfwd,LOW);
}

void right_Reverse()
{
 digitalWrite(Rmotorrev,HIGH);
 digitalWrite(Rmotorfwd,LOW);
}

void RotateLeft(int a){
  a = LeftDistance + a;
  while(LeftDistance <= a)
  {
    Serial2.print("Left Encoder count= ");
    Serial2.print(ShaftCountLeft);
    Serial2.print("Right Encoder count= ");
    Serial2.print(ShaftCountRight);
    left(255);
  }
  brake();
  delay(50);
}

void RotateRight(int a){
  a = LeftDistance + a;
  while(LeftDistance <= a)
  {
    Serial2.print("Left Encoder count= ");
    Serial2.print(ShaftCountLeft);
    Serial2.print("Right Encoder count= ");
    Serial2.print(ShaftCountRight);
    right(255);
  }
  brake();
  delay(50);
}

//Initial Motion for odd tables
void InitialMotionOdd(){
//  MoveDistance(1000);
//  RotateLeft(10);
//  MoveDistance(1000);
//  RotateRight(10);
}

//Initial Motion for Odd tables
void InitialMotionEven(){
//  MoveDistance(1000);
//  RotateRight(10);
//  MoveDistance(1000);
//  RotateLeft(10);
}

//Functions to Move to INPUT TABLE
void MoveToTableOne(){
  MoveDistance(400);
  RotateLeft(40);
  MoveDistance(400);
  RotateRight(40);
  MoveDistance(400);
}
void MoveToTableTwo(){
    MoveDistance(400);
  RotateRight(40);
  MoveDistance(400);
  RotateLeft(404);
  MoveDistance(400);
}
void MoveToTableThree(){
   MoveDistance(400);
  RotateLeft(40);
  MoveDistance(400);
  RotateRight(40);
  MoveDistance(800);
}
void MoveToTableFour(){
  MoveDistance(400);
  RotateRight(40);
  MoveDistance(400);
  RotateLeft(40);
  MoveDistance(800);

}
void MoveToTableFive(){
   MoveDistance(400);
  RotateLeft(40);
  MoveDistance(400);
  RotateRight(40);
  MoveDistance(1200);
}
void MoveToTableSix(){
   MoveDistance(400);
  RotateRight(40);
  MoveDistance(400);
  RotateLeft(40);
  MoveDistance(1200);
}

void loop() 
{
  Serial2.print("Left Encoder count = ");
  Serial2.print(ShaftCountLeft);
  Serial2.print("\tRight Encoder count = ");
  Serial2.print(ShaftCountLeft);
  //MoveDistance(20);
  delay(300);
  //For Table movement
// while(1)
//   {
//    if(Serial2.available()>0)
//    Serial2Data=Serial2.read();     
//    //Manual Controls:
//    if(Serial2Data=='a') 
//    MoveToTableOne(); 
//    if(Serial2Data=='b')
//    MoveToTableTwo();
//    if(Serial2Data=='c')
//    MoveToTableThree();
//    if(Serial2Data=='d')
//    MoveToTableFour();
//    if(Serial2Data=='e')
//    MoveToTableFive();
//    if(Serial2Data == 'l')
//    RotateLeft(35);
//   }

//Manual Control
    while(1)
   {
    if(Serial2.available()>0)
    Serial2Data=Serial2.read();     
    //Manual Controls:
    if(Serial2Data=='w') 
    forward(255); 
    if(Serial2Data=='s')
    reverse(255);
    if(Serial2Data=='v')
    brake();
    if(Serial2Data=='a')
    RotateLeft(40);
    if(Serial2Data=='d')
    RotateRight(40);
   }
 brake();

   
}
