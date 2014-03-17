//*****MARS ROVER************

/*This is hindbrain code for ENGR3392 Robotics II Mars Rover Software Phase Project
*/  
//***************************************************************************

#include <Servo.h>  //for RC servos
#include <ax12.h>  //for Dynamixel servos

//****************************GLOBALS*****************************************
boolean ledON;               //used to toggle the led
unsigned long newtime;       //the time at the start of loop()
unsigned long timeperiod;    //time for loop() in milliseconds.
int estop_val;               //etop_val == LOW is stopped

//create RC servo objects
Servo starboard_motor;
Servo port_motor;
Servo right_rudder;
Servo left_rudder;

int sm_pos = 1500;    //starboard motor position 0 to 2500
int pm_pos = 1500;    //port motor position
int rr_pos = 1500;    //right rudder position
int lr_pos = 1500;    //left rudder position
int cp_pos = 1500;    //camera pan position
int h_code = 1;      //Status code from PC 1 == OK, 100 == ESTOP
int ESTOP = 100;

float bat_volt = 8.00;  //Lipo battery voltage
float ir_volt_1 = 1.00;  //IR distance voltage
float ir_volt_2 = 1.00;
//float so_volt; //Sonar distance voltage
float color_power;
float red_volt;
float green_volt;
float blue_volt;
boolean bump_mode_1;
boolean bump_mode_2;
//float encoder_value;

const int buffer_size = 40;
char readbuffer[buffer_size];
//******************MAPPING OF CONTROLLER BOARD PINS***************************
//
//*******************MAPPING OF DIO PINS *************************************
int led_pin = 0;   //USER LED on the ArbotiX Robocontroller, the blinking light
int ESTOP_pin = 23;     //manual switch on the model...stops all servos
int blink_pin = 1;    //running LED
int color_pin = 4;  //powering the color sensor
int bump_pin_1 = 2; //bump pin
int bump_pin_2 = 3; //bump pin
//int encoder_pin = 23;
//******************MAPPING OF ANALOG INPUT PINS******************************
//int bat_pin = 1;   //Lipo battery voltage
int ir_pin_1 = 0;    //Sharp IR distance device
int ir_pin_2 = 1;
//int so_pin = 3;  //XL MaxSonar device
int red_pin = 2;
int green_pin = 3;
int blue_pin = 4;
//******************MAPPING OF RC SERVO DEVICES*****************************
int sm_pin = 20;  //starboard motor speed
int pm_pin =21;  //port motor speed
int rr_pin = 18;  //right rudder position
int lr_pin = 19;  //left rudder position
//********************MAPPING OF DYNAMIXEL SERVO DEVICES**********************
int cp_num = 1;  //the camera panning servo number
//****************************************************************************

//***********************THE SETUP LOOP**************************************
void setup()
{
  // start serial port at 38400 bps:
  Serial.begin(38400);
  
  // wait to connect
  delay(1000);

  //setup for the Dynamixel serovs
  SetPosition(1,0); //set the position of servo # 1 to '0'

  //setup up the RC servos 
  starboard_motor.attach(sm_pin);
  starboard_motor.writeMicroseconds(sm_pos);
  port_motor.attach(pm_pin);
  port_motor.writeMicroseconds(pm_pos);
  right_rudder.attach(rr_pin);
  right_rudder.writeMicroseconds(rr_pos);
  left_rudder.attach(lr_pin);
  left_rudder.writeMicroseconds(lr_pos);  
  
  //setup input pins
  pinMode(ESTOP_pin, INPUT);           // set pin to input
  digitalWrite(ESTOP_pin, HIGH);       // turn on pullup resistors, 20k resitor to 5VDC on input pin
  pinMode(bump_pin_1, INPUT);
  digitalWrite(bump_pin_1, HIGH);
  pinMode(bump_pin_2, INPUT);
  digitalWrite(bump_pin_2, HIGH);
  
  //setup output pins
  pinMode(led_pin, OUTPUT);  // initialize the digital pin as an output for onboard LED
  pinMode(color_pin, OUTPUT); // initialize color sensors
  pinMode(blink_pin, OUTPUT);  //output for remote blinking LED
  digitalWrite(color_pin, HIGH);
  //pinMode(encoder_pin, INPUT);
  
  
  
  //setup the timing and blinky led
  ledON = false;        //used to toggle the board led with loop timer
  timeperiod = 100;    //loop time in milliseconds
  
}// end setup
//*************************THE MAIN PROGRAM LOOP******************************
void loop()
{
  
  //use the computer clock to create a timed loop, read the present time to start the timer
  newtime = millis();
  
  //read the value of the ESTOP switch, value is LOW when button is pressed, otherwise HIGH
  estop_val = digitalRead(ESTOP_pin);

  //Read the sensor values
  Read_sensors();
  
  //Read the data bytes from the PC
  int numread = 0;
  numread = Read_Command_String();
  
  //send a response in bytes to the PC if one has been read
  if (numread > 0)
  {
    Send_Return_String(numread);
  }
  
  //Parse the Command_String
  Parse_command_string();
  
  //Run the arbiter
  
  //Move the motors
  Move_the_motors();
  
  //Move the rudders
  Move_the_rudders();
  
  //Move the camera
  Move_the_camera();  
  
  
  //use computer clock to time the loop
   while(millis() < (newtime + timeperiod) )
  {
    //wait until time elaspes
  }
  
  //change the state of the on board LED
  Toggle_led();
  
}// end void
//****************************************************

//*************SUBROUTINES******************************
int Read_Command_String(void)  //returns number of bytes found
{
  //read the string as an array of type char
  int num = 0;  //number of bytes read
  if ( Serial.available() > 0)
  {
    //read until the '\n' is found
    num = Serial.readBytesUntil('\n', readbuffer, buffer_size);
  } 
  
  return num;
}//end Read_Command_String
//****************************************
void Send_Return_String(int numread)
{
  //send the response as a byte array
  byte returnbuffer[3];
    
  //if something has been sent, send a return
  if (numread > 0)
  {
    returnbuffer[0] = 0x24;  //$
    returnbuffer[1] = 0x52;  //R
    returnbuffer[2] = 0x49;  //I
    
    Serial.write(returnbuffer, 3);  //return the bytes read
    Serial.print('i');
    Serial.print(ir_volt_1, 3);  //sent the IR distance sensor
    Serial.print('j');
    Serial.print(ir_volt_2, 3);  //sent the IR distance sensor
    //Serial.print('s');
    //Serial.print(so_volt,3); //sent the Sonar distance Sensor
    Serial.print('r');
    Serial.print(red_volt,3);
    Serial.print('g');
    Serial.print(green_volt,3);
    Serial.print('b');
    Serial.print(blue_volt,3);
    Serial.print('t');
    Serial.print(bump_mode_1,3);
    Serial.print('u');
    Serial.print(bump_mode_2,3);
    //Serial.print('e');
    //Serial.print(encoder_value,3);
    Serial.write(0x0A);  //a newline, \n
  }
  
  return;
}// end Send_Return_String
//******************************************
void Toggle_led(void)
{
  //toggle the led on the board
  if (ledON == false)
  {
    ledON = true;
    digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(blink_pin, HIGH);  //turn external LED on
  }  
  else
  {
    ledON = false;
    digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(blink_pin, LOW);  //turn external LED off
  }
}//end Toggle_led
//*******************************************************
void Move_the_motors(void)
{
  //move the motors which respond to RC servo microsecond values
  //the motor controller is assumed to be a Marine 15
  
  //check ESTOP, value is low when button is pressed
  //and the passed stop value h_code
  if (estop_val == LOW || h_code == ESTOP)
  {
    //stop the motors
    starboard_motor.writeMicroseconds(1470);  //tuned to motor??????????
    port_motor.writeMicroseconds(1500);
  }
  else
  {
    //move the motors to the passed positions
    starboard_motor.writeMicroseconds(sm_pos);
    port_motor.writeMicroseconds(pm_pos);
  }
}//end Move_the_motors
//******************************************************
void Read_sensors(void)
{
  //This assumes the 5.0 VDC voltage reference is being used on the Arbotix-m board, the default
  
  //read the Sharp IR distance sensor
  int ir_value_1;
  ir_value_1 = analogRead(ir_pin_1);
  
  int ir_value_2;
  ir_value_2 = analogRead(ir_pin_2);
  
  
  //read the Sonar distance sensor
  //so_volt = analogRead(so_pin)/2;
  //convert to volts
  ir_volt_1 = (float)ir_value_1 * 5.0 / 1024.0;
  ir_volt_2 = (float) ir_value_2 * 5.0 / 1024.0;
  
  //color sensor reading
  red_volt = analogRead(red_pin);
  green_volt = analogRead(green_pin);
  blue_volt = analogRead(blue_pin);
  
  bump_mode_1 = digitalRead(bump_pin_1);
  bump_mode_2 = digitalRead(bump_pin_2);
  
  //encoder_value = digitalRead(encoder_pin);


}//end Read_sensors
//********************************************************
void Parse_command_string(void)
{
  //the following converts the 4 char number values into integers
  //the readbuffer is assumed to be $CS----P----R----L----C----H----'\n' 34 bytes
  char dummy[4];    
  
  //for starboard motor
  dummy[0] = readbuffer[3];
  dummy[1] = readbuffer[4];
  dummy[2] = readbuffer[5];
  dummy[3] = readbuffer[6];
  
  sm_pos = atoi(dummy);    //convert char/byte array into interger
  
  //for port motor    
  dummy[0] = readbuffer[8];
  dummy[1] = readbuffer[9];
  dummy[2] = readbuffer[10];
  dummy[3] = readbuffer[11];
  
  pm_pos = atoi(dummy);
  
  //for right rudder
  dummy[0] = readbuffer[13];
  dummy[1] = readbuffer[14];
  dummy[2] = readbuffer[15];
  dummy[3] = readbuffer[16];
  
  rr_pos = atoi(dummy);
  
  //for left rudder
  dummy[0] = readbuffer[18];
  dummy[1] = readbuffer[19];
  dummy[2] = readbuffer[20];
  dummy[3] = readbuffer[21];
  
  lr_pos = atoi(dummy);
  
  //for camera pan
  dummy[0] = readbuffer[23];
  dummy[1] = readbuffer[24];
  dummy[2] = readbuffer[25];
  dummy[3] = readbuffer[26];
  
  cp_pos = atoi(dummy);
  
  //for status
  dummy[0] = readbuffer[28];
  dummy[1] = readbuffer[29];
  dummy[2] = readbuffer[30];
  dummy[3] = readbuffer[31];
  
  h_code = atoi(dummy);

}//end Parse_commmand_string
//*********************************************************
void Move_the_camera(void)
{
  //the camera is panned using a Dynamixel servo motor AX-12
  
  //below are the values of m and b in y = m*x + b 
  //this moves the range 500 to 2500 into the range 0 to 1023 required by the Dynamixel servo
  float m = 0.51;
  float b = -255.0;
  float cp_value;
  
  //calculate the camera position of 0 to 1023;
  cp_value = m * (float)cp_pos + b;
  
  //check ESTOP, value is low when button is pressed
  //and the passed stop value h_code
  if (estop_val == LOW || h_code == ESTOP)
  {
    //leave the camera alone
    
  }
  else
  {
    //move the camera to the passed positions
    SetPosition(1, (int)cp_value);
  }
}//end Move_the_camera
//*******************************************************
void Move_the_rudders(void)
{
  //the rudders are assumed to be operated by RC servos
  
  //check ESTOP, value is low when button is pressed
  //and the passed stop value h_code
  if (estop_val == LOW || h_code == ESTOP)
  {
    //leave the camera alone
    
  }
  else
  {
    //move the rudders
    right_rudder.writeMicroseconds(rr_pos);
    left_rudder.writeMicroseconds(lr_pos);
  }
  
}//end Move_the_rudders
//************* end of program****************************

