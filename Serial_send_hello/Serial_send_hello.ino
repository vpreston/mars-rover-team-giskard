//***** Serial_send_hello************

//This routine tests the serial communications by sending "Hello World\n" every second

int led = 0;  // Pin 0 maps to the USER LED on the ArbotiX Robocontroller.

//*******************************************************************************
void setup()
{
  // start serial port at 38400 bps:
  Serial.begin(38400);
  
  // wait to connect
  delay(1000);
  
  pinMode(led, OUTPUT);  // initialize the digital pin as an output.
  
}

//******************************************************************************
void loop()
{ 
  
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level) 

  // Send the string Hello world
  Serial.print("Hello World\n");
     
  delay(100);
  digitalWrite(led, LOW);

  delay(900);
  
}


