//set Port D pins 3,5,7 as OUTPUT

int fps_factor = 20;
int count = 0;
int incomingByte = 0;	              // for incoming serial data

void setup()
{
  Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
  DDRD = B10101000;
}

void loop()
{
  if (Serial.available() > 0)
    incomingByte = Serial.read();
  
  if (count == fps_factor/2)
  {
    PORTD = B10000000; //Set pins 5 to LOW, 7 to HIGH
  }
  else if (count == fps_factor)
  {
    if (incomingByte > 0)
    {
      PORTD = B10101000; //Set pins 3, 5, 7 to HIGH
      incomingByte = 0;
    }
    else
      PORTD = B10100000; //Set pins 5, 7 to HIGH
   
    count = 0;
  }
  else
  {
    PORTD = (1<<PD7); //Set pin 7 to HIGH
  }
  
  delayMicroseconds(500);
    
  PORTD = (0<<PD7) && (0<<PD3); //Set pins 3 and 7 to LOW
  delayMicroseconds(500);  
  
  count++;
}
