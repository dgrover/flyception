// TTL pulse to Arduino Uno pins 5,7
//set Port D pins 5,7 as OUTPUT

int count = 1;
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
  
  if (count >10)
  {
    if (incomingByte > 0)
    {
      PORTD = B10101000; //Set pins 3, 5, 7 to HIGH
      incomingByte = 0;
    }
    else
      PORTD = B10100000; //Set pins 5, 7 to HIGH
   
    count = 1;
  }
  else
    PORTD = B10000000; //Set pin 7 to HIGH
  
  delayMicroseconds(100);
    
  PORTD = B00000000; //Set pins LOW
  delayMicroseconds(100);
  
  count++;
}
