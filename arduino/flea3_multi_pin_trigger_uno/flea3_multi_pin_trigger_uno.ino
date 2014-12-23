// TTL pulse to Arduino Uno pins 5,7
//set Port D pins 5,7 as OUTPUT
int count = 1;
void setup()
{
  DDRD = B10100000;
}

void loop()
{
  if (count >10)
  {
    PORTD = B10100000; //Set pins HIGH
    count = 1;
  }
  else
    PORTD = B10000000; //Set pins HIGH
  
  delayMicroseconds(1000);
    
  PORTD = B00000000; //Set pins LOW
  delayMicroseconds(1000);
  
  count++;
}
