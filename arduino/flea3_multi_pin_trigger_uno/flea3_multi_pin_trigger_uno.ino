// TTL pulse to Arduino Uno pins 3,5,7
//set Port D pins 3,5,7 as OUTPUT
void setup()
{
  DDRD = B10100000;
}

void loop()
{
  PORTD = B10100000; //Set pins HIGH
  delayMicroseconds(60);
  PORTD = B00000000; //Set pins LOW
  delayMicroseconds(60);
}
