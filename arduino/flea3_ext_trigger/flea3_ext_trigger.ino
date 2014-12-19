int ledPin = 7;                 // connected to digital pin 7

void setup()
{
  pinMode(ledPin, OUTPUT);      // sets the digital pin as output
}

void loop()
{
  digitalWrite(ledPin, HIGH);   
  delayMicroseconds(60);                  
  digitalWrite(ledPin, LOW);
  delayMicroseconds(60);                  
}
