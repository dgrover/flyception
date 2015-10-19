//set Port D pins 5,6,7 as OUTPUT

int fps_factor = 22;
int count = 0;
bool flash_fired = false;
int flash_count = 0;

int incomingByte = 0;	              // for incoming serial data

void setup()
{
  Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
  DDRD = B11100000;
  PORTD = B01000000;   //init P5-low (imaging camera), P6-high (flash), P7-low (tracking camera)
  
}

void loop()
{
  if (Serial.available() > 0)
    incomingByte = Serial.read();  //if incoming flash trigger from keyboard
  
  if (count == fps_factor/2)
  {
    PORTD = B11000000; //Set pins 5 to LOW, 7 to HIGH, pin 6 is high and Flash is Armed
  }
  else if (count == fps_factor)
  {
    if (incomingByte > 0)  //if there was a trigger for flash
    {
      PORTD = B10100000; //Set pins 5, 7 to HIGH and fire Flash by setting pin 6 to LOW
      incomingByte = 0;
      flash_fired = true;
      flash_count = 0;
    }
    else
      PORTD = B11100000; //Set pins 5, 7 to HIGH
   
    count = 0;  //reset count
    
    if (flash_fired)  //start counting for automatic flash fire 3000 imaging frames later
      flash_count++;
      
    if (flash_count == 3000)
    {
      PORTD = B10100000; //Set pins 5, 7 to HIGH and fire Flash by setting pin 6 to LOW
      flash_fired = false;
      flash_count = 0;
    }
  }
  else
  {
    PORTD = PORTD | B10000000;  //only set pin 7 to HIGH
  }
  
  delayMicroseconds(100);
    
  PORTD = PORTD & B01111111;  //only toggle pin 7, all other pins manipulated above
  
  delayMicroseconds(800);  

  count++;
}
