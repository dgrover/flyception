int incomingByte = 0;   // for incoming serial data
const int led0 = 5;  //port 1
const int led1 = 4;  //port 2
//const int led1 = 3;  //port 3
//const int led1 = 2;  //port 4

int qt = 2000;

void setup() {
        Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
        pinMode(led0,OUTPUT);
        pinMode(led1,OUTPUT);
        digitalWrite(led0,LOW);
        digitalWrite(led1,HIGH);
}
void loop() 
{
        // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                
                if( incomingByte > 48)
                {

                  digitalWrite(led0,LOW);
                }
                
                if( incomingByte > 49)
                {

                  digitalWrite(led0,HIGH);
                  digitalWrite(led1,LOW);
                  delay(qt);
                  digitalWrite(led1,HIGH);
                  digitalWrite(led0,LOW);
                }
                
        }
}
