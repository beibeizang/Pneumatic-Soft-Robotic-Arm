
#include <SoftwareSerial.h>
SoftwareSerial BlueTooth(8,9);//从机接pneumatic
char val; // Data received from the serial port
int pinRelay1 = 4; // Set the pin to digital I/O 4
int pinRelay2 = 5;
//SoftwareSerial mySerial(10,11);

void deflate() {
  Serial.println("deflate");
  digitalWrite(pinRelay1,LOW);
  digitalWrite(pinRelay2,LOW);
}
void hold() {
  Serial.println("hold");
  digitalWrite(pinRelay1,HIGH);
  digitalWrite(pinRelay2,LOW);
}

void inflate() {
  Serial.println("inflate");
  digitalWrite(pinRelay2,HIGH);
  digitalWrite(pinRelay1,HIGH);
}

 
 void setup() {
 pinMode(pinRelay1, OUTPUT); // Set pin as OUTPUT
 pinMode(pinRelay2,OUTPUT);
 Serial.begin(9600); // Start serial communication at 9600 bps
 BlueTooth.begin(9600);
 }
 
 void loop() {
  
// if(mySerial.available());
 //Serial.write(mySerial.read());
 while (BlueTooth.available()) { // If data is available to read,
 val = BlueTooth.read(); // read it and store it in val
 //mySerial.write(Serial.read());
 }
 if (val == 'I') { // If H was received
 //digitalWrite(ledPin, HIGH); // turn the LED on
 inflate();
 } 
 else if(val == 'D') {
 //digitalWrite(ledPin, HIGH); // Otherwise turn it OFF
 deflate();
 }
 else  {
 hold();
 //digitalWrite(ledPin, LOW); // turn the LED on
 }
 delay(100); // Wait 100 milliseconds for next reading
 }
 
 
