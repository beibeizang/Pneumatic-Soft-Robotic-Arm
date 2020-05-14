#include<SoftwareSerial.h>
SoftwareSerial BT(8,9);//主机
char val;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
BT.begin(9600);
}

void loop() {
 while (Serial.available()) { // If data is available to read,
 val = Serial.read(); // read it and store it in val
 }
 Serial.print(val);
 BT.print(val);
 delay(100); // Wait 100 milliseconds for next reading
 }
