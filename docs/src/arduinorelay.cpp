#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial = SoftwareSerial(10, 11);; // RX, TX
const uint8_t direction_pin = 2;

void setTxEnable()
{
  digitalWrite(direction_pin, HIGH);
}
void setTxDisable()
{
  digitalWrite(direction_pin, LOW);
}

void setup() {
  pinMode(10,INPUT);
  pinMode(11,OUTPUT);
  pinMode(direction_pin, OUTPUT);
  mySerial.begin(57600);// this connects to the other USART device
  Serial.begin(57600);  // this one connects to a computer via USB
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
} 

void loop() { 
  if (mySerial.available() > 0) {
    // relay everything received from mySerial to Serial    
    Serial.write(mySerial.read()); 
  }
  if (Serial.available() > 0) {
    // relay everything received from Serial to mySerial
    setTxEnable();
    mySerial.write(Serial.read());
    setTxDisable();       
  }  
} 