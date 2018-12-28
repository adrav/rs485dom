
#include <SoftwareSerial.h>
#include "rs485dom.h"

//#include <SPI.h>
//#include <SD.h>

#define PinRS485RX        10  // RS485 Serial Receive pin
#define PinRS485TX        11  // RS485 Serial Transmit pin
#define PinRS485TXControl 3   // RS485 Direction control

#define Pin13LED         13

SoftwareSerial RS485Serial(PinRS485RX, PinRS485TX); // RX, TX

#define DEBUG

RS485Dom *rs485DOM;

void setup()
{
  Serial.begin(9600);
  
  pinMode(Pin13LED, OUTPUT);
  pinMode(PinRS485TXControl, OUTPUT);    
  
  RS485Serial.begin(9600);   // set the data rate 

  rs485DOM = new RS485Dom('G', &RS485Serial, Pin13LED, Pin13LED, PinRS485TXControl);

  rs485DOM->sendStartupMesssage();

  Serial.println(F("G Started"));

}

void loop() 
{

  uint8_t res = rs485DOM->loop();

  if ((res != LINE_STILL_READING) && (res != LINE_EMPTY)) {
    Serial.print(F("#G: "));
    Serial.println(res);
  }


}


