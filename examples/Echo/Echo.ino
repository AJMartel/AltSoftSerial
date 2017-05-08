#include <AltSoftSerial.h>
#include <SoftwareSerial.h>

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45
// Wiring-S           5         6          4
// Sanguino          13        14         12

//AltSoftSerial altSerial;
//SoftwareSerial altSerial(0, 1);

//#define altSerial Serial

void setup() {
  // Serial.begin(9600);
  // while (!Serial) ; // wait for Arduino Serial Monitor to open
  // Serial.println("AltSoftSerial Test Begin");

  AltSoftSerial5.begin(9600);
  AltSoftSerial5.println("Hello World");
}

uint8_t i = 0;

void loop() {
  char c;

  // if (Serial.available()) {
  //   c = Serial.read();
  //   altSerial.print(c);
  // }
  // if (altSerial.available()) {
  //   c = altSerial.read();
  //   Serial.print(c);
  // }

  if (AltSoftSerial5.available()) {
    c = AltSoftSerial5.read();
    AltSoftSerial5.print(c);
  }

//  altSerial.print(i++);
//  altSerial.print('\n');
}

