// Proxy to configure HC-05 via Arduino Terminal
// RX/TX from Arduino Terminal are forwarded to HC-05 on pin 2 and 3 (via 38400 baud)
// Connect HC-05 RX to pin 3
// Connect HC-05 TX to pin 2
// Press EN button on power connect
// Use CRNL line ending

#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Ready to go. Please type your AT commands.");
  mySerial.begin(38400); // For ATing
}

void loop() {
  // forward and vice versa
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}
