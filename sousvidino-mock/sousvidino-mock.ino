// Mock example sousvidino output via software serial on hc-05

#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Ready to go");
  mySerial.begin(38400);
}

unsigned long last_t = 0;
uint8_t power = 10;
float t0 = 50.0;
float t1 = -127.0;
float t2 = -127.0;
double output = 10.0;
double setpoint = 60.0;
double kp = 30;
double ki = 0.03;
double kd = 0.0;


void loop() {
  unsigned long t = millis() - last_t;
  last_t = millis();
  
  // serial out
  String out;
  out += "p=";
  out += power;
  out += ";t0=";
  out += t0;
  out += ";t1=";
  out += t1;
  out += ";t2=";
  out += t2;
  out += ";sp=";
  out += setpoint;
  out += ";kp=";
  out += kp;
  out += ";ki=";
  out += ki;
  out += ";kd=";
  out += kd;
  out += ";output=";
  out += output;
  out += ";loop_time=";
  out += t;
  out += ";time=";
  out += last_t;
  out += "\n";

  // fake changing temperature
  t0 += 0.25;
  if(t0 > 100.0) {
    t0 = -4.0;
  }

  // Write out on both arduino serial and software serial
  Serial.write(out.c_str());
  mySerial.write(out.c_str());
  
  delay(77); // approximate delay with one temperature sensor
}
