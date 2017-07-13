// Sousvidino. Low-cost Sous vide controller using a one-wire temperature sensor and a solid state relay for heater power switching
//
// Author: Markus Roth <mail@rothmark.us>
// 
// setpoint:
//   burger: 55: rare, 60: medium, 65: medium well    
//
// PID parameters (dependent on water volume, heater power)
// 1.5l water boiler 2kw:
//   kp = 1.0;
//   ki = 0.03;
//   kd = 0.0; 
// 6l water, immersion heater 1kW:
//   kp = 30.0;
//   ki = 0.03;
//   kd = 0.0;

#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#define SOUSVIDINO_EEPROM_VERSION 43

#define MAX_POWER 50 // has to be smaller than 65535 cycles / 1250 prescaled timer cycles per ac period

#define PIN_BT_TX        2
#define PIN_BT_RX        3
#define PIN_BT_GND       4
#define PIN_BT_VCC       5
#define PIN_BT_ENABLE    6
#define PIN_ONEWIRE_TEMP 7  // yellow
#define PIN_ONEWIRE_VCC  8  // red
#define PIN_ONEWIRE_GND  9  // black
#define PIN_SSR          13 // solid state relay

OneWire one_wire(PIN_ONEWIRE_TEMP);
DallasTemperature sensors(&one_wire);

// bluetooth serial
SoftwareSerial bluetoothSerial(PIN_BT_RX, PIN_BT_TX); // RX, TX
uint8_t bluetooth_enable = LOW;

// PID
double input, output, setpoint;
double kp, ki, kd;
// setpoint, kp, ki, kd initialized in setup();

uint8_t n_sensors = 0;
static DeviceAddress temp_sensor0 = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 
static DeviceAddress temp_sensor1 = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; 

void writeEEPROM() {
  size_t addr = 0;
  // store version, sp, p, i, d in first addresses
  uint8_t version = SOUSVIDINO_EEPROM_VERSION;
  EEPROM.put(addr, version);
  addr += sizeof(version);
  EEPROM.put(addr, setpoint);
  addr += sizeof(setpoint);
  EEPROM.put(addr, kp);
  addr += sizeof(kp);
  EEPROM.put(addr, ki);
  addr += sizeof(ki);
  EEPROM.put(addr, kd);
  addr += sizeof(kd);
  for(size_t i=0; i<8; ++i) {
    EEPROM.put(addr, temp_sensor0[i]);
    addr += sizeof(temp_sensor0[i]);
  }
  for(size_t i=0; i<8; ++i) {
    EEPROM.put(addr, temp_sensor1[i]);
    addr += sizeof(temp_sensor1[i]);
  }
}

void readEEPROM() {
  size_t addr = 0;
  // Check version first. Important for first-time flashing
  uint8_t version;
  EEPROM.get(addr, version);
  if(version != SOUSVIDINO_EEPROM_VERSION) {
    // set defaults
    setpoint = 58.0;
    kp = 30.0;
    ki = 0.03;
    kd = 0.0;
    memset(temp_sensor0, 0, 8);
    memset(temp_sensor1, 0, 8);
  } else {
    addr += sizeof(version);
    // read sp, p, i, d from first addresses
    EEPROM.get(addr, setpoint);
    addr += sizeof(setpoint);
    EEPROM.get(addr, kp);
    addr += sizeof(kp);
    EEPROM.get(addr, ki);
    addr += sizeof(ki);
    EEPROM.get(addr, kd);
    addr += sizeof(kd);
    for(size_t i=0; i<8; ++i) {
      EEPROM.get(addr, temp_sensor0[i]);
      addr += sizeof(temp_sensor0[i]);
    }
    for(size_t i=0; i<8; ++i) {
      EEPROM.get(addr, temp_sensor1[i]);
      addr += sizeof(temp_sensor1[i]);
    }
  }
}

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// output
uint8_t ssr_state = LOW;
uint8_t power = 0; // 0: off, MAX_POWER: full

// serial in buffer
String serial_buffer_usb = "";
String serial_buffer_bluetooth = "";
// last complete command (bluetooth and USB) (tailing \n)
String serial_command_usb = "";
String serial_command_bluetooth = "";

void flip_ssr_state() {
  ssr_state = !ssr_state;
  digitalWrite(PIN_SSR, ssr_state);
}

void set_power(uint8_t p) {
  // clamp at 0..MAX_POWER
  power = min(MAX_POWER, p);

  switch(power) {
    case 0:
      TCCR1B = 0x00; // disable timer
      TCNT1 = 0;
      ssr_state = LOW;
      digitalWrite(PIN_SSR, ssr_state);
    break;

    case MAX_POWER:
      TCCR1B = 0x00; // disable timer
      TCNT1 = 0;
      ssr_state = HIGH;
      digitalWrite(PIN_SSR, ssr_state);
    break;
    
    default:
      // enable timer
      TCCR1B = 0x04;
    break;
  }
}

void setup() {

  // disable global interrupts
  noInterrupts(); // cli()

  // set up Timer1
  TIMSK1 = 0x02;  // enable comparator A interrupt
  TIMSK1 |= 0x01; // enable overflow interrupt
  TCCR1A = 0x00;  // no pwm, etc
  TCCR1B = 0x04;  // 256 prescaler. 1 50Hz period == 20ms == 1250 cycles 
  OCR1A = 1250;   // initialize the comparator. 1 period == 20ms == 1250 cycles
  TCNT1 = 0;      // timer variable

  // solid state relay
  ssr_state = LOW;
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, ssr_state);

  // convenience for temperature sensors
  pinMode(PIN_ONEWIRE_VCC, OUTPUT);
  pinMode(PIN_ONEWIRE_GND, OUTPUT);
  digitalWrite(PIN_ONEWIRE_VCC, HIGH);
  digitalWrite(PIN_ONEWIRE_GND, LOW);


  // power
  set_power(0);
    
  // serial
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // bluetooth serial
  bluetoothSerial.begin(38400);
  pinMode(PIN_BT_ENABLE, OUTPUT);
  digitalWrite(PIN_BT_ENABLE, LOW);
  pinMode(PIN_BT_GND, OUTPUT); // convenience, not for arduino nano
  digitalWrite(PIN_BT_GND, LOW);
  pinMode(PIN_BT_VCC, OUTPUT); // convenience, not for arduino nano
  digitalWrite(PIN_BT_VCC, HIGH);
  
  // sensors
  sensors.begin();
  
  // set the resolution to 12 bit
  sensors.setResolution(12);

  // PID
  readEEPROM();
  pid.SetTunings(kp, ki, kd);
  pid.SetSampleTime(5000); // ms. a little longer than normal loop time
  // restrict max power
  pid.SetOutputLimits(0, MAX_POWER);
  pid.SetMode(AUTOMATIC);


  // Onewire discovery, if none saved
//  if(temp_sensor0[0] == 0) 
  {
    DeviceAddress addr;
    if(one_wire.search(temp_sensor0)) {
      ++n_sensors;
  //    if(OneWire::crc8(temp_sensor0, 7) != addr[7]) { }
    }
    if(one_wire.search(temp_sensor1)) {
  //    if(OneWire::crc8(temp_sensor1, 7) != addr[7]) { }
      ++n_sensors;
    }
    one_wire.reset_search();
    writeEEPROM();
  }
  
  // enable global interrupts
  interrupts(); // sei
}

// timer1 comparator A match
ISR(TIMER1_COMPA_vect) {
  TCCR1B = 0x00; // disable timer
  TCNT1 = 0;

  // assume that 0 < power < MAX_POWER
  // with both power == 0, and power == MAX_POWER, timer is disabled 
  
  flip_ssr_state();

  if(ssr_state) {
    OCR1A = 1250 * power;
  } else {
    OCR1A = 1250 * (MAX_POWER - power);
  }

  // enable timer
  TCCR1B = 0x04;
}

unsigned long last_t = 0;


// whole loop around 77ms
void loop() {
  
  unsigned long t = millis() - last_t;
  last_t = millis();
  
  // get temperatures
//  sensors.requestTemperatures(); // 766ms!
  sensors.requestTemperaturesByAddress(temp_sensor0); // 46ms
  sensors.requestTemperaturesByAddress(temp_sensor1); // 46ms
  float t0 = sensors.getTempC(temp_sensor0); // 13ms 
  float t1 = sensors.getTempC(temp_sensor1); // 13ms

  // Accumulate bluetooth serial command
  while (bluetoothSerial.available()) {
    char in_char = (char) bluetoothSerial.read();
    serial_buffer_bluetooth += in_char;
    
    // command complete
    if (in_char == '\n') {
      // not likely, that both serial_command_usb from bluetooth and usb are present
      serial_buffer_bluetooth.trim();
      serial_command_bluetooth = serial_buffer_bluetooth;
      serial_buffer_bluetooth = "";
    }
  }

  String serial_command;
  if (serial_command_bluetooth.length() > 0) {
    if (bluetooth_enable) {
      // forward bluetooth serial to usb serial
      Serial.print(serial_command_bluetooth + "\n");
    } else {
      serial_command = serial_command_bluetooth;
    }
    serial_command_bluetooth = "";
  }

  if (serial_command_usb.length() > 0) {
    if (bluetooth_enable) {
      // forward usb serial to bluetooth serial
      Serial.print(serial_command_usb + "\n"); // echo
      bluetoothSerial.print(serial_command_usb + "\r\n");
    }
    // always use usb serial commands to allow for setting bluetooth_enable
    serial_command = serial_command_usb;
    serial_command_usb = "";
  }

  // serial command from either bluetooth or USB complete
  if (serial_command.length() > 0) {
    if (serial_command.startsWith("p=")) {
      uint8_t p = serial_command.substring(2).toInt();
      set_power(p);
    }
    else if(serial_command.startsWith("sp=")) {
      setpoint = serial_command.substring(3).toFloat();
    }
    else if(serial_command.startsWith("kp=")) {
      double kp = serial_command.substring(3).toFloat();
      pid.SetTunings(kp, pid.GetKi(), pid.GetKd());
    }
    else if(serial_command.startsWith("ki=")) {
      double ki = serial_command.substring(3).toFloat();
      pid.SetTunings(pid.GetKp(), ki, pid.GetKd());
    }
    else if(serial_command.startsWith("kd=")) {
      double kd = serial_command.substring(3).toFloat();
      pid.SetTunings(pid.GetKp(), pid.GetKi(), kd);
    }
    else if (serial_command.startsWith("bt_en=")) {
      bluetooth_enable = serial_command.substring(6).toInt() > 0 ? HIGH : LOW;
      if (bluetooth_enable) {
        // restart bluetooth module in AT mode
        digitalWrite(PIN_BT_ENABLE, HIGH);
      } else {
        digitalWrite(PIN_BT_ENABLE, LOW);
      }
      // restart bluetooth module
      digitalWrite(PIN_BT_VCC, LOW);
      delay(10);
      digitalWrite(PIN_BT_VCC, HIGH);
    }
    else if (serial_command.startsWith("write_eeprom=1")) {
      writeEEPROM();
    }

    serial_command = ""; // reset last serial command
  }

  // pid
  input = t0;

  pid.Compute();


  /*
  // relay controller
  if(input < setpoint) {
    output = 10;
  } else {
    output = 0;
  }
  */
  
  uint8_t p = uint8_t(round(output));
  set_power(p);

  {      
    // serial out
    String out;
    out += "p=";
    out += power;
    out += ";t0=";
    out += t0;
    out += ";t1=";
    out += t1;
    out += ";sp=";
    out += setpoint;
    out += ";kp=";
    out += pid.GetKp();
    out += ";ki=";
    out += pid.GetKi();
    out += ";kd=";
    out += pid.GetKd();
    out += ";output=";
    out += output;
    out += ";loop_time=";
    out += t;
    out += ";time=";
    out += last_t;
    out += ";bt_en=";
    out += bluetooth_enable;

    out += ";temp_sensor0=";
    out += temp_sensor0[0];
    for(size_t i=1; i<8; ++i) {
      out += ",";
      out += temp_sensor0[i];
    }

    out += ";temp_sensor1=";
    out += temp_sensor1[0];
    for(size_t i=1; i<8; ++i) {
      out += ",";
      out += temp_sensor1[i];
    }

    out += ";n_sensors=";
    out += n_sensors;

    out += "\n";

    if (!bluetooth_enable) {
      Serial.print(out);
      bluetoothSerial.print(out);
    } 
  }

  // delay to 100ms total loop time
  long delay_time = 100 - (millis() - last_t);
  if(delay_time > 0) {
    delay(delay_time);
  }
}

// USB serial
void serialEvent() {
  while (Serial.available()) {
    char in_char = (char) Serial.read();
    serial_buffer_usb += in_char;

    // command complete
    if (in_char == '\n') {
      serial_buffer_usb.trim();
      serial_command_usb = serial_buffer_usb;
      serial_buffer_usb = "";
    }
  }
}

