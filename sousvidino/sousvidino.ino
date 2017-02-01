// Sousvidino. Low-cost Sous vide controller using a one-wire temperature sensor and a solid state relay for heater power switching
//
// Author: Markus Roth <mail@rothmark.us>
// 
// setpoint:
//   burger: 55: rare, 60: medium, 65: medium well    
//
// PID parameters (dependent on water volume, heater power
// 1.5l water boiler 2kw:
//   kp = 1.0;
//   ki = 0.03;
//   kd = 0.0;
// 6l water, immersion heater 1kW:
//   kp = 30.0;
//   ki = 0.03;
//   kd = 0.0;

#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define SOUSVIDINO_EEPROM_VERSION 42

#define PIN_SSR 13 // solid state relay
#define PIN_GND_ONEWIRE 12
#define PIN_VCC_ONEWIRE 11
#define PIN_TEMP 3
#define MAX_POWER 50 // has to be smaller than 65535 cycles / 1250 prescaled timer cycles per ac period

// for display/control (second row)
#define CONTROL_STATE_SP 0
#define CONTROL_STATE_P 1
#define CONTROL_STATE_I 2
#define CONTROL_STATE_D 3

// button
#define ANALOG_BUTTON_MIN_PRESS_TIME_MS 200
#define ANALOG_BUTTON_PIN 0 // from lcd keypad shield
int lastButtonPressTime = 0;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// from https://www.dfrobot.com/wiki/index.php/Arduino_LCD_KeyPad_Shield_(SKU:_DFR0009)
uint8_t read_LCD_button()
{
 int adc_key_in = analogRead(ANALOG_BUTTON_PIN);      // read the value from the sensor 

 if (adc_key_in > 1000) return btnNONE;
 // For V1.1 us this threshold
 if (adc_key_in < 50)   return btnRIGHT;  
 if (adc_key_in < 250)  return btnUP; 
 if (adc_key_in < 450)  return btnDOWN; 
 if (adc_key_in < 650)  return btnLEFT; 
 if (adc_key_in < 850)  return btnSELECT;  

 return btnNONE;  // when all others fail, return this...
}


LiquidCrystal lcd(8, 9, 4, 5, 6, 7);   // select the pins used on the LCD panel
OneWire one_wire(PIN_TEMP);
DallasTemperature sensors(&one_wire);

// PID
double input, output, setpoint;
double kp, ki, kd;
// setpoint, kp, ki, kd initialized in setup();

uint8_t controlState = CONTROL_STATE_SP;
static double * const controlValues[] = { &setpoint, &kp, &ki, &kd };
static const double controlIncrements[] = { 0.5, 0.1, 0.001, 0.1 };
static const char*  controlNames[] = { "SP", "P  ", "I  ", "D  " };

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
  }
}

PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

static const DeviceAddress temp_sensor0 = { 0x28, 0xFF, 0x24, 0x79, 0x01, 0x15, 0x02, 0x90 }; 
static const DeviceAddress temp_sensor1 = { 0x28, 0xFF, 0x95, 0x94, 0x01, 0x15, 0x02, 0xC6 }; 
static const DeviceAddress temp_sensor2 = { 0x28, 0xFF, 0xE7, 0x57, 0x01, 0x15, 0x02, 0x99 }; 

// output
uint8_t ssr_state = LOW;
uint8_t power = 0; // 0: off, MAX_POWER: full

// serial in buffer
String serial_buffer = "";
// last complete command (tailing \n)
String serial_command = "";

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

  // GND and VCC for one wire temp sensor
  pinMode(PIN_GND_ONEWIRE, OUTPUT);
  digitalWrite(PIN_GND_ONEWIRE, LOW);
  pinMode(PIN_VCC_ONEWIRE, OUTPUT);
  digitalWrite(PIN_VCC_ONEWIRE, HIGH);

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

  // power
  set_power(0);
    
  // LCD
  lcd.begin(16, 2);
  lcd.clear();

  // serial
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // sensors
  sensors.begin();
  
  // set the resolution to 12 bit
  sensors.setResolution(temp_sensor0, 12);
//  sensors.setResolution(temp_sensor1, 12);
//  sensors.setResolution(temp_sensor2, 12);

  // PID
  readEEPROM();
  pid.SetTunings(kp, ki, kd);
  pid.SetSampleTime(5000); // ms. a little longer than normal loop time
  // restrict max power
  pid.SetOutputLimits(0, MAX_POWER);
  pid.SetMode(AUTOMATIC);
  
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

uint8_t loop_it = 0;

int last_t = 0;


// whole loop around 77ms
void loop() {
  
  int t = millis() - last_t;
  last_t = millis();

  uint8_t button_value = read_LCD_button();
  if(millis() - lastButtonPressTime > ANALOG_BUTTON_MIN_PRESS_TIME_MS) {
    lastButtonPressTime = millis();
    
    switch(button_value) {
      case btnSELECT:
        controlState += 1;
        controlState %= 4;
        break;
      case btnUP:
        *controlValues[controlState] += controlIncrements[controlState];
        pid.SetTunings(kp, ki, kd);
        break;
      case btnDOWN:
        *controlValues[controlState] -= controlIncrements[controlState];
        pid.SetTunings(kp, ki, kd);
        break;
      case btnLEFT:
        readEEPROM();
        break;
      case btnRIGHT:
        writeEEPROM();
        break;
      default:
        break;
    }
  }
  
  // get temperatures
//  sensors.requestTemperatures(); // 766ms!
  sensors.requestTemperaturesByAddress(temp_sensor0); // 46ms
  float t0 = sensors.getTempC(temp_sensor0); // 13ms 
//  float t1 = sensors.getTempC(temp_sensor1); // 13ms
//  float t2 = sensors.getTempC(temp_sensor2); // 13ms
  float t1 = -127.0;
  float t2 = -127.0;

  // serial in
  if(serial_command.length() > 0) {
    serial_command.trim(); // remove \n at end
    if(serial_command.startsWith("p=")) {
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
    lcd.setCursor(0,0);
    lcd.write(serial_command.c_str());
    delay(500);
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
  
  ++loop_it;
  loop_it %= 10;
  if(loop_it == 0) {
    loop_it=0;
  
    // lcd
    lcd.setCursor(0,0);
    lcd.print("PV ");
    lcd.setCursor(3,0);
    lcd.print(t0, 2);

    lcd.setCursor(10,0);
    lcd.print("OUT");
    lcd.setCursor(14,0);
    {
    // right align output value
    char out_str[4];
    sprintf(out_str, "%2d", p*2); // show output as percentage
    lcd.print(out_str);
    }

    // second line 
    lcd.setCursor(0,1);
    lcd.print(controlNames[controlState]);
    lcd.setCursor(3,1);
    lcd.print(*controlValues[controlState], 2);

    // time
    unsigned long sec = millis() / 1000;
    unsigned long minutes = sec / 60;
    sec %= 60;

    char sec_str[3];
    sprintf(sec_str, "%02ld", sec);
    char minutes_str[5];
    sprintf(minutes_str, "%4ld", minutes);
    lcd.setCursor(9,1);
    lcd.print(minutes_str);
    lcd.setCursor(13,1);
    lcd.print(":");
    lcd.setCursor(14,1);
    lcd.print(sec_str);
    
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
    out += pid.GetKp();
    out += ";ki=";
    out += pid.GetKi();
    out += ";kd=";
    out += pid.GetKd();
    out += ";output=";
    out += output;
    out += ";loop_time=";
    out += t;
  
    out += "\n";

    Serial.print(out.c_str());
  }
}

void serialEvent() {
  while (Serial.available()) {
    char in_char = (char) Serial.read();
    serial_buffer += in_char;
    
    // command complete
    if (in_char == '\n') {
      serial_command = serial_buffer;
      serial_buffer = "";
    }
  }
}
