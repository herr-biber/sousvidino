#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define PIN_SSR 13
#define PIN_GND 12 // for one wire
#define PIN_VCC 11 // for one wire
#define PIN_TEMP 3
#define MAX_POWER 50 // has to be smaller than 65535 cycles / 1250 prescaled timer cycles per ac period

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);   // select the pins used on the LCD panel
OneWire one_wire(PIN_TEMP);
DallasTemperature sensors(&one_wire);

// PID
double input, output, setpoint;

// used for first beef
//double kp = 4.0;
//double ki = 0.1;
//double kd = 3.0;

// Wasserkocher 2kw
//double kp = 1.0;
//double ki = 0.03;
//double kd = 0.0;

double kp = 30.0;
double ki = 0.03;
double kd = 0.0;


PID pid(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// APID auto tune
double aTuneStep = 10, aTuneNoise = 1.0;
int aTuneLookBack = 150;
//PID_ATune aTune(&input, &output);
boolean autotune = false;

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
  pinMode(PIN_GND, OUTPUT);
  digitalWrite(PIN_GND, LOW);
  pinMode(PIN_VCC, OUTPUT);
  digitalWrite(PIN_VCC, HIGH);

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
//  lcd.setCursor(0,0);
//  lcd.print("sousvidino");

  // serial
  Serial.begin(115200);

  // sensors
  sensors.begin();
  
  // set the resolution to 12 bit
  sensors.setResolution(temp_sensor0, 12);
//  sensors.setResolution(temp_sensor1, 12);
//  sensors.setResolution(temp_sensor2, 12);

  // PID
  // restrict max power
  setpoint = 58.0; // sp burger: 55: rare, 60: medium, 65: mediu well    
//  output = 10.0; // only set on autotune
  pid.SetSampleTime(5000); // ms. a little longer than normal loop time
  pid.SetOutputLimits(0, MAX_POWER);
  pid.SetMode(AUTOMATIC);

  // PID auto tune
// aTune.SetNoiseBand(aTuneNoise);
// aTune.SetOutputStep(aTuneStep);
//  aTune.SetLookbackSec(aTuneLookBack);
//  aTune.SetControlType(CONTROL_TYPE_PI);
  autotune = false;
  
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
  
  // get temperatures
//  sensors.requestTemperatures(); // 766ms!/
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
    else if(serial_command.startsWith("autotune=")) {
      autotune = serial_command.substring(9).toInt();
    }
    lcd.setCursor(0,0);
    lcd.write(serial_command.c_str());
    delay(500);
    serial_command = ""; // reset last serial command
  }

  // pid
  input = t0;

  if(autotune) {
//    int is_autotune_finished = aTune.Runtime();/
    
//    if (is_autotune_finished) {
//      autotune = false;
//    }
    // set pid with current settings, even, if not running
//    aTune.FinishUp();/
//    pid.SetTunings(aTune.GetKp(), aTune.GetKi(), aTune.GetKd());/
  }
  else {
    pid.Compute();
  }

  /*
  // relay controller
  if(input < setpoint) {
    output = 10;
  } else {
    output = 0;
  }
  */
  
  
  //output = round(output); // round output such that pid knows the a
  uint8_t p = uint8_t(round(output));
  set_power(p);
  
  // make sure autotune knows what output was actually taken
  if(autotune) {
    output = power;
  }

  ++loop_it;
  loop_it %= 10;
  if(loop_it == 0) {
  
    // lcd
    lcd.setCursor(0,1);
    lcd.print("P   ");
    lcd.setCursor(2,1);
    lcd.print(power);
  
    lcd.setCursor(0,0);
    lcd.print("PV ");
    lcd.setCursor(3,0);
    lcd.print(t0);
  
    lcd.setCursor(8,0);
    lcd.print("SP ");
    lcd.setCursor(11,0);
    lcd.print(setpoint);
  
    lcd.setCursor(8,1);
    lcd.print("O ");
    lcd.setCursor(10,1);
    lcd.print(output);
    lcd.setCursor(15,1);
    lcd.print(autotune ? "+" : "-");
  
  
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
    out += ";autotune=";
    out += autotune;
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
