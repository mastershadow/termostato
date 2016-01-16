#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <math.h>

#define SERIAL_BAUD 9600
#define RELAY_PIN 2
#define THERMISTOR_PIN 0
#define RESISTOR 10000.0
#define BASE 1024.0
#define K_TO_C 273.15
#define A 0.001129148
#define B 0.000234125
#define C 0.0000000876741
#define SWITCH_PIN 8

#define RX 5
#define TX 6
#define SAMPLES 5

#define ARG_ON "ON"
#define ARG_OFF "OFF"
#define CMD_SETRELAY "SETRELAY"
#define CMD_GETRELAY "GETRELAY"
#define CMD_GETTEMP "GETTEMP"
#define CMD_OK "OK"
#define CMD_WHAT "WHAT?"


boolean isRelayOn = false;
int switchStatus = HIGH;
int lastSwitchStatus = HIGH;
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

// SerialCommand SCmd;
SoftwareSerial EspSerial = SoftwareSerial(RX,TX);
SerialCommand SCmd(EspSerial);

int currentSample = 0;
double currentTemperature = 0;
double temperatures[SAMPLES];

// Steinhart-Hart Thermistor Equation:
// Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
double getTemperatureFromRawInput(int rawInput) {
  double lnResistance = 0;
  double temperature = 0;
  
  lnResistance = log(RESISTOR * (BASE / rawInput - 1));
  temperature = 1 / (A + (B + (C * lnResistance * lnResistance)) * lnResistance);
  temperature = temperature - K_TO_C;
  
  return temperature;
}

void setRelay(boolean status) {
  if (status == true) {
    digitalWrite(RELAY_PIN, LOW);
  } else {
    digitalWrite(RELAY_PIN, HIGH); // high is off
  }
  isRelayOn = status;
}

boolean getRelay() {
  return isRelayOn;
}

double getTemperature() {
  return getTemperatureFromRawInput(analogRead(THERMISTOR_PIN));
}

void sendToSerial(const char *command, const char *argument) {
  char _buffer[64];
  memset(_buffer, 0, sizeof(_buffer));
  if (argument != NULL) {
    snprintf(_buffer, sizeof(_buffer) - 1, "+%s %s\n", command, argument);
  } else {
    snprintf(_buffer, sizeof(_buffer) - 1, "+%s\n", command);
  }
  // Serial.print(_buffer);
  EspSerial.print(_buffer);
}

void SerialSetRelay() {
  char *arg = SCmd.next();
  const char *realArg = NULL;
  if (arg != NULL) {
    if (strncmp(arg, ARG_ON, 2) == 0) {
      realArg = ARG_ON;
      setRelay(true);
    } else {
      realArg = ARG_OFF;
      setRelay(false);
    }
    
    sendToSerial(CMD_SETRELAY, realArg);
  }
}

void SerialGetRelay() {
  sendToSerial(CMD_GETRELAY, getRelay() ? ARG_ON : ARG_OFF);
}

void SerialGetTemperature() {
  char _strTemp[10];
  double t = currentTemperature; // getTemperature();
  memset(_strTemp, 0, sizeof(_strTemp));
  dtostrf(t, 2, 1, _strTemp);
  sendToSerial(CMD_GETTEMP, _strTemp);
}

void SerialUnrecognized() {
  sendToSerial(CMD_WHAT, NULL);
}

void readInputs() {
  // read switch status
  int reading = digitalRead(SWITCH_PIN);
  if (reading != lastSwitchStatus) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != switchStatus) {
      switchStatus = reading;
      
      if (switchStatus == LOW) {
        setRelay(!getRelay());
      }
    }
  }
  lastSwitchStatus = reading;

  // read a new sample
  temperatures[currentSample] = getTemperature();
  currentSample = (currentSample + 1) % SAMPLES;
  // update 
  int samplesCount = 0;
  double t = 0;
  for (int i = 0; i < SAMPLES; i++) {
    if (temperatures[i] > 0) {
      t += temperatures[i];
      samplesCount++;  
    }
  }
  if (samplesCount > 0) {
    currentTemperature = t / samplesCount;
  } else {
    currentTemperature = 0;
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  // switch
  lastSwitchStatus = HIGH;
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(SWITCH_PIN, lastSwitchStatus);
  
  setRelay(false);

  // clear and init temperature sampling array
  memset(&temperatures, 0, sizeof(temperatures));

  // Serial manangement
  Serial.begin(SERIAL_BAUD); 
  EspSerial.begin(SERIAL_BAUD);
  delay(10); 
  // Setup callbacks for SerialCommand commands 
  SCmd.addCommand(CMD_SETRELAY, SerialSetRelay);
  SCmd.addCommand(CMD_GETRELAY, SerialGetRelay);
  SCmd.addCommand(CMD_GETTEMP, SerialGetTemperature);
  SCmd.addDefaultHandler(SerialUnrecognized);
  sendToSerial(CMD_OK, NULL);
}

void loop() {
  SCmd.readSerial();
  delay(10);
  readInputs();
}
