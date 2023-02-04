#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "keypad.h"
#include "rotaryEncoder.h"

#define R1 32
#define R2 33
#define R3 25
#define R4 26
#define C1 27
#define C2 14
#define C3 12
#define C4 16

#define relayPin 17
#define loadButtonPin 5
#define loadButtonLed 18

OneWire oneWire(15);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

LiquidCrystal_I2C lcd(0x27, 16, 4);
Adafruit_MCP4725 dac;
Adafruit_ADS1115 ads;

/* Debug */
const int debug = 0;
unsigned long prevMillis;
unsigned long prevMicros;

/* Fan Config */
const int fanPin = 13;
const int fanChannel = 1;
const double fanPwmFrequency = 10;
const int fanPwmResolution = 8;
int fanSpeed = 0;

/* Buzzer config */
const int buzPin = 4;
const int buzChannel = 8;

/* Flags */
bool outputOn = 0;
bool senseFlag = 0;

/* Telemetry Variables*/

// ADC Readings
const int16_t adcResolution = 32768 - 1; //15 Bit ADC
int16_t voltageAdcRaw = 0;
int16_t currentAdcRaw = 0;
float   currentAdcRaw_f = 0;
float   voltageAdcRaw_f = 0;

// Calculated telemetry from ADC
float measuredCurrent = 0;
float measuredVoltage = 0;
float measuredPower = 0;

// DAC & current set values
int   dacCounts = 0;
int   dacCountsPrevious = 0;
float setCurrent = 0;
float setCurrentPrevious = 0;

/* (Calibration) constants */
const float voltageCal = 99.74;

const float hardwareMaxCurrent = 10.0;

/* Keypad Variables */
int keypress = 0;
int keypress_t = 0;

/* Temperature value */
float tempC = 0;

float fanSet(float Temp) {

  fanSpeed = 0.0042*(pow(Temp, 2.4)); // Fan Curve formula (0.0042*temp^2.4)

    if (fanSpeed > 100){
      fanSpeed = 100;
    }
    if (fanSpeed < 20){
      fanSpeed = 0;
    }

  ledcWrite(fanChannel, (fanSpeed / 100.0) * 255);
  return fanSpeed;
}

// function to print the temperature for a device
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);

  if (tempC == DEVICE_DISCONNECTED_C)
  {
    Serial.println("Error: Could not read temperature data");
    return 0;
  }
  return tempC;
}

void setup() {
  pinMode(C1, OUTPUT);
  pinMode(C2, OUTPUT);
  pinMode(C3, OUTPUT);
  pinMode(C4, OUTPUT);
  pinMode(R1, INPUT_PULLUP);
  pinMode(R2, INPUT_PULLUP);
  pinMode(R3, INPUT_PULLUP);
  pinMode(R4, INPUT_PULLUP);

  pinMode(relayPin, OUTPUT);
  pinMode(loadButtonPin, INPUT_PULLUP);
  pinMode(loadButtonLed, OUTPUT);

  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  digitalWrite(C3, LOW);

  Serial.begin(115200);

  dac.begin(0x60);

  ledcSetup(fanChannel, fanPwmFrequency, fanPwmResolution);
  ledcAttachPin(fanPin, fanChannel);
  ledcAttachPin(buzPin, buzChannel);

  ads.setGain(GAIN_TWO);
  ads.begin();

  lcd.init();
  
  ledcWriteNote(buzChannel, NOTE_C, 5); delay(150);
  ledcWriteNote(buzChannel, NOTE_E, 5); delay(150);
  ledcWriteNote(buzChannel, NOTE_C, 6); delay(300);
  ledcWrite(buzChannel,0);
  
  fanSet(100.0);    // Give fake temperature to test fan
  ads.setDataRate(250);
  Serial.println(ads.getDataRate());
  Serial.println(ads.getGain());

  delay(2000);

  lcd.backlight();

  /* Onewire Bus initialisation */
  sensors.begin();

  /* Initialize Encoder */
  encoderInit();

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

  sensors.setResolution(insideThermometer, 8);
}

void loop() {
  prevMillis = millis();
  prevMicros = micros();
  keypress = keypadRead();

  /* Decision switch case for keypress */
  if (keypress > 0) {
    switch (keypress) {
      case 1:
          setCurrent = 1.0;
      break;
      case 2:
          setCurrent = 2.0;
      break;
      case 3:
          setCurrent = 3.0;
      break;
      case 4:
      break;
      case 5:
        setCurrent = 4.0;
      break;
      case 6:
        setCurrent = 5.0;
      break;
      case 7:
        setCurrent = 6.0;
      break;
      case 8:
      break;
      case 9:
        setCurrent = 7.0;
      break;
      case 10:
        setCurrent = 8.0;
      break;
      case 11:
        setCurrent = 9.0;
      break;
      case 12:
      break;
      case 13:
        digitalWrite(relayPin, HIGH);
      break;
      case 14:
        setCurrent = 10.0;
      break;
      case 15:
        digitalWrite(relayPin, LOW);
      break;
      case 16:
       lcd.clear();
      break;
    }
  }

  /* Determine dacCounts for set current if setCurrent has changed */
  if (setCurrent != setCurrentPrevious){
  dacCounts = (setCurrent / hardwareMaxCurrent) * 4095;
  setCurrentPrevious = setCurrent;
  }

  /* Offset correction for current */
  if (outputOn != 0 && measuredVoltage > 0.1 && setCurrent != 0){
    if ((measuredCurrent - setCurrent) > 0.005){
      if ((dacCounts - 1) >= 0){
        dacCounts--;
      }  
    }
    else if ((measuredCurrent - setCurrent) < -0.005){
      if ((dacCounts + 1) <= 4095){
        dacCounts++;
      }  
    }
  }
  
  /* Reading ADC Inputs */
  currentAdcRaw = ads.readADC_SingleEnded(3); // Current measurement
  voltageAdcRaw = ads.readADC_SingleEnded(0); // Voltage measurement

  if (currentAdcRaw < 0) {
    currentAdcRaw = 0;
  }

  if (voltageAdcRaw < 0){
    voltageAdcRaw = 0;
  }

  currentAdcRaw_f = float(currentAdcRaw);
  voltageAdcRaw_f = float(voltageAdcRaw);

  measuredCurrent = (hardwareMaxCurrent / adcResolution) * currentAdcRaw_f;

  measuredVoltage = (80.0 / adcResolution) * voltageAdcRaw_f;
  measuredVoltage = (measuredVoltage * voltageCal) / 100;
  measuredPower = measuredCurrent * measuredVoltage;

  sensors.requestTemperatures();
  tempC = printTemperature(insideThermometer);

  fanSpeed = fanSet(tempC);

  if ((digitalRead(loadButtonPin)) == 0){
    outputOn = !outputOn;
  }

  if (outputOn == 0){
    dacCounts = 0;
    digitalWrite(loadButtonLed, LOW);
  }
  else{
    digitalWrite(loadButtonLed, HIGH);
  }

  if (dacCounts != dacCountsPrevious){
    if (dacCounts < 0){
      dacCounts = 0;
    }
      dac.setVoltage(dacCounts, false);
  }

  // lcd.setCursor(0, 0);
  // lcd.print("Iset:");
  // lcd.print(setCurrent, 2);
  // lcd.print("A");
  // lcd.setCursor(0, 1);
  // lcd.print("I:");
  // lcd.print(measuredCurrent, 2);
  // lcd.print("A");
  // lcd.setCursor(12, 0);
  // lcd.print("V:");
  // lcd.print(measuredVoltage, 2);
  // lcd.print("V");
  // lcd.setCursor(0, 2);
  // lcd.print("P:");
  // lcd.print(measuredPower, 2);
  // lcd.print("W");
  // lcd.setCursor(17, 3);
  // lcd.print(tempC, 0);
  // lcd.print("C");
  // lcd.setCursor(0, 3);
  // lcd.print("FAN:");
  // lcd.print(fanSpeed, 0);
  // lcd.print("%");

  Serial.print("setI:");
  Serial.print(setCurrent, 3);
  Serial.print("A|IADC:");
  Serial.print(currentAdcRaw);
  Serial.print("|VADC:");
  Serial.print(voltageAdcRaw);
  Serial.print("|DAC:");
  Serial.print(dacCounts);
  Serial.print("|Imeas:");
  Serial.print(measuredCurrent, 4);
  Serial.print("A|Vmeas:");
  Serial.print(measuredVoltage, 4);
  Serial.print("V|Pmeas: ");
  Serial.print(measuredPower, 3);
  Serial.print("W|Fan:");
  Serial.print(fanSpeed, 0);
  Serial.print("%|temp:");
  Serial.print(tempC, 2);  
  Serial.print("°C|loopTime: ");
  Serial.print(millis() - prevMillis);
  Serial.print("ms ");  
  Serial.print(micros() - prevMicros);
  Serial.println("µs");

  int encoder = detectMovement();

 if (encoder > 0) {
    switch (encoder) {
      case 1:
        if (setCurrent + 0.01 <= hardwareMaxCurrent) {
          setCurrent = setCurrent + 0.01;
        }
        else if (setCurrent + 0.01 > 10.0) {
          setCurrent = hardwareMaxCurrent;
        }
      break;
      case 2:
        if (setCurrent - 0.01 >= 0) {
          setCurrent = setCurrent - 0.01;
        }
        else if (setCurrent - 0.01 < 0) {
          setCurrent = 0.0;
        }
      break;
    }
  }
}


