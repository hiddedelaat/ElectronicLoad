#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "keypad.h"

OneWire oneWire(15);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

LiquidCrystal_I2C lcd(0x27, 16, 4);
Adafruit_MCP4725 dac;
Adafruit_ADS1115 ads;

/* Debug */
int debug = 0;
unsigned long prevMillis;
unsigned long prevMicros;

const int fanPin = 13;
const int fanChannel = 1;
const double fanPwmFrequency = 30;
const int fanPwmResolution = 8;

float fanSpeed = 0;

const int buzPin = 4;
const int buzChannel = 8;

bool      outputOn = 0;
bool      senseFlag = 0;

/* Telemetry Variables*/
int16_t voltageAdcRaw = 0;
int16_t currentAdcRaw = 0; 

/* Calibration constants */
const float voltage_cal = 99.58;

#define R1 32
#define R2 33
#define R3 25
#define R4 26
#define C1 27
#define C2 14
#define C3 12
#define C4 16

#define relay 17
#define load_sw 5
#define load_sw_led 18

int keypress = 0;
int keypress_t = 0;
int dacv = 0;
float Iset = 10.0;
float tempC = 0;

float fanSet(float Power, float Temp) {

  fanSpeed = 0.0;

  fanSpeed = 0.0042*(pow(Temp, 2.4)); // Fan Curve (Excel File)
  if (fanSpeed > 100.0){
    fanSpeed = 100.0;
  }

  ledcWrite(fanChannel, (fanSpeed / 100.0) * 255);

  return fanSpeed;

}
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
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

  pinMode(relay, OUTPUT);
  pinMode(load_sw, INPUT_PULLUP);
  pinMode(load_sw_led, OUTPUT);


  digitalWrite(C1, LOW);
  digitalWrite(C2, LOW);
  digitalWrite(C3, LOW);

  Serial.begin(115200);

  dac.begin(0x60);
  dac.setVoltage(0x000, true);

  ledcSetup(fanChannel, fanPwmFrequency, fanPwmResolution);
  ledcAttachPin(fanPin, fanChannel);
  ledcAttachPin(buzPin, buzChannel);

  ads.setGain(GAIN_TWO);
  ads.begin();

  lcd.init();
  lcd.backlight();

  ledcWriteNote(buzChannel, NOTE_D, 6);
  delay(250);
  ledcWrite(buzChannel,0);

  sensors.begin();

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0");

  sensors.setResolution(insideThermometer, 8);
}

void loop() {
  prevMillis = millis();
  prevMicros = micros();
  keypress = keypadRead();

  if (keypress > 0) {
    switch (keypress) {
      case 1:
        if (dacv + 10 <= 4095) {
          dacv = dacv + 10;
        }
        else if (dacv + 10 > 4095) {
          dacv = 4095;
        }
        break;
      case 2:
        if (dacv - 10 >= 0) {
          dacv = dacv - 10;
        }
        else if (dacv - 10 < 0) {
          dacv = 0;
        }
        break;
      case 3:
        dacv = 0;
        break;
      case 5:
        dacv = 4095;
        break;
      case 9:
        lcd.clear();
        break;
      case 10:
        digitalWrite(relay, LOW);
        senseFlag = false;
        break;
      case 11:
        digitalWrite(relay, HIGH);
        senseFlag = true;
        break;
    }
  }

  float dacv_f = float(dacv);
  float I_set = (Iset / 4095.0) * dacv_f;

  currentAdcRaw = ads.readADC_SingleEnded(3); // Current ADC 16 bit

  if (senseFlag == true){
  voltageAdcRaw = ads.readADC_Differential_0_1();
  }
  else{
  voltageAdcRaw = ads.readADC_SingleEnded(0);
  }

  if (voltageAdcRaw < 0) {
    voltageAdcRaw = 0;
  }

  if (currentAdcRaw < 0) {
    currentAdcRaw = 0;
  }

  float currentAdcRaw_t = float(currentAdcRaw);
  float voltageAdcRaw_t = float(voltageAdcRaw);

  float I_meas = (Iset / (32768 - 1)) * currentAdcRaw_t;
  float V_meas = (80.0 / (32768 - 1)) * voltageAdcRaw_t;
        V_meas = (V_meas * voltage_cal) / 100;
  float P_set  = I_set * V_meas;
  float P_meas = I_meas * V_meas;

  sensors.requestTemperatures();
  tempC = printTemperature(insideThermometer);

  float F_speed = fanSet(P_meas, tempC);

  if ((digitalRead(load_sw)) == 0){
    outputOn = !outputOn;
  }

  if (outputOn == 0){
    dac.setVoltage(0, false);
    digitalWrite(load_sw_led, LOW);
  }
  else{
    dac.setVoltage(dacv, false);
    digitalWrite(load_sw_led, HIGH);
  }

  lcd.setCursor(0, 0);
  lcd.print("Iset: ");
  lcd.print(I_set, 2);
  lcd.print("A ");
  lcd.print(currentAdcRaw);
  lcd.setCursor(0, 1);
  lcd.print("I: ");
  lcd.print(I_meas, 2);
  lcd.setCursor(7, 1);
  lcd.print("A");
  lcd.setCursor(11, 1);
  lcd.print("V: ");
  lcd.print(V_meas, 2);
  lcd.print("V");
  lcd.setCursor(0, 2);
  lcd.print("P: ");
  lcd.print(P_meas, 2);
  lcd.setCursor(15, 2);
  lcd.print(tempC, 1);
  lcd.setCursor(19, 2);
  lcd.print("c");
  lcd.setCursor(8, 2);
  lcd.print("W");
  lcd.setCursor(0, 3);
  lcd.print("FAN: ");
  lcd.print(F_speed, 1);
  lcd.setCursor(10, 3);
  lcd.print("%");

  Serial.print("I_SET:");
  Serial.print(I_set, 4);
  Serial.print("A | I_ADC_RAW:");
  Serial.print(currentAdcRaw);
  Serial.print(" | V_ADC_RAW:");
  Serial.print(voltageAdcRaw);
  Serial.print(" | I_MEAS:");
  Serial.print(I_meas, 4);
  Serial.print("A | V_MEAS:");
  Serial.print(V_meas, 4);
  Serial.print("V | P_SET:");
  Serial.print(P_set, 3);
  Serial.print("W | P_MEAS:");
  Serial.print(P_meas, 3);
  Serial.print("W | ");
  Serial.print(F_speed, 2);
  Serial.print("% | ");
  Serial.print("sinkTemp:");
  Serial.print(tempC, 2);  
  Serial.print("°C | loopTime (ms): ");
  Serial.print(millis() - prevMillis);
  Serial.print("ms | loopTime (µs):  ");  
  Serial.print(micros() - prevMicros);
  Serial.println("µs"); 

}

