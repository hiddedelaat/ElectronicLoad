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
const double fanPwmFrequency = 30;
const int fanPwmResolution = 8;
float fanSpeed = 0;

/* Buzzer config */
const int buzPin = 4;
const int buzChannel = 8;

/* Flags */
bool outputOn = 0;
bool senseFlag = 0;
bool cursorFlag = 0;

/* Telemetry Variables*/
int16_t voltageAdcRaw = 0;
int16_t currentAdcRaw = 0;
float currentAdcRaw_t = 0;
float voltageAdcRaw_t = 0;

float I_meas = 0;
float V_meas = 0;
float P_set = 0;
float P_meas = 0;
float dacv_f = 0;

/* Calibration constants */
const float voltage_cal = 99.58;

int keypress = 0;
int keypress_t = 0;
int dacv = 0;
float Iset = 10.0;
float tempC = 0;

float fanSet(float Temp) {

  fanSpeed = 0.0;

  fanSpeed = 0.0042*(pow(Temp, 2.4)); // Fan Curve (Excel File)
  if (fanSpeed > 100.0){
    fanSpeed = 100.0;
  }
  if (fanSpeed < 20.0){
    fanSpeed = 0.0;
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

  pinMode(relayPin, OUTPUT);
  pinMode(loadButtonPin, INPUT_PULLUP);
  pinMode(loadButtonLed, OUTPUT);

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

  encoderInit();

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
        break;
      case 11:
        break;
    }
  }

  float dacv_f = float(dacv);
  float I_set = (Iset / 4095.0) * dacv_f;

  currentAdcRaw = ads.readADC_SingleEnded(3); // Current ADC 16 bit

  /* Differential Voltage Measurement is work in progress! */

  if (senseFlag == true){
  voltageAdcRaw = ads.readADC_Differential_0_1();
  voltageAdcRaw = (voltageAdcRaw + 4462) * 2.1378;
  }
  else{
  voltageAdcRaw = ads.readADC_SingleEnded(0);
    if (voltageAdcRaw < 0) {
      voltageAdcRaw = 0;
    }
  }

  if (currentAdcRaw < 0) {
    currentAdcRaw = 0;
  }

  currentAdcRaw_t = float(currentAdcRaw);
  voltageAdcRaw_t = float(voltageAdcRaw);

  I_meas = (Iset / (32768 - 1)) * currentAdcRaw_t;
  V_meas = (80.0 / (32768 - 1)) * voltageAdcRaw_t;
  V_meas = (V_meas * voltage_cal) / 100;
  P_set  = I_set * V_meas;
  P_meas = I_meas * V_meas;

  sensors.requestTemperatures();
  tempC = printTemperature(insideThermometer);

  fanSpeed = fanSet(tempC);

  if ((digitalRead(loadButtonPin)) == 0){
    outputOn = !outputOn;
  }

  if (outputOn == 0){
    dac.setVoltage(0, false);
    digitalWrite(loadButtonLed, LOW);
  }
  else{
    dac.setVoltage(dacv, false);
    digitalWrite(loadButtonLed, HIGH);
  }

  if (cursorFlag != false){
  lcd.noCursor();
  cursorFlag = false;
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
  lcd.setCursor(17, 2);
  lcd.print(tempC, 0);
  lcd.setCursor(19, 2);
  lcd.print("c");
  lcd.setCursor(8, 2);
  lcd.print("W");
  lcd.setCursor(0, 3);
  lcd.print("FAN: ");
  lcd.print(fanSpeed, 0);
  lcd.setCursor(8, 3);
  lcd.print("%");
  lcd.setCursor(11, 3);

  lcd.setCursor(6, 0);
  if (cursorFlag != true){
  lcd.cursor();
  cursorFlag = true;
  }

  // lcd.setCursor(17, 3);
  // if(senseFlag == 0){
  //   lcd.print("Off");
  // }
  // else{
  //   lcd.print("On");
  // }

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
  Serial.print(fanSpeed, 0);
  Serial.print("% | ");
  Serial.print("sinkTemp:");
  Serial.print(tempC, 2);  
  Serial.print("°C | loopTime: ");
  Serial.print(millis() - prevMillis);
  Serial.print("ms | ");  
  Serial.print(micros() - prevMicros);
  Serial.println("µs");
  

}

