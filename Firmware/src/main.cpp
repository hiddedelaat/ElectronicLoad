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

//DEBUG--------------------------------------------

int debug = 0;

//DEBUG--------------------------------------------

const int fanPin = 13;
const int fanChannel = 1;
const int buzPin = 4;
const int buzChannel = 8;
bool output_on = 0;

const double freq = 200 ;

const int resolution = 8;

int adc3_cal = 0;

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



float fanSet(float P, float T) {

  float fanSpeed = 0.0;

  if ((P >= 25.0 && P < 50.0) | (T >= 35.0 && P < 45.0)) {
    fanSpeed = 25.0;
    //leds[0] = CRGB::Green;
  }

  if ((P >= 50.0 && P < 100.0) | (T >= 45.0 && P < 55.0)) {
    fanSpeed = 60.0;
    //leds[0] = CRGB::Yellow;
  }

  if (P >= 100.0 || T >= 50.0) {
    fanSpeed = 100.0;
    //leds[0] = CRGB::Red;
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

  Serial.begin(9600);

  dac.begin(0x60);
  dac.setVoltage(0x000, true);

  ledcSetup(fanChannel, freq, resolution);
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

  if (debug == 1) {

    Serial.println("Locating devices...");

    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    Serial.print("Parasite power is: ");
    if (sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");

    Serial.print("D18B20 Adr: ");
    printAddress(insideThermometer);
    Serial.println();

    Serial.print("Res: ");

    Serial.println(sensors.getResolution(insideThermometer), DEC);

    sensors.requestTemperatures();

    tempC = printTemperature(insideThermometer);
    Serial.print("Temp: ");
    Serial.print(tempC, 4);
    Serial.println("C");

    delay(500);

  }
}

void loop() {
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
      case 4:
        dacv = 4095;
        break;
      case 9:
        lcd.clear();
        break;
      case 10:
        digitalWrite(relay, LOW);
        break;
      case 12:
        digitalWrite(relay, HIGH);
        break;
    }
  }

  float dacv_f = float(dacv);
  float I_set = (Iset / 4095.0) * dacv_f;

  int16_t adc3 = ads.readADC_SingleEnded(3); // Current ADC 16 bit
  int16_t adc0 = ads.readADC_SingleEnded(0); // Voltage ADC 16 bit

  adc3 = adc3 - adc3_cal; //Offset removal

  if (adc0 < 0) {
    adc0 = 0;
  }

  if (adc3 < 0) {
    adc3 = 0;
  }

  float adc3_f = float(adc3);
  float adc0_f = float(adc0);

  float I_meas = (Iset / (32768 - adc3_cal)) * adc3_f;
  float V_meas = (80.0 / 32768) * adc0_f;
  float P_set = I_set * V_meas;
  float P_meas = I_meas * V_meas;

  sensors.requestTemperatures();
  tempC = printTemperature(insideThermometer);

  float F_speed = fanSet(P_meas, tempC);

  // Serial.print("I_SET:");
  // Serial.print(I_set, 4);
  // Serial.print("A | I_RAW_ADC:");
  // Serial.print(adc3);
  // Serial.print(" | I_MEAS:");
  // Serial.print(I_meas, 4);
  // Serial.print("A | V_MEAS:");
  // Serial.print(V_meas, 4);
  // Serial.print("V | P_SET:");
  // Serial.print(P_set, 3);
  // Serial.print("W | P_MEAS:");
  // Serial.print(P_meas, 3);
  // Serial.print("W | ");
  // Serial.print(F_speed, 2);
  // Serial.print("% | ");

  // Serial.print("Fuse Status:");
  // Serial.print(fuseStatus);
  // Serial.print(" | HeatS_Temp_C:");

  Serial.println(tempC, 4);

  lcd.setCursor(0, 0);
  lcd.print("Iset: ");
  lcd.print(I_set, 2);
  lcd.print("A ");
  lcd.print(adc3);
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

  delay(10);

  //lcd.clear();

  if ((digitalRead(load_sw)) == 0){
    output_on = !output_on;
  }

  if (output_on == 0){
    dac.setVoltage(0, false);
    digitalWrite(load_sw_led, LOW);
  }
  else{
    dac.setVoltage(dacv, false);
    digitalWrite(load_sw_led, HIGH);
  }
  

}

