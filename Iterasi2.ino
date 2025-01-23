#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Data wire is connected to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

#define TdsSensorPin A1
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample points

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 20, 4);

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, temperature = 25;
int tdsValue = 0;

// pH
#define ph_Pin A0
#define Offset 0.00 // deviation compensate
#define samplingInterval 20
#define ArrayLenth 40 // times of collection
int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;

float PHEmpat = 1.73;
float PHTujuh = 1.93;
float voltage, pHValue, ph;

float volt;
int ntu;
float voltageKekeruhan;
float suhu;
int tds;
int kekeruhanVal;
int kekeruhan;

void setup() {
  Serial.begin(9600);
  sensors.begin();
  pinMode(TdsSensorPin, INPUT);
  pinMode(ph_Pin, INPUT);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("      Welcome to     ");
  lcd.setCursor(0, 2);
  lcd.print("         WQMS  ");
  delay(2000);
  lcd.clear();
}

void loop() {
  kirim();
  String minta = "";
  while (Serial.available() > 0) {
    minta += char(Serial.read());
  }
  minta.trim();
  if (minta == "Ya") {
    String data = String(suhu) + "#" + String(tds) + "#" + String(ph) + "#" + String(kekeruhan);
    Serial.println(data);
  }
  minta = "";

  displayData();
  delay(1000);
}

void kirim() {
  sensorSuhu();
  sensorTds();
  sensorPh();
  sensorKekeruhan();
  Serial.print("Suhu : ");
  Serial.print(suhu);
  Serial.print(" °C, TDS : ");
  Serial.print(tds);
  Serial.print(" PPM, pH : ");
  Serial.print(ph);
  Serial.print(", Kekeruhan : ");
  Serial.print(kekeruhan);
  Serial.println(" NTU");
}

void displayData() {
  lcd.setCursor(0, 0);
  lcd.print("Suhu : ");
  lcd.print(suhu);
  lcd.print(char(223));
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Tds  : ");
  lcd.print(tds);
  lcd.print(" PPM");

  lcd.setCursor(0, 2);
  lcd.print("PH   : ");
  lcd.print(ph);

  lcd.setCursor(0, 3);
  lcd.print("Kekeruhan   : ");
  lcd.print(kekeruhan);
}

void sensorSuhu() {
  sensors.requestTemperatures();
  suhu = sensors.getTempCByIndex(0);
}

void sensorTds() {
   static unsigned long analogSampleTimepoint = millis();
   if (millis() - analogSampleTimepoint > 40U) { // every 40 milliseconds, read the analog value from the ADC
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
     analogBufferIndex++;
     if (analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if (millis() - printTimepoint > 800U) {
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVoltage = averageVoltage / compensationCoefficient; // temperature compensation
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                 - 255.86 * compensationVoltage * compensationVoltage 
                 + 857.39 * compensationVoltage) * 0.5; // convert voltage value to tds value
      tds = tdsValue;
   }
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (int i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void sensorPh() {
  static unsigned long samplingTime = millis();
  if (millis() - samplingTime > samplingInterval) {
    samplingTime = millis();
    pHArray[pHArrayIndex++] = analogRead(ph_Pin);
    if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;

    voltage = avergearray(pHArray, ArrayLenth) * VREF / 1024.0;
    pHValue = 3.5 * voltage + Offset;
    ph = pHValue;

    Serial.print("pH Voltage: ");
    Serial.println(voltage, 3);
    Serial.print("pH Value: ");
    Serial.println(pHValue, 2);
  }
}

double avergearray(int* arr, int number) {
  long sum = 0;
  for (int i = 0; i < number; i++) {
    sum += arr[i];
  }
  return (double)sum / number;
}

void sensorKekeruhan() {
  int sensorValue = analogRead(A2); // read the input on analog pin A2
  float voltage = sensorValue * (5.0 / 1024.0); // convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
  kekeruhan = (voltage - 4.1739) / (-0.0022);
  if (kekeruhan < 0) {
    kekeruhan = 0;
  }
  Serial.print("Kekeruhan Voltage: ");
  Serial.println(voltage, 3);
  Serial.print("Kekeruhan Value: ");
  Serial.println(kekeruhan);
}
