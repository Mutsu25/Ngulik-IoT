#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin definitions
#define ONE_WIRE_BUS 4
#define TdsSensorPin A1
#define ph_Pin A0
#define TurbiditySensorPin A2

// Constants
#define VREF 5.0
#define SCOUNT 30
#define Offset 0.00
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40

// Sensor objects
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Variables
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
float temperature = 25.0;
int tdsValue = 0;

int pHArray[ArrayLenth];
int pHArrayIndex = 0;
float voltage, pHValue;

float suhu;
float ph;
int tds;
int kekeruhan;

void setup() {
  Serial.begin(9600);
  sensors.begin();
  pinMode(TdsSensorPin, INPUT);
  pinMode(ph_Pin, INPUT);
  pinMode(TurbiditySensorPin, INPUT);

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
  updateSensors();

  String request = "";
  while (Serial.available() > 0) {
    request += char(Serial.read());
  }
  request.trim();
  if (request == "Ya") {
    String data = String(suhu) + "#" + String(tds) + "#" + String(ph) + "#" + String(kekeruhan);
    Serial.println(data);
  }

  displayData();
  delay(1000);
}

void updateSensors() {
  sensorSuhu();
  sensorTds();
  sensorPh();
  sensorKekeruhan();
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
  lcd.print("Kekeruhan: ");
  lcd.print(kekeruhan);
}

void sensorSuhu() {
  sensors.requestTemperatures();
  suhu = sensors.getTempCByIndex(0);
}

void sensorTds() {
   static unsigned long sampleTime = millis();
   if (millis() - sampleTime > 40) {
       sampleTime = millis();
       analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
       analogBufferIndex = (analogBufferIndex + 1) % SCOUNT;
   }

   static unsigned long printTime = millis();
   if (millis() - printTime > 800) {
       printTime = millis();

       // Copy buffer for processing
       for (int i = 0; i < SCOUNT; i++) {
           analogBufferTemp[i] = analogBuffer[i];
       }

       // Median filtering and voltage calculation
       float averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * VREF / 1024.0;

       // Debug: Print raw voltage
       Serial.print("TDS Raw Voltage (V): ");
       Serial.println(averageVoltage, 3); // Print voltage with 3 decimal places

       float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
       float compensationVoltage = averageVoltage / compensationCoefficient;

       // Debug: Print compensation details
       Serial.print("Compensation Coefficient: ");
       Serial.println(compensationCoefficient, 3);
       Serial.print("Compensated Voltage (V): ");
       Serial.println(compensationVoltage, 3);

       // TDS calculation
       tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
                   - 255.86 * compensationVoltage * compensationVoltage
                   + 857.39 * compensationVoltage) * 0.5;

       // Adjust sensitivity and constrain TDS value
       tdsValue *= 0.25; // Reduce sensitivity (adjust this factor as needed)
       tdsValue = constrain(tdsValue, 0, 200); // Limit TDS value to a max of 200

       // Debug: Print final TDS value
       Serial.print("TDS Value (PPM): ");
       Serial.println(tdsValue);

       tds = tdsValue;
   }
}


int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  memcpy(bTab, bArray, iFilterLen * sizeof(int));
  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int temp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = temp;
      }
    }
  }
  return (iFilterLen & 1) ? bTab[iFilterLen / 2] : (bTab[iFilterLen / 2 - 1] + bTab[iFilterLen / 2]) / 2;
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
    int sensorValue = analogRead(A2); // Read the input on analog pin A2
    float voltage = sensorValue * (5.0 / 1024.0); // Convert to voltage
    Serial.print("Raw Value: ");
    Serial.println(sensorValue);
    Serial.print("Voltage: ");
    Serial.println(voltage);

    // Example calculation (adjust based on your sensor datasheet)
    kekeruhan = (voltage - 4.1739) / -0.0022; // Formula for NTU conversion

    // Handle negative values and constrain the output
    if (kekeruhan < 0) {
        kekeruhan = 0; // Ensure no negative values
    }

    //Serial.print("Turbidity (NTU): ");
    //Serial.println(kekeruhan);
}
