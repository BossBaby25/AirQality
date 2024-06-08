#define BLYNK_TEMPLATE_ID "TMPL6XkdOYF7A"
#define BLYNK_TEMPLATE_NAME "Weather Monitoring"
#define BLYNK_AUTH_TOKEN "QqvVLgfpY3wQMJPOLQ-rmypq4hZfjP9d"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>

// Dust sensor definitions
#define COV_RATIO 0.2 // ug/mmm / mv
#define NO_DUST_VOLTAGE 400 // mv
#define SYS_VOLTAGE 5000 // system voltage in mV

const int iled = 5; // drive the LED of sensor
const int vout = 35; // analog input for dust sensor

float density, voltage;
int adcvalue;

// Function to filter dust sensor readings
int Filter(int m) {
  static int flag_first = 0, _buff[10], sum;
  const int _buff_max = 10;
  int i;

  if(flag_first == 0) {
    flag_first = 1;
    for(i = 0, sum = 0; i < _buff_max; i++) {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  } else {
    sum -= _buff[0];
    for(i = 0; i < (_buff_max - 1); i++) {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];

    i = sum / 10.0;
    return i;
  }
}

// DHT sensor definitions
#define DHTPIN 4 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(DHTPIN, DHTTYPE);
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Meraj"; // your WiFi name
char pass[] = "0987654321"; // your WiFi password

// Wind sensor variables
const byte PulsesPerRevolution = 2; // Number of pulses per revolution
const unsigned long ZeroTimeout = 100000; // Timeout to consider zero speed
const byte numReadings = 2; // Number of readings for averaging
const float WHEEL_CIRCUMFERENCE_METERS = 3.0; // Circumference of the wheel in meters

volatile unsigned long LastTimeWeMeasured = 0;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned long FrequencyRaw = 0;
unsigned long FrequencyReal = 0;
unsigned long RPM = 0;
unsigned int PulseCounter = 1;
unsigned long PeriodSum = 0;

unsigned long LastTimeCycleMeasure = 0;
unsigned long CurrentMicros = 0;
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra = 0;
unsigned long readings[numReadings] = {0};
unsigned long readIndex = 0;
unsigned long total = 0;
unsigned long averageRPM = 0;

float SpeedKmph = 0.0; // Speed in km/h

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);

  lcd.init(); // Initialize the LCD
  lcd.backlight();
  dht.begin();
  pinMode(iled, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event, RISING); // Attach interrupt to pin 2
  delay(1000);
  lcd.clear();
}

void loop() {
  // Dust sensor handling
  digitalWrite(iled, HIGH);
  delayMicroseconds(280);
  adcvalue = analogRead(vout);
  digitalWrite(iled, LOW);
  
  adcvalue = Filter(adcvalue);

  voltage = (SYS_VOLTAGE / 1024.0) * adcvalue * 11; // convert to voltage

  if(voltage >= NO_DUST_VOLTAGE) {
    voltage -= NO_DUST_VOLTAGE;
    density = voltage * COV_RATIO;
  } else {
    density = 0;
  }

  Serial.print("Dust: ");
  Serial.print(density);
  Serial.println(" ug/m3");

  lcd.setCursor(0, 2);
  lcd.print("Dust: ");
  lcd.print(density);
  lcd.print(" ug/m3");

  // Wind sensor handling
  LastTimeCycleMeasure = LastTimeWeMeasured;
  CurrentMicros = micros();

  if (CurrentMicros < LastTimeCycleMeasure) {
    LastTimeCycleMeasure = CurrentMicros;
  }

  FrequencyRaw = 10000000000UL / PeriodAverage;

  if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra) {
    FrequencyRaw = 0;
    ZeroDebouncingExtra = 2000;
  } else {
    ZeroDebouncingExtra = 0;
  }

  FrequencyReal = FrequencyRaw / 10000;
  RPM = FrequencyRaw / PulsesPerRevolution * 60 / 10000;

  total = total - readings[readIndex];
  readings[readIndex] = RPM;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  averageRPM = total / numReadings;

  SpeedKmph = (averageRPM * WHEEL_CIRCUMFERENCE_METERS) / 1000;

  Serial.print("Speed: ");
  Serial.print(SpeedKmph);
  Serial.println(" km/h");

  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(SpeedKmph);
  lcd.print(" km/h");

  // DHT sensor handling
  float temperature = dht.readTemperature(); // Read temperature
  float humidity = dht.readHumidity();       // Read humidity

  Serial.print("T: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.println(humidity);

  lcd.setCursor(0, 1);
  lcd.print("T: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(7, 1);
  lcd.print("H: ");
  lcd.print(humidity);
  lcd.print("%");

  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, humidity);
  Blynk.virtualWrite(V0, SpeedKmph);
  Blynk.virtualWrite(V3, density);

  Blynk.run();
  delay(1000); // Short delay for readability and stability
}

void Pulse_Event() {
  unsigned long currentMicros = micros();
  PeriodBetweenPulses = currentMicros - LastTimeWeMeasured;
  LastTimeWeMeasured = currentMicros;

  if (PulseCounter >= AmountOfReadings) {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter = 1;
    PeriodSum = PeriodBetweenPulses;

    int RemappedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
    RemappedAmountOfReadings = constrain(RemappedAmountOfReadings, 1, 10);
    AmountOfReadings = RemappedAmountOfReadings;
  } else {
    PulseCounter++;
    PeriodSum += PeriodBetweenPulses;
  }
}
