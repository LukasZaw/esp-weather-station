#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <HttpClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <MQ135.h>

#include "secrets.h" // Wifi credentials

#define MQ135_PIN A0
#define RAIN_SENSOR_PIN D5
#define WIND_SENSOR_PIN D6


const char* host = "http://heroku-nodejs-8802db802387.herokuapp.com"; // URL backendu

Adafruit_BME280 bme;
BH1750 lightMeter;
MQ135 mq135_sensor = MQ135(MQ135_PIN);

//Reading Error estimations
float tempError = -0.5; // -1.0
float humiError = 0;    // ok?
float presError = 32;   // 33

volatile uint32_t windCount = 0;
volatile uint32_t lastEdgeUs = 0;

void IRAM_ATTR countWind() {
  unsigned long now = micros();
  if (now - lastEdgeUs > 4000) { // 4 ms debounce
    windCount++;
    lastEdgeUs = now;
  }
}

unsigned long lastPing = 0;
const unsigned long pingInterval = 5 * 60 * 1000; // Interval 5 min


// utrzymanie polaczenia Wi-Fi
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    yield();
  }
}


// --------------------------------------------

void setup() {
  Serial.begin(9600);
  Wire.begin();

  ensureWiFi();

  unsigned status;
    
  // default settings
  status = bme.begin(0x76);  
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
      Serial.println("BME280 not found!");
      while (1) delay(10);
  }

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 not found!");
  }

  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(WIND_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_SENSOR_PIN), countWind, FALLING);

  Serial.println("--- Czujniki aktywne ---");
}

// --------------------------------------------



void loop() {
  ensureWiFi();

  if (WiFi.status() == WL_CONNECTED) {
    sendBME280();     delay(100);
    sendBH1750();     delay(100);
    sendMQ135();      delay(100);
    sendRainSensor(); delay(100);
    sendWindSensor();
  }

  unsigned long now = millis();
  if (now - lastPing >= pingInterval) {
    sendPing();
    lastPing = now;
  }

  delay(40000 - (5*100));
}

// --------------------------------------------

void sendPing() {
  postData("/api/ping", "{\"device_id\":\"esp8266_primary\"}");
}

void sendBME280() {
  float temp = bme.readTemperature() + tempError;
  float hum = bme.readHumidity() + humiError;
  float pres = (bme.readPressure() / 100.0F) + presError;

  String json = "{\"temperature\":" + String(temp, 2) + 
                ",\"humidity\":" + String(hum, 2) + 
                ",\"pressure\":" + String(pres, 2) + "}";
  postData("/api/bme280", json);
}

void sendBH1750() {
  float lux = lightMeter.readLightLevel();
  String json = "{\"light_level\":" + String(lux, 2) + "}";
  postData("/api/bh1750", json);
}

void sendMQ135() {
  float air_quality = mq135_sensor.getPPM();
  String json = "{\"air_quality_index\":" + String(air_quality, 2) + "}";
  postData("/api/mq135", json);
}

void sendRainSensor() {
  int rain = digitalRead(RAIN_SENSOR_PIN);
  String json = "{\"is_raining\":" + String(rain == LOW ? "true" : "false") + "}";
  postData("/api/yl83", json);
}

void sendWindSensor() {
  const unsigned long MEAS_MS = 20000;   // 20 s okno
  const float KMH_PER_HZ = 3.2f;        // 1 Hz = 3.2 km/h

  // Zerowanie licznika i mierzenie przez MEAS_MS
  noInterrupts();
  windCount = 0;
  interrupts();

  delay(MEAS_MS);

  // Czytanie wyniku
  uint32_t pulses;
  noInterrupts();
  pulses = windCount;
  interrupts();

  float seconds = MEAS_MS / 1000.0f;
  float hz = pulses / seconds; // impulsy/s
  float speed_kmh = hz * KMH_PER_HZ;
  float speed_ms = speed_kmh / 3.6f;

  // Wygładzanie (EMA)
  static float ema_ms = NAN;
  const float alpha = 0.5f;   //wygladzanie
  if (isnan(ema_ms)) ema_ms = speed_ms; else ema_ms = alpha * speed_ms + (1 - alpha) * ema_ms;

  String json = "{\"wind_speed\":" + String(ema_ms, 2) + "}";
  postData("/api/anemometer", json);
}

// --------------------------------------------



void postData(String endpoint, String jsonData) {
  WiFiClient client;
  HTTPClient http;
  String url = String(host) + endpoint;

  if (!http.begin(client, url)) {
    Serial.println(F("HTTP begin failed"));
    return;
  }
  http.setTimeout(8000); // 8s
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(jsonData);
  Serial.printf("POST %s -> %d\n", endpoint.c_str(), code);
  if (code <= 0) {
    Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
  }
  http.end();
}
