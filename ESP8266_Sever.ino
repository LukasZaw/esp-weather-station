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

#define LED_STATUS D7   // Żółta dioda – status
#define LED_ERROR  D8   // Czerwona dioda – błędy


const char* host = "http://heroku-nodejs-8802db802387.herokuapp.com"; // URL backendu

Adafruit_BME280 bme;
BH1750 lightMeter;
MQ135 mq135_sensor = MQ135(MQ135_PIN);

//Reading Error estimations
float tempError = -0.5; // -1.0?
float humiError = 0;    // ok?
float presError = 33;


volatile uint32_t windCount = 0;
volatile uint32_t lastEdgeUs = 0;
volatile uint8_t lastState = HIGH; // zapamiętanie stanu pinu

void IRAM_ATTR countWind() {
  unsigned long now = micros();
  uint8_t state = digitalRead(WIND_SENSOR_PIN);

  if (state == LOW && lastState == HIGH) {      // przejście z HIGH -> LOW
    if (now - lastEdgeUs > 15000) {             // 15 ms = 66 Hz max (fizycznie realne)
      windCount++;
      lastEdgeUs = now;
    }
  }
  lastState = state;
}


unsigned long lastPing = 0;
const unsigned long pingInterval = 5 * 60 * 1000; // Interval 5 min

void ledStatusBlink() {
  digitalWrite(LED_STATUS, HIGH);
  delay(100);
  digitalWrite(LED_STATUS, LOW);
}

void ledErrorBlink(int times = 3) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_ERROR, HIGH);
    delay(250);
    digitalWrite(LED_ERROR, LOW);
    delay(250);
  }
}

// utrzymanie polaczenia Wi-Fi
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    yield();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi reconnect failed!");
    ledErrorBlink(5);
  }
}


// --------------------------------------------

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_ERROR, LOW);

  ledStatusBlink();

  ensureWiFi();

  unsigned status;
    
  // default settings
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("BME280 not found!");
    ledErrorBlink(5);
  }

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 not found!");
    ledErrorBlink(5);
  }

  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(WIND_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WIND_SENSOR_PIN), countWind, CHANGE);

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
  } else {
    ledErrorBlink(3);
  }

  unsigned long now = millis();
  if (now - lastPing >= pingInterval) {
    sendPing();
    lastPing = now;
  }

  ledStatusBlink();
  delay(40000 - (5*100));
}

// --------------------------------------------

void sendPing() {
  String json = "{\"device_id\":\"esp8266_primary\","
                "\"uptime_ms\":" + String(millis()) + ","
                "\"free_heap\":" + String(ESP.getFreeHeap()) + ","
                "\"wifi_rssi\":" + String(WiFi.RSSI()) + "}";
  postData("/api/ping", json);
}


void sendBME280() {
  float temp = bme.readTemperature() + tempError;
  float hum = bme.readHumidity() + humiError;
  float pres = (bme.readPressure() / 100.0F) + presError;

  if (isnan(temp) || isnan(hum) || isnan(pres)) {
    Serial.println("Błąd odczytu z BME280!");
    ledErrorBlink(2);
    return;
  }

  String json = "{\"temperature\":" + String(temp, 2) + 
                ",\"humidity\":" + String(hum, 2) + 
                ",\"pressure\":" + String(pres, 2) + "}";
  postData("/api/bme280", json);
}

void sendBH1750() {
  float lux = lightMeter.readLightLevel();
  if (isnan(lux) || lux < 0) {
    Serial.println("Błąd odczytu BH1750!");
    ledErrorBlink(2);
    return;
  }

  String json = "{\"light_level\":" + String(lux, 2) + "}";
  postData("/api/bh1750", json);
}

void sendMQ135() {
  float air_quality = mq135_sensor.getPPM();
  if (isnan(air_quality) || air_quality < 0) {
    Serial.println("Błąd odczytu MQ135!");
    ledErrorBlink(2);
    return;
  }

  String json = "{\"air_quality_index\":" + String(air_quality, 2) + "}";
  postData("/api/mq135", json);
}

void sendRainSensor() {
  int rain = digitalRead(RAIN_SENSOR_PIN);
  String json = "{\"is_raining\":" + String(rain == LOW ? "true" : "false") + "}";
  postData("/api/yl83", json);
}

void sendWindSensor() {
  const unsigned long MEAS_MS = 20000;
  const float KMH_PER_HZ = 3.2f;         // 1 Hz = 3.2 km/h

  // Zerowanie licznika impulsów
  noInterrupts();
  windCount = 0;
  interrupts();

  // Pomiar przez określony czas
  delay(MEAS_MS);

  // Pobranie wyniku
  uint32_t pulses;
  noInterrupts();
  pulses = windCount;
  interrupts();

  float seconds = MEAS_MS / 1000.0f;
  float hz = pulses / seconds;           // częstotliwość w Hz
  float speed_kmh = hz * KMH_PER_HZ;     // km/h

  if (speed_kmh > 100) speed_kmh = 0.0;

  if (speed_kmh < 0) {
    Serial.println("Błąd odczytu wiatru!");
    ledErrorBlink(2);
    return;
  }

  // Tworzenie JSON i wysłanie danych
  String json = "{\"wind_speed\":" + String(speed_kmh, 2) + "}";
  postData("/api/anemometer", json);
}

// --------------------------------------------



void postData(String endpoint, String jsonData) {
  WiFiClient client;
  HTTPClient http;
  String url = String(host) + endpoint;

  if (!http.begin(client, url)) {
    Serial.println(F("HTTP begin failed"));
    ledErrorBlink(3);  // błąd rozpoczęcia połączenia
    return;
  }

  http.setTimeout(8000); // 8s
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", api_key);
  

  int code = http.POST(jsonData);
  Serial.printf("POST %s -> %d\n", endpoint.c_str(), code);

  if (code <= 0) {
    Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
    ledErrorBlink(3);
  } 
  else if (code >= 400) {
    Serial.printf("Server returned error code %d\n", code);
    
    if (code == 401) {
      Serial.println("Unauthorized (401) - check API key or token");
      ledErrorBlink(2);  //błąd autoryzacji
    } else {
      ledErrorBlink(4);  //błędy HTTP
    }
  }

  http.end();
}

