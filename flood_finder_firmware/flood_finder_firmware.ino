#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include "secrets.h"

// === PIN DEFINITIONS ===
#define EXT_SDA   47
#define EXT_SCL   48
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define TRIG_PIN  2
#define ECHO_PIN  3
#define ENC_CLK   5
#define ENC_DT    6
#define ENC_SW    7
#define VBAT_PIN  1
#define ADC_CTRL  37
#define VEXT_PIN  36
#define GPS_EN    34
#define GPS_RX    39
#define GPS_TX    38
#define MPU6050_ADDR 0x68

// Heltec V4 LoRa SX1262 pins
#define LORA_SS    8
#define LORA_RST   12
#define LORA_DIO1  14
#define LORA_BUSY  13
#define LORA_FREQ  915E6
#define LORA_BW    125E3
#define LORA_SF    7
#define LORA_TX_POWER 14

// === OBJECTS ===
Adafruit_SSD1306 display(128, 64, &Wire1, -1);
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

// === STATE ===
volatile int encoderPos = 0;
int lastEncPos = 0;
int currentPage = 0;
const int PAGES = 6;
bool bmpOK = false, mpuOK = false, wifiConnected = false, loraOK = false;
bool transmitting = false;
bool isCharging = false;
int txMode = 0;  // 0 = WiFi, 1 = LoRa
unsigned long lastSend = 0;
unsigned long lastBtn = 0;
unsigned long btnHoldStart = 0;
const unsigned long SEND_INTERVAL = 30000;

// Sensor data
float temperature, pressure, distance, tiltAngle;
int16_t ax, ay, az;
double gpsLat = 0, gpsLng = 0;
int gpsSats = 0;
float battVoltage;
int battPercent;

// === ENCODER ISR ===
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_DT) == digitalRead(ENC_CLK)) encoderPos++;
  else encoderPos--;
}

// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(500);

  // OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW); delay(50);
  digitalWrite(OLED_RST, HIGH); delay(50);
  Wire1.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  showMsg("FLOOD FINDER v2", "Booting...");

  // Sensor I2C
  Wire.begin(EXT_SDA, EXT_SCL);

  // Vext + GPS power control (V4 board internal)
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);   // enable GPS/peripherals
  pinMode(GPS_EN, OUTPUT);
  digitalWrite(GPS_EN, LOW);     // enable L76K
  pinMode(ADC_CTRL, OUTPUT);
  digitalWrite(ADC_CTRL, HIGH);  // enable battery voltage divider

  // GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, FALLING);

  // BMP390 max precision
  bmpOK = bmp.begin_I2C(0x77, &Wire);
  if (bmpOK) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_127);
    bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);
  }

  // MPU-6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  mpuOK = Wire.available() && (Wire.read() == 0x68);

  // LoRa
  showMsg("Init LoRa...", "915 MHz");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO1);
  loraOK = LoRa.begin(LORA_FREQ);
  if (loraOK) {
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setTxPower(LORA_TX_POWER);
  }

  showMsg("BMP:" + String(bmpOK?"OK":"FAIL"),
          "MPU:" + String(mpuOK?"OK":"FAIL") +
          " LoRa:" + String(loraOK?"OK":"FAIL"));
  delay(1500);

  // WiFi
  showMsg("Connecting WiFi", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); tries++;
  }
  wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected) {
    showMsg("WiFi Connected!", WiFi.localIP().toString());
  } else {
    showMsg("WiFi FAILED", "LoRa mode available");
  }
  delay(1500);
}

// === MAIN LOOP ===
void loop() {
  readSensors();
  readGPS();
  readBattery();
  handleEncoder();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (currentPage) {
    case 0: pageSensors(); break;
    case 1: pageGPS(); break;
    case 2: pageSystem(); break;
    case 3: pageMode(); break;
    case 4: pageWifi(); break;
    case 5: pageTransmit(); break;
  }

  // Page dots
  display.setCursor(0, 57);
  display.print("[");
  for (int i = 0; i < PAGES; i++)
    display.print(i == currentPage ? "*" : "-");
  display.print("] ");
  const char* n[] = {"SENS","GPS","SYS","MODE","WIFI","TX"};
  display.print(n[currentPage]);
  display.display();

  // Auto-send
  if (transmitting && millis() - lastSend > SEND_INTERVAL) {
    if (txMode == 0 && wifiConnected) sendToSupabase();
    else if (txMode == 1 && loraOK) sendViaLoRa();
    lastSend = millis();
  }

  delay(150);
}

// === SENSOR READS ===
void readSensors() {
  if (bmpOK && bmp.performReading()) {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;
  }
  if (mpuOK) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)6);
    if (Wire.available() == 6) {
      ax = (Wire.read()<<8)|Wire.read();
      ay = (Wire.read()<<8)|Wire.read();
      az = (Wire.read()<<8)|Wire.read();
    }
  }
  tiltAngle = atan2((float)ax, (float)az) * 180.0 / PI;

  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 30000);
  distance = d * 0.034 / 2.0;
}

void readGPS() {
  while (GPSSerial.available()) gps.encode(GPSSerial.read());
  if (gps.location.isValid()) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
  }
  gpsSats = gps.satellites.value();
}

void readBattery() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  battVoltage = analogRead(VBAT_PIN) / 4095.0 * 3.3 * 2.0;
  battPercent = constrain((int)((battVoltage - 3.0) / 1.2 * 100), 0, 100);
  isCharging = (battVoltage > 4.5);
}

// === ENCODER ===
void handleEncoder() {
  if (encoderPos != lastEncPos) {
    int diff = encoderPos - lastEncPos;
    currentPage = (currentPage + diff + PAGES) % PAGES;
    lastEncPos = encoderPos;
  }
}

bool buttonPressed() {
  if (digitalRead(ENC_SW) == LOW && millis() - lastBtn > 300) {
    lastBtn = millis();
    return true;
  }
  return false;
}

// === PAGES ===
void pageSensors() {
  display.println("=== SENSORS ===");
  display.print(temperature, 2); display.println(" C");
  display.print(pressure, 4); display.println(" hPa");
  display.print("Tilt: "); display.print(tiltAngle, 1); display.println(" deg");
  display.print("Dist: "); display.print(distance, 1); display.println(" cm");
  display.print("Enc: "); display.println(encoderPos);
}

void pageGPS() {
  display.println("=== GPS ===");
  if (gpsLat != 0) {
    display.print("Lat: "); display.println(gpsLat, 6);
    display.print("Lng: "); display.println(gpsLng, 6);
  } else {
    display.println("Searching...");
  }
  display.print("Sats: "); display.println(gpsSats);
  display.print("Age: ");
  display.print(gps.location.age()); display.println("ms");
}

void pageSystem() {
  display.println("=== SYSTEM ===");
  display.print("Bat: "); display.print(battVoltage, 2);
  display.print("V "); display.print(battPercent); display.println("%");
  display.print("Chrg: "); display.println(isCharging ? "YES (USB)" : "No");
  display.print("BMP: "); display.print(bmpOK ? "OK" : "ERR");
  display.print(" MPU: "); display.println(mpuOK ? "OK" : "ERR");
  display.print("LoRa: "); display.print(loraOK ? "OK" : "ERR");
  display.print(" WiFi: "); display.println(wifiConnected ? "OK" : "ERR");
}

void pageMode() {
  bool btn = buttonPressed();
  display.println("=== TX MODE ===");
  display.println();

  if (btn) {
    txMode = (txMode + 1) % 2;
  }

  if (txMode == 0) {
    display.println("  > WiFi Direct");
    display.println("    LoRa -> Cellular");
    display.println();
    display.print("  WiFi: ");
    display.println(wifiConnected ? "Connected" : "No connection");
  } else {
    display.println("    WiFi Direct");
    display.println("  > LoRa -> Cellular");
    display.println();
    display.print("  LoRa: ");
    display.println(loraOK ? "Ready 915MHz" : "Not available");
  }
  display.println();
  display.println("  Press to switch");
}

void pageWifi() {
  display.println("=== WIFI ===");
  if (wifiConnected) {
    display.println(WiFi.SSID());
    display.print("IP: "); display.println(WiFi.localIP());
    display.print("RSSI: "); display.print(WiFi.RSSI()); display.println("dB");
  } else {
    display.println("Not connected");
    if (buttonPressed()) {
      showMsg("Reconnecting...", "");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      int t = 0;
      while (WiFi.status() != WL_CONNECTED && t < 15) { delay(500); t++; }
      wifiConnected = (WiFi.status() == WL_CONNECTED);
    }
    display.println("Press to reconnect");
  }
}

void pageTransmit() {
  bool btn = buttonPressed();
  display.println("=== TRANSMIT ===");
  if (btn) transmitting = !transmitting;

  display.println();
  if (transmitting) {
    display.println("  STATUS: ACTIVE");
    display.print("  Via: ");
    display.println(txMode == 0 ? "WiFi" : "LoRa");
    display.println("  Every 30 seconds");
    display.print("  Last: ");
    display.print((millis()-lastSend)/1000); display.println("s ago");
    display.println("  Press to STOP");
  } else {
    display.println("  STATUS: STOPPED");
    display.println();
    display.println("  Press knob to");
    display.println("  START sending data");
  }
}

// === SEND VIA WIFI (Supabase POST) ===
void sendToSupabase() {
  if (!wifiConnected) return;

  HTTPClient http;
  String url = String(SUPABASE_URL) + "/rest/v1/sensor_readings";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", SUPABASE_KEY);
  http.addHeader("Authorization", "Bearer " + String(SUPABASE_KEY));
  http.addHeader("Prefer", "return=minimal");

  String json = "{";
  json += "\"temperature\":" + String(temperature, 4) + ",";
  json += "\"pressure\":" + String(pressure, 4) + ",";
  json += "\"distance_cm\":" + String(distance, 1) + ",";
  json += "\"tilt_angle\":" + String(tiltAngle, 2) + ",";
  json += "\"accel_x\":" + String(ax) + ",";
  json += "\"accel_y\":" + String(ay) + ",";
  json += "\"accel_z\":" + String(az) + ",";
  json += "\"gps_lat\":" + String(gpsLat, 6) + ",";
  json += "\"gps_lng\":" + String(gpsLng, 6) + ",";
  json += "\"battery_voltage\":" + String(battVoltage, 2) + ",";
  json += "\"battery_percent\":" + String(battPercent) + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"encoder_pos\":" + String(encoderPos) + ",";
  json += "\"is_charging\":" + String(isCharging ? "true" : "false") + ",";
  json += "\"tx_mode\":\"wifi\"";
  json += "}";

  int code = http.POST(json);
  Serial.print("Supabase POST: ");
  Serial.println(code);
  http.end();
}

// === SEND VIA LORA ===
void sendViaLoRa() {
  if (!loraOK) return;

  // Pack sensor data into a compact LoRa packet
  // Gateway receives this and forwards to Supabase via cellular
  String pkt = "FF|";  // Flood Finder header
  pkt += String(temperature, 2) + "|";
  pkt += String(pressure, 2) + "|";
  pkt += String(distance, 1) + "|";
  pkt += String(tiltAngle, 1) + "|";
  pkt += String(gpsLat, 6) + "|";
  pkt += String(gpsLng, 6) + "|";
  pkt += String(battVoltage, 2) + "|";
  pkt += String(battPercent) + "|";
  pkt += String(isCharging ? 1 : 0) + "|";
  pkt += String(encoderPos);

  LoRa.beginPacket();
  LoRa.print(pkt);
  LoRa.endPacket();

  Serial.print("LoRa TX: ");
  Serial.println(pkt);
}

// === HELPERS ===
void showMsg(String line1, String line2) {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(line1);
  display.println(line2);
  display.display();
}
