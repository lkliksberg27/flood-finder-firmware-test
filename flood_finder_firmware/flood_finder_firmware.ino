#define SUPABASE_URL  "https://ygoentpdkizwwskagacw.supabase.co"
#define SUPABASE_KEY  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Inlnb2VudHBka2l6d3dza2FnYWN3Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzY3NDQxMjYsImV4cCI6MjA5MjMyMDEyNn0.kUISJBxYocLYx6FUg3TDWf7Q6fpal-KO7gitvxin43Q"

#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define EXT_SDA   33
#define EXT_SCL   34
#define OLED_SDA  17
#define OLED_SCL  18
#define OLED_RST  21
#define TRIG_PIN  2
#define ECHO_PIN  3
#define ENC_CLK   5
#define ENC_DT    6
#define ENC_SW    7
#define VBAT_PIN  1
#define GPS_RX    46
#define GPS_TX    45
#define MPU6050_ADDR 0x68

#define LORA_SS    8
#define LORA_RST   12
#define LORA_DIO1  14
#define LORA_BUSY  13
#define LORA_FREQ  915E6
#define LORA_BW    125E3
#define LORA_SF    7
#define LORA_TX_POWER 14

Adafruit_SSD1306 display(128, 64, &Wire1, -1);
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
WiFiManager wifiManager;

volatile int encoderPos = 0;
int lastEncPos = 0;
int currentPage = 0;
// Pages 0..5 = original. Pages 6,7,8 = AWAKE / SEMI / SLEEP power mode pages.
// Rotation onto a sleep page activates that mode after a 2-second grace.
const int PAGES = 9;
bool bmpOK = false, mpuOK = false, wifiConnected = false, loraOK = false;
bool transmitting = false;
bool isCharging = false;
int txMode = 0;
unsigned long lastSend = 0;
unsigned long lastRead = 0;
unsigned long lastBtn = 0;
const unsigned long SEND_INTERVAL = 10000;   // 10s (was 30s)
const unsigned long READ_INTERVAL = 10000;   // 10s reads in AWAKE mode

// Power mode state (persisted through deep sleep via RTC memory)
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int rtcSleepPage = -1;          // Page we slept from; -1 = wasn't sleeping
const unsigned long SEMI_SLEEP_SEC = 600;     // 10 min between SEMI-sleep reads
const unsigned long MODE_ENTER_DELAY = 2000;  // 2s grace before a sleep page actually sleeps

unsigned long pageEnterMs = 0;

float temperature, pressure, distance, tiltAngle;
int16_t ax, ay, az;
double gpsLat = 0, gpsLng = 0;
int gpsSats = 0;
float battVoltage;
int battPercent;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_DT) == digitalRead(ENC_CLK)) encoderPos++;
  else encoderPos--;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  bootCount++;

  // Wake-cause routing:
  //   EXT0 (encoder press) -> force AWAKE, land on SENS page
  //   TIMER (from SEMI sleep) -> resume SEMI page, take one reading, sleep again
  esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
  if (wake == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("[WAKE] Encoder button -> AWAKE page");
    currentPage = 0;
    rtcSleepPage = -1;
  } else if (wake == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("[WAKE] Timer -> resume SEMI page");
    currentPage = (rtcSleepPage >= 0) ? rtcSleepPage : 7;
  }

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW); delay(50);
  digitalWrite(OLED_RST, HIGH); delay(50);
  Wire1.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (wake == ESP_SLEEP_WAKEUP_TIMER) {
    showMsg("Semi-sleep wake", "Reading sensors...");
  } else if (wake == ESP_SLEEP_WAKEUP_EXT0) {
    showMsg("WOKE UP", "Mode: AWAKE");
  } else {
    showMsg("FLOOD FINDER v2", "Booting...");
  }

  Wire.begin(EXT_SDA, EXT_SCL);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, FALLING);

  bmpOK = bmp.begin_I2C(0x77, &Wire);
  if (bmpOK) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_127);
    bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
  mpuOK = Wire.available() && (Wire.read() == 0x68);

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

  // WiFiManager — creates "Flood Finder" hotspot if no saved WiFi.
  // Connect from your phone, captive portal lets you pick a network.
  showMsg("WiFi Setup", "Connect phone to:");
  display.println("\"Flood Finder\"");
  display.println("to configure WiFi");
  display.display();

  wifiManager.setConfigPortalTimeout(120);
  wifiManager.setAPCallback([](WiFiManager *mgr) {
    showMsg("Connect phone to:", "\"Flood Finder\"");
  });

  wifiConnected = wifiManager.autoConnect("Flood Finder");

  if (wifiConnected) showMsg("WiFi Connected!", WiFi.localIP().toString());
  else showMsg("WiFi skipped", "LoRa mode available");
  delay(1500);

  // If we woke from the SEMI-sleep timer, do one reading + send and go back to sleep.
  // (loop() never runs in this path.)
  if (wake == ESP_SLEEP_WAKEUP_TIMER) {
    readSensors();
    readGPS();
    readBattery();
    if (txMode == 0 && wifiConnected) sendToSupabase();
    else if (txMode == 1 && loraOK) sendViaLoRa();
    showMsg("Semi-sleep", "Back to sleep...");
    delay(800);
    rtcSleepPage = 7;
    goToDeepSleep(SEMI_SLEEP_SEC);
  }
  pageEnterMs = millis();
}

void loop() {
  // SEMI page (7): after 2s grace, take one reading + send, deep sleep 10 min.
  // Timer wake comes back through setup() to repeat.
  if (currentPage == 7 && millis() - pageEnterMs > MODE_ENTER_DELAY) {
    readSensors(); readGPS(); readBattery();
    if (txMode == 0 && wifiConnected) sendToSupabase();
    else if (txMode == 1 && loraOK) sendViaLoRa();
    showMsg("SEMI SLEEP", "Sleep 10 min...");
    delay(1500);
    rtcSleepPage = 7;
    goToDeepSleep(SEMI_SLEEP_SEC);
  }
  // SLEEP page (8): after 2s grace, full deep sleep, only knob press wakes.
  if (currentPage == 8 && millis() - pageEnterMs > MODE_ENTER_DELAY) {
    showMsg("FULL SLEEP", "Press knob to wake");
    delay(1500);
    rtcSleepPage = 8;
    goToDeepSleep(0);
  }

  // AWAKE: throttle sensor reads to once every READ_INTERVAL (10s)
  if (millis() - lastRead > READ_INTERVAL || lastRead == 0) {
    readSensors();
    readGPS();
    readBattery();
    lastRead = millis();
  }
  handleEncoder();
  wifiConnected = (WiFi.status() == WL_CONNECTED);

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
    case 6: pageAwake(); break;
    case 7: pageSemi(); break;
    case 8: pageSleep(); break;
  }

  display.setCursor(0, 57);
  display.print("[");
  for (int i = 0; i < PAGES; i++)
    display.print(i == currentPage ? "*" : "-");
  display.print("] ");
  const char* n[] = {"SENS","GPS","SYS","MODE","WIFI","TX","AWAKE","SEMI","SLEEP"};
  display.print(n[currentPage]);
  display.display();

  if (transmitting && millis() - lastSend > SEND_INTERVAL) {
    if (txMode == 0 && wifiConnected) sendToSupabase();
    else if (txMode == 1 && loraOK) sendViaLoRa();
    lastSend = millis();
  }

  delay(150);
}

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
  // Heltec V4: 100/(100+390) = 0.204 divider on VBAT, gated by ADC_CTRL (GPIO 37).
  // V_battery = V_adc * 4.9.  analogReadMilliVolts uses factory ADC calibration.
  analogSetAttenuation(ADC_11db);
  uint32_t v_adc_mV = analogReadMilliVolts(VBAT_PIN);
  battVoltage = (v_adc_mV / 1000.0) * 4.9;
  battPercent = constrain((int)((battVoltage - 3.0) / (4.2 - 3.0) * 100), 0, 100);
  isCharging = (battVoltage > 4.5);
}

void handleEncoder() {
  if (encoderPos != lastEncPos) {
    int diff = encoderPos - lastEncPos;
    int prevPage = currentPage;
    currentPage = (currentPage + diff + PAGES) % PAGES;
    if (currentPage != prevPage) pageEnterMs = millis();
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
  if (btn) txMode = (txMode + 1) % 2;

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
    display.println();
    display.println("Press to reset WiFi");
    if (buttonPressed()) {
      wifiManager.resetSettings();
      showMsg("WiFi reset!", "Restarting...");
      delay(1000);
      ESP.restart();
    }
  } else {
    display.println("Not connected");
    display.println();
    display.println("Press to start");
    display.println("WiFi setup portal");
    if (buttonPressed()) {
      showMsg("Connect phone to:", "\"Flood Finder\"");
      wifiManager.startConfigPortal("Flood Finder");
      wifiConnected = (WiFi.status() == WL_CONNECTED);
    }
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
    display.println("  Every 10 seconds");
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

void sendViaLoRa() {
  if (!loraOK) return;

  String pkt = "FF|";
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

// === POWER MODE PAGES ===
// Three pages — rotating onto a sleep page activates that mode after 2s grace.
//   AWAKE : normal sensor view, screen on, 10s read interval (default behavior).
//   SEMI  : 2s grace then sleeps 10min, wakes on timer, reads + sends, sleeps again.
//   SLEEP : 2s grace then full deep sleep, only knob press wakes.
// Press knob from ANY sleep state -> wake to SENS page (page 0).
void pageAwake() {
  display.println("=== AWAKE MODE ===");
  display.println();
  display.println("  Always on");
  display.println("  Reads every 10s");
  display.println("  Screen stays on");
  display.println();
  display.println("(turn to leave)");
}

void pageSemi() {
  display.println("=== SEMI SLEEP ===");
  display.println();
  display.println("  Sleeps 10 min");
  display.println("  Wakes for 1 reading");
  display.println("  Sends + sleeps again");
  int rem = (MODE_ENTER_DELAY - (millis() - pageEnterMs) + 999) / 1000;
  if (rem < 0) rem = 0;
  display.print("Entering in ");
  display.print(rem);
  display.println("s...");
}

void pageSleep() {
  display.println("=== FULL SLEEP ===");
  display.println();
  display.println("  Everything OFF");
  display.println("  Press knob to wake");
  display.println();
  int rem = (MODE_ENTER_DELAY - (millis() - pageEnterMs) + 999) / 1000;
  if (rem < 0) rem = 0;
  display.print("Entering in ");
  display.print(rem);
  display.println("s...");
}

// === DEEP SLEEP HELPER ===
// secs > 0: wake on timer OR encoder press. secs == 0: only encoder press wakes.
void goToDeepSleep(int secs) {
  Serial.printf("[SLEEP] entering deep sleep for %d sec (button wake = AWAKE)\n", secs);
  Serial.flush();

  display.clearDisplay();
  display.display();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  LoRa.end();

  // Encoder switch on GPIO 7 wakes the device (RTC-capable on ESP32-S3)
  rtc_gpio_pullup_en((gpio_num_t)ENC_SW);
  rtc_gpio_pulldown_dis((gpio_num_t)ENC_SW);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)ENC_SW, 0);

  if (secs > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
  }

  esp_deep_sleep_start();
}

void showMsg(String line1, String line2) {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(line1);
  display.println(line2);
  display.display();
}
