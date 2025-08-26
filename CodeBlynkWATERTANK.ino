#define BLYNK_TEMPLATE_ID    "TMPL6R8to5deT"
#define BLYNK_TEMPLATE_NAME  "TankMonitorWeb"
#define BLYNK_AUTH_TOKEN     "9DaUpNzpYguEvKrfTZVGV5mv1xSWIf65"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Wi-Fi credentials
const char* ssid = "CHL";
const char* pass = "G4G9o5_CD8";

// Blynk cloud server
const char* blynkServer = "blynk.cloud";
const uint16_t blynkPort = 80;

// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Ultrasonic A02YYUW setup on Serial2
#define RXD2_PIN 17
#define TXD2_PIN 16
HardwareSerial sensorSerial(2);

unsigned char packet[4];
float distanceCm = 0;

// Tank calibration (in cm)
const float fullDistance        = 32.8;    // 100% (updated here)
const float emptyDistance       = 90.0;    // 0%
const float aboveFullThreshold  = 27.0;    // still show 100%
const float maxSensorRange      = 26.99;   // ABOVE FULL if below this

// Notification flags
bool notifiedLow = false;
bool notifiedFull = false;
bool notifiedAboveFull = false;

void setup() {
  Serial.begin(115200);
  delay(100);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, blynkServer, blynkPort);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Penampung");
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.println("Starting sensor...");
  display.display();
  delay(2000);

  sensorSerial.begin(9600, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
}

void loop() {
  Blynk.run();
  readAndReport();
}

void readAndReport() {
  if (!readSensorFrame()) return;

  float percentFull = 0.0;
  bool isAboveFull = false;

  // Map distance to tank level
  if (distanceCm >= emptyDistance) {
    percentFull = 0.0;
  } else if (distanceCm <= maxSensorRange) {
    isAboveFull = true;
    percentFull = 100.0;
  } else if (distanceCm <= aboveFullThreshold) {
    percentFull = 100.0;
  } else if (distanceCm <= fullDistance) {
    percentFull = 100.0;
  } else {
    percentFull = ((emptyDistance - distanceCm) / (emptyDistance - fullDistance)) * 100.0;
    if (percentFull < 1.0) percentFull = 0.0;
  }

  // — Notifications —
  if (isAboveFull && !notifiedAboveFull) {
    Blynk.logEvent("above_full", "Your Water Tank is Above. Please Check your Tank");
    notifiedAboveFull = true;
    notifiedLow = false;
    notifiedFull = false;
  } else if (percentFull <= 0.0 && !notifiedLow && !isAboveFull) {
    Blynk.logEvent("empty_level", "Your Water Tank is Empty. Please Refill your Tank");
    notifiedLow = true;
    notifiedFull = false;
    notifiedAboveFull = false;
  } else if (percentFull >= 99.0 && !notifiedFull && !isAboveFull) {
    Blynk.logEvent("tank_full", "Water tank is full!");
    notifiedFull = true;
    notifiedLow = false;
    notifiedAboveFull = false;
  } else if (percentFull > 0.0 && percentFull < 99.0 && !isAboveFull) {
    notifiedLow = false;
    notifiedFull = false;
    notifiedAboveFull = false;
  }

  // — OLED Display —
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("WATER LEVEL:");

  display.setTextSize(3);
  display.setCursor(0, 20);
  if (isAboveFull) {
    display.println("ABOVE");
    display.setTextSize(2);
    display.setCursor(0, 48);
    display.println("FULL");
  } else {
    display.print(percentFull, 1);
    display.println("%");
  }
  display.display();

  // — Serial Debug —
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.print(" cm => ");
  if (isAboveFull) {
    Serial.println("ABOVE FULL");
  } else {
    Serial.print(percentFull, 1);
    Serial.println(" %");
  }

  // — Send to Blynk —
  if (isAboveFull) {
    Blynk.virtualWrite(V0, "ABOVE FULL");
  } else {
    Blynk.virtualWrite(V0, percentFull);
  }
}

// Reads one valid 4-byte frame from A02YYUW
bool readSensorFrame() {
  if (!sensorSerial.available()) return false;
  if (sensorSerial.read() != 0xFF) return false;

  uint32_t start = millis();
  while (sensorSerial.available() < 3) {
    if (millis() - start > 10) return false;
  }

  packet[0] = 0xFF;
  for (int i = 1; i < 4; i++) {
    packet[i] = sensorSerial.read();
  }

  if (uint8_t((packet[0] + packet[1] + packet[2]) & 0xFF) != packet[3]) {
    return false;
  }

  distanceCm = ((uint16_t(packet[1]) << 8) | packet[2]) / 10.0;
  return true;
}
