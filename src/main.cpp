// --- Pin Definitions ---
// 1 (GND)=gnd
// 2 (VDD)=3_3v
// 3 (GDO0)=gp7
// 4 (CSN)=gp9
// 5 (SCK)=gp10
// 6 (MOSI)=gp11
// 7 (MISO/GDO1)=gp8
// 8 (DGO2)=gp6
#include <Arduino.h>
#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <math.h>

// --- Pin Definitions ---
constexpr int PIN_GDO0 = 7;   // CC1101 GDO0
constexpr int PIN_CSN = 9;    // Chip select
constexpr int PIN_SCK = 10;   // SPI clock
constexpr int PIN_MOSI = 11;  // SPI MOSI
constexpr int PIN_MISO = 8;   // SPI MISO
constexpr int PIN_GDO2 = 6;  // CC1101 DGO2

// On-board LED Pins
constexpr int LED_PIN_B = 12; // BLUE
constexpr int LED_PIN_G = 2;  // GREEN
constexpr int LED_PIN_R = 1;  // RED

constexpr int BTN1 = 5;
constexpr int BTN2 = 4;

// OLED Pins
constexpr int PIN_OLED_SDA = 18;
constexpr int PIN_OLED_SCL = 19;

#define RADIO_ENABLED 1
#define DISPLAY_ENABLED 0
#define ESP32 1

// --- Config Constants ---
constexpr float SCAN_START_FREQ = 380.0f;
constexpr float SCAN_END_FREQ = 470.0f;
constexpr float RADIO_BIT_RATE_KBPS = 4.8f;
constexpr float RADIO_FREQ_DEV_KHZ = 5.0f;
constexpr float RADIO_RX_BW_KHZ = 135.0f;
constexpr int8_t RADIO_TX_POWER_DBM = 10;

constexpr int RX_BUFFER_SIZE = 64;
constexpr int RSSI_THRESHOLD_DBM = -80;

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// --- Globals ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);
bool oledReady = false;

float scanSteps[SCREEN_WIDTH];
float rssiData[SCREEN_WIDTH];
float avgRssi[SCREEN_WIDTH];
unsigned long lastSmoothMillis = 0;
float globalMaxRssiLog = -130.0;
float globalMaxRssiFreq = 0.0;
unsigned long globalMaxRssiTime = 0;
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_UPDATE_MS = 500;

unsigned long logBaseMillis = 0;
unsigned long logBaseOffset = 0;
const char* LOG_FILENAME = "/scan_log.csv";

// --- Helper Functions ---
void initRadio() {
  Serial.println("Initializing CC1101 with ELECHOUSE library...");
  
  // Set SPI pins first
  ELECHOUSE_cc1101.setSpiPin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  Serial.println("SPI Pins set.");
  ELECHOUSE_cc1101.setGDO(PIN_GDO0, PIN_GDO2);
  Serial.println("GDO0 Pin set.");
  // Initialize
  ELECHOUSE_cc1101.Init(Serial);
  Serial.println("CC1101 Init called.");
  // Set frequency (in MHz)
  ELECHOUSE_cc1101.setMHZ(SCAN_START_FREQ);
  Serial.println("Frequency set.");
  // Configure for OOK/ASK (mode 2)
  ELECHOUSE_cc1101.setModulation(2); // 0=2-FSK, 1=GFSK, 2=ASK/OOK, 3=4-FSK, 4=MSK
  Serial.println("Modulation set to OOK.");
  // Set data rate (in kBaud)
  ELECHOUSE_cc1101.setDRate(RADIO_BIT_RATE_KBPS);
  Serial.println("Data rate set.");
  // Set frequency deviation (in kHz)
  ELECHOUSE_cc1101.setDeviation(RADIO_FREQ_DEV_KHZ);
  Serial.println("Frequency deviation set.");
  // Set receiver bandwidth (in kHz)
  ELECHOUSE_cc1101.setRxBW(RADIO_RX_BW_KHZ);
  Serial.println("RX Bandwidth set.");
  // Set output power (-30 to +10 dBm for 433MHz)
  int powerLevel = 0; // Default (max)
  if (RADIO_TX_POWER_DBM == 10) powerLevel = 10; // 10 dBm
  else if (RADIO_TX_POWER_DBM == 0) powerLevel = 7; // 0 dBm
  else if (RADIO_TX_POWER_DBM == -6) powerLevel = 5; // -6 dBm
  ELECHOUSE_cc1101.setPA(powerLevel);
  Serial.println("TX Power set.");
  // Set to receive mode
  ELECHOUSE_cc1101.SetRx();
  Serial.println("CC1101 Initialized!");

  
  // Test RSSI
  delay(100);
  float rssi = ELECHOUSE_cc1101.getRssi();
  Serial.print("Initial RSSI: ");
  Serial.println(rssi);
  
  digitalWrite(LED_PIN_G, HIGH);
  delay(200);
  digitalWrite(LED_PIN_G, LOW);
}

float getRSSI() {
  // ELECHOUSE library returns RSSI in dBm
  return ELECHOUSE_cc1101.getRssi();
}

void setFrequency(float freqMHz) {
  ELECHOUSE_cc1101.setMHZ(freqMHz);
}

void logData(float freq, float rssi, uint8_t* data, size_t len) {
  File f = LittleFS.open(LOG_FILENAME, "a");
  if (f) {
    f.print(freq, 2);
    f.print(",");
    f.print(millis() + logBaseOffset);
    f.print(",");
    f.print(rssi, 1);
    f.print(",");
    
    if (data != nullptr && len > 0) {
      for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) f.print("0");
        f.print(data[i], HEX);
      }
    } else {
      f.print("N/A");
    }
    f.println();
    f.close();
  }
}

void renderDisplay() {
  if (!oledReady) return;
  // Your existing display code
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Scanning ");
  display.print((int)SCAN_START_FREQ);
  display.print("-");
  display.print((int)SCAN_END_FREQ);
  display.print(" MHz");
  display.display();
}

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println("\n\n--- RADIO ANALYZER with ELECHOUSE ---");
  
  // Setup pins
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_G, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  digitalWrite(LED_PIN_G, HIGH);
  
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  
  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed!");
  } else {
    Serial.println("LittleFS Mounted.");
  }
  
  // Initialize CSV
  if (!LittleFS.exists(LOG_FILENAME)) {
    File f = LittleFS.open(LOG_FILENAME, "w");
    if (f) {
      f.println("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex");
      f.close();
      Serial.println("CSV Created");
    }
  }
  
  // Initialize display if enabled
#if DISPLAY_ENABLED
  Wire1.setSDA(PIN_OLED_SDA);
  Wire1.setSCL(PIN_OLED_SCL);
  Wire1.begin();
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledReady = true;
    Serial.println("OLED Initialized");
  }
#endif
  
  // Initialize radio
#if RADIO_ENABLED
  initRadio();
#else
  Serial.println("RADIO DISABLED");
#endif
  
  // Pre-calculate frequencies
  float step = (SCAN_END_FREQ - SCAN_START_FREQ) / (SCREEN_WIDTH - 1);
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    scanSteps[i] = SCAN_START_FREQ + (i * step);
    rssiData[i] = -150.0f;
    avgRssi[i] = -150.0f;
  }
  
  Serial.println("Setup complete!");
}

void loop() {
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
    Serial.print("Loop alive. Mem: ");
    Serial.println(rp2040.getFreeHeap());
    lastHeartbeat = millis();
  }
  
#if RADIO_ENABLED
  float maxRssiThisSweep = -150.0;
  float maxRssiFreq = 0;
  int rssiLogsThisSweep = 0;
  
  // Sweep all frequencies
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    float currentFreq = scanSteps[i];
    
    // Set frequency
    setFrequency(currentFreq);
    
    // Wait for PLL to lock (critical!)
    delayMicroseconds(1000); // 1ms
    
    // Read RSSI
    float rssi = getRSSI();
    rssiData[i] = rssi;
    
    // Debug output
    Serial.print("Freq ");
    Serial.print(currentFreq);
    Serial.print(": RSSI = ");
    Serial.println(rssi);
    
    // Track max
    if (rssi > maxRssiThisSweep) {
      maxRssiThisSweep = rssi;
      maxRssiFreq = currentFreq;
    }
    
    // Log if above threshold
    if (rssi > RSSI_THRESHOLD_DBM) {
      logData(currentFreq, rssi, nullptr, 0);
      rssiLogsThisSweep++;
    }
    
    // LED feedback
    if (rssi > -60.0) {
      digitalWrite(LED_PIN_B, HIGH);
      delay(10);
      digitalWrite(LED_PIN_B, LOW);
    }
  }
  
  // Update display
  if (millis() - lastDisplayMs >= DISPLAY_UPDATE_MS) {
    lastDisplayMs = millis();
    renderDisplay();
  }
  
  Serial.print("Sweep complete. Max: ");
  Serial.print(maxRssiThisSweep);
  Serial.print(" dBm @ ");
  Serial.print(maxRssiFreq);
  Serial.println(" MHz");
  
  // Simple smoothing
  unsigned long now = millis();
  float dt = (float)(now - lastSmoothMillis);
  float alpha = 0.1; // Simple smoothing factor
  
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    if (rssiData[i] > -140) {
      avgRssi[i] = avgRssi[i] + alpha * (rssiData[i] - avgRssi[i]);
    }
  }
  lastSmoothMillis = now;
  
#else
  // Fake mode
  delay(1000);
  Serial.println("Fake mode - no radio");
#endif
  
  delay(100); // Small delay between sweeps
}