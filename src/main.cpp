#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <math.h>

// --- Pin Definitions ---
// 1 (GND)=gnd
// 2 (VDD)=3_3v
// 3 (GDO0)=gp7
// 4 (CSN)=gp9
// 5 (SCK)=gp10
// 6 (MOSI)=gp11
// 7 (MISO/GDO1)=gp8
// 8 (DGO2)=gp6
constexpr int PIN_GDO0 = 7;   // CC1101 GDO0 (data ready)
constexpr int PIN_CSN = 9;    // Chip select (CSN)
constexpr int PIN_SCK = 10;    // SPI clock
constexpr int PIN_MOSI = 11;   // SPI MOSI
constexpr int PIN_MISO = 8;   // SPI MISO
constexpr int PIN_DGO2 = 6;   // DGO2, idk what that is

// On-board Tri-Color LED Pins
constexpr int LED_PIN_B = 12; // On-board LED BLUE
constexpr int LED_PIN_G = 2; // On-board LED GREEN
constexpr int LED_PIN_R = 1; // On-board LED RED

constexpr int BTN1 = 5;
constexpr int BTN2 = 4;       // Used for CSV Reset

// OLED Pins - I2C1 on pins 18/19
constexpr int PIN_OLED_SDA = 18;
constexpr int PIN_OLED_SCL = 19;

// Enable or disable the actual radio hardware at compile time.
// Set to 1 to use real radio scanning, or 0 to run in fake-data mode
// that writes to a separate CSV for testing without the radio.
#define RADIO_ENABLED 1  // 1 = enabled, 0 = disabled

// Enable or disable the OLED display at compile time.
// Set to 1 to initialize and use the OLED, or 0 to keep it off entirely.
#define DISPLAY_ENABLED 0  // 1 = enable OLED, 0 = disable OLED
// Test transmit mode: when enabled the main loop will still iterate
// frequencies but will transmit random payloads at full power on each.
// Only meaningful when RADIO_ENABLED==1.
#define TEST_TX_DEFAULT 0  // 1 = start in TX test mode, 0 = start in receive mode
bool testTxMode = (TEST_TX_DEFAULT != 0); // runtime flag toggled by BTN1

// Timestamp base: use (millis() - logBaseMillis) + logBaseOffset for CSV timestamps
unsigned long logBaseMillis = 0;
unsigned long logBaseOffset = 0;

// --- Config Constants ---
constexpr float SCAN_START_FREQ = 900.0f;
constexpr float SCAN_END_FREQ = 930.0f;
constexpr float RADIO_BIT_RATE_KBPS = 4.8f;
constexpr float RADIO_FREQ_DEV_KHZ = 5.0f;
constexpr float RADIO_RX_BW_KHZ = 135.0f;
constexpr int8_t RADIO_TX_POWER_DBM = 10;
constexpr uint8_t RADIO_PREAMBLE_BITS = 32;

constexpr int RX_BUFFER_SIZE = 64;
constexpr int RSSI_THRESHOLD_DBM = -80; // Only log signals stronger than -80 dBm

#define SCREEN_ADDRESS 0x3C
#define SSD1306_128_64 true
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Enable to probe 907.000 MHz for 5s before each sweep. If no
// signal >= -73 dBm is found the main sweep will be skipped and a
// debug failure will be logged.
#define DEBUG_907 1

// Display layout constants
#define HEADER_HEIGHT 10
#define GRAPH_TOP (HEADER_HEIGHT + 2)
#define GRAPH_HEIGHT (SCREEN_HEIGHT - GRAPH_TOP)
#define BAR_WIDTH 4  // Each bar is 4 pixels wide
#define NUM_BARS (SCREEN_WIDTH / BAR_WIDTH)  // 32 bars

// --- Globals ---
// Provide GDO2 pin so RadioLib (and advanced CC1101 usage) can use the
// second GDO line for interrupts/asynchronous mode.
Module cc1101Module(PIN_CSN, PIN_GDO0, PIN_DGO2, RADIOLIB_NC, SPI1);
CC1101 radio(&cc1101Module);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
bool oledReady = false;

// Counters and runtime state
int consecutiveTxErrors = 0;

float scanSteps[SCREEN_WIDTH]; // Map frequencies to screen X pixels (128 freqs)
float rssiData[SCREEN_WIDTH];
float avgRssi[SCREEN_WIDTH];
uint8_t rx_buffer[RX_BUFFER_SIZE];
unsigned long lastSmoothMillis = 0;
float globalMaxRssiLog = -130.0;
float globalMaxRssiFreq = 0.0;
unsigned long globalMaxRssiTime = 0;
// redLedUntil removed — LED state is centralized via updateLedState()
// legacy timestampOffset removed — using logBaseMillis/logBaseOffset for CSV timestamps
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_UPDATE_MS = 500;
const unsigned long MAX_TRACK_TIME_MS = 300000; // 5 minutes

#if RADIO_ENABLED
const char* LOG_FILENAME = "/scan_log.csv";
#else
const char* LOG_FILENAME = "/scan_log_fake.csv";
#endif

// Storage thresholds
// If the filesystem has less than this many free bytes we'll consider
// storage "low" and enter the pause behavior to avoid crashing due to full flash.
#define STORAGE_MIN_FREE_BYTES (16 * 1024) // 16 KB

// Legacy CSV size constant retained for compatibility (used only when
// the LittleFS free/used API isn't available). Raise the fallback to 1MB to
// avoid false positives on devices with larger flash.
#define CSV_MAX_BYTES (1024 * 1024)  // 1 MB (fallback)


// Forward declarations
void logData(float freq, float rssi, uint8_t* data, size_t len);
void renderDisplay();
void scheduleStatus(const String &msg, unsigned long ms = 1000);

// Forward-declare CSV check used by LED updater
bool isCsvNearFull();
void updateLedState();

// Status overlay
String pendingStatus = "";
unsigned long statusUntil = 0;

// --- Helpers ---

void initRadio() {
  Serial.print("Initializing CC1101... ");
  int16_t state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
     Serial.println("Basic Init SUCCESS.");
     radio.setFrequency(SCAN_START_FREQ);
     radio.setBitRate(RADIO_BIT_RATE_KBPS);
     radio.setFrequencyDeviation(RADIO_FREQ_DEV_KHZ);
     radio.setRxBandwidth(RADIO_RX_BW_KHZ);
     radio.setOutputPower(RADIO_TX_POWER_DBM);
     radio.setOOK(true);
     Serial.println("Radio configured for OOK Scan.");
     
      digitalWrite(LED_PIN_R, LOW);
      digitalWrite(LED_PIN_G, HIGH); delay(200); digitalWrite(LED_PIN_G, LOW);
      if (oledReady) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Radio ready");
        display.display();
      }

  } else {
    Serial.print("INIT FAILED, code ");
    Serial.println(state);
    
    if (oledReady) {
      scheduleStatus(String("Radio Fail: ") + String(state), 2000);
    }
    
    digitalWrite(LED_PIN_R, HIGH);
  }
}

// Render the OLED display with proper formatting
void renderDisplay() {
  if (!oledReady) return;

  display.clearDisplay();
  
  // --- HEADER: "Scanning 900-930 MHz, MAX: 915.5" ---
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // Calculate how long ago we saw the max
  unsigned long timeSinceMax = millis() - globalMaxRssiTime;
  bool maxExpired = (timeSinceMax > MAX_TRACK_TIME_MS);

  // Compose header in a single string and print with println
  {
    String header;
    if (maxExpired || globalMaxRssiLog < -120.0) {
      header = String("Scan ") + String((int)SCAN_START_FREQ) + "-" + String((int)SCAN_END_FREQ) + "MHz";
    } else {
      header = String("Scan ") + String((int)SCAN_START_FREQ) + "-" + String((int)SCAN_END_FREQ) + " MAX:" + String(globalMaxRssiFreq, 1);
    }
    display.println(header);
  }

  // Draw separator line under header
  //display.drawFastHLine(0, HEADER_HEIGHT, SCREEN_WIDTH, SSD1306_WHITE);
  
  // --- GRAPH AREA: 4-pixel-wide bars ---
  // We have 128 frequency samples, average 4 together for each bar (32 bars total)
  
  for (int bar = 0; bar < NUM_BARS; bar++) {
    // Average 4 consecutive frequency samples for this bar
    float sum = 0;
    int count = 0;
    for (int i = 0; i < 4; i++) {
      int sampleIdx = bar * 4 + i;
      if (sampleIdx < SCREEN_WIDTH) {
        float rssi = avgRssi[sampleIdx];
        if (rssi > -140.0f) {
          sum += rssi;
          count++;
        }
      }
    }
    
    if (count == 0) continue; // No valid data for this bar
    
    float avgBarRssi = sum / count;
    
    // Scale: -90 dBm = 0 pixels, each dB higher = 0.5 pixels
    // So -74 dBm = 16 dB above -90 = 8 pixels tall
    const float RSSI_FLOOR = -90.0f;
    const float PIXELS_PER_DB = 0.5f;
    
    float dbAboveFloor = avgBarRssi - RSSI_FLOOR;
    if (dbAboveFloor < 0) dbAboveFloor = 0;
    
    int barHeight = (int)(dbAboveFloor * PIXELS_PER_DB);
    if (barHeight < 1 && avgBarRssi > RSSI_FLOOR) barHeight = 1; // Show at least 1 pixel
    if (barHeight > GRAPH_HEIGHT) barHeight = GRAPH_HEIGHT;
    
    // Draw filled rectangle (4 pixels wide)
    int xPos = bar * BAR_WIDTH;
    int yPos = SCREEN_HEIGHT - barHeight;
    
    if (barHeight > 0) {
      display.fillRect(xPos, yPos, BAR_WIDTH, barHeight, SSD1306_WHITE);
    }
  }
  
  // Draw status overlay if present
  if (pendingStatus.length() > 0 && millis() < statusUntil) {
    // Draw a filled box for contrast
    display.fillRect(0, HEADER_HEIGHT + 2, SCREEN_WIDTH, 10, SSD1306_BLACK);
    display.drawRect(0, HEADER_HEIGHT + 2, SCREEN_WIDTH, 10, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(2, HEADER_HEIGHT + 3);
    display.print(pendingStatus);
  }

  display.display();
  delay(5); // Allow time for display to update
  Serial.println("Display updated.");
}

void scheduleStatus(const String &msg, unsigned long ms) {
  pendingStatus = msg;
  statusUntil = millis() + ms;
}

// Update the tri-color LED to reflect current global state:
// - Blue when storage full/pause
// - Red when in TX test mode
// - Green when in RX/normal mode
void updateLedState() {
  if (isCsvNearFull()) {
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, HIGH);
    return;
  }

  if (testTxMode) {
    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_G, LOW);
    digitalWrite(LED_PIN_B, LOW);
  } else {
    digitalWrite(LED_PIN_R, LOW);
    digitalWrite(LED_PIN_G, HIGH);
    digitalWrite(LED_PIN_B, LOW);
  }
}

void initCSV() {
  
  if (!LittleFS.exists(LOG_FILENAME)) {
    File f = LittleFS.open(LOG_FILENAME, "w");
    if (f) {
      f.println("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex");
      f.close();
      // start new logs anchored to now
      logBaseMillis = millis();
      logBaseOffset = 0;
      Serial.println("CSV Created - timestamp starts at 0");
    } else {
      Serial.println("Failed to create CSV");
    }
  } else {
      Serial.println("CSV exists. Syncing timestamp...");
      File f = LittleFS.open(LOG_FILENAME, "r");
      if (f) {
         if (f.size() > 150) {
             f.seek(f.size() - 150);
             f.readStringUntil('\n');
         }
         
         unsigned long maxTs = 0;
         while (f.available()) {
             String line = f.readStringUntil('\n');
             line.trim();
             if (line.length() == 0) continue;
             
             int c1 = line.indexOf(',');
             int c2 = line.indexOf(',', c1+1);
             if (c1 > 0 && c2 > c1) {
                 String tsPart = line.substring(c1+1, c2);
                 if (isdigit(tsPart.charAt(0))) {
                     unsigned long ts = strtoul(tsPart.c_str(), NULL, 10);
                     if (ts > maxTs) maxTs = ts;
                 }
             }
         }
         f.close();
         if (maxTs > 0) {
           // anchor new timestamps so they continue after maxTs
           logBaseMillis = millis();
           logBaseOffset = maxTs + 100;
         }
         Serial.print("Resuming Log from MS: "); Serial.println(logBaseOffset);
      }
  }
}

void resetCSV() {
  // Remove both possible log files to ensure a full reset
  LittleFS.remove("/scan_log.csv");
  LittleFS.remove("/scan_log_fake.csv");
  logBaseOffset = 0;            // new logs start from 0
  logBaseMillis = millis();     // anchor time for new timestamps
  globalMaxRssiLog = -130.0;
  globalMaxRssiTime = 0;

  // After a reset, make sure the LEDs show the normal idle state
  // (updateLedState will set the appropriate LED based on mode)
  File f = LittleFS.open(LOG_FILENAME, "w");
  if (f) {
    f.println("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex");
    f.close();
    Serial.println("CSV Reset by User - timestamp reset to 0");
  }

  updateLedState();
  if (oledReady) {
      scheduleStatus("CSV RESET!", 800);
  }
  delay(500);
  // Ensure LEDs reflect the current mode (green/red) after reset
  updateLedState();
}

// Human-readable RadioLib error name (best-effort using defines when available)
const char* radioErrorName(int16_t code) {
#ifdef RADIOLIB_ERR_NONE
  if (code == RADIOLIB_ERR_NONE) return "RADIOLIB_ERR_NONE";
#endif
#ifdef RADIOLIB_ERR_ANTENNA
  if (code == RADIOLIB_ERR_ANTENNA) return "RADIOLIB_ERR_ANTENNA";
#endif
#ifdef RADIOLIB_ERR_SPI
  if (code == RADIOLIB_ERR_SPI) return "RADIOLIB_ERR_SPI";
#endif
#ifdef RADIOLIB_ERR_TX_TIMEOUT
  if (code == RADIOLIB_ERR_TX_TIMEOUT) return "RADIOLIB_ERR_TX_TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_RX_TIMEOUT
  if (code == RADIOLIB_ERR_RX_TIMEOUT) return "RADIOLIB_ERR_RX_TIMEOUT";
#endif
#ifdef RADIOLIB_ERR_CRC_MISMATCH
  if (code == RADIOLIB_ERR_CRC_MISMATCH) return "RADIOLIB_ERR_CRC_MISMATCH";
#endif
#ifdef RADIOLIB_ERR_INVALID_STATE
  if (code == RADIOLIB_ERR_INVALID_STATE) return "RADIOLIB_ERR_INVALID_STATE";
#endif
#ifdef RADIOLIB_ERR_NONE
  (void)0; // ensure at least one define is referenced
#endif
  return "UNKNOWN_RADIO_ERROR";
}

// Return true if either CSV is at or above the configured threshold
bool isCsvNearFull() {
  // Prefer checking filesystem free space so we avoid a full flash crash.
  // LittleFS provides totalBytes() and usedBytes() on this platform.
  size_t total = 0;
  size_t used = 0;

  // Defensive: call available APIs if present
  // Some LittleFS ports define totalBytes()/usedBytes(). Try using them when
  // available; otherwise fall back to the CSV-size heuristic.
  #if defined(LittleFS) && defined(LITTLEFS)
    total = LittleFS.totalBytes();
    used = LittleFS.usedBytes();
    if (total == 0) {
      // If the API exists but returned 0, fall back
      Serial.println("LittleFS.totalBytes() returned 0 — falling back to file-size check");
    } else {
      size_t freeBytes = (total > used) ? (total - used) : 0;
      return (freeBytes <= (size_t)STORAGE_MIN_FREE_BYTES);
    }
  #endif

  // Fallback: approximate by summing CSV sizes (previous behavior)
  const char* filesToCheck[] = { "/scan_log.csv", "/scan_log_fake.csv" };
  size_t totalFilesSize = 0;
  for (const char* fn : filesToCheck) {
    if (LittleFS.exists(fn)) {
      File f = LittleFS.open(fn, "r");
      if (f) {
        totalFilesSize += f.size();
        f.close();
      }
    }
  }
  // If total files size exceeds CSV_MAX_BYTES, declare near full (fallback)
  return (totalFilesSize >= CSV_MAX_BYTES);
}

void dumpCSV() {
  Serial.println("\n--- START CSV DUMP ---");
  File f = LittleFS.open(LOG_FILENAME, "r");
  if (f) {
    while (f.available()) {
      rp2040.wdt_reset();
      Serial.write(f.read());
    }
    f.close();
  } else {
    Serial.println("Failed to open CSV");
  }
  Serial.println("\n--- END CSV DUMP ---");
}

// Compute and return a timestamp suitable for CSV rows. Uses millis()-based
// base to provide small timestamps when starting a fresh log and to resume
// after an existing log by anchoring to previous max via logBaseOffset.
unsigned long currentLogTimestamp() {
  return (unsigned long)(millis() - logBaseMillis) + logBaseOffset;
}

// Log to an already-open File reference (avoids open/close per sample)
void logData(File &f, float freq, float rssi, uint8_t* data, size_t len) {
  if (!f) return;
  f.print(freq, 2);
  f.print(",");
  unsigned long ts = currentLogTimestamp();
  f.print(ts);
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
}

// Backwards-compatible wrapper for callers that don't have an open File
void logData(float freq, float rssi, uint8_t* data, size_t len) {
  File f = LittleFS.open(LOG_FILENAME, "a");
  if (f) {
    logData(f, freq, rssi, data, len);
    f.close();
  } else {
    Serial.println("Failed to open CSV for appending");
  }
}

void setup() {
  rp2040.wdt_begin(8000);
  
  pinMode(LED_PIN_G, OUTPUT);
  digitalWrite(LED_PIN_G, HIGH);

  Serial.begin(9600);
  
  delay(2000);
  rp2040.wdt_reset();
  
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  
  // Power On Self Test
  digitalWrite(LED_PIN_G, LOW); 
  digitalWrite(LED_PIN_R, HIGH); delay(100); digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_G, HIGH); delay(100); digitalWrite(LED_PIN_G, LOW);
  digitalWrite(LED_PIN_B, HIGH); delay(100); digitalWrite(LED_PIN_B, LOW);
  digitalWrite(LED_PIN_G, HIGH);

  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 5000)) {
     delay(10);
     rp2040.wdt_reset();
  }
  
  Serial.println("\n\n--- RADIO ANALYZER STARTING v2.3 ---");
  Serial.println("System Life Check: CLOCK TICKING");
  // Helpful tip: CC1101 may draw peaks during TX/RX. If you observe TX errors
  // or resets, try a different USB cable/port or add a 10uF cap across 3.3V/GND
  // close to the radio module to reduce brownout risk.
  Serial.println("TIP: If you see TX errors or resets, try a better USB cable/port or add a 10uF cap across 3.3V-GND on the radio module.");

  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_DGO2, INPUT); // Enable GDO2 for CC1101 interrupts/asynchronous data
  Serial.print("GDO2 configured on pin "); Serial.println(PIN_DGO2);
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed!");
  } else {
    Serial.println("LittleFS Mounted.");
  }
  initCSV();
  logData(SCAN_START_FREQ, -120.0, nullptr, 0); 
  
  rp2040.wdt_reset();

#if DISPLAY_ENABLED
  Wire1.setSDA(PIN_OLED_SDA);
  Wire1.setSCL(PIN_OLED_SCL);
  Wire1.begin();

  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledReady = true;
    Serial.print("OLED Initialized at address 0x"); Serial.println(SCREEN_ADDRESS, HEX);
  } else {
    const uint8_t altAddr = (SCREEN_ADDRESS == 0x3C) ? 0x3D : 0x3C;
    Serial.print("SSD1306 init with 0x"); Serial.print(SCREEN_ADDRESS, HEX); Serial.println(" failed, trying alternate");
    if (display.begin(SSD1306_SWITCHCAPVCC, altAddr)) {
      oledReady = true;
      Serial.print("OLED Initialized at address 0x"); Serial.println(altAddr, HEX);
    } else {
      oledReady = false;
      Serial.println(F("SSD1306 allocation failed at both addresses"));
    }
  }
#else
  oledReady = false;
  Serial.println(F("DISPLAY_DISABLED: OLED disabled at compile time."));
  Serial.println("SETUP_LOG: after display init (DISPLAY_DISABLED)");
#endif

  rp2040.wdt_reset();

  if (oledReady) {
      Serial.println("SETUP_LOG: oledReady true - starting display splash");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      
      display.setCursor(10, 5);
      display.setTextSize(2);
      display.println("RADIO");
      display.setCursor(5, 25);
      display.println("ANALYZER");
      
      display.setTextSize(1);
      display.setCursor(30, 45);
      display.print("v2.3");
      display.setCursor(5, 55);
      display.print((int)SCAN_START_FREQ);
      display.print("-");
      display.print((int)SCAN_END_FREQ);
      display.print(" MHz");
      
      display.display();
      
      unsigned long startupStart = millis();
      while (millis() - startupStart < 2000) {
        rp2040.wdt_reset();
        delay(100);
      }
      
      display.clearDisplay();
      display.display();
      Serial.println("SETUP_LOG: display splash complete");
  }
  Serial.println("SETUP_LOG: after display splash and before SPI remap");

  // REMAP SPI PINS TO MATCH HARDWARE (RP2040 / arduino-pico core)
  // Must map SCK/MOSI/MISO and CS before calling SPI.begin() so the
  // CC1101 sees SPI traffic on the correct GPIOs.
  SPI1.setSCK(PIN_SCK);
  Serial.print("SPI remapped to SCK="); Serial.print(PIN_SCK);
  SPI1.setTX(PIN_MOSI);
  Serial.print(" MOSI="); Serial.print(PIN_MOSI);
  SPI1.setRX(PIN_MISO);
  Serial.print(" MISO="); Serial.print(PIN_MISO);
  //SPI.setCS(PIN_CSN); // Ensure CS/SS uses the correct GPIO (GPn)

  SPI1.begin();
  Serial.println("SETUP_LOG: SPI.begin() done");

#if RADIO_ENABLED
  Serial.println("SETUP_LOG: about to initRadio()");
  initRadio();
  Serial.println("SETUP_LOG: returned from initRadio()");
#if DISPLAY_ENABLED
  Wire1.end();
  delay(10);
  Wire1.setSDA(PIN_OLED_SDA);
  Wire1.setSCL(PIN_OLED_SCL);
  Wire1.setClock(5000);
  Wire1.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED re-init failed!");
  } else {
    Serial.println("OLED re-init OK");
  }
#else
  Serial.println("DISPLAY_DISABLED: Skipping OLED re-init.");
#endif

if (testTxMode) {
  Serial.println("TEST_TX_MODE enabled: sending random payloads at each frequency");
  scheduleStatus("TX TEST MODE", 1500);
  // Seed RNG for payload generation - already seeded on toggle, but ensure here too
  randomSeed(millis());
  Serial.println("SETUP_LOG: TEST_TX_MODE active");
}

#else
  Serial.println("RADIO DISABLED: Skipping radio init.");
  scheduleStatus("RADIO DISABLED", 1000);
#endif

  // Pre-calculate frequencies for each screen column (128 samples)
  float step = (SCAN_END_FREQ - SCAN_START_FREQ) / SCREEN_WIDTH;
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    scanSteps[i] = SCAN_START_FREQ + (i * step);
    rssiData[i] = -150.0f;
    avgRssi[i] = -150.0f;
  }
  lastSmoothMillis = millis();
  lastDisplayMs = millis();
  // Ensure LED is set to the correct mode on startup
  updateLedState();
  Serial.println("SETUP_LOG: updateLedState done - setup complete");
  delay(500);
}

void checkForSerialCommand() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      dumpCSV();
    }
  }
}

void loop() {
  rp2040.wdt_reset();
  
  checkForSerialCommand();

  // Check Radio Reset Button
  if (digitalRead(BTN1) == LOW) {
      // Toggle TX test mode with the button
      testTxMode = !testTxMode;
      if (testTxMode) {
        Serial.println("Button 1 Pressed: ENTERING TX TEST MODE");
        scheduleStatus("TX MODE", 800);
        // Seed RNG for payloads
        randomSeed(millis());
      } else {
        Serial.println("Button 1 Pressed: ENTERING RX MODE");
        scheduleStatus("RX MODE", 800);
      }

      // Update LED to reflect new mode
      updateLedState();
      delay(500);
  }

  // Check CSV Reset Button
  if (digitalRead(BTN2) == LOW) {
    Serial.println("Button 2 Pressed: Resetting CSV...");
    resetCSV();
    delay(500); 
  }
  
  // Heartbeat log every 2 seconds
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
      Serial.print("Loop alive. RSSI Scan in progress... Mem: ");
      Serial.println(rp2040.getFreeHeap());
      lastHeartbeat = millis();
  }

  // If storage is low, log diagnostic info but do NOT pause scans or change
  // the main LED state — just notify so the user can act.
  if (isCsvNearFull()) {
#if defined(LittleFS) && defined(LITTLEFS)
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    size_t freeBytes = (total > used) ? (total - used) : 0;
    Serial.print("STORAGE WARNING: Free bytes = "); Serial.print(freeBytes);
    Serial.print(" / total = "); Serial.print(total);
    Serial.print(" used = "); Serial.println(used);
#else
    // Fallback: print sizes of known CSV files
    size_t s1 = 0, s2 = 0;
    if (LittleFS.exists("/scan_log.csv")) {
      File f1 = LittleFS.open("/scan_log.csv", "r");
      if (f1) { s1 = f1.size(); f1.close(); }
    }
    if (LittleFS.exists("/scan_log_fake.csv")) {
      File f2 = LittleFS.open("/scan_log_fake.csv", "r");
      if (f2) { s2 = f2.size(); f2.close(); }
    }
    Serial.print("STORAGE WARNING: CSV sizes -> scan_log.csv="); Serial.print(s1);
    Serial.print(" bytes, scan_log_fake.csv="); Serial.print(s2); Serial.println(" bytes");
#endif
    // Brief status message on the display (if available)
    if (oledReady) scheduleStatus("LOW STORAGE", 2000);

    // Do NOT pause scans or force LED to blue; allow scanning to continue.
  }

  // Optional debug probe: check 907.000 MHz for 5s and verify a strong signal
  // If DEBUG_907 is enabled and no signal >= -73 dBm is observed, skip the
  // full sweep to avoid flooding logs with meaningless data.
  float maxRssiThisSweep = -150.0;
  float maxRssiFreq = 0;
  int rssiLogsThisSweep = 0;

#if defined(DEBUG_907) && DEBUG_907
  Serial.println("DEBUG_907: probing 907.000 MHz for 5 seconds...");
  if (RADIO_ENABLED) {
    unsigned long probeEnd = millis() + 5000;
    float debugMax = -200.0f;
    while (millis() < probeEnd) {
      rp2040.wdt_reset();
      radio.setFrequency(907.0f);
      delayMicroseconds(200); // allow PLL/RX to settle
      int16_t s = radio.startReceive();
      if (s == RADIOLIB_ERR_NONE) {
        delayMicroseconds(200);
        float r = radio.getRSSI();
        if (r > debugMax) debugMax = r;
      }
      // small pause to avoid hammering the radio
      delay(50);
    }

    Serial.print("DEBUG_907: max RSSI seen = "); Serial.print(debugMax); Serial.println(" dBm");
    if (debugMax < -73.0f) {
      Serial.println("DEBUG_907: No strong debug signal found; skipping sweep and logging failure");
      // Log a failure row so the host can see the diagnostic
      logData(907.0f, debugMax, nullptr, 0);
      // back off a bit to avoid tight-looping
      delay(2000);
      return; // skip the sweep this iteration (exit loop() so Arduino will call it again)
    } else {
      Serial.println("DEBUG_907: Debug signal found; proceeding with sweep");
    }
  } else {
    Serial.println("DEBUG_907: RADIO_DISABLED - skipping debug probe");
  }
#endif
  
#if RADIO_ENABLED
  // Sweep Loop - scan all 128 frequencies
  // Open CSV once per sweep to avoid repeated open/close overhead.
  File sweepLog = LittleFS.open(LOG_FILENAME, "a");
  if (!sweepLog) {
    Serial.println("Warning: could not open CSV for sweep logging");
  }
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    checkForSerialCommand();
    rp2040.wdt_reset();

    if (digitalRead(BTN2) == LOW) break;

    float currentFreq = scanSteps[i];
    radio.setFrequency(currentFreq);

    // CRITICAL FIX: Give radio time to settle on new frequency
    // Increased from 10us to 200us to allow the PLL & RX path to stabilize
    delayMicroseconds(200); // Allow PLL to lock and RSSI to stabilize

if (testTxMode) {
    // Transmit random payload at full configured power on this frequency
    radio.setOutputPower(RADIO_TX_POWER_DBM);
    const int TX_LEN = 16;
    uint8_t txBuf[TX_LEN];
    for (int b = 0; b < TX_LEN; b++) txBuf[b] = (uint8_t)random(0, 256);

    int16_t txState = radio.transmit(txBuf, TX_LEN);
    if (txState == RADIOLIB_ERR_NONE) {
      // record transmit attempt into CSV (rssi not applicable)
      if (sweepLog) logData(sweepLog, currentFreq, 0.0f, txBuf, TX_LEN);
      consecutiveTxErrors = 0;
    } else {
      Serial.print("TX ERROR: "); Serial.print(txState);
      Serial.print(" -> "); Serial.println(radioErrorName(txState));
      consecutiveTxErrors++;
      // After repeated errors, reinitialize radio and back off briefly
      if (consecutiveTxErrors >= 8) {
        Serial.println("Multiple TX errors; reinitializing radio");
        scheduleStatus("RADIO RESET", 1000);
        initRadio();
        consecutiveTxErrors = 0;
        delay(200);
      }
    }

    // small settle time — increase to 5 ms to avoid hammering the radio
    delay(5);
    // mark no received RSSI
    rssiData[i] = -150.0;
} else {
    int16_t state = radio.startReceive();

    if (state == RADIOLIB_ERR_NONE) {
      // CRITICAL FIX: Longer settling time for accurate RSSI
      // Increased to 200us to prevent erratic spikes and false logs
      delayMicroseconds(200); // Wait 200us for RSSI to stabilize
      float rssi = radio.getRSSI(); 
      rssiData[i] = rssi;
      
      if (rssi > maxRssiThisSweep) {
        maxRssiThisSweep = rssi;
        maxRssiFreq = currentFreq;
      }

      // DATA SNIFFER LOGIC
      if (rssi > -60.0) {
          // Log packet when found; do not change LED here to avoid flicker
          size_t len = radio.getPacketLength(); 
          String strData;
          rp2040.wdt_reset();
          int16_t rxState = radio.receive(strData, 50);
          rp2040.wdt_reset();

          if (rxState == RADIOLIB_ERR_NONE) {
             if (sweepLog) logData(sweepLog, currentFreq, rssi, (uint8_t*)strData.c_str(), strData.length());
             // Could schedule a brief status instead of LED flash
             if (oledReady) scheduleStatus("PKT RX", 500);
          } else {
             if (sweepLog) logData(sweepLog, currentFreq, rssi, nullptr, 0);
          }
      } else if (rssi > RSSI_THRESHOLD_DBM) {
          if (sweepLog) logData(sweepLog, currentFreq, rssi, nullptr, 0);
          rssiLogsThisSweep++;
      }

    } else {
      rssiData[i] = -150.0;
    }
    
    } // end receive branch
  
  } // end sweep for-loop
  if (sweepLog) sweepLog.close();
#else
  // Fake mode: simulate a quick sweep filling rssiData[] with generated values and occasional logs
  File sweepLog = LittleFS.open(LOG_FILENAME, "a");
  if (!sweepLog) Serial.println("Warning: could not open CSV for sweep logging (fake)");
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    checkForSerialCommand();
    rp2040.wdt_reset();

    if (digitalRead(BTN2) == LOW) break;

    float currentFreq = scanSteps[i];
    float t = (millis() / 1000.0f);
    // Generate a low-cost deterministic waveform with small noise
    float base = -100.0f + 10.0f * sinf((i / 8.0f) + t);
    float noise = (float)(random(-5, 5)) * 0.5f;
    float rssi = base + noise;
    rssiData[i] = rssi;

    // Occasionally inject a strong peak and log it
    if ((i % 40) == ((unsigned long)(t) % 40)) {
      rssi = -45.0f;
      rssiData[i] = rssi;
      if (sweepLog) logData(sweepLog, currentFreq, rssi, nullptr, 0);
      if (rssi > maxRssiThisSweep) {
        maxRssiThisSweep = rssi;
        maxRssiFreq = currentFreq;
      }
    } else if (rssi > RSSI_THRESHOLD_DBM) {
      if (sweepLog) logData(sweepLog, currentFreq, rssi, nullptr, 0);
      rssiLogsThisSweep++;
    }

    // No per-sample LED changes in fake mode either; defer to updateLedState
    (void)0; // noop placeholder

    // small throttle to simulate scanning time
    delayMicroseconds(50);
  }
  // short delay to emulate total scan time
  if (sweepLog) sweepLog.close();
  delay(20);
#endif

  // Ensure LED matches current mode (no per-sample flicker)
  updateLedState();
  
  // Update global max tracking
  if (maxRssiThisSweep > globalMaxRssiLog) {
    globalMaxRssiLog = maxRssiThisSweep;
    globalMaxRssiFreq = maxRssiFreq;
    globalMaxRssiTime = millis();
    Serial.print("NEW MAX: "); Serial.print(globalMaxRssiLog);
    Serial.print(" dBm @ "); Serial.print(globalMaxRssiFreq);
    Serial.println(" MHz");
  }
  
  // Log sweep stats
  float sweepAvgRssi = 0;
  int validCount = 0;
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    if (rssiData[i] > -140) {
      sweepAvgRssi += rssiData[i];
      validCount++;
    }
  }
  if (validCount > 0) sweepAvgRssi /= validCount;
  
  Serial.print("Sweep complete: Logged "); Serial.print(rssiLogsThisSweep);
  Serial.print(" RSSI values. Max: "); Serial.print(maxRssiThisSweep);
  Serial.print(" dBm @ "); Serial.print(maxRssiFreq);
  Serial.print(" MHz. Avg: "); Serial.print(sweepAvgRssi);
  Serial.print(" dBm ("); Serial.print(validCount); Serial.println(" samples)");
  
  // Smooth rssiData into avgRssi using EMA
  unsigned long now = millis();
  float dt = (float)(now - lastSmoothMillis);
  if (dt < 0) dt = 0;
  const float tau = 2000.0f; // 2 second smoothing (faster response)
  float alpha = (dt > 0) ? (1.0f - expf(-dt / tau)) : 0.1f;
  
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    if (rssiData[i] > -140) {
      avgRssi[i] = avgRssi[i] + alpha * (rssiData[i] - avgRssi[i]);
    } else {
      avgRssi[i] = avgRssi[i] + alpha * (-150.0f - avgRssi[i]);
    }
  }
  lastSmoothMillis = now;

  // Decay global max slowly
  if (millis() - globalMaxRssiTime > 1000) {
    globalMaxRssiLog -= 0.2; // Slow decay
  }

  // Update display every DISPLAY_UPDATE_MS
  if (millis() - lastDisplayMs >= DISPLAY_UPDATE_MS) {
    lastDisplayMs = millis();
    renderDisplay();
  }
}