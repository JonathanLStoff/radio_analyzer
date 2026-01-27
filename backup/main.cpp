#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <math.h>

// --- Pin Definitions ---
constexpr int PIN_GDO0 = 7;   // CC1101 GDO0 (data ready)
constexpr int PIN_CSN = 5;    // Chip select (CSN)
constexpr int PIN_SCK = 2;    // SPI clock
constexpr int PIN_MOSI = 3;   // SPI MOSI
constexpr int PIN_MISO = 4;   // SPI MISO
constexpr int LED_PIN_B = 12; // On-board LED BLUE
constexpr int LED_PIN_G = 11; // On-board LED GREEN
constexpr int LED_PIN_R = 10; // On-board LED RED

constexpr int BTN1 = 8;
constexpr int BTN2 = 9;       // Used for CSV Reset

// OLED Pins - CHANGED TO VALID I2C1 PAIR (User SCK=19 kept, SDA moved to 18)
// NOTE: Please move SDA wire from 21 to 18!
constexpr int PIN_OLED_SDA = 18;
constexpr int PIN_OLED_SCL = 19;

// --- Config Constants ---
constexpr float SCAN_START_FREQ = 900.0f;
constexpr float SCAN_END_FREQ = 930.0f;
constexpr float RADIO_BIT_RATE_KBPS = 4.8f;
constexpr float RADIO_FREQ_DEV_KHZ = 5.0f;
constexpr float RADIO_RX_BW_KHZ = 135.0f;
constexpr int8_t RADIO_TX_POWER_DBM = 10;
constexpr uint8_t RADIO_PREAMBLE_BITS = 32;

constexpr int RX_BUFFER_SIZE = 64;
constexpr int RSSI_THRESHOLD_DBM = -80; // Only log signals stronger than -80 dBm (raised from -95 to reduce noise)

#define SSD1306_128_64 true
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Globals ---
Module cc1101Module(PIN_CSN, PIN_GDO0, RADIOLIB_NC);
CC1101 radio(&cc1101Module);
// Use Wire1 because Pins 18/19 are I2C1 on RP2040
// Create the display after I2C is configured in setup to ensure Adafruit uses Wire1
Adafruit_SSD1306* display = nullptr;
bool oledReady = false;

float scanSteps[SCREEN_WIDTH]; // Map frequencies to screen X pixels
float rssiData[SCREEN_WIDTH];
float avgRssi[SCREEN_WIDTH]; // exponential moving average over ~5s
uint8_t rx_buffer[RX_BUFFER_SIZE];
unsigned long lastSmoothMillis = 0;
// A simple tracking variable for the 'background' signal level to detect 2x stronger
float globalMaxRssiLog = -130.0; 
unsigned long redLedUntil = 0;
unsigned long timestampOffset = 0;
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_UPDATE_MS = 500;

const char* LOG_FILENAME = "/scan_log.csv";

// Forward declarations for functions used above their definitions
void logData(float freq, float rssi, uint8_t* data, size_t len);
void renderDisplay(const float pixelVals[]);
void scheduleStatus(const String &msg, unsigned long ms = 1000);

// Status overlay shown on next render
String pendingStatus = "";
unsigned long statusUntil = 0;

// Display persistence - holds peak values to reduce flicker
float displayPersistence[SCREEN_WIDTH];
unsigned long lastPersistenceDecay = 0;

// --- Helpers ---

void initRadio() {
  Serial.print("Initializing CC1101... ");
  // Using begin() default first to check basic connectivity
  int16_t state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
     Serial.println("Basic Init SUCCESS.");
     // Now apply config
     radio.setFrequency(SCAN_START_FREQ);
     radio.setBitRate(RADIO_BIT_RATE_KBPS);
     radio.setFrequencyDeviation(RADIO_FREQ_DEV_KHZ);
     radio.setRxBandwidth(RADIO_RX_BW_KHZ);
     radio.setOutputPower(RADIO_TX_POWER_DBM);
     radio.setOOK(true);
     Serial.println("Radio configured for OOK Scan.");
     
      digitalWrite(LED_PIN_R, LOW); // Turn off Red if it was on
     // Blink Green to confirm
     digitalWrite(LED_PIN_G, HIGH); delay(200); digitalWrite(LED_PIN_G, LOW);
      if (oledReady && display != nullptr) {
        scheduleStatus("Radio: OK", 800);
      }

  } else {
    Serial.print("INIT FAILED, code ");
    Serial.println(state);
    
    if (oledReady && display != nullptr) {
      scheduleStatus(String("Radio Fail: ") + String(state), 2000);
    }
    
    // Solid RED to indicate radio death, but do NOT block execution so button can reset
    digitalWrite(LED_PIN_R, HIGH);
  }
}

// Read the CSV tail and draw only entries within the last windowMs milliseconds.
void updateDisplayFromCSVWindow(unsigned long windowMs) {
  if (!oledReady || display == nullptr) return;

  unsigned long nowTs = timestampOffset + millis();
  unsigned long cutoff = (windowMs >= nowTs) ? 0 : (nowTs - windowMs);

  // Debug trace: log when the display update runs and its time window
  Serial.print("updateDisplayFromCSVWindow called; nowTs="); Serial.print(nowTs);
  Serial.print(" cutoff="); Serial.println(cutoff);
 
  rp2040.wdt_reset();

  File f = LittleFS.open(LOG_FILENAME, "r");
  if (!f) {
    Serial.println("Failed to open CSV log for reading");
    // show diagnostic
    display->clearDisplay();
    display->setCursor(0,0);
    display->println("CSV open fail");
    display->display();
    return;
  }

  // Seek near end for performance (tail read)
  size_t tailSize = f.size() > 32768 ? 32768 : f.size();
  if (tailSize > 0) {
    f.seek(f.size() - tailSize);
    // discard partial line
    f.readStringUntil('\n');
  }

  // Prepare buckets - each pixel covers a FREQUENCY RANGE, not a time window
  // We'll average ALL recent readings in each freq bucket
  float bucketSum[SCREEN_WIDTH];
  int bucketCount[SCREEN_WIDTH];
  float bucketMax[SCREEN_WIDTH];
  float bucketPeakFreq[SCREEN_WIDTH];
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    bucketSum[x] = 0.0f;
    bucketCount[x] = 0;
    bucketMax[x] = -200.0f;
    bucketPeakFreq[x] = 0.0f;
  }

  // Read lines and accumulate those whose timestamp >= cutoff
  int linesProcessed = 0;
  int linesSkippedOldTimestamp = 0;
  int linesSkippedBadParse = 0;
  
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    // skip header
    if (line.startsWith("Freq_")) continue;
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 1 || c2 < 1) {
      linesSkippedBadParse++;
      continue;
    }
    String freqS = line.substring(0, c1);
    String tsS = line.substring(c1 + 1, c2);
    String rssiS = (c2 > 0 && c3 > c2) ? line.substring(c2 + 1, c3) : line.substring(c2 + 1);

    unsigned long ts = strtoul(tsS.c_str(), NULL, 10);
    if (ts < cutoff) {
      linesSkippedOldTimestamp++;
      continue;
    }

    float freq = freqS.toFloat();
    float rssi = rssiS.toFloat();
    if (freq <= 0) {
      linesSkippedBadParse++;
      continue;
    }

    linesProcessed++;
    
    // Debug: Print first few lines to see what we're processing
    if (linesProcessed <= 3) {
      Serial.print("  Sample CSV line: Freq="); Serial.print(freq);
      Serial.print(" RSSI="); Serial.print(rssi);
      Serial.print(" TS="); Serial.println(ts);
    }

    // Map freq to pixel x - each pixel covers (30MHz / 128) = ~0.234 MHz
    int x = (int)round(((freq - SCAN_START_FREQ) / (SCAN_END_FREQ - SCAN_START_FREQ)) * (SCREEN_WIDTH - 1));
    x = constrain(x, 0, SCREEN_WIDTH - 1);
    bucketSum[x] += rssi;
    bucketCount[x]++;
    if (rssi > bucketMax[x]) {
      bucketMax[x] = rssi;
      bucketPeakFreq[x] = freq;
    }
  }
  f.close();

  Serial.print("Lines processed in window: "); Serial.print(linesProcessed);
  Serial.print(" (skipped old: "); Serial.print(linesSkippedOldTimestamp);
  Serial.print(", bad parse: "); Serial.print(linesSkippedBadParse); Serial.println(")");

  // Compute per-pixel value - USE AVERAGE for smooth spectrum display
  float pixelVal[SCREEN_WIDTH];
  int nonEmptyBuckets = 0;
  float minRSSI = 0.0f;
  float maxRSSI = -200.0f;
  
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    if (bucketCount[x] > 0) {
      // Average all readings in this frequency bucket
      pixelVal[x] = bucketSum[x] / bucketCount[x];
      nonEmptyBuckets++;
      if (pixelVal[x] > maxRSSI) maxRSSI = pixelVal[x];
      if (pixelVal[x] < minRSSI) minRSSI = pixelVal[x];
    } else {
      pixelVal[x] = -150.0f; // Keep at -150, don't raise it
    }
    // REMOVED: pixelVal[x] = max(pixelVal[x], -120.0f); 
    // This was forcing empty pixels to render as bars!
  }
  
  Serial.print("Buckets with data: "); Serial.print(nonEmptyBuckets);
  Serial.print(" RSSI range: ["); Serial.print(minRSSI);
  Serial.print(" to "); Serial.print(maxRSSI); Serial.println("]");
  
  // Detect baseline (noise floor) - use 10th percentile
  if (nonEmptyBuckets > 10) {
    float sortedVals[SCREEN_WIDTH];
    int validIdx = 0;
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      if (pixelVal[x] > -145.0f) {
        sortedVals[validIdx++] = pixelVal[x];
      }
    }
    // Simple bubble sort for 10th percentile
    for (int i = 0; i < validIdx - 1; i++) {
      for (int j = 0; j < validIdx - i - 1; j++) {
        if (sortedVals[j] > sortedVals[j + 1]) {
          float temp = sortedVals[j];
          sortedVals[j] = sortedVals[j + 1];
          sortedVals[j + 1] = temp;
        }
      }
    }
    float baseline = sortedVals[validIdx / 10]; // 10th percentile
    Serial.print("Detected baseline (noise floor): "); Serial.print(baseline); Serial.println(" dBm");
    
    // Subtract baseline from all values to show relative strength
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      if (pixelVal[x] > -145.0f) {
        pixelVal[x] = pixelVal[x] - baseline; // Now 0 = noise floor, positive = signal
      }
    }
  }

  // Horizontal smoothing - ONLY smooth pixels that have actual data
  // Don't smooth across empty regions or it spreads noise everywhere
  float smoothVal[SCREEN_WIDTH];
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    // Check if THIS pixel has data (pixelVal > -145)
    if (pixelVal[x] > -145.0f) {
      // This pixel has data, apply 3-point smoothing with neighbors that also have data
      float a = pixelVal[x];
      float b = (x > 0 && pixelVal[x-1] > -145.0f) ? pixelVal[x - 1] : a;
      float c = (x < SCREEN_WIDTH - 1 && pixelVal[x+1] > -145.0f) ? pixelVal[x + 1] : a;
      smoothVal[x] = (a + b + c) / 3.0f;
    } else {
      // No data, keep at -150
      smoothVal[x] = -150.0f;
    }
  }
  
  // Debug: Count how many smoothed values are valid
  int smoothedCount = 0;
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    if (smoothVal[x] > -145.0f) smoothedCount++;
  }
  Serial.print("Smoothed pixels: "); Serial.println(smoothedCount);

  // Compute peak across the window
  float peakVal = -200.0f;
  int peakX = -1;
  float peakFreq = 0.0f;
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    if (bucketCount[x] > 0 && pixelVal[x] > peakVal) {
      peakVal = pixelVal[x];
      peakX = x;
      peakFreq = bucketPeakFreq[x];
    }
  }

  // NO LOGGING HERE - it creates a feedback loop!
  // The peak gets logged from the scan loop, not from display rendering

  // Pass smoothed values directly to renderer (no persistence to avoid ghost trails)
  renderDisplay(smoothVal);
}


// Render the entire OLED frame from the provided per-pixel RSSI values.
// This is the ONLY place that calls `display->display()` so updates happen
// at most once per DISPLAY_UPDATE_MS when triggered by updateDisplayFromCSVWindow().
void renderDisplay(const float pixelVals[]) {
  if (!oledReady || display == nullptr) return;

  display->clearDisplay();
  display->drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);

  bool any = false;
  int barsDrawn = 0;
  int tallBars = 0; // height > 10
  
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    float val = pixelVals[x];
    if (val <= -140.0f) continue;
    any = true;
    barsDrawn++;
    
    // Scale for RELATIVE strength: 0 dBm = noise floor, +10 dBm = strong signal
    const float rmin = -2.0f;   // Allow 2dB below baseline to show
    const float rmax = 15.0f;   // Signals up to +15dB above baseline
    float t = (val - rmin) / (rmax - rmin);
    t = constrain(t, 0.0f, 1.0f);
    
    // Debug first few bars
    if (barsDrawn <= 3) {
      Serial.print("  Bar "); Serial.print(x);
      Serial.print(": val="); Serial.print(val);
      Serial.print(" t="); Serial.print(t);
    }
    
    int y = (int)round((1.0f - t) * (SCREEN_HEIGHT - 12));
    y = constrain(y, 0, SCREEN_HEIGHT - 13);
    int height = (SCREEN_HEIGHT - 12) - y;
    
    // Minimum height for any visible signal (even at noise floor)
    if (height < 2 && val > -5.0f) height = 2;
    
    // Strong signals (>3dB above baseline) get taller minimum
    if (val > 3.0f && height < 10) height = 10;
    
    if (barsDrawn <= 3) {
      Serial.print(" y="); Serial.print(y);
      Serial.print(" h="); Serial.println(height);
    }
    
    if (height > 10) tallBars++;
    
    display->drawFastVLine(x, y, height, SSD1306_WHITE);
    
    // Add indicator at bottom for strong signals (>5dB above baseline)
    if (val > 5.0f) {
      display->fillRect(x, SCREEN_HEIGHT - 11, 1, 5, SSD1306_WHITE);
    }
  }
  
  Serial.print("Bars drawn: "); Serial.print(barsDrawn);
  Serial.print(" (tall: "); Serial.print(tallBars);
  
  // Show range of bars
  int firstBar = -1, lastBar = -1;
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    if (pixelVals[x] > -140.0f) {
      if (firstBar == -1) firstBar = x;
      lastBar = x;
    }
  }
  if (firstBar >= 0) {
    Serial.print(", range: pixels "); Serial.print(firstBar);
    Serial.print("-"); Serial.print(lastBar);
  }
  Serial.println(")");

  if (!any) {
    display->setCursor(8, (SCREEN_HEIGHT/2)-4);
    display->print("No scan data");
    for (int x = 0; x < SCREEN_WIDTH; x += 6) display->drawFastVLine(x, SCREEN_HEIGHT - 18, 6, SSD1306_WHITE);
  }

  // Bottom labels - show frequency range
  display->setCursor(0, SCREEN_HEIGHT - 10);
  display->print(SCAN_START_FREQ, 0);
  display->print("MHz");
  display->setCursor(SCREEN_WIDTH - 42, SCREEN_HEIGHT - 10);
  display->print(SCAN_END_FREQ, 0);
  display->print("MHz");

  // Left side: show it's relative scale
  display->setCursor(30, SCREEN_HEIGHT - 10);
  display->print("Rel");
  display->setCursor(0, 10);
  display->print("Max:");
  display->print(globalMaxRssiLog, 1);
  display->print("dB");

  // Draw status overlay if present
  if (pendingStatus.length() > 0 && millis() < statusUntil) {
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(2, 2);
    display->print(pendingStatus);
  }

  display->display();
}

// Show strong dbm spike on oled
void showSpikeOnOLED(float freq, float rssi) {
  if (!oledReady || display == nullptr) return;

  String msg = "Spike!";
  msg += String(" ");
  msg += String(freq, 2);
  msg += String("MHz ");
  msg += String(rssi, 1);
  msg += String("dBm");
  
  scheduleStatus(msg, 1500);
}
// Schedule a short status string to be shown on next render. Does not call
// `display->display()` directly; `renderDisplay()` shows it when it runs.
void scheduleStatus(const String &msg, unsigned long ms) {
  pendingStatus = msg;
  statusUntil = millis() + ms;
}

void initCSV() {
  // ALWAYS reset timestamp offset when initializing CSV
  timestampOffset = 0;
  
  if (!LittleFS.exists(LOG_FILENAME)) {
    File f = LittleFS.open(LOG_FILENAME, "w");
    if (f) {
      f.println("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex");
      f.close();
      Serial.println("CSV Created - timestamp starts at 0");
    } else {
      Serial.println("Failed to create CSV");
    }
  } else {
      Serial.println("CSV exists. Syncing timestamp...");
      File f = LittleFS.open(LOG_FILENAME, "r");
      if (f) {
         // Optimization: Seek near end to find last line quickly
         if (f.size() > 150) {
             f.seek(f.size() - 150);
             f.readStringUntil('\n'); // consume partial line
         }
         
         unsigned long maxTs = 0;
         while (f.available()) {
             String line = f.readStringUntil('\n');
             line.trim();
             if (line.length() == 0) continue;
             
             // Parse: Freq,Timestamp,RSSI...
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
         // Add 100ms gap just to ensure no overlap
         if (maxTs > 0) {
           timestampOffset = maxTs + 100;
         }
         Serial.print("Resuming Log from MS: "); Serial.println(timestampOffset);
      }
  }
}

void resetCSV() {
  LittleFS.remove(LOG_FILENAME);
  timestampOffset = 0; // CRITICAL: Reset timestamp offset
  
  // Recreate the CSV file with header
  File f = LittleFS.open(LOG_FILENAME, "w");
  if (f) {
    f.println("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex");
    f.close();
    Serial.println("CSV Reset by User - timestamp reset to 0");
  }
  
  if (oledReady && display != nullptr) {
      scheduleStatus("CSV RESET!", 800);
  }
  delay(500); // Reduced from 1000ms
}

void dumpCSV() {
  Serial.println("\n--- START CSV DUMP ---");
  File f = LittleFS.open(LOG_FILENAME, "r");
  if (f) {
    while (f.available()) {
      rp2040.wdt_reset(); // Prevent watchdog reset during large file dumps
      Serial.write(f.read());
    }
    f.close();
  } else {
    Serial.println("Failed to open CSV");
  }
  Serial.println("\n--- END CSV DUMP ---");
}

void logData(float freq, float rssi, uint8_t* data, size_t len) {
  File f = LittleFS.open(LOG_FILENAME, "a");
  if (f) {
    f.print(freq, 2);
    f.print(",");
    f.print(timestampOffset + millis());
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
  } else {
    Serial.println("Failed to open CSV for appending");
  }
}

void setup() {
  // Setup Watchdog IMMEDIATELY - 8s timeout
  rp2040.wdt_begin(8000);
  
  // HARDWARE SAFETY: Turn on Green LED IMMEDIATELY to prove power/boot
  pinMode(LED_PIN_G, OUTPUT);
  digitalWrite(LED_PIN_G, HIGH);

  Serial.begin(9600);
  
  // Give USB time to enumerate before we do anything crazy
  delay(2000);
  rp2040.wdt_reset();
  
  // Initialize other LED Pins
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  
  // Power On Self Test - Colors (Green is already on, cycle others)
  digitalWrite(LED_PIN_G, LOW); 
  digitalWrite(LED_PIN_R, HIGH); delay(100); digitalWrite(LED_PIN_R, LOW);
  digitalWrite(LED_PIN_G, HIGH); delay(100); digitalWrite(LED_PIN_G, LOW);
  digitalWrite(LED_PIN_B, HIGH); delay(100); digitalWrite(LED_PIN_B, LOW);
  digitalWrite(LED_PIN_G, HIGH); // Leave Green on

  // Wait for Serial, but execute code after timeout so device works standalone
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 5000)) {
     delay(10);
     rp2040.wdt_reset(); // Pet watchdog while waiting
  }
  
  Serial.println("\n\n--- RADIO ANALYZER STARTING v2.2 ---");
  Serial.println("System Life Check: CLOCK TICKING");

  // Initialize Pins
  pinMode(PIN_GDO0, INPUT);
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH); // SPI Deselect

  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS Mount Failed!");
    // Blink Red Fast
  } else {
    Serial.println("LittleFS Mounted.");
  }
  initCSV();
  // Force a test log on boot so the CSV is never empty
  logData(SCAN_START_FREQ, -120.0, nullptr, 0); 
  
  rp2040.wdt_reset();

  // Initialize I2C and OLED
  Serial.print("Configuring I2C: SDA="); Serial.print(PIN_OLED_SDA); Serial.print(" SCL="); Serial.println(PIN_OLED_SCL);
  Serial.flush(); // Ensure text sends before potential crash

  // DIAGNOSTIC CHECK
  pinMode(PIN_OLED_SDA, INPUT_PULLUP);
  pinMode(PIN_OLED_SCL, INPUT_PULLUP);
  delay(50);
  int sdaVal = digitalRead(PIN_OLED_SDA);
  int sclVal = digitalRead(PIN_OLED_SCL);
  Serial.print("Pin Check -> SDA: "); Serial.print(sdaVal); Serial.print(" (expect 1), SCL: "); Serial.println(sclVal);
  Serial.flush();
  
  if (sdaVal == LOW || sclVal == LOW) {
     Serial.println("CRITICAL ERROR: I2C Pins are stuck LOW! Wiring short detected.");
     Serial.println("Skipping I2C Init to prevent crash.");
     oledReady = false;
  } else {
      // Use Wire1 for Pins 18/19 (I2C1)
      Wire1.setSDA(PIN_OLED_SDA);
      Wire1.setSCL(PIN_OLED_SCL);
      Wire1.begin();
      Wire1.beginTransmission(0x3D); // OLED I2C address
      // I2C Scanner to debug connection
      
      
      oledReady = true;
      
      
      if (display == nullptr) {
        display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
      }
      if(display == nullptr || !display->begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        Serial.println(F("SSD1306 allocation failed"));
        oledReady = false;
      } else {
        Serial.println("OLED Initialized.");
      }
  } // end else (valid pins)
  
  // Reset Watchdog
  rp2040.wdt_reset();
  
  if (oledReady && display != nullptr) {
      // Request a short status to appear on the first renderer pass
      scheduleStatus("Analyzer Setup... Radio Init...", 800);
  }

  // Initialize SPI
  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  // Initialize Radio
  // Extracted to function for button reset
  initRadio();

  // Pre-calculate frequencies for each screen column
  float step = (SCAN_END_FREQ - SCAN_START_FREQ) / SCREEN_WIDTH;
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    scanSteps[i] = SCAN_START_FREQ + (i * step);
    rssiData[i] = -150.0f;
    avgRssi[i] = -150.0f;
    displayPersistence[i] = -150.0f; // Initialize to -150 (below render threshold)
  }
  lastSmoothMillis = millis();
  lastDisplayMs = millis(); // Initialize display timer
  lastPersistenceDecay = millis(); // Initialize decay timer
}

void showTop3FromCSVWindow(unsigned long windowMs = 5000) {
  if (!oledReady || display == nullptr) return;

  unsigned long nowTs = timestampOffset + millis();
  unsigned long cutoff = (windowMs >= nowTs) ? 0 : (nowTs - windowMs);
  rp2040.wdt_reset();

  File f = LittleFS.open(LOG_FILENAME, "r");
  if (!f) {
    Serial.println("Failed to open CSV log for top3");
    display->clearDisplay();
    display->setCursor(0,0);
    display->println("No CSV");
    display->display();
    return;
  }

  // Tail read
  size_t tailSize = f.size() > 32768 ? 32768 : f.size();
  if (tailSize > 0) {
    f.seek(f.size() - tailSize);
    f.readStringUntil('\n');
  }

  float bucketSum[SCREEN_WIDTH];
  int bucketCount[SCREEN_WIDTH];
  float bucketPeakFreq[SCREEN_WIDTH];
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    bucketSum[x] = 0.0f;
    bucketCount[x] = 0;
    bucketPeakFreq[x] = 0.0f;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (line.startsWith("Freq_")) continue;
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 1 || c2 < 1) continue;
    String freqS = line.substring(0, c1);
    String tsS = line.substring(c1 + 1, c2);
    String rssiS = (c2 > 0 && c3 > c2) ? line.substring(c2 + 1, c3) : line.substring(c2 + 1);

    unsigned long ts = strtoul(tsS.c_str(), NULL, 10);
    if (ts < cutoff) continue;

    float freq = freqS.toFloat();
    float rssi = rssiS.toFloat();
    if (freq <= 0) continue;

    int x = (int)round(((freq - SCAN_START_FREQ) / (SCAN_END_FREQ - SCAN_START_FREQ)) * (SCREEN_WIDTH - 1));
    x = constrain(x, 0, SCREEN_WIDTH - 1);
    bucketSum[x] += rssi;
    bucketCount[x]++;
    if (rssi > bucketPeakFreq[x]) bucketPeakFreq[x] = freq;
  }
  f.close();

  // Collect averages
  struct Entry { float avg; int x; };
  Entry entries[SCREEN_WIDTH];
  int entryCount = 0;
  for (int x = 0; x < SCREEN_WIDTH; x++) {
    if (bucketCount[x] > 0) {
      entries[entryCount++] = { bucketSum[x] / bucketCount[x], x };
    }
  }

  // If no entries, display --- lines
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);

  // Header centered
  char hdr[64];
  snprintf(hdr, sizeof(hdr), "Scanning %.2f - %.2fMHz", SCAN_START_FREQ, SCAN_END_FREQ);
  int16_t tw = (int16_t)strlen(hdr) * 6; // approx
  int16_t tx = max(0, (SCREEN_WIDTH - tw) / 2);
  display->setCursor(tx, 0);
  display->print(hdr);

  if (entryCount == 0) {
    for (int i = 0; i < 3; i++) {
      display->setCursor(2, 16 + (i * 12));
      display->print("+ -------");
    }
    display->display();
    return;
  }

  // Find top 3 by average. Simple selection with tie detection (within 0.1 dB)
  for (int rank = 0; rank < 3; rank++) {
    int bestIdx = -1;
    float bestVal = -10000.0f;
    for (int i = 0; i < entryCount; i++) {
      bool used = false;
      // check if already used in previous ranks
      for (int r = 0; r < rank; r++) {
        // stored in entries[r].avg == -INFINITY marker? We'll mark used by setting avg to -10000
      }
      // find max
      if (entries[i].avg > bestVal) {
        bestVal = entries[i].avg;
        bestIdx = i;
      }
    }

    // If no valid best, show ---
    if (bestIdx < 0) {
      display->setCursor(2, 16 + (rank * 12));
      display->print("+ -------");
      continue;
    }

    // Check for tie: any other entry within 0.1 dB of bestVal
    bool tie = false;
    for (int i = 0; i < entryCount; i++) {
      if (i == bestIdx) continue;
      if (fabs(entries[i].avg - bestVal) < 0.1f) { tie = true; break; }
    }

    if (tie) {
      display->setCursor(2, 16 + (rank * 12));
      display->print("+ -------");
      // mark the best as used to avoid repeated ties
      entries[bestIdx].avg = -10000.0f;
      continue;
    }

    // Use bucketPeakFreq to get frequency
    int x = entries[bestIdx].x;
    float freq = bucketPeakFreq[x];
    float rssi = entries[bestIdx].avg;

    char buf[32];
    snprintf(buf, sizeof(buf), "+ %.2fMHz | %+.0fdB", freq, rssi);
    display->setCursor(2, 16 + (rank * 12));
    display->print(buf);

    // mark used
    entries[bestIdx].avg = -10000.0f;
  }

  display->display();
}

void checkForSerialCommand() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'd' || c == 'D') {
      dumpCSV();
    } else if (c == 's' || c == 'S') {
      // show top 3 strongest freqs from last 5 seconds
      showTop3FromCSVWindow(5000);
    }
  }
}

void loop() {
  // Feed the watchdog every loop to prevent reset
  rp2040.wdt_reset();
  
  checkForSerialCommand();

  // Check Radio Reset Button (BTN1 - GPIO 8)
  if (digitalRead(BTN1) == LOW) {
      Serial.println("Button 1 Pressed: Resetting Radio...");
      if (oledReady && display != nullptr) {
        display->clearDisplay();
        display->setCursor(0,0);
        display->println("Resetting Radio...");
        display->display();
      }
      initRadio();
      Serial.println("Radio Reset Complete");
      delay(500); // Debounce
  }

  // Check CSV Reset Button
  if (digitalRead(BTN2) == LOW) {
    Serial.println("Button 2 Pressed: Resetting CSV...");
    resetCSV();
    delay(500); 
  }
  
  // Heartbeat log every 2 seconds to prove loop is alive
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
      Serial.print("Loop alive. RSSI Scan in progress... Mem: ");
      Serial.println(rp2040.getFreeHeap());
      lastHeartbeat = millis();
  }

  float maxRssiThisSweep = -150.0;
  float maxRssiFreq = 0;
  int rssiLogsThisSweep = 0; // Count how many we log
  
  // Sweep Loop
  // We scan the range. Each 'i' is a column on the screen.
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    // Check serial constantly during sweep so we don't miss commands
    checkForSerialCommand();

    // Pet watchdog inside the sweep so long scans don't trigger reset
    rp2040.wdt_reset();

    // Break if button pressed
    if (digitalRead(BTN2) == LOW) break;

    float currentFreq = scanSteps[i];
    radio.setFrequency(currentFreq);

    int16_t state = radio.startReceive();

    
    if (state == RADIOLIB_ERR_NONE) {
      delayMicroseconds(500); // Wait for RSSI to settle
      float rssi = radio.getRSSI(); 
      rssiData[i] = rssi;
      
      // Update max for this sweep
      if (rssi > maxRssiThisSweep) {
        maxRssiThisSweep = rssi;
        maxRssiFreq = currentFreq;
      }

      // DATA SNIFFER LOGIC
      // If signal is strong (> -60dBm), stop and try to read the packet!
      if (rssi > -60.0) {
          digitalWrite(LED_PIN_B, HIGH); // Blue indicates sniffing
          
          // Try to receive a packet with a 50ms timeout
          size_t len = radio.getPacketLength(); 
          // Note: In OOK/Raw mode, getPacketLength might be tricky, assuming variable length
          // RadioLib's receive() is blocking.
          String strData;
          // Ensure receive cannot block indefinitely: use a short timeout (50ms)
          rp2040.wdt_reset();
          int16_t rxState = radio.receive(strData, 50);
          rp2040.wdt_reset();

          if (rxState == RADIOLIB_ERR_NONE) {
             // We caught data!
             logData(currentFreq, rssi, (uint8_t*)strData.c_str(), strData.length());
             digitalWrite(LED_PIN_G, HIGH); // Green flash for success
             delay(50);
             digitalWrite(LED_PIN_G, LOW);
          } else {
             // Just log the strong signal presence
             logData(currentFreq, rssi, nullptr, 0);
          }
           digitalWrite(LED_PIN_B, LOW);
      } else if (rssi > RSSI_THRESHOLD_DBM) {
          // Weak signal, just log RSSI
          logData(currentFreq, rssi, nullptr, 0);
          rssiLogsThisSweep++;
      }
      
      // LED Logic
      // Requirement: Always GREEN. 
      // If signal is 2x stronger (approx +6dB) than background max, turn RED for 1 second.
      
      if (rssi > (globalMaxRssiLog + 6.0)) {
         redLedUntil = millis() + 1000;
      }

      if (millis() < redLedUntil) {
         digitalWrite(LED_PIN_R, HIGH);
         digitalWrite(LED_PIN_G, LOW);
         digitalWrite(LED_PIN_B, LOW);
      } else {
         digitalWrite(LED_PIN_R, LOW);
         digitalWrite(LED_PIN_G, HIGH); // Default Green
         digitalWrite(LED_PIN_B, LOW);
      }

    } else {
      rssiData[i] = -150.0;
    }
  }
  
  // Log sweep stats with RSSI variation
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
  
  // Smooth rssiData into avgRssi using time-based EMA (tau = 5s)
  unsigned long now = millis();
  float dt = (float)(now - lastSmoothMillis);
  if (dt < 0) dt = 0;
  const float tau = 5000.0f; // 5 seconds
  float alpha = (dt > 0) ? (1.0f - expf(-dt / tau)) : 0.05f;
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    // Only update if rssiData valid
    if (rssiData[i] > -140) {
      avgRssi[i] = avgRssi[i] + alpha * (rssiData[i] - avgRssi[i]);
    } else {
      // slowly decay invalids to background
      avgRssi[i] = avgRssi[i] + alpha * (-150.0f - avgRssi[i]);
    }
  }
  lastSmoothMillis = now;

  // Update global max from smoothed data
  float smoothMax = -200.0f;
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    if (avgRssi[i] > smoothMax) smoothMax = avgRssi[i];
  }
  if (smoothMax > globalMaxRssiLog) {
      globalMaxRssiLog = smoothMax;
  } else {
      globalMaxRssiLog -= 0.5; // decay
  }

  // Only update the visible OLED every DISPLAY_UPDATE_MS using CSV window
  if (millis() - lastDisplayMs >= DISPLAY_UPDATE_MS) {
    lastDisplayMs = millis();
    // Draw from recent CSV rows - NO LOGGING inside this function
    // Window set to 10 seconds to capture multiple full sweeps for complete picture
    // updateDisplayFromCSVWindow(10000);
  }
}
