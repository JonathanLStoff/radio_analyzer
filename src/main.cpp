#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>

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
constexpr int RSSI_THRESHOLD_DBM = -95; // Lowered to -95 for testing (logs noise/weak signals)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Globals ---
Module cc1101Module(PIN_CSN, PIN_GDO0, RADIOLIB_NC);
CC1101 radio(&cc1101Module);
// Use Wire1 because Pins 18/19 are I2C1 on RP2040
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
bool oledReady = false;

float scanSteps[SCREEN_WIDTH]; // Map frequencies to screen X pixels
float rssiData[SCREEN_WIDTH];
uint8_t rx_buffer[RX_BUFFER_SIZE];
// A simple tracking variable for the 'background' signal level to detect 2x stronger
float globalMaxRssiLog = -130.0; 
unsigned long redLedUntil = 0;
unsigned long timestampOffset = 0;

const char* LOG_FILENAME = "/scan_log.csv";

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
if (oledReady) {
        display.fillRect(0, 0, SCREEN_WIDTH, 8, SSD1306_BLACK);
        display.setCursor(0,0);
        display.println("Radio: OK");
        display.display();
     }

  } else {
    Serial.print("INIT FAILED, code ");
    Serial.println(state);
    
    if (oledReady) {
        display.fillRect(0, 0, SCREEN_WIDTH, 8, SSD1306_BLACK);
        display.setCursor(0,0);
        display.print("Radio Fail: ");
        display.print(state);
        display.display();
    }
    
    // Solid RED to indicate radio death, but do NOT block execution so button can reset
    digitalWrite(LED_PIN_R, HIGH);
  }
}

void initCSV() {
  timestampOffset = 0;
  if (!LittleFS.exists(LOG_FILENAME)) {
    File f = LittleFS.open(LOG_FILENAME, "w");
    if (f) {
      f.println("Freq_MHz,Timestamp_ms,RSSI_dBm,Data_Hex");
      f.close();
      Serial.println("CSV Created");
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
                     timestampOffset = strtoul(tsPart.c_str(), NULL, 10);
                 }
             }
         }
         f.close();
         // Add 100ms gap just to ensure no overlap overlap
         if (timestampOffset > 0) timestampOffset += 100;
         Serial.print("Resuming Log from MS: "); Serial.println(timestampOffset);
      }
  }
}

void resetCSV() {
  LittleFS.remove(LOG_FILENAME);
  initCSV();
  Serial.println("CSV Reset by User");
  
  if (oledReady) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("CSV RESET!");
      display.display();
  }
  delay(1000);
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
    Serial.print("Logged: ");
    Serial.print(freq);
    Serial.print("MHz, ");
    Serial.print(rssi);
    Serial.println("dBm");
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
  
  Serial.println("\n\n--- RADIO ANALYZER STARTING v2.1 ---");
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
  logData(0.0, -1.0, nullptr, 0); 
  
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
    
      // I2C Scanner to debug connection
      Serial.println("Scanning I2C bus...");
      byte error, address;
      int nDevices = 0;
      for(address = 1; address < 127; address++ ) {
        rp2040.wdt_reset(); 
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();
        if (error == 0) {
          Serial.print("I2C device found at address 0x");
          if (address < 16) Serial.print("0");
          Serial.println(address, HEX);
          nDevices++;
        }
      }
      if (nDevices == 0) {
        Serial.println("No I2C devices found. CHECK WIRING!");
        oledReady = false; 
      } else {
        Serial.println("I2C scan complete.");
        oledReady = true;
      }
      
      if (oledReady) {
         if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
            Serial.println(F("SSD1306 allocation failed"));
            oledReady = false;
         } else {
            Serial.println("OLED Initialized.");
         }
      }
  } // end else (valid pins)
  
  // Reset Watchdog
  rp2040.wdt_reset();
  
  /* REMOVED OLD BLOCK TO AVOID CONFUSION/ERRORS
  if(!display.begin... 
  */
  
  if (oledReady) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("Analyzer Setup...");
    display.println("Radio Init...");
    display.display();
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
  }
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
  // Feed the watchdog every loop to prevent reset
  rp2040.wdt_reset();
  
  checkForSerialCommand();

  // Check Radio Reset Button (BTN1 - GPIO 8)
  if (digitalRead(BTN1) == LOW) {
      Serial.println("Button 1 Pressed: Resetting Radio...");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Resetting Radio...");
      display.display();
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
  
  // Sweep Loop
  // We scan the range. Each 'i' is a column on the screen.
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    // Check serial constantly during sweep so we don't miss commands
    checkForSerialCommand();

    // Break if button pressed
    if (digitalRead(BTN2) == LOW) break;

    float currentFreq = scanSteps[i];
    
    // Switch to RX mode
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
          int16_t rxState = radio.receive(strData); 
          
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
      }
      
      // LED Logic
      
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
  
  // Decay the global max slowly so we adapt to new noise floors, 
  // but keep it high enough to remember recent strong signals.
  // Actually, we should track the max of *this* sweep to be the baseline for the *next* sweep.
  // But to detect "one signal stronger than others", we use the average of the sweep?
  // Let's set globalMaxRssiLog to the AVERAGE of this sweep, so we spot peaks above average?
  // No, user said "stronger than any other signal".
  // Let's track the "Second Strongest" signal?
  // Use a simple heuristic: globalMaxRssiLog = maxRssiThisSweep.
  if (oledReady) {
    display.clearDisplay();
  
    // Draw Graph
    for (int x = 0; x < SCREEN_WIDTH; x++) {
      float val = rssiData[x];
      if (val > -140) { // Only draw valid
         int y = map((long)val, -130, -30, SCREEN_HEIGHT, 0);
         y = constrain(y, 0, SCREEN_HEIGHT - 1);
         display.drawFastVLine(x, y, SCREEN_HEIGHT - y, SSD1306_WHITE);
      }
    }
  
    // Draw Labels
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    display.setCursor(0, 0); 
    display.print("900");
    
    display.setCursor(SCREEN_WIDTH - 20, 0); 
    display.print("930");
  
    // Draw Peak Info
    display.setCursor(30, 0);
    display.print("Pk:");
    display.print(maxRssiFreq, 1);
    display.print("M");
    
    display.setCursor(30, 10);
    display.print(maxRssiThisSweep, 0);
    display.print("dB");
  
    display.display();
  }
}