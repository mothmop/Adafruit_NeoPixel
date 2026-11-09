#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include <cstdarg>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLE2902.h>

// ============================================================================
// Hardware & Display Configuration (FeatherWing OLED 128x32)
// ============================================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1  // Display shares reset button with Feather board

// Button Pin Definitions for FeatherWing OLED
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static bool hasDisplay = false;

// ============================================================================
// Sony RMT-P1BT Remote GATT Service & Characteristic UUIDs
// ============================================================================
static BLEUUID SERVICE_UUID("8000FF00-FF00-FFFF-FFFF-FFFFFFFFFFFF");
static BLEUUID CHAR_CONTROL_UUID("0000FF01-0000-1000-8000-00805F9B34FB");

// Sony Bluetooth SIG Company Identifier (0x012D -> Sony Corporation)
static constexpr uint16_t SONY_COMPANY_ID = 0x012D;

// --- Sony RMT-P1BT Shutter & AF Commands (2-byte payloads) ---
static const uint8_t CMD_AF_DOWN[]      = {0x01, 0x07}; // Half-press focus engage
static const uint8_t CMD_AF_UP[]        = {0x01, 0x06}; // Half-press focus release
static const uint8_t CMD_SHUTTER_DOWN[] = {0x01, 0x09}; // Full shutter engage
static const uint8_t CMD_SHUTTER_UP[]   = {0x01, 0x08}; // Full shutter release

// --- Sony RMT-P1BT Manual Focus Commands (3-byte payloads for Focus +/-) ---
// Byte 0: 0x02 (Length)
// Byte 1: Command Opcode (0x6B = Focus Near / In, 0x6D = Focus Far / Out)
// Byte 2: Speed / Velocity (0x10 = Slow, 0x30 = Medium, 0x7F = Fast)
static const uint8_t CMD_FOCUS_NEAR_PRESS[] = {0x02, 0x6B, 0x30}; // Focus Near (+)
static const uint8_t CMD_FOCUS_NEAR_REL[]   = {0x02, 0x6A, 0x00}; // Focus Near Release
static const uint8_t CMD_FOCUS_FAR_PRESS[]  = {0x02, 0x6D, 0x30}; // Focus Far (-)
static const uint8_t CMD_FOCUS_FAR_REL[]    = {0x02, 0x6C, 0x00}; // Focus Far Release

// Timing Delays (Milliseconds)
static constexpr int MECHANICAL_SETTLE_MS = 400;
static constexpr int SHUTTER_CYCLE_MS    = 1200;
static constexpr int BACKLASH_REVERSE_MS = 300;

// Default Stacking Parameters
static float currentRangeMm = 4.0f;
static int currentFrames    = 10;
static float currentRulerCm = 3.5f;

// ============================================================================
// Command & Output Logging System (Console + LittleFS)
// ============================================================================
static const char* LOG_FILE_PATH     = "/stacker.log";
static const char* LOG_OLD_FILE_PATH = "/stacker.log.old";
static constexpr size_t MAX_LOG_SIZE = 64 * 1024; // 64 KB per log file
static bool fsReady = false;

void getTimestampStr(char* buffer, size_t maxLen) {
    uint32_t ms_total = millis();
    uint32_t ms = ms_total % 1000;
    uint32_t sec_total = ms_total / 1000;
    uint32_t s = sec_total % 60;
    uint32_t m = (sec_total / 60) % 60;
    uint32_t h = sec_total / 3600;
    snprintf(buffer, maxLen, "[%02u:%02u:%02u.%03u]", h, m, s, ms);
}

void logMessage(const char* tag, const char* format, ...) {
    char timeBuf[16];
    getTimestampStr(timeBuf, sizeof(timeBuf));

    char msgBuf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(msgBuf, sizeof(msgBuf), format, args);
    va_end(args);

    // 1. Output to Serial Monitor
    Serial.printf("%s [%s] %s\n", timeBuf, tag, msgBuf);

    // 2. Persist to LittleFS log file (if filesystem is ready)
    if (fsReady) {
        if (LittleFS.exists(LOG_FILE_PATH)) {
            File check = LittleFS.open(LOG_FILE_PATH, FILE_READ);
            if (check) {
                size_t sz = check.size();
                check.close();
                if (sz >= MAX_LOG_SIZE) {
                    LittleFS.remove(LOG_OLD_FILE_PATH);
                    LittleFS.rename(LOG_FILE_PATH, LOG_OLD_FILE_PATH);
                }
            }
        }

        File logFile = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
        if (logFile) {
            logFile.printf("%s [%s] %s\n", timeBuf, tag, msgBuf);
            logFile.close();
        }
    }
}

void dumpLog() {
    Serial.println(F("\n================= BEGIN STACKER LOG ================="));
    if (!fsReady || !LittleFS.exists(LOG_FILE_PATH)) {
        Serial.println(F("[LOG] No log file exists on flash."));
    } else {
        File file = LittleFS.open(LOG_FILE_PATH, FILE_READ);
        if (!file) {
            Serial.println(F("[LOG] Failed to open log file for reading."));
        } else {
            while (file.available()) {
                String line = file.readStringUntil('\n');
                Serial.println(line);
            }
            file.close();
        }
    }
    Serial.println(F("================== END STACKER LOG ==================\n"));
}

void clearLog() {
    if (fsReady) {
        if (LittleFS.exists(LOG_FILE_PATH)) LittleFS.remove(LOG_FILE_PATH);
        if (LittleFS.exists(LOG_OLD_FILE_PATH)) LittleFS.remove(LOG_OLD_FILE_PATH);
    }
    logMessage("LOG", "Log file cleared by user request.");
}

void printLogStatus() {
    size_t curSize = 0;
    if (fsReady && LittleFS.exists(LOG_FILE_PATH)) {
        File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
        if (f) { curSize = f.size(); f.close(); }
    }
    size_t used = fsReady ? LittleFS.usedBytes() : 0;
    size_t total = fsReady ? LittleFS.totalBytes() : 0;
    logMessage("LOG", "Log Size: %u / %u bytes | LittleFS Total: %u KB, Used: %u KB",
               (unsigned int)curSize, (unsigned int)MAX_LOG_SIZE,
               (unsigned int)(total / 1024), (unsigned int)(used / 1024));
}

// ============================================================================
// Calibration Table & Non-linear Interpolation
// ============================================================================
struct CalPoint {
    float ruler_cm;
    float depth_mm;
    float ms_per_mm;
};

static std::vector<CalPoint> calTable;
static const char* CAL_FILE_PATH = "/macro_calibration.csv";

static const CalPoint DEFAULT_CAL_TABLE[] = {
    {2.0f,  10.0f, 320.0f},
    {3.0f,  15.0f, 240.0f},
    {4.0f,  20.0f, 185.0f},
    {6.0f,  30.0f, 125.0f},
    {8.0f,  40.0f,  90.0f},
    {10.0f, 50.0f,  68.0f}
};
static const size_t DEFAULT_CAL_SIZE = sizeof(DEFAULT_CAL_TABLE) / sizeof(DEFAULT_CAL_TABLE[0]);

// ============================================================================
// BLE State & Handles
// ============================================================================
static BLEClient* pClient = nullptr;
static BLERemoteCharacteristic* pRemoteControlChar = nullptr;
static BLEAdvertisedDevice* targetDevice = nullptr;
static bool doConnect  = false;
static bool connected  = false;
static bool isScanning = false;
static bool isStacking = false;

// Display state
static char lastStatusMsg[32] = "Ready";

// ============================================================================
// Forward Declarations
// ============================================================================
void updateDisplay(const char* statusMsg = nullptr);
void showStackProgress(int frame, int total, float current_z, const char* stage);
void startBleScan();
bool delayWithAbortCheck(uint32_t ms);
void triggerShutter();
void pulseFocusNear(int durationMs);
void pulseFocusFar(int durationMs);
void executeStackingSweep(float range_mm, int frame_count, float ruler_index_cm);
void handleSerial();

// ============================================================================
// Calibration File Operations
// ============================================================================
void writeDefaultCalibrationFile() {
    if (!fsReady) return;
    File file = LittleFS.open(CAL_FILE_PATH, FILE_WRITE);
    if (!file) {
        logMessage("FS", "Failed to create default calibration file %s", CAL_FILE_PATH);
        return;
    }
    file.println("Ruler_Distance_cm,True_Depth_mm,Ms_Per_mm");
    for (size_t i = 0; i < DEFAULT_CAL_SIZE; ++i) {
        file.printf("%.1f,%.1f,%.1f\n", 
                    DEFAULT_CAL_TABLE[i].ruler_cm, 
                    DEFAULT_CAL_TABLE[i].depth_mm, 
                    DEFAULT_CAL_TABLE[i].ms_per_mm);
    }
    file.close();
    logMessage("FS", "Generated default macro_calibration.csv in flash");
}

bool loadCalibrationCSV() {
    calTable.clear();

    if (!fsReady) {
        logMessage("CAL", "Filesystem not ready; loading hardcoded defaults");
        for (size_t i = 0; i < DEFAULT_CAL_SIZE; ++i) calTable.push_back(DEFAULT_CAL_TABLE[i]);
        return false;
    }

    if (!LittleFS.exists(CAL_FILE_PATH)) {
        logMessage("FS", "Calibration file missing. Creating default template...");
        writeDefaultCalibrationFile();
    }

    File file = LittleFS.open(CAL_FILE_PATH, FILE_READ);
    if (!file) {
        logMessage("FS", "Error opening %s. Using hardcoded defaults.", CAL_FILE_PATH);
        for (size_t i = 0; i < DEFAULT_CAL_SIZE; ++i) calTable.push_back(DEFAULT_CAL_TABLE[i]);
        return false;
    }

    file.readStringUntil('\n'); // Skip header
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        float c1 = 0, c2 = 0, c3 = 0, c4 = 0;
        int parsed = sscanf(line.c_str(), "%f,%f,%f,%f", &c1, &c2, &c3, &c4);
        if (parsed == 4) {
            // Format with 4 columns: ruler_cm, depth_mm, steps_per_mm, ms_per_mm
            calTable.push_back({c1, c2, c4});
        } else if (parsed == 3) {
            // Format with 3 columns: ruler_cm, depth_mm, ms_per_mm
            calTable.push_back({c1, c2, c3});
        }
    }
    file.close();

    if (calTable.empty()) {
        for (size_t i = 0; i < DEFAULT_CAL_SIZE; ++i) {
            calTable.push_back(DEFAULT_CAL_TABLE[i]);
        }
    }

    std::sort(calTable.begin(), calTable.end(), [](const CalPoint& a, const CalPoint& b) {
        return a.ruler_cm < b.ruler_cm;
    });

    logMessage("CAL", "Loaded %u calibration points from %s", (unsigned int)calTable.size(), CAL_FILE_PATH);
    return true;
}

float interpolatePulsePerMm(float ruler_cm) {
    if (calTable.empty()) return 150.0f;
    if (ruler_cm <= calTable.front().ruler_cm) return calTable.front().ms_per_mm;
    if (ruler_cm >= calTable.back().ruler_cm) return calTable.back().ms_per_mm;

    for (size_t i = 0; i < calTable.size() - 1; ++i) {
        if (ruler_cm >= calTable[i].ruler_cm && ruler_cm <= calTable[i + 1].ruler_cm) {
            float t = (ruler_cm - calTable[i].ruler_cm) / (calTable[i + 1].ruler_cm - calTable[i].ruler_cm);
            return calTable[i].ms_per_mm + t * (calTable[i + 1].ms_per_mm - calTable[i].ms_per_mm);
        }
    }
    return calTable.back().ms_per_mm;
}

// ============================================================================
// Display Functions
// ============================================================================
void updateDisplay(const char* statusMsg) {
    if (statusMsg != nullptr) {
        strncpy(lastStatusMsg, statusMsg, sizeof(lastStatusMsg) - 1);
        lastStatusMsg[sizeof(lastStatusMsg) - 1] = '\0';
    }
    if (!hasDisplay) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Line 0: BLE connection status
    display.setCursor(0, 0);
    if (connected) {
        display.print(F("SONY A7C: CONNECTED"));
    } else if (isScanning) {
        display.print(F("BLE: SCANNING..."));
    } else if (doConnect) {
        display.print(F("BLE: CONNECTING..."));
    } else {
        display.print(F("BLE: DISCONNECTED"));
    }

    // Line 1: Stacking parameters
    display.setCursor(0, 8);
    display.printf("R:%.1fmm N:%d L:%.1fcm", currentRangeMm, currentFrames, currentRulerCm);

    // Line 2: Button guide
    display.setCursor(0, 16);
    if (connected) {
        display.print(F("A:Run  B:Shot  C:Near"));
    } else {
        display.print(F("Press button to scan"));
    }

    // Line 3: Dynamic status message
    display.setCursor(0, 24);
    display.print(lastStatusMsg);

    display.display();
}

void showStackProgress(int frame, int total, float current_z, const char* stage) {
    if (!hasDisplay) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Line 0: Frame count and Z depth
    display.setCursor(0, 0);
    display.printf("STACK %d/%d Z:%.2fmm", frame, total, current_z);

    // Line 1: Current stage description
    display.setCursor(0, 9);
    display.print(stage);

    // Line 2 & 3: Progress bar outline and fill
    display.drawRect(0, 20, 128, 10, SSD1306_WHITE);
    if (total > 0 && frame > 0) {
        int barWidth = (frame * 124) / total;
        if (barWidth > 124) barWidth = 124;
        display.fillRect(2, 22, barWidth, 6, SSD1306_WHITE);
    }
    display.display();
}

// ============================================================================
// BLE Remote Commands
// ============================================================================
void sendBleCommand(const uint8_t* cmd, size_t len) {
    if (connected && pRemoteControlChar != nullptr) {
        pRemoteControlChar->writeValue((uint8_t*)cmd, len, false);
    }
}

void pulseFocusNear(int durationMs) {
    logMessage("FOCUS", "Pulse Near (%d ms)", durationMs);
    sendBleCommand(CMD_FOCUS_NEAR_PRESS, 3);
    delay(durationMs);
    sendBleCommand(CMD_FOCUS_NEAR_REL, 3);
}

void pulseFocusFar(int durationMs) {
    logMessage("FOCUS", "Pulse Far (%d ms)", durationMs);
    sendBleCommand(CMD_FOCUS_FAR_PRESS, 3);
    delay(durationMs);
    sendBleCommand(CMD_FOCUS_FAR_REL, 3);
}

void triggerShutter() {
    logMessage("SHUTTER", "Triggering shutter cycle (Half-press AF -> Full shutter -> Settle)");
    // 1. Engage AF half-press
    sendBleCommand(CMD_AF_DOWN, 2);
    delay(100);
    // 2. Full shutter engage
    sendBleCommand(CMD_SHUTTER_DOWN, 2);
    delay(100);
    // 3. Release full shutter
    sendBleCommand(CMD_SHUTTER_UP, 2);
    delay(50);
    // 4. Release AF half-press
    sendBleCommand(CMD_AF_UP, 2);
    delayWithAbortCheck(SHUTTER_CYCLE_MS);
}

// Non-blocking delay helper during stacking that checks for abort input
bool delayWithAbortCheck(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        // Button A aborts running stack
        if (digitalRead(BUTTON_A) == LOW) {
            delay(20);
            if (digitalRead(BUTTON_A) == LOW) {
                logMessage("CMD", "[BUTTON] Button A pressed -> ABORT_SWEEP");
                return true;
            }
        }
        // Serial abort command check
        if (Serial.available()) {
            int peekChar = Serial.peek();
            if (peekChar == 's' || peekChar == 'S' || peekChar == 'q' || peekChar == 'Q') {
                Serial.readStringUntil('\n');
                logMessage("CMD", "[SERIAL] Abort command received");
                return true;
            }
        }
        delay(10);
    }
    return false;
}

void executeStackingSweep(float range_mm, int frame_count, float ruler_index_cm) {
    if (!connected) {
        logMessage("ERROR", "Bluetooth connection not established. Sweep canceled.");
        updateDisplay("Error: Not Connected");
        return;
    }
    if (frame_count < 2) frame_count = 2;

    float ms_per_mm = interpolatePulsePerMm(ruler_index_cm);
    float step_dist_mm = range_mm / (frame_count - 1);
    int step_pulse_ms = max(15, (int)round(step_dist_mm * ms_per_mm));

    logMessage("STACK", "Starting sweep: Range = %.2f mm | Frames = %d | Ruler = %.1f cm",
               range_mm, frame_count, ruler_index_cm);
    logMessage("CAL", "Interpolated rate: %.1f ms/mm -> Step dist: %.3f mm -> Pulse: %d ms",
               ms_per_mm, step_dist_mm, step_pulse_ms);

    isStacking = true;

    // Backlash Elimination at Start Plane (Z = 0)
    showStackProgress(0, frame_count, 0.0f, "Backlash Settle...");
    logMessage("STACK", "Backlash margin: Reversing Far (%d ms)", BACKLASH_REVERSE_MS);
    pulseFocusFar(BACKLASH_REVERSE_MS);
    if (delayWithAbortCheck(MECHANICAL_SETTLE_MS)) {
        logMessage("STACK", "Sweep aborted by user during backlash reversal!");
        isStacking = false;
        updateDisplay("Sweep Aborted!");
        return;
    }

    logMessage("STACK", "Backlash seating: Advancing Near (%d ms)", BACKLASH_REVERSE_MS);
    pulseFocusNear(BACKLASH_REVERSE_MS);
    if (delayWithAbortCheck(MECHANICAL_SETTLE_MS)) {
        logMessage("STACK", "Sweep aborted by user during backlash seating!");
        isStacking = false;
        updateDisplay("Sweep Aborted!");
        return;
    }

    // Stacking Sweep
    for (int frame = 0; frame < frame_count; ++frame) {
        float current_z = frame * step_dist_mm;
        logMessage("STACK", "Slice %d/%d (Z = %.3f mm)", frame + 1, frame_count, current_z);

        showStackProgress(frame + 1, frame_count, current_z, "Exposing...");

        if (delayWithAbortCheck(MECHANICAL_SETTLE_MS)) {
            logMessage("STACK", "Sweep aborted by user before slice %d exposure!", frame + 1);
            isStacking = false;
            updateDisplay("Sweep Aborted!");
            return;
        }

        triggerShutter();

        if (frame < frame_count - 1) {
            showStackProgress(frame + 1, frame_count, current_z, "Stepping Focus...");
            pulseFocusNear(step_pulse_ms);
        }
    }

    logMessage("STACK", "Sweep complete. All %d frames captured successfully.", frame_count);
    isStacking = false;
    updateDisplay("Sweep Complete!");
}

// ============================================================================
// BLE Callbacks & Connection Management
// ============================================================================
class SecurityCallback : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() override { return 0; }
    void onPassKeyNotify(uint32_t pass_key) override {}
    bool onConfirmPIN(uint32_t pass_key) override { return true; }
    bool onSecurityRequest() override { return true; }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
        if (cmpl.success) {
            logMessage("BLE", "Authentication & Bonding Succeeded");
        } else {
            logMessage("BLE", "Authentication Failed (Reason: %d)", cmpl.fail_reason);
        }
    }
};

class ClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) override {
        connected = true;
        logMessage("BLE", "Connected to Sony camera");
    }
    void onDisconnect(BLEClient* pclient) override {
        connected = false;
        pRemoteControlChar = nullptr;
        logMessage("BLE", "Disconnected from Sony camera");
    }
};

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
        bool hasService = advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID);
        bool isSonyMfg = false;

        // Check BLE Manufacturer Specific Data for Sony Corporation Company ID (0x012D)
        if (advertisedDevice.haveManufacturerData()) {
            std::string mfg = advertisedDevice.getManufacturerData();
            if (mfg.length() >= 2) {
                // Bluetooth SIG Company ID is stored in little-endian byte order
                uint16_t companyId = ((uint8_t)mfg[1] << 8) | (uint8_t)mfg[0];
                if (companyId == SONY_COMPANY_ID) {
                    isSonyMfg = true;
                }
            }
        }

        // Filter: Device must advertise the Sony Remote Service
        if (hasService) {
            if (advertisedDevice.haveManufacturerData() && !isSonyMfg) {
                logMessage("BLE", "Ignoring non-Sony device advertising service UUID: %s",
                           advertisedDevice.getAddress().toString().c_str());
                return;
            }

            logMessage("BLE", "Found target Sony Camera: %s (RSSI: %d, SonyMfg: %s)",
                       advertisedDevice.getAddress().toString().c_str(),
                       advertisedDevice.getRSSI(),
                       isSonyMfg ? "VERIFIED (0x012D)" : "UNSPECIFIED");

            BLEDevice::getScan()->stop();
            if (targetDevice != nullptr) {
                delete targetDevice;
            }
            targetDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            isScanning = false;
        }
    }
};

void onScanComplete(BLEScanResults results) {
    isScanning = false;
    logMessage("BLE", "Scan complete. Found %d devices", results.getCount());
}

void startBleScan() {
    if (connected || isScanning) return;

    logMessage("BLE", "Starting BLE scan for Sony camera (window: 449, interval: 1349, duration: 15s)...");
    isScanning = true;
    updateDisplay("Scanning...");

    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    pScan->setInterval(1349);
    pScan->setWindow(449);
    pScan->setActiveScan(true);
    pScan->start(15, onScanComplete, false);
}

bool connectToCamera() {
    if (!targetDevice) return false;

    logMessage("BLE", "Connecting to camera: %s...", targetDevice->getAddress().toString().c_str());
    updateDisplay("Connecting...");

    if (!pClient) {
        pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCallback());
    }

    if (!pClient->connect(targetDevice)) {
        logMessage("BLE", "Connection attempt failed");
        updateDisplay("Connect Failed");
        return false;
    }
    pClient->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);

    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (!pRemoteService) {
        logMessage("BLE", "Sony remote GATT service UUID not found");
        pClient->disconnect();
        updateDisplay("No Remote Service");
        return false;
    }

    pRemoteControlChar = pRemoteService->getCharacteristic(CHAR_CONTROL_UUID);
    if (!pRemoteControlChar) {
        logMessage("BLE", "Remote control characteristic UUID not found");
        pClient->disconnect();
        updateDisplay("No Control Char");
        return false;
    }

    connected = true;
    logMessage("BLE", "Sony Remote GATT Service and Characteristic acquired successfully!");
    updateDisplay("Ready");
    return true;
}

// ============================================================================
// Button Debouncing & Press Detection
// ============================================================================
enum ButtonEvent { NONE, SHORT_PRESS, LONG_PRESS };

struct ButtonState {
    uint8_t pin;
    bool lastReading;
    bool state;
    uint32_t lastDebounceTime;
    uint32_t pressStartTime;
    bool longPressHandled;
};

static ButtonState btnA = {BUTTON_A, HIGH, HIGH, 0, 0, false};
static ButtonState btnB = {BUTTON_B, HIGH, HIGH, 0, 0, false};
static ButtonState btnC = {BUTTON_C, HIGH, HIGH, 0, 0, false};

ButtonEvent checkButton(ButtonState& btn, uint32_t now) {
    int reading = digitalRead(btn.pin);
    ButtonEvent evt = NONE;

    if (reading != btn.lastReading) {
        btn.lastDebounceTime = now;
    }
    btn.lastReading = reading;

    if ((now - btn.lastDebounceTime) > 30) {
        if (reading != btn.state) {
            btn.state = reading;
            if (btn.state == LOW) {
                // Button just pressed down
                btn.pressStartTime = now;
                btn.longPressHandled = false;
            } else {
                // Button released up
                if (!btn.longPressHandled && (now - btn.pressStartTime >= 30)) {
                    evt = SHORT_PRESS;
                }
            }
        } else if (btn.state == LOW && !btn.longPressHandled) {
            if (now - btn.pressStartTime >= 600) {
                btn.longPressHandled = true;
                evt = LONG_PRESS;
            }
        }
    }
    return evt;
}

// ============================================================================
// Serial Command Processor
// ============================================================================
void handleSerial() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    logMessage("CMD", "[SERIAL] %s", line.c_str());

    if (line.startsWith("RUN") || line.startsWith("run")) {
        float r_mm = currentRangeMm, ruler_cm = currentRulerCm;
        int frames = currentFrames;
        int parsed = sscanf(line.c_str() + 3, "%f %d %f", &r_mm, &frames, &ruler_cm);
        if (parsed >= 1) currentRangeMm = r_mm;
        if (parsed >= 2) currentFrames = frames;
        if (parsed >= 3) currentRulerCm = ruler_cm;

        executeStackingSweep(currentRangeMm, currentFrames, currentRulerCm);
    } 
    else if (line.startsWith("SET") || line.startsWith("set")) {
        float r_mm = currentRangeMm, ruler_cm = currentRulerCm;
        int frames = currentFrames;
        int parsed = sscanf(line.c_str() + 3, "%f %d %f", &r_mm, &frames, &ruler_cm);
        if (parsed >= 1) currentRangeMm = r_mm;
        if (parsed >= 2) currentFrames = frames;
        if (parsed >= 3) currentRulerCm = ruler_cm;

        logMessage("CONFIG", "Parameters updated: Range = %.2f mm | Frames = %d | Ruler = %.1f cm",
                   currentRangeMm, currentFrames, currentRulerCm);
        updateDisplay("Config Updated");
    }
    else if (line.equalsIgnoreCase("SHUTTER") || line.equalsIgnoreCase("SNAP")) {
        if (connected) {
            updateDisplay("Shutter firing...");
            triggerShutter();
            updateDisplay("Shutter fired");
        } else {
            logMessage("ERROR", "Cannot trigger shutter: Camera not connected");
        }
    }
    else if (line.startsWith("NEAR") || line.startsWith("near")) {
        int duration = 100;
        sscanf(line.c_str() + 4, "%d", &duration);
        if (duration <= 0) duration = 100;
        if (connected) {
            updateDisplay("Focus Near...");
            pulseFocusNear(duration);
            updateDisplay("Ready");
        } else {
            logMessage("ERROR", "Cannot pulse focus: Camera not connected");
        }
    }
    else if (line.startsWith("FAR") || line.startsWith("far")) {
        int duration = 100;
        sscanf(line.c_str() + 3, "%d", &duration);
        if (duration <= 0) duration = 100;
        if (connected) {
            updateDisplay("Focus Far...");
            pulseFocusFar(duration);
            updateDisplay("Ready");
        } else {
            logMessage("ERROR", "Cannot pulse focus: Camera not connected");
        }
    }
    else if (line.equalsIgnoreCase("SCAN")) {
        startBleScan();
    }
    else if (line.equalsIgnoreCase("LOG") || line.equalsIgnoreCase("DUMP_LOG")) {
        dumpLog();
    }
    else if (line.equalsIgnoreCase("CLEAR_LOG")) {
        clearLog();
    }
    else if (line.equalsIgnoreCase("LOG_STATUS")) {
        printLogStatus();
    }
    else if (line.equalsIgnoreCase("HELP") || line.equalsIgnoreCase("?")) {
        Serial.println(F("\n--- Available Commands ---"));
        Serial.println(F("RUN [range_mm] [frames] [ruler_cm] - Execute stacking sweep"));
        Serial.println(F("SET [range_mm] [frames] [ruler_cm] - Update stacking parameters"));
        Serial.println(F("SHUTTER                             - Trigger camera shutter"));
        Serial.println(F("NEAR [duration_ms]                 - Pulse manual focus near (default: 100ms)"));
        Serial.println(F("FAR  [duration_ms]                 - Pulse manual focus far  (default: 100ms)"));
        Serial.println(F("SCAN                               - Start BLE scan for Sony camera"));
        Serial.println(F("STOP                               - Abort active stacking sweep"));
        Serial.println(F("LOG / DUMP_LOG                     - Dump flash log file (/stacker.log)"));
        Serial.println(F("CLEAR_LOG                          - Clear flash log file"));
        Serial.println(F("LOG_STATUS                         - Show log size and LittleFS usage"));
        Serial.println(F("HELP                               - Show this menu\n"));
    }
    else {
        logMessage("WARN", "Unknown command: %s. Type HELP for commands.", line.c_str());
    }
}

// ============================================================================
// Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 800); // Wait briefly for CDC Serial on ESP32-S3

    // Configure FeatherWing buttons with internal pullups
    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    // Initialize display on standard I2C pins
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        hasDisplay = true;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("Sony a7C Stacker"));
        display.println(F("Initializing..."));
        display.display();
        delay(300);
    } else {
        hasDisplay = false;
    }

    // Initialize LittleFS & Load Calibration
    fsReady = LittleFS.begin(true);
    if (fsReady) {
        logMessage("FS", "LittleFS mounted successfully");
        loadCalibrationCSV();
    } else {
        logMessage("WARN", "LittleFS mount failed. Using hardcoded calibration defaults.");
        for (size_t i = 0; i < DEFAULT_CAL_SIZE; ++i) {
            calTable.push_back(DEFAULT_CAL_TABLE[i]);
        }
    }

    logMessage("BOOT", "Sony a7C BLE Focus Stacker Controller initialized");
    logMessage("OLED", "%s", hasDisplay ? "SSD1306 display initialized (128x32)" : "SSD1306 not detected (headless mode)");

    // Initialize BLE Subsystem
    BLEDevice::init("ESP32-RMT-P1BT");
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);
    BLEDevice::setSecurityCallbacks(new SecurityCallback());

    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    logMessage("BLE", "Device initialized as 'ESP32-RMT-P1BT' with MITM bonding");

    startBleScan();
}

void loop() {
    uint32_t now = millis();

    // Check for disconnection / connection transitions to refresh UI
    static bool lastConnectedState = false;
    if (lastConnectedState != connected) {
        lastConnectedState = connected;
        updateDisplay(connected ? "Connected!" : "Disconnected");
    }

    // Check for scan finish transition
    static bool lastScanningState = false;
    if (lastScanningState != isScanning) {
        lastScanningState = isScanning;
        if (!isScanning && !connected && !doConnect) {
            updateDisplay("Scan Finished");
        }
    }

    // Check if target device was found and connection needs to be initiated
    if (doConnect) {
        doConnect = false;
        if (connectToCamera()) {
            logMessage("READY", "Camera connected! Ready for commands.");
            updateDisplay("Connected!");
        } else {
            updateDisplay("Connect Failed");
        }
    }

    // Process button events (debounced)
    ButtonEvent evtA = checkButton(btnA, now);
    ButtonEvent evtB = checkButton(btnB, now);
    ButtonEvent evtC = checkButton(btnC, now);

    // Button A: Run Stacking Sweep (or start BLE scan if disconnected)
    if (evtA == SHORT_PRESS || evtA == LONG_PRESS) {
        if (!connected) {
            logMessage("CMD", "[BUTTON] Button A pressed -> START_SCAN (camera disconnected)");
            startBleScan();
        } else {
            logMessage("CMD", "[BUTTON] Button A pressed -> START_SWEEP (R:%.2fmm, N:%d, L:%.1fcm)",
                       currentRangeMm, currentFrames, currentRulerCm);
            executeStackingSweep(currentRangeMm, currentFrames, currentRulerCm);
        }
    }

    // Button B: Short -> Trigger Shutter, Long -> Focus Far (-)
    if (evtB == SHORT_PRESS) {
        if (connected) {
            logMessage("CMD", "[BUTTON] Button B short press -> TRIGGER_SHUTTER");
            updateDisplay("Shutter firing...");
            triggerShutter();
            updateDisplay("Shutter fired");
        } else {
            logMessage("CMD", "[BUTTON] Button B pressed -> START_SCAN (camera disconnected)");
            startBleScan();
        }
    } else if (evtB == LONG_PRESS) {
        if (connected) {
            logMessage("CMD", "[BUTTON] Button B long press -> FOCUS_FAR (100 ms)");
            updateDisplay("Focus Far (-100ms)");
            pulseFocusFar(100);
            updateDisplay("Ready");
        }
    }

    // Button C: Short -> Focus Near (+), Long -> Focus Far (-)
    if (evtC == SHORT_PRESS) {
        if (connected) {
            logMessage("CMD", "[BUTTON] Button C short press -> FOCUS_NEAR (100 ms)");
            updateDisplay("Focus Near (+100ms)");
            pulseFocusNear(100);
            updateDisplay("Ready");
        } else {
            logMessage("CMD", "[BUTTON] Button C pressed -> START_SCAN (camera disconnected)");
            startBleScan();
        }
    } else if (evtC == LONG_PRESS) {
        if (connected) {
            logMessage("CMD", "[BUTTON] Button C long press -> FOCUS_FAR (100 ms)");
            updateDisplay("Focus Far (-100ms)");
            pulseFocusFar(100);
            updateDisplay("Ready");
        }
    }

    // Process serial incoming commands
    handleSerial();

    delay(10);
}
