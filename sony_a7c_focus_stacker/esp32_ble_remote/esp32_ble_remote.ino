#include <Arduino.h>
#include <LittleFS.h>
#include <vector>
#include <algorithm>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLE2902.h>

// Sony RMT-P1BT Remote GATT Service & Characteristic UUIDs
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

static BLERemoteCharacteristic* pRemoteControlChar = nullptr;
static BLEAdvertisedDevice* targetDevice = nullptr;
static bool doConnect = false;
static bool connected = false;

void writeDefaultCalibrationFile() {
    File file = LittleFS.open(CAL_FILE_PATH, FILE_WRITE);
    if (!file) {
        Serial.println("[FS] Failed to create default calibration file.");
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
    Serial.println("[FS] Generated default macro_calibration.csv in flash.");
}

bool loadCalibrationCSV() {
    calTable.clear();

    if (!LittleFS.exists(CAL_FILE_PATH)) {
        Serial.println("[FS] File missing. Creating default template...");
        writeDefaultCalibrationFile();
    }

    File file = LittleFS.open(CAL_FILE_PATH, FILE_READ);
    if (!file) {
        Serial.println("[FS] Error opening CSV file.");
        return false;
    }

    file.readStringUntil('\n'); // Skip header
    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;

        float r_cm = 0, d_mm = 0, ms_mm = 0;
        if (sscanf(line.c_str(), "%f,%f,%f", &r_cm, &d_mm, &ms_mm) == 3) {
            calTable.push_back({r_cm, d_mm, ms_mm});
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

    Serial.printf("[FS] Loaded %u calibration points from %s.\n", (unsigned int)calTable.size(), CAL_FILE_PATH);
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

void sendBleCommand(const uint8_t* cmd, size_t len) {
    if (connected && pRemoteControlChar != nullptr) {
        pRemoteControlChar->writeValue((uint8_t*)cmd, len, false);
    }
}

void pulseFocusNear(int durationMs) {
    sendBleCommand(CMD_FOCUS_NEAR_PRESS, 3);
    delay(durationMs);
    sendBleCommand(CMD_FOCUS_NEAR_REL, 3);
}

void pulseFocusFar(int durationMs) {
    sendBleCommand(CMD_FOCUS_FAR_PRESS, 3);
    delay(durationMs);
    sendBleCommand(CMD_FOCUS_FAR_REL, 3);
}

void triggerShutter() {
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
    delay(SHUTTER_CYCLE_MS);
}

void executeStackingSweep(float range_mm, int frame_count, float ruler_index_cm) {
    if (!connected) {
        Serial.println("[ERROR] Bluetooth connection not established.");
        return;
    }
    if (frame_count < 2) frame_count = 2;

    float ms_per_mm = interpolatePulsePerMm(ruler_index_cm);
    float step_dist_mm = range_mm / (frame_count - 1);
    int step_pulse_ms = max(15, (int)round(step_dist_mm * ms_per_mm));

    Serial.printf("[STACK] Range: %.2f mm | Frames: %d | Ruler: %.1f cm | Pulse: %d ms\n",
                  range_mm, frame_count, ruler_index_cm, step_pulse_ms);

    // Backlash Elimination at Start Plane (Z = 0)
    // Reverse Far, then advance Near by the exact same duration to seat gear teeth right at Z=0
    Serial.println("[STACK] Reversing Far (Backlash Margin)...");
    pulseFocusFar(BACKLASH_REVERSE_MS);
    delay(MECHANICAL_SETTLE_MS);

    Serial.println("[STACK] Seating Geartrain Near to return to Z=0 origin...");
    pulseFocusNear(BACKLASH_REVERSE_MS);
    delay(MECHANICAL_SETTLE_MS);

    // Stacking Sweep
    for (int frame = 0; frame < frame_count; ++frame) {
        float current_z = frame * step_dist_mm;
        Serial.printf("[STACK] Slice %d/%d (Z = %.3f mm)\n", frame + 1, frame_count, current_z);

        delay(MECHANICAL_SETTLE_MS);
        triggerShutter();

        if (frame < frame_count - 1) {
            pulseFocusNear(step_pulse_ms);
        }
    }
    Serial.println("[STACK] Sweep complete.");
}

class SecurityCallback : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() { return 0; }
    void onPassKeyNotify(uint32_t pass_key) {}
    bool onConfirmPIN(uint32_t pass_key) { return true; }
    bool onSecurityRequest() { return true; }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
        if (cmpl.success) Serial.println("[BLE] Authentication & Bonding Succeeded.");
        else Serial.printf("[BLE] Authentication Failed: %d\n", cmpl.fail_reason);
    }
};

class ClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        connected = true;
        Serial.println("[BLE] Connected to Sony camera.");
    }
    void onDisconnect(BLEClient* pclient) {
        connected = false;
        Serial.println("[BLE] Disconnected from Sony camera.");
    }
};

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
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
            // If manufacturer data is present, confirm it matches Sony ID to reject unrelated devices
            if (advertisedDevice.haveManufacturerData() && !isSonyMfg) {
                Serial.printf("[BLE] Ignoring non-Sony device advertising service UUID: %s\n",
                              advertisedDevice.getAddress().toString().c_str());
                return;
            }

            Serial.printf("[BLE] Found target Sony Camera: %s (RSSI: %d, SonyMfg: %s)\n",
                          advertisedDevice.getAddress().toString().c_str(),
                          advertisedDevice.getRSSI(),
                          isSonyMfg ? "VERIFIED (0x012D)" : "UNSPECIFIED");

            BLEDevice::getScan()->stop();
            targetDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
        }
    }
};

bool connectToCamera() {
    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCallback());

    if (!pClient->connect(targetDevice)) return false;
    pClient->setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);

    BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
    if (!pRemoteService) {
        pClient->disconnect();
        return false;
    }

    pRemoteControlChar = pRemoteService->getCharacteristic(CHAR_CONTROL_UUID);
    if (!pRemoteControlChar) {
        pClient->disconnect();
        return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    if (LittleFS.begin(true)) {
        loadCalibrationCSV();
    }

    BLEDevice::init("ESP32-RMT-P1BT");
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);
    BLEDevice::setSecurityCallbacks(new SecurityCallback());

    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    BLEScan* pScan = BLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
    pScan->setInterval(1349);
    pScan->setWindow(449);
    pScan->setActiveScan(true);
    Serial.println("[BLE] Scanning for Sony camera in pairing mode...");
    pScan->start(15, false);
}

void loop() {
    if (doConnect) {
        if (connectToCamera()) {
            Serial.println("[READY] Send: RUN <range_mm> <frames> <ruler_cm>");
        }
        doConnect = false;
    }

    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.startsWith("RUN")) {
            float r_mm = 4.0f, ruler_cm = 3.5f;
            int frames = 10;
            int parsed = sscanf(line.c_str(), "RUN %f %d %f", &r_mm, &frames, &ruler_cm);
            if (parsed >= 2) {
                executeStackingSweep(r_mm, frames, (parsed == 3) ? ruler_cm : 3.5f);
            }
        }
    }
    delay(20);
}
