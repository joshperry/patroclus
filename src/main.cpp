/*
 * Patroclus - VE.CAN Energy Meter Reader for Victron GX
 * 
 * Taps internal CAN bus of Victron MV-3P75CT 3-phase meter,
 * decodes per-phase measurements, and publishes to Victron GX as
 * virtual grid meters via dbus-mqtt-devices driver.
 * 
 * Features:
 * - Flexible phase-to-meter mapping (combine physical phases into virtual meters)
 * - HTTP OTA for remote firmware updates (POST to /update)
 * - MQTT command interface for remote control
 * - Remote CAN capture streaming
 * - Energy accumulation (kWh) from meter registers
 * - Per-phase power factor
 * 
 * Hardware: Seeed Studio XIAO ESP32-S3
 * 
 * See config.h for configuration options.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <driver/twai.h>
#include "config.h"

#if MQTT_USE_TLS
#include <WiFiClientSecure.h>
#endif
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#if ENABLE_HTTP_OTA
#include <WebServer.h>
#include <Update.h>
#endif

// ============================================================================
// Data Structures
// ============================================================================

// Per-phase measurements from CAN (physical phases from meter)
struct PhaseMeasurement {
    float voltage;          // V AC
    float current;          // A
    float power;            // W (real power, negative = import)
    float energyForward;    // kWh (bought/consumed) - per phase if available
    float energyReverse;    // kWh (sold/exported)
    float powerFactor;      // 0.0 - 1.0
    unsigned long lastUpdate;   // millis() of last CAN update
    bool valid;             // Have we received data?
};

// Physical meter data (3 phases from MV-3P75CT)
struct MeterData {
    float frequency;            // Hz (common to all phases)
    float totalEnergyForward;   // kWh (total from meter register 0x50)
    PhaseMeasurement phases[3]; // Physical L1, L2, L3
};

// Virtual meter configuration (published to Venus OS)
struct VirtualMeterConfig {
    const char* serviceId;
    const char* name;
    const char* deviceType;
    int phaseCount;         // 1, 2, or 3
    int physicalPhase[3];   // Map virtual L1/L2/L3 to physical phase index (-1 = unused)
};

// Virtual meter registration state
struct VirtualMeterState {
    bool registered;
    char topicPath[64];     // W/portalId/grid/N path from registration
    int deviceInstance;
};

// State machine states
enum DeviceState {
    STATE_INIT,
    STATE_WIFI_CONNECT,
    STATE_MQTT_CONNECT,
    STATE_REGISTER,
    STATE_WAIT_REGISTRATION,
    STATE_RUNNING
};

// CAN capture frame
struct CanCapture {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    bool extd;
};

// ============================================================================
// Virtual Meter Configuration (from config.h defines)
// ============================================================================

const VirtualMeterConfig virtualMeters[METER_COUNT] = {
    {
        METER0_SERVICE_ID,
        METER0_NAME,
        METER0_DEVICE_TYPE,
        METER0_PHASE_COUNT,
        { METER0_VIRT_L1_PHYS, METER0_VIRT_L2_PHYS, METER0_VIRT_L3_PHYS }
    },
#if METER_COUNT > 1
    {
        METER1_SERVICE_ID,
        METER1_NAME,
        METER1_DEVICE_TYPE,
        METER1_PHASE_COUNT,
        { METER1_VIRT_L1_PHYS, METER1_VIRT_L2_PHYS, METER1_VIRT_L3_PHYS }
    },
#endif
#if METER_COUNT > 2
    {
        METER2_SERVICE_ID,
        METER2_NAME,
        METER2_DEVICE_TYPE,
        METER2_PHASE_COUNT,
        { METER2_VIRT_L1_PHYS, METER2_VIRT_L2_PHYS, METER2_VIRT_L3_PHYS }
    },
#endif
};

// ============================================================================
// Global State
// ============================================================================

#if MQTT_USE_TLS
WiFiClientSecure wifiClient;
#else
WiFiClient wifiClient;
#endif
PubSubClient mqttClient(wifiClient);

#if ENABLE_HTTP_OTA
WebServer httpServer(HTTP_OTA_PORT);
#endif

MeterData meterData;                            // Physical phase data
VirtualMeterState meterStates[METER_COUNT];     // Virtual meter states
DeviceState deviceState = STATE_INIT;

char portalId[32] = "";
bool allMetersRegistered = false;
IPAddress mqttServerIP;

// Timing
unsigned long lastPublishTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long stateEnteredTime = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;
unsigned long lastLedToggle = 0;
unsigned long lastCapturePublish = 0;
bool ledState = false;

// CAN dump/capture modes
bool canDumpSerial = false;         // Dump to serial
bool canCaptureEnabled = false;     // Capture to MQTT
CanCapture captureBuffer[CAN_CAPTURE_BUFFER_SIZE];
int captureCount = 0;

// OTA state
bool otaInProgress = false;

#if ENABLE_GENSET_HANDOFF
// Generator handoff state. We watch any Venus genset's /StatusCode and, while it's
// running, move the transfer-switched inlet meter's readings onto the genset's AC
// paths while zeroing its grid meter (see config.h). All state resets per connection.
int  gensetInstance   = -1;       // genset instance we're tracking (-1 = none yet)
bool gensetRunning    = false;    // last seen StatusCode == GENSET_RUNNING_STATUSCODE
bool gensetAcCapable  = false;    // genset advertises writable Ac/* paths (else can't hand off)
bool gensetHandoffActive = false; // currently routing inlet -> genset + zeroing grid
bool gensetSubscribed = false;    // subscribed to N/<portal>/genset/... this connection
bool gensetFirstKa    = true;     // next keepalive is the connection's first
unsigned long gensetKaCount = 0;  // keepalives sent (gates periodic full republish)
unsigned long lastGensetKa = 0;
String topicGensetStatus;         // N/<portal>/genset/+/StatusCode
String topicGensetAc;             // N/<portal>/genset/+/Ac/#
String topicGensetKeepalive;      // R/<portal>/keepalive
#endif

// ============================================================================
// MQTT Topics
// ============================================================================

String topicCommand;    // device/patroclus/Command - incoming commands
String topicLog;        // device/patroclus/Log - outgoing logs/responses
String topicCapture;    // device/patroclus/Capture - CAN frame stream
String topicDBus;       // device/patroclus/DBus - registration responses
String topicStatus;     // device/patroclus/Status - registration

// ============================================================================
// Debug Logging
// ============================================================================

#if ENABLE_SERIAL_DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif

// Forward declarations
void publishLog(const char* message);
void publishLog(const String& message);
void publishVirtualMeter(int meterIndex, bool forceZero = false);

// ============================================================================
// LED Status Functions
// ============================================================================

#if ENABLE_LED_STATUS
void ledSetup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);
}

void ledSolid() {
    digitalWrite(LED_PIN, LED_ON);
}

void ledOff() {
    digitalWrite(LED_PIN, LED_OFF);
}

void ledBlink(unsigned long intervalMs) {
    unsigned long now = millis();
    if (now - lastLedToggle >= intervalMs) {
        lastLedToggle = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
    }
}

void ledDoubleBlink() {
    unsigned long phase = (millis() / 100) % 20;
    if (phase == 0 || phase == 2) {
        digitalWrite(LED_PIN, LED_ON);
    } else {
        digitalWrite(LED_PIN, LED_OFF);
    }
}

void updateLedStatus() {
    if (otaInProgress) {
        ledBlink(50);   // Very fast blink during OTA
        return;
    }
    
    switch (deviceState) {
        case STATE_WIFI_CONNECT:
            ledBlink(500);      // 1Hz - slow blink
            break;
        case STATE_MQTT_CONNECT:
            ledBlink(125);      // 4Hz - fast blink
            break;
        case STATE_REGISTER:
        case STATE_WAIT_REGISTRATION:
            ledDoubleBlink();
            break;
        case STATE_RUNNING:
            ledSolid();
            break;
        default:
            ledOff();
            break;
    }
}
#else
void ledSetup() {}
void ledSolid() {}
void ledOff() {}
void updateLedStatus() {}
#endif

// ============================================================================
// HTTP OTA Functions
// ============================================================================

#if ENABLE_HTTP_OTA

void handleOtaIndex() {
    httpServer.send(200, "text/html",
        "<html><head><title>Patroclus OTA</title></head><body>"
        "<h1>Patroclus OTA Update</h1>"
        "<p>Version: " DEVICE_VERSION "</p>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin'><br><br>"
        "<input type='submit' value='Upload Firmware'>"
        "</form></body></html>"
    );
}

void handleOtaUpdate() {
    httpServer.sendHeader("Connection", "close");
    if (Update.hasError()) {
        httpServer.send(500, "text/plain", "Update failed: " + String(Update.errorString()));
    } else {
        httpServer.send(200, "text/plain", "OK - Rebooting...");
        delay(500);
        ESP.restart();
    }
}

void handleOtaUpload() {
    HTTPUpload& upload = httpServer.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        otaInProgress = true;
        DEBUG_PRINTF("OTA Start: %s\n", upload.filename.c_str());
        publishLog("OTA update starting: " + upload.filename);
        
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            DEBUG_PRINTF("Update.begin failed: %s\n", Update.errorString());
            publishLog("OTA begin failed: " + String(Update.errorString()));
        }
    } 
    else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            DEBUG_PRINTF("Update.write failed: %s\n", Update.errorString());
        }
        // Progress indicator
        static size_t lastReport = 0;
        if (upload.totalSize - lastReport > 100000) {
            DEBUG_PRINTF("OTA Progress: %u bytes\n", upload.totalSize);
            lastReport = upload.totalSize;
        }
    } 
    else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            DEBUG_PRINTF("OTA Success: %u bytes\n", upload.totalSize);
            publishLog("OTA complete: " + String(upload.totalSize) + " bytes");
        } else {
            DEBUG_PRINTF("Update.end failed: %s\n", Update.errorString());
            publishLog("OTA end failed: " + String(Update.errorString()));
        }
        otaInProgress = false;
    }
    else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaInProgress = false;
        DEBUG_PRINTLN("OTA Aborted");
        publishLog("OTA aborted");
    }
}

void handleOtaVersion() {
    String json = "{\"version\":\"" DEVICE_VERSION "\",\"client_id\":\"" CLIENT_ID "\"}";
    httpServer.send(200, "application/json", json);
}

void httpOtaSetup() {
    httpServer.on("/", HTTP_GET, handleOtaIndex);
    httpServer.on("/version", HTTP_GET, handleOtaVersion);
    httpServer.on("/update", HTTP_POST, handleOtaUpdate, handleOtaUpload);
    httpServer.begin();
    DEBUG_PRINTF("HTTP OTA server on port %d\n", HTTP_OTA_PORT);
}

#else
void httpOtaSetup() {}
#endif

// ============================================================================
// TWAI (CAN) Functions
// ============================================================================

bool twaiSetup() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN,
        CAN_RX_PIN,
        #if CAN_ACTIVE_MODE
        TWAI_MODE_NORMAL
        #else
        TWAI_MODE_LISTEN_ONLY
        #endif
    );
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        DEBUG_PRINTF("TWAI driver install failed: %d\n", result);
        return false;
    }
    
    result = twai_start();
    if (result != ESP_OK) {
        DEBUG_PRINTF("TWAI start failed: %d\n", result);
        return false;
    }
    
    DEBUG_PRINTLN("TWAI initialized");
    #if CAN_ACTIVE_MODE
    DEBUG_PRINTLN("  Mode: ACTIVE (will ACK frames)");
    #else
    DEBUG_PRINTLN("  Mode: LISTEN_ONLY (passive)");
    #endif
    
    return true;
}

// Process a single CAN frame - VE.CAN energy meter format
void processCanFrame(const twai_message_t& msg) {
    // Dump to serial if enabled
    if (canDumpSerial) {
        if (msg.extd) {
            Serial.printf("CAN: %08X [%d]", msg.identifier, msg.data_length_code);
        } else {
            Serial.printf("CAN: %03X [%d]", msg.identifier, msg.data_length_code);
        }
        for (int i = 0; i < msg.data_length_code; i++) {
            Serial.printf(" %02X", msg.data[i]);
        }
        Serial.println();
    }
    
    // Buffer for MQTT capture if enabled
    if (canCaptureEnabled && captureCount < CAN_CAPTURE_BUFFER_SIZE) {
        CanCapture& cap = captureBuffer[captureCount++];
        cap.id = msg.identifier;
        cap.len = msg.data_length_code;
        cap.extd = msg.extd;
        memcpy(cap.data, msg.data, 8);
    }

    unsigned long now = millis();
    
    // Handle register-based messages (0x19EFFF40)
    // Format: [seq_lo][seq_hi][register][sub_seq][value 32-bit LE]
    if (msg.identifier == CAN_ID_REGISTER) {
        uint8_t reg = msg.data[2];
        uint32_t regVal = (uint32_t)msg.data[4] | 
                          ((uint32_t)msg.data[5] << 8) | 
                          ((uint32_t)msg.data[6] << 16) | 
                          ((uint32_t)msg.data[7] << 24);
        
        switch (reg) {
            case REG_ENERGY_FORWARD:
                // Energy in Wh, convert to kWh
                meterData.totalEnergyForward = regVal / 1000.0f;
                break;
            //case REG_L1_PF:
            //    meterData.phases[0].powerFactor = regVal / 1000.0f;
            //    break;
            //case REG_L2_PF:
            //    meterData.phases[1].powerFactor = regVal / 1000.0f;
            //    break;
            //case REG_L3_PF:
            //    meterData.phases[2].powerFactor = regVal / 1000.0f;
            //    break;
        }
        return;  // Don't fall through to 0x19F30xxx parsing
    }

    // Parse 0x19F30xxx VE.CAN energy meter messages
    // Format: [seq_lo][seq_hi][val1_lo][val1_hi][val2_b0][val2_b1][val2_b2][val2_b3]
    
    uint16_t val16 = (msg.data[3] << 8) | msg.data[2];
    int32_t val32 = (int32_t)msg.data[4] | 
                    ((int32_t)msg.data[5] << 8) | 
                    ((int32_t)msg.data[6] << 16) | 
                    ((int32_t)msg.data[7] << 24);
    
    switch (msg.identifier) {
        // L1 Voltage/Frequency
        case CAN_ID_L1_VOLTAGE:
            meterData.phases[0].voltage = val16 / 10.0f;
            meterData.frequency = ((msg.data[7] << 8) | msg.data[6]) / 10.0f;
            meterData.phases[0].lastUpdate = now;
            meterData.phases[0].valid = true;
            break;
            
        // L1 Current/Power
        case CAN_ID_L1_POWER:
            meterData.phases[0].current = val16 / 10.0f;
            meterData.phases[0].power = (float)val32;  // Watts, negative = import
            meterData.phases[0].lastUpdate = now;
            meterData.phases[0].valid = true;
            break;
            
        // L2 Voltage/Frequency
        case CAN_ID_L2_VOLTAGE:
            meterData.phases[1].voltage = val16 / 10.0f;
            meterData.phases[1].lastUpdate = now;
            meterData.phases[1].valid = true;
            break;
            
        // L2 Current/Power
        case CAN_ID_L2_POWER:
            meterData.phases[1].current = val16 / 10.0f;
            meterData.phases[1].power = (float)val32;
            meterData.phases[1].lastUpdate = now;
            meterData.phases[1].valid = true;
            break;
            
        // L3 Voltage/Frequency
        case CAN_ID_L3_VOLTAGE:
            meterData.phases[2].voltage = val16 / 10.0f;
            meterData.phases[2].lastUpdate = now;
            meterData.phases[2].valid = true;
            break;
            
        // L3 Current/Power
        case CAN_ID_L3_POWER:
            meterData.phases[2].current = val16 / 10.0f;
            meterData.phases[2].power = (float)val32;
            meterData.phases[2].lastUpdate = now;
            meterData.phases[2].valid = true;
            break;
    }
}

// Poll for CAN frames (non-blocking)
void twaiPoll() {
    twai_message_t msg;
    
    // Process all available frames
    while (twai_receive(&msg, 0) == ESP_OK) {
        processCanFrame(msg);
    }
}

// Publish captured CAN frames to MQTT
void publishCapturedFrames() {
    if (!canCaptureEnabled || captureCount == 0) return;
    if (!mqttClient.connected()) return;
    
    unsigned long now = millis();
    if (now - lastCapturePublish < CAN_CAPTURE_INTERVAL_MS) return;
    lastCapturePublish = now;
    
    // Build JSON array of captured frames
    JsonDocument doc;
    JsonArray frames = doc.to<JsonArray>();
    
    for (int i = 0; i < captureCount; i++) {
        CanCapture& cap = captureBuffer[i];
        JsonObject frame = frames.add<JsonObject>();
        
        char idStr[12];
        if (cap.extd) {
            snprintf(idStr, sizeof(idStr), "%08X", cap.id);
        } else {
            snprintf(idStr, sizeof(idStr), "%03X", cap.id);
        }
        frame["id"] = idStr;
        
        char dataStr[24];
        int pos = 0;
        for (int j = 0; j < cap.len; j++) {
            pos += snprintf(dataStr + pos, sizeof(dataStr) - pos, "%02X", cap.data[j]);
        }
        frame["d"] = dataStr;
    }
    
    char payload[2048];
    size_t len = serializeJson(doc, payload, sizeof(payload));
    
    if (len > 0 && len < sizeof(payload)) {
        mqttClient.publish(topicCapture.c_str(), payload);
    }
    
    captureCount = 0;
}

// ============================================================================
// WiFi Functions
// ============================================================================

void wifiSetup() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
}

bool wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }
    
    unsigned long now = millis();
    if (now - lastWiFiAttempt < WIFI_RECONNECT_DELAY_MS) {
        return false;
    }
    lastWiFiAttempt = now;
    
    DEBUG_PRINTF("Connecting to WiFi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    return false;
}

bool wifiIsConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool resolveMqttServer() {
    if (mqttServerIP.fromString(MQTT_SERVER)) {
        DEBUG_PRINTF("MQTT server is IP: %s\n", mqttServerIP.toString().c_str());
        return true;
    }
    
    DEBUG_PRINTF("Resolving %s...\n", MQTT_SERVER);
    if (WiFi.hostByName(MQTT_SERVER, mqttServerIP)) {
        DEBUG_PRINTF("Resolved %s to %s\n", MQTT_SERVER, mqttServerIP.toString().c_str());
        return true;
    }
    
    DEBUG_PRINTF("Failed to resolve %s\n", MQTT_SERVER);
    return false;
}

// ============================================================================
// MQTT Log/Response Functions
// ============================================================================

void publishLog(const char* message) {
    if (mqttClient.connected()) {
        mqttClient.publish(topicLog.c_str(), message);
    }
    DEBUG_PRINTLN(message);
}

void publishLog(const String& message) {
    publishLog(message.c_str());
}

// ============================================================================
// Generator Handoff
// ============================================================================

#if ENABLE_GENSET_HANDOFF

// Parse "N/<portalId>/genset/<inst>/<rest>". Returns the instance and points
// *restOut at the remainder ("StatusCode", "Ac/L1/Power", ...); -1 if not a genset
// topic for our portal.
int parseGensetTopic(const char* topic, const char** restOut) {
    if (portalId[0] == '\0') return -1;
    char prefix[48];
    int plen = snprintf(prefix, sizeof(prefix), "N/%s/genset/", portalId);
    if (plen <= 0 || strncmp(topic, prefix, plen) != 0) return -1;
    const char* p = topic + plen;           // "<inst>/<rest>"
    const char* slash = strchr(p, '/');
    if (!slash || slash == p) return -1;
    *restOut = slash + 1;
    return atoi(p);
}

// Digest an inbound genset N/ message: track run state and AC-path capability.
void handleGensetMessage(int inst, const char* rest, const char* payload) {
    if (GENSET_INSTANCE >= 0 && inst != GENSET_INSTANCE) return;   // not the one we follow

    if (strcmp(rest, "StatusCode") == 0) {
        JsonDocument doc;
        if (deserializeJson(doc, payload)) return;
        if (doc["value"].isNull()) return;
        int sc = doc["value"].as<int>();
        gensetInstance = inst;
        bool run = (sc == GENSET_RUNNING_STATUSCODE);
        if (run != gensetRunning) {
            gensetRunning = run;
            DEBUG_PRINTF("Genset %d StatusCode=%d -> %s\n", inst, sc, run ? "RUNNING" : "stopped");
        }
    } else if (strncmp(rest, "Ac/", 3) == 0) {
        // The path existing on N/ (even null) means it's declared & writable. A genset
        // without AC paths never publishes these, so we never become capable -> we
        // safely leave the inlet meter on grid rather than dropping its power.
        gensetInstance = inst;
        if (!gensetAcCapable) {
            gensetAcCapable = true;
            DEBUG_PRINTF("Genset %d advertises writable AC paths\n", inst);
        }
    }
}

// Write one genset dbus path directly (W/<portal>/genset/<inst>/<leaf>). Patroclus
// isn't the genset's registrant, so we can't use the freakent Proxy channel - but raw
// W/ writes go straight through the flashmq dbus bridge (same path the GX uses).
void writeGensetPath(const char* leaf, float value) {
    char topic[96];
    snprintf(topic, sizeof(topic), "W/%s/genset/%d/%s", portalId, gensetInstance, leaf);
    char payload[48];
    snprintf(payload, sizeof(payload), "{\"value\":%.2f}", value);
    mqttClient.publish(topic, payload);
}

// Publish the inlet meter's phases onto the genset's AC paths (or zero them). Phase
// mapping mirrors the handoff meter's grid mapping, so the genset reports exactly what
// the inlet sees while the generator is the live source.
void publishGensetAc(bool zero) {
    if (gensetInstance < 0 || portalId[0] == '\0') return;
    VirtualMeterConfig const& meter = virtualMeters[GENSET_HANDOFF_METER];
    const char* legs[] = {"Ac/L1", "Ac/L2", "Ac/L3"};
    float totalPower = 0;
    char leaf[32];
    for (int v = 0; v < 3; v++) {
        int phys = meter.physicalPhase[v];
        if (phys < 0 || phys >= 3) continue;
        PhaseMeasurement& d = meterData.phases[phys];
        float volt = zero ? 0 : d.voltage;
        float curr = zero ? 0 : d.current;
        float powr = zero ? 0 : d.power;
        snprintf(leaf, sizeof(leaf), "%s/Voltage", legs[v]); writeGensetPath(leaf, volt);
        snprintf(leaf, sizeof(leaf), "%s/Current", legs[v]); writeGensetPath(leaf, curr);
        snprintf(leaf, sizeof(leaf), "%s/Power",   legs[v]); writeGensetPath(leaf, powr);
        totalPower += powr;
    }
    writeGensetPath("Ac/Power",     totalPower);
    writeGensetPath("Ac/Frequency", zero ? 0 : meterData.frequency);
}

// Zero the handoff meter's grid service via DIRECT W/ writes (using its registration
// topicPath, e.g. W/<portal>/grid/<inst>). Used on the handoff ON edge: the per-cycle
// Proxy zero travels freakent's slower proxy path and lands ~0.7s after the genset's
// direct W/ write, briefly showing the inlet on both services (systemcalc sums them).
// Writing the grid zero on the same direct bridge makes both land together.
void writeGridZero() {
    VirtualMeterState& state = meterStates[GENSET_HANDOFF_METER];
    if (!state.registered || state.topicPath[0] == '\0') return;
    VirtualMeterConfig const& meter = virtualMeters[GENSET_HANDOFF_METER];
    const char* legs[]   = {"Ac/L1", "Ac/L2", "Ac/L3"};
    const char* leaves[] = {"Voltage", "Current", "Power"};
    char topic[96];
    const char* zeroBody = "{\"value\":0}";
    for (int v = 0; v < 3; v++) {
        int phys = meter.physicalPhase[v];
        if (phys < 0 || phys >= 3) continue;
        for (int k = 0; k < 3; k++) {
            snprintf(topic, sizeof(topic), "%s/%s/%s", state.topicPath, legs[v], leaves[k]);
            mqttClient.publish(topic, zeroBody);
        }
    }
    snprintf(topic, sizeof(topic), "%s/Ac/Power", state.topicPath);
    mqttClient.publish(topic, zeroBody);
    snprintf(topic, sizeof(topic), "%s/Ac/Frequency", state.topicPath);
    mqttClient.publish(topic, zeroBody);
}

// Recompute whether we should be handing off, and act on the transition edges.
void updateGensetHandoff() {
    // One-shot notice if a generator is running but we can't move its power to it.
    static bool warnedNoAc = false;
    if (gensetRunning && gensetInstance >= 0 && !gensetAcCapable) {
        if (!warnedNoAc) {
            warnedNoAc = true;
            publishLog("Genset running but exposes no writable AC paths; inlet meter stays on grid");
        }
    } else if (!gensetRunning) {
        warnedNoAc = false;
    }

    bool want = (gensetInstance >= 0) && gensetRunning && gensetAcCapable;
    if (want == gensetHandoffActive) return;

    gensetHandoffActive = want;
    lastPublishTime = 0;    // force an immediate publish of the new routing
    if (want) {
        publishLog("Genset handoff ON: inlet meter -> genset, grid zeroed");
        // Zero the grid on the edge via the fast direct-W/ path (symmetric with the OFF
        // edge zeroing the genset below), so we never briefly report the inlet on BOTH
        // services. The genset's real readings follow on the forced publish next loop.
        writeGridZero();
    } else {
        publishLog("Genset handoff OFF: inlet meter -> grid");
        publishGensetAc(true);  // clear the genset AC so it doesn't hold stale power
    }
}

// Subscribe to genset topics once the portal is known, and run the keepalive that
// keeps the GX pushing N/ genset changes to us (it stops after ~60s of silence).
void gensetMaintenance() {
    if (!mqttClient.connected() || portalId[0] == '\0') return;

    if (!gensetSubscribed) {
        topicGensetStatus    = String("N/") + portalId + "/genset/+/StatusCode";
        topicGensetAc        = String("N/") + portalId + "/genset/+/Ac/#";
        topicGensetKeepalive = String("R/") + portalId + "/keepalive";
        mqttClient.subscribe(topicGensetStatus.c_str());
        mqttClient.subscribe(topicGensetAc.c_str());
        gensetSubscribed = true;
        gensetFirstKa = true;
        gensetKaCount = 0;
        lastGensetKa = 0;
        DEBUG_PRINTLN("Genset handoff: subscribed, watching for a genset");
    }

    unsigned long now = millis();
    if (gensetFirstKa || now - lastGensetKa >= GENSET_KEEPALIVE_MS) {
        lastGensetKa = now;
        // Every Nth keepalive (incl. the first) triggers a full republish so we learn
        // a genset that appeared late + its AC capability; the rest just hold the session.
        bool republish = (gensetKaCount % GENSET_REPUBLISH_EVERY) == 0;
        const char* payload = republish ? "" : "{\"keepalive-options\":[\"suppress-republish\"]}";
        mqttClient.publish(topicGensetKeepalive.c_str(), payload);
        gensetKaCount++;
        gensetFirstKa = false;
    }
}

#endif // ENABLE_GENSET_HANDOFF

// ============================================================================
// MQTT Command Handler
// ============================================================================

void handleCommand(const char* cmd) {
    String command = String(cmd);
    command.trim();
    command.toUpperCase();
    
    DEBUG_PRINTF("Command received: %s\n", command.c_str());
    
    if (command == "STATUS") {
        String response = "--- Status ---\n";
        response += "Version: " + String(DEVICE_VERSION) + "\n";
        response += "State: " + String(deviceState) + "\n";
        response += "WiFi: " + WiFi.localIP().toString() + "\n";
        response += "MQTT: " + String(mqttClient.connected() ? "Connected" : "Disconnected") + "\n";
        response += "OTA: http://" + WiFi.localIP().toString() + ":" + String(HTTP_OTA_PORT) + "/update\n";
        response += "Freq: " + String(meterData.frequency, 2) + " Hz\n";
        response += "Total Energy: " + String(meterData.totalEnergyForward, 3) + " kWh\n";
        
        // Physical phases
        response += "\nPhysical Phases:\n";
        const char* phaseNames[] = {"L1", "L2", "L3"};
        for (int i = 0; i < 3; i++) {
            PhaseMeasurement& data = meterData.phases[i];
            response += "  " + String(phaseNames[i]) + ": ";
            response += "V=" + String(data.voltage, 1);
            response += " I=" + String(data.current, 2);
            response += " P=" + String(data.power, 0);
            response += " PF=" + String(data.powerFactor, 3);
            response += " (" + String(data.valid ? "ok" : "stale") + ")\n";
        }
        
        // Virtual meters
        response += "\nVirtual Meters:\n";
        for (int i = 0; i < METER_COUNT; i++) {
            response += "  " + String(virtualMeters[i].name) + ": ";
            response += String(meterStates[i].registered ? "registered" : "pending") + "\n";
        }
        
        publishLog(response);
    }
    else if (command == "CANDUMP") {
        canDumpSerial = !canDumpSerial;
        publishLog(String("Serial CAN dump: ") + (canDumpSerial ? "ON" : "OFF"));
    }
    else if (command == "CAPTURE") {
        canCaptureEnabled = !canCaptureEnabled;
        captureCount = 0;
        publishLog(String("MQTT CAN capture: ") + (canCaptureEnabled ? "ON" : "OFF"));
    }
    else if (command == "REBOOT") {
        publishLog("Rebooting...");
        delay(500);
        ESP.restart();
    }
    else if (command == "HELP") {
        String help = "Commands:\n";
        help += "  STATUS  - Show device status\n";
        help += "  CANDUMP - Toggle serial CAN dump\n";
        help += "  CAPTURE - Toggle MQTT CAN capture\n";
        help += "  REBOOT  - Restart device\n";
        help += "  HELP    - Show this help\n";
        help += "OTA: POST firmware to http://<ip>:" + String(HTTP_OTA_PORT) + "/update";
        publishLog(help);
    }
    else {
        publishLog("Unknown command. Type HELP for list.");
    }
}

// ============================================================================
// MQTT Functions
// ============================================================================

void mqttCallback(char* topic, byte* payload, unsigned int length);

void mqttSetup() {
    // Build topic strings
    topicCommand = String("device/") + CLIENT_ID + "/Command";
    topicLog = String("device/") + CLIENT_ID + "/Log";
    topicCapture = String("device/") + CLIENT_ID + "/Capture";
    topicDBus = String("device/") + CLIENT_ID + "/DBus";
    topicStatus = String("device/") + CLIENT_ID + "/Status";
    
    #if MQTT_USE_TLS
    wifiClient.setInsecure();   // Skip cert verification for self-signed
    #endif
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(2048);  // Larger buffer for captures
}

String buildLastWillPayload() {
    JsonDocument doc;
    doc["clientId"] = CLIENT_ID;
    doc["connected"] = 0;
    doc["version"] = DEVICE_VERSION;
    doc["services"].to<JsonObject>();
    
    String payload;
    serializeJson(doc, payload);
    return payload;
}

bool mqttConnect() {
    if (mqttClient.connected()) {
        return true;
    }
    
    unsigned long now = millis();
    if (now - lastMQTTAttempt < MQTT_RECONNECT_DELAY_MS) {
        return false;
    }
    lastMQTTAttempt = now;
    
    DEBUG_PRINTF("Connecting to MQTT: %s:%d\n", mqttServerIP.toString().c_str(), MQTT_PORT);
    
    String willPayload = buildLastWillPayload();
    
    if (mqttClient.connect(CLIENT_ID, MQTT_USER, MQTT_PASS, 
                           topicStatus.c_str(), 0, false, willPayload.c_str())) {
        DEBUG_PRINTLN("MQTT connected");
        
        // Subscribe to DBus responses
        mqttClient.subscribe(topicDBus.c_str());
        DEBUG_PRINTF("Subscribed to: %s\n", topicDBus.c_str());
        
        // Subscribe to command topic
        mqttClient.subscribe(topicCommand.c_str());
        DEBUG_PRINTF("Subscribed to: %s\n", topicCommand.c_str());
        
        // Announce presence
        publishLog(String("Online - ") + DEVICE_VERSION + " @ " + WiFi.localIP().toString() + 
                   " OTA: http://" + WiFi.localIP().toString() + ":" + String(HTTP_OTA_PORT) + "/update");
        
        return true;
    } else {
        DEBUG_PRINTF("MQTT connect failed, rc=%d\n", mqttClient.state());
        return false;
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char buffer[1024];
    if (length >= sizeof(buffer)) {
        DEBUG_PRINTLN("MQTT payload too large");
        return;
    }
    memcpy(buffer, payload, length);
    buffer[length] = '\0';
    
    DEBUG_PRINTF("MQTT received [%s]: %s\n", topic, buffer);
    
    // Check if this is a command
    if (String(topic) == topicCommand) {
        handleCommand(buffer);
        return;
    }

#if ENABLE_GENSET_HANDOFF
    // Genset run-state / AC-capability updates (N/<portal>/genset/<inst>/...)
    {
        const char* rest;
        int inst = parseGensetTopic(topic, &rest);
        if (inst >= 0) {
            handleGensetMessage(inst, rest, buffer);
            return;
        }
    }
#endif

    // Check if this is a DBus response
    if (String(topic) != topicDBus) {
        return;
    }
    
    // Parse registration response
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, buffer);
    
    if (error) {
        DEBUG_PRINTF("JSON parse error: %s\n", error.c_str());
        return;
    }
    
    // Extract portalId
    if (doc["portalId"].is<const char*>()) {
        strlcpy(portalId, doc["portalId"] | "", sizeof(portalId));
        DEBUG_PRINTF("Portal ID: %s\n", portalId);
    }
    
    // Extract device instances and topic paths for each virtual meter
    JsonObject deviceInstances = doc["deviceInstance"];
    JsonObject topicPaths = doc["topicPath"];
    
    for (int i = 0; i < METER_COUNT; i++) {
        const char* serviceId = virtualMeters[i].serviceId;
        
        if (deviceInstances[serviceId].is<int>()) {
            meterStates[i].deviceInstance = deviceInstances[serviceId];
            DEBUG_PRINTF("Meter %s instance: %d\n", serviceId, meterStates[i].deviceInstance);
        }
        
        if (topicPaths[serviceId].is<JsonObject>()) {
            JsonObject paths = topicPaths[serviceId];
            if (paths["W"].is<const char*>()) {
                strlcpy(meterStates[i].topicPath, paths["W"] | "", sizeof(meterStates[i].topicPath));
                meterStates[i].registered = true;
                DEBUG_PRINTF("Meter %s topic: %s\n", serviceId, meterStates[i].topicPath);
            }
        }
    }
    
    // Check if all meters registered
    allMetersRegistered = true;
    for (int i = 0; i < METER_COUNT; i++) {
        if (!meterStates[i].registered) {
            allMetersRegistered = false;
            break;
        }
    }
    
    if (allMetersRegistered) {
        DEBUG_PRINTLN("All meters registered successfully");
        publishLog("All meters registered");
    }
}

// ============================================================================
// Registration Functions
// ============================================================================

void sendRegistration() {
    JsonDocument doc;
    
    doc["clientId"] = CLIENT_ID;
    doc["connected"] = 1;
    doc["version"] = DEVICE_VERSION;
    
    JsonObject services = doc["services"].to<JsonObject>();
    for (int i = 0; i < METER_COUNT; i++) {
        services[virtualMeters[i].serviceId] = "grid";
    }
    
    char payload[512];
    serializeJson(doc, payload, sizeof(payload));
    
    DEBUG_PRINTF("Publishing registration to %s: %s\n", topicStatus.c_str(), payload);
    mqttClient.publish(topicStatus.c_str(), payload, true);
}

void clearRegistration() {
    portalId[0] = '\0';
    allMetersRegistered = false;
    for (int i = 0; i < METER_COUNT; i++) {
        meterStates[i].registered = false;
        meterStates[i].topicPath[0] = '\0';
        meterStates[i].deviceInstance = 0;
    }
#if ENABLE_GENSET_HANDOFF
    // Re-bind + re-learn the genset on the next connection (instances/AC capability
    // are re-asserted by the republish after we re-subscribe).
    gensetSubscribed = false;
    gensetInstance = -1;
    gensetRunning = false;
    gensetAcCapable = false;
    gensetHandoffActive = false;
    gensetFirstKa = true;
#endif
}

// ============================================================================
// Publishing Functions
// ============================================================================

// Check if a physical phase has valid, fresh data
bool isPhaseValid(int physicalPhase) {
    if (physicalPhase < 0 || physicalPhase >= 3) return false;
    PhaseMeasurement& data = meterData.phases[physicalPhase];
    if (!data.valid) return false;
    unsigned long age = millis() - data.lastUpdate;
    return age <= CAN_TIMEOUT_MS;
}

// Publish a virtual meter with its mapped phases. When `forceZero` is set the meter
// is published as dead (0 V/I/P, 0 Hz) regardless of CAN data and with no energy
// keys - used to retire the grid meter while its inlet is handed off to the genset
// (omitting energy freezes the cumulative register rather than crediting genset energy
// to the grid meter).
void publishVirtualMeter(int meterIndex, bool forceZero) {
    VirtualMeterConfig const& meter = virtualMeters[meterIndex];
    VirtualMeterState& state = meterStates[meterIndex];

    if (!state.registered || state.topicPath[0] == '\0') {
        return;
    }

    if (!forceZero) {
        // Check if at least one mapped phase has valid data
        bool hasValidData = false;
        for (int v = 0; v < 3; v++) {
            if (isPhaseValid(meter.physicalPhase[v])) {
                hasValidData = true;
                break;
            }
        }
        if (!hasValidData) return;
    }

    JsonDocument doc;
    doc["topicPath"] = state.topicPath;

    JsonObject values = doc["values"].to<JsonObject>();

    // Metadata
    values["ProductId"] = 0xFFFF;
    values["CustomName"] = meter.name;
    values["DeviceType"] = meter.deviceType;
    values["ErrorCode"] = 0;

    // Aggregates
    float totalPower = 0;
    float totalEnergyRev = 0;

    // Virtual phase labels
    const char* phasePaths[] = {"Ac/L1", "Ac/L2", "Ac/L3"};

    // Map each virtual phase
    for (int v = 0; v < 3; v++) {
        int phys = meter.physicalPhase[v];
        if (phys < 0 || phys >= 3) continue;

        PhaseMeasurement& data = meterData.phases[phys];

        String base = String(phasePaths[v]);
        if (forceZero) {
            values[base + "/Voltage"] = 0;
            values[base + "/Current"] = 0;
            values[base + "/Power"] = 0;
            values[base + "/PowerFactor"] = 0;
            continue;   // no energy keys -> cumulative register frozen
        }
        if (!isPhaseValid(phys)) continue;

        values[base + "/Voltage"] = data.voltage;
        values[base + "/Current"] = data.current;
        values[base + "/Power"] = data.power;
        values[base + "/PowerFactor"] = data.powerFactor;
        values[base + "/Energy/Forward"] = data.energyForward;
        values[base + "/Energy/Reverse"] = data.energyReverse;

        totalPower += data.power;
        totalEnergyRev += data.energyReverse;
    }

    // Aggregates
    values["Ac/Power"] = forceZero ? 0 : totalPower;
    values["Ac/Frequency"] = forceZero ? 0 : meterData.frequency;
    if (!forceZero) {
        values["Ac/Energy/Forward"] = meterData.totalEnergyForward;
        values["Ac/Energy/Reverse"] = totalEnergyRev;
    }

    char payload[1024];
    serializeJson(doc, payload, sizeof(payload));

    char topic[64];
    snprintf(topic, sizeof(topic), "device/%s/Proxy", CLIENT_ID);

    mqttClient.publish(topic, payload);
}

void publishAllMeters() {
    for (int i = 0; i < METER_COUNT; i++) {
#if ENABLE_GENSET_HANDOFF
        if (i == GENSET_HANDOFF_METER && gensetHandoffActive) {
            publishGensetAc(false);         // inlet readings -> genset device
            publishVirtualMeter(i, true);   // inlet grid meter -> zeroed
            continue;
        }
#endif
        publishVirtualMeter(i);
    }
}

// ============================================================================
// State Machine
// ============================================================================

void changeState(DeviceState newState) {
    if (deviceState != newState) {
        DEBUG_PRINTF("State: %d -> %d\n", deviceState, newState);
        deviceState = newState;
        stateEnteredTime = millis();
    }
}

void runStateMachine() {
    unsigned long now = millis();
    unsigned long stateTime = now - stateEnteredTime;
    
    switch (deviceState) {
        case STATE_INIT:
            clearRegistration();
            changeState(STATE_WIFI_CONNECT);
            break;
            
        case STATE_WIFI_CONNECT:
            if (wifiIsConnected()) {
                DEBUG_PRINTF("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
                
                // Start mDNS
                if (MDNS.begin(CLIENT_ID)) {
                    DEBUG_PRINTF("mDNS: %s.local\n", CLIENT_ID);
                }
                
                // Start HTTP OTA server
                httpOtaSetup();
                
                if (resolveMqttServer()) {
                    mqttClient.setServer(mqttServerIP, MQTT_PORT);
                    changeState(STATE_MQTT_CONNECT);
                } else {
                    DEBUG_PRINTLN("MQTT server resolution failed, retrying...");
                }
            } else {
                wifiConnect();
                if (stateTime > WIFI_CONNECT_TIMEOUT_MS) {
                    DEBUG_PRINTLN("WiFi timeout, retrying...");
                    WiFi.disconnect();
                    stateEnteredTime = now;
                }
            }
            break;
            
        case STATE_MQTT_CONNECT:
            if (!wifiIsConnected()) {
                changeState(STATE_WIFI_CONNECT);
                break;
            }
            
            if (mqttConnect()) {
                changeState(STATE_REGISTER);
            } else if (stateTime > MQTT_CONNECT_TIMEOUT_MS) {
                DEBUG_PRINTLN("MQTT timeout, retrying...");
                stateEnteredTime = now;
            }
            break;
            
        case STATE_REGISTER:
            if (!mqttClient.connected()) {
                changeState(STATE_MQTT_CONNECT);
                break;
            }
            
            sendRegistration();
            changeState(STATE_WAIT_REGISTRATION);
            break;
            
        case STATE_WAIT_REGISTRATION:
            if (!mqttClient.connected()) {
                clearRegistration();
                changeState(STATE_MQTT_CONNECT);
                break;
            }
            
            if (allMetersRegistered) {
                changeState(STATE_RUNNING);
                lastPublishTime = 0;    // Force immediate publish
            } else if (stateTime > REGISTRATION_TIMEOUT_MS) {
                DEBUG_PRINTLN("Registration timeout, retrying...");
                changeState(STATE_REGISTER);
            }
            break;
            
        case STATE_RUNNING:
            if (!wifiIsConnected()) {
                changeState(STATE_WIFI_CONNECT);
                break;
            }
            
            if (!mqttClient.connected()) {
                clearRegistration();
                changeState(STATE_MQTT_CONNECT);
                break;
            }
            break;
    }
}

// ============================================================================
// Serial Command Interface (local)
// ============================================================================

void handleSerialInput() {
    static String inputBuffer;
    
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer.c_str());
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}

// ============================================================================
// Arduino Setup & Loop
// ============================================================================

void setup() {
    #if ENABLE_SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    Serial.println("\n\n================================");
    Serial.println("Patroclus - VE.CAN Meter Reader");
    Serial.printf("Version: %s\n", DEVICE_VERSION);
    Serial.printf("Client ID: %s\n", CLIENT_ID);
    Serial.printf("Virtual meters: %d\n", METER_COUNT);
    Serial.println("================================\n");
    
    // Print meter mapping
    for (int i = 0; i < METER_COUNT; i++) {
        Serial.printf("Meter %d: %s (%d-phase)\n", i, virtualMeters[i].name, virtualMeters[i].phaseCount);
        for (int v = 0; v < 3; v++) {
            int phys = virtualMeters[i].physicalPhase[v];
            if (phys >= 0) {
                Serial.printf("  Virtual L%d <- Physical L%d\n", v+1, phys+1);
            }
        }
    }
    Serial.println();
    #endif
    
    // Initialize meter data
    memset(&meterData, 0, sizeof(meterData));
    for (int i = 0; i < 3; i++) {
        meterData.phases[i].valid = false;
    }
    for (int i = 0; i < METER_COUNT; i++) {
        meterStates[i].registered = false;
        meterStates[i].topicPath[0] = '\0';
        meterStates[i].deviceInstance = 0;
    }
    
    ledSetup();
    wifiSetup();
    mqttSetup();
    
    if (!twaiSetup()) {
        DEBUG_PRINTLN("TWAI setup failed - CAN will not work");
    }
    
    DEBUG_PRINTLN("Setup complete, starting state machine");
}

void loop() {
    unsigned long now = millis();
    
    // Handle HTTP server (including OTA uploads)
    #if ENABLE_HTTP_OTA
    if (wifiIsConnected()) {
        httpServer.handleClient();
    }
    #endif
    
    // Skip normal processing during OTA
    if (otaInProgress) {
        updateLedStatus();
        return;
    }
    
    // Always poll CAN - even before WiFi/MQTT is up
    twaiPoll();
    
    // Process MQTT messages
    if (mqttClient.connected()) {
        mqttClient.loop();
    }
    
    // Run state machine
    runStateMachine();
    
    // Update LED
    updateLedStatus();
    
    // Handle serial commands
    handleSerialInput();
    
    // Publish captured frames if enabled
    publishCapturedFrames();
    
    // Only publish meter data when running
    if (deviceState != STATE_RUNNING) {
        return;
    }

#if ENABLE_GENSET_HANDOFF
    // Keep the genset subscription + keepalive alive and re-evaluate the handoff edge
    // every loop (cheap; the publish itself is still rate-limited below).
    gensetMaintenance();
    updateGensetHandoff();
#endif

    // Publish periodically
    if (now - lastPublishTime >= PUBLISH_INTERVAL_MS) {
        lastPublishTime = now;
        publishAllMeters();
    }
}
