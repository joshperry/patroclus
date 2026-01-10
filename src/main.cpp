/*
 * Patroclus - VE.CAN Energy Meter Reader for Victron GX
 * 
 * Taps internal CAN bus of Precision Circuits MV-3P75CT 3-phase meter,
 * decodes per-phase measurements, and publishes to Victron GX as
 * virtual grid meters via dbus-mqtt-devices driver.
 * 
 * Features:
 * - Flexible phase-to-meter mapping (combine physical phases into virtual meters)
 * - HTTP OTA for remote firmware updates (POST to /update)
 * - MQTT command interface for remote control
 * - Remote CAN capture streaming
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
    float energyForward;    // kWh (bought/consumed)
    float energyReverse;    // kWh (sold/exported)
    float powerFactor;      // 0.0 - 1.0
    unsigned long lastUpdate;   // millis() of last CAN update
    bool valid;             // Have we received data?
};

// Physical meter data (3 phases from MV-3P75CT)
struct MeterData {
    float frequency;        // Hz (common to all phases)
    PhaseMeasurement phases[3];  // Physical L1, L2, L3
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

    // Parse 19F30xxx VE.CAN energy meter messages
    // Format: [seq_lo][seq_hi][val1_lo][val1_hi][val2_b0][val2_b1][val2_b2][val2_b3]
    
    uint16_t val16 = (msg.data[3] << 8) | msg.data[2];
    int32_t val32 = (int32_t)msg.data[4] | 
                    ((int32_t)msg.data[5] << 8) | 
                    ((int32_t)msg.data[6] << 16) | 
                    ((int32_t)msg.data[7] << 24);
    
    unsigned long now = millis();
    
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
        
        // Physical phases
        response += "\nPhysical Phases:\n";
        const char* phaseNames[] = {"L1", "L2", "L3"};
        for (int i = 0; i < 3; i++) {
            PhaseMeasurement& data = meterData.phases[i];
            response += "  " + String(phaseNames[i]) + ": ";
            response += "V=" + String(data.voltage, 1);
            response += " I=" + String(data.current, 2);
            response += " P=" + String(data.power, 0);
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

// Publish a virtual meter with its mapped phases
void publishVirtualMeter(int meterIndex) {
    VirtualMeterConfig const& meter = virtualMeters[meterIndex];
    VirtualMeterState& state = meterStates[meterIndex];
    
    if (!state.registered || state.topicPath[0] == '\0') {
        return;
    }
    
    // Check if at least one mapped phase has valid data
    bool hasValidData = false;
    for (int v = 0; v < 3; v++) {
        if (isPhaseValid(meter.physicalPhase[v])) {
            hasValidData = true;
            break;
        }
    }
    if (!hasValidData) return;
    
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
    float totalEnergyFwd = 0;
    float totalEnergyRev = 0;
    
    // Virtual phase labels
    const char* phasePaths[] = {"Ac/L1", "Ac/L2", "Ac/L3"};
    
    // Map each virtual phase
    for (int v = 0; v < 3; v++) {
        int phys = meter.physicalPhase[v];
        if (phys < 0 || phys >= 3) continue;
        
        PhaseMeasurement& data = meterData.phases[phys];
        if (!isPhaseValid(phys)) continue;
        
        String base = String(phasePaths[v]);
        values[base + "/Voltage"] = data.voltage;
        values[base + "/Current"] = data.current;
        values[base + "/Power"] = data.power;
        values[base + "/Energy/Forward"] = data.energyForward;
        values[base + "/Energy/Reverse"] = data.energyReverse;
        
        totalPower += data.power;
        totalEnergyFwd += data.energyForward;
        totalEnergyRev += data.energyReverse;
    }
    
    // Aggregates
    values["Ac/Power"] = totalPower;
    values["Ac/Energy/Forward"] = totalEnergyFwd;
    values["Ac/Energy/Reverse"] = totalEnergyRev;
    
    // Frequency (shared)
    values["Ac/Frequency"] = meterData.frequency;
    
    char payload[768];
    serializeJson(doc, payload, sizeof(payload));
    
    char topic[64];
    snprintf(topic, sizeof(topic), "device/%s/Proxy", CLIENT_ID);
    
    mqttClient.publish(topic, payload);
}

void publishAllMeters() {
    for (int i = 0; i < METER_COUNT; i++) {
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
    
    // Publish periodically
    if (now - lastPublishTime >= PUBLISH_INTERVAL_MS) {
        lastPublishTime = now;
        publishAllMeters();
    }
}
