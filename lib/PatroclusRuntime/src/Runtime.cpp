#include "Runtime.h"

#include <ESPmDNS.h>
#if ENABLE_HTTP_OTA
#include <Update.h>
#endif

namespace patroclus {

Runtime* Runtime::s_active = nullptr;

Runtime::Runtime()
    : mqttClient_(wifiClient_)
#if ENABLE_HTTP_OTA
    , httpServer_(HTTP_OTA_PORT)
#endif
{
}

// ============================================================================
// App-facing composition
// ============================================================================

void Runtime::setIdentity(const char* clientId, const char* version) {
    clientId_ = clientId;
    version_ = version;
}

Meter& Runtime::addMeter(const char* serviceId, const char* name, const char* deviceType) {
    if (meterCount_ >= GX_MAX_SERVICES) {
        // Out of service slots (GX_MAX_SERVICES). Hand back the last meter so the
        // caller's fluent chain still has something to touch; the meter is dropped.
        DEBUG_PRINTLN("addMeter: too many meters (raise GX_MAX_SERVICES)");
        return meters_[GX_MAX_SERVICES - 1];
    }
    Meter& m = meters_[meterCount_++];
    m.serviceId = serviceId;
    m.name = name;
    m.deviceType = deviceType;
    return m;
}

// ============================================================================
// Lifecycle
// ============================================================================

void Runtime::begin() {
    s_active = this;

#if ENABLE_SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD);
    delay(1000);
    Serial.println("\n\n================================");
    Serial.println("Patroclus - VE.CAN Meter Reader");
    Serial.printf("Version: %s\n", version_);
    Serial.printf("Client ID: %s\n", clientId_);
    Serial.printf("Virtual meters: %d\n", (int)meterCount_);
    Serial.println("================================\n");

    for (size_t i = 0; i < meterCount_; i++) {
        Serial.printf("Meter %d: %s (%d-phase)\n", (int)i, meters_[i].name, meters_[i].phaseCount());
        for (int v = 0; v < 3; v++) {
            int phys = meters_[i].physicalPhase[v];
            if (phys >= 0) {
                Serial.printf("  Virtual L%d <- Physical L%d\n", v + 1, phys + 1);
            }
        }
    }
    Serial.println();
#endif

    // gx-projector-client service table: every virtual meter registers as dbus type
    // "grid" (as v1 did); the grid-vs-acload identity travels as DeviceType in the
    // registration init instead (see fillMeterInitCb).
    for (size_t i = 0; i < meterCount_; i++) {
        services_[i].tag = meters_[i].serviceId;
        services_[i].type = "grid";
    }

    // The session exposes the portal id + per-meter W/ topic bases; the client runs the
    // whole projector contract prologue and binds asynchronously.
    gxSession_ = new gx::GxSession(clientId_, version_, services_, meterCount_);
    gxClient_ = new gx::GxClient(mqttClient_, *gxSession_);
    gxClient_->setInitFiller(&Runtime::fillMeterInitCb, this);
    gxClient_->setMessageHandler(&Runtime::onProjectMessageCb, this);

    ledSetup();
    wifiSetup();
    mqttSetup();

    if (!canMeter_.begin()) {
        DEBUG_PRINTLN("TWAI setup failed - CAN will not work");
    }

    deviceState_ = STATE_INIT;
    stateEnteredTime_ = millis();
    DEBUG_PRINTLN("Setup complete, starting state machine");
}

void Runtime::loop() {
    unsigned long now = millis();

#if ENABLE_HTTP_OTA
    if (wifiIsConnected()) {
        httpServer_.handleClient();
    }
#endif

    // Skip normal processing during OTA
    if (otaInProgress_) {
        updateLedStatus();
        return;
    }

    // Always poll CAN - even before WiFi/MQTT is up
    canMeter_.poll();

    if (mqttClient_.connected()) {
        mqttClient_.loop();
    }

    runStateMachine();
    updateLedStatus();
    handleSerialInput();
    publishCapturedFrames();

    if (deviceState_ != STATE_RUNNING) {
        return;
    }

    if (now - lastPublishTime_ >= PUBLISH_INTERVAL_MS) {
        lastPublishTime_ = now;
        publishAllMeters();
    }
}

// ============================================================================
// State Machine
// ============================================================================

void Runtime::changeState(DeviceState newState) {
    if (deviceState_ != newState) {
        DEBUG_PRINTF("State: %d -> %d\n", deviceState_, newState);
        deviceState_ = newState;
        stateEnteredTime_ = millis();
    }
}

void Runtime::runStateMachine() {
    unsigned long now = millis();
    unsigned long stateTime = now - stateEnteredTime_;

    switch (deviceState_) {
        case STATE_INIT:
            changeState(STATE_WIFI_CONNECT);
            break;

        case STATE_WIFI_CONNECT:
            if (wifiIsConnected()) {
                DEBUG_PRINTF("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

                if (MDNS.begin(clientId_)) {
                    DEBUG_PRINTF("mDNS: %s.local\n", clientId_);
                }

                httpOtaSetup();

                if (resolveMqttServer()) {
                    mqttClient_.setServer(mqttServerIP_, MQTT_PORT);
                    changeState(STATE_MQTT_CONNECT);
                } else {
                    DEBUG_PRINTLN("MQTT server resolution failed, retrying...");
                }
            } else {
                wifiConnect();
                if (stateTime > WIFI_CONNECT_TIMEOUT_MS) {
                    DEBUG_PRINTLN("WiFi timeout, retrying...");
                    WiFi.disconnect();
                    stateEnteredTime_ = now;
                }
            }
            break;

        case STATE_MQTT_CONNECT:
            if (!wifiIsConnected()) {
                changeState(STATE_WIFI_CONNECT);
                break;
            }

            if (mqttConnect()) {
                // gxClient.connect() already published the registration; wait for the
                // async DBus binding.
                changeState(STATE_WAIT_REGISTRATION);
            } else if (stateTime > MQTT_CONNECT_TIMEOUT_MS) {
                DEBUG_PRINTLN("MQTT timeout, retrying...");
                stateEnteredTime_ = now;
            }
            break;

        case STATE_WAIT_REGISTRATION:
            if (!mqttClient_.connected()) {
                changeState(STATE_MQTT_CONNECT);
                break;
            }

            if (gxSession_->bound()) {
                DEBUG_PRINTLN("All meters registered successfully");
                publishLog("All meters registered");
                changeState(STATE_RUNNING);
                lastPublishTime_ = 0;    // Force immediate publish
            } else if (stateTime > REGISTRATION_TIMEOUT_MS) {
                // Registration and binding are per-connection: retry means redoing the
                // whole handshake on a fresh connection.
                DEBUG_PRINTLN("Registration timeout, reconnecting...");
                mqttClient_.disconnect();
                changeState(STATE_MQTT_CONNECT);
            }
            break;

        case STATE_RUNNING:
            if (!wifiIsConnected()) {
                changeState(STATE_WIFI_CONNECT);
                break;
            }

            if (!mqttClient_.connected()) {
                changeState(STATE_MQTT_CONNECT);
                break;
            }

            if (gxClient_->needsRebind()) {
                // A re-announce reply carried different instances: our topic bases are
                // stale. Reconnect to rebind cleanly (connect() clears the flag).
                DEBUG_PRINTLN("Instance rebind required, reconnecting...");
                mqttClient_.disconnect();
                changeState(STATE_MQTT_CONNECT);
                break;
            }
            break;
    }
}

// ============================================================================
// WiFi
// ============================================================================

void Runtime::wifiSetup() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
}

bool Runtime::wifiConnect() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    unsigned long now = millis();
    if (now - lastWiFiAttempt_ < WIFI_RECONNECT_DELAY_MS) {
        return false;
    }
    lastWiFiAttempt_ = now;

    DEBUG_PRINTF("Connecting to WiFi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    return false;
}

bool Runtime::wifiIsConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool Runtime::resolveMqttServer() {
    if (mqttServerIP_.fromString(MQTT_SERVER)) {
        DEBUG_PRINTF("MQTT server is IP: %s\n", mqttServerIP_.toString().c_str());
        return true;
    }

    DEBUG_PRINTF("Resolving %s...\n", MQTT_SERVER);
    if (WiFi.hostByName(MQTT_SERVER, mqttServerIP_)) {
        DEBUG_PRINTF("Resolved %s to %s\n", MQTT_SERVER, mqttServerIP_.toString().c_str());
        return true;
    }

    DEBUG_PRINTF("Failed to resolve %s\n", MQTT_SERVER);
    return false;
}

// ============================================================================
// MQTT / projector
// ============================================================================

void Runtime::mqttSetup() {
    topicCommand_ = String("device/") + clientId_ + "/Command";
    topicLog_ = String("device/") + clientId_ + "/Log";
    topicCapture_ = String("device/") + clientId_ + "/Capture";

#if MQTT_USE_TLS
    wifiClient_.setInsecure();   // Skip cert verification for self-signed
#endif
    mqttClient_.setCallback(&Runtime::mqttCallbackCb);
    mqttClient_.setBufferSize(2048);  // Larger buffer for captures
}

bool Runtime::mqttConnect() {
    if (mqttClient_.connected()) {
        return true;
    }

    unsigned long now = millis();
    if (now - lastMQTTAttempt_ < MQTT_RECONNECT_DELAY_MS) {
        return false;
    }
    lastMQTTAttempt_ = now;

    DEBUG_PRINTF("Connecting to MQTT: %s:%d\n", mqttServerIP_.toString().c_str(), MQTT_PORT);

    // The lib runs the whole contract prologue: CONNECT with will = RETAINED "0" on
    // device/<id>/online, retained online=1, subscribe DBus, publish the NON-retained v2
    // registration. Binding completes asynchronously in STATE_WAIT_REGISTRATION.
    if (gxClient_->connect(MQTT_USER, MQTT_PASS)) {
        DEBUG_PRINTLN("MQTT connected");

        mqttClient_.subscribe(topicCommand_.c_str());
        DEBUG_PRINTF("Subscribed to: %s\n", topicCommand_.c_str());

        publishLog(String("Online - ") + version_ + " @ " + WiFi.localIP().toString() +
                   " OTA: http://" + WiFi.localIP().toString() + ":" + String(HTTP_OTA_PORT) + "/update");

        return true;
    } else {
        DEBUG_PRINTF("MQTT connect failed, rc=%d\n", mqttClient_.state());
        return false;
    }
}

void Runtime::publishLog(const char* message) {
    if (mqttClient_.connected()) {
        mqttClient_.publish(topicLog_.c_str(), message);
    }
    DEBUG_PRINTLN(message);
}

void Runtime::publishLog(const String& message) {
    publishLog(message.c_str());
}

// Board-authored initial values for one meter's registration init, rebuilt on every
// (re-)announce. Under v2 these seed the dbus paths once at device build; the
// grid-vs-acload identity rides here as DeviceType.
void Runtime::fillMeterInitCb(const char* tag, JsonObject init, void* ctx) {
    Runtime* self = static_cast<Runtime*>(ctx);
    for (size_t i = 0; i < self->meterCount_; i++) {
        if (strcmp(self->meters_[i].serviceId, tag) != 0) continue;
        init["CustomName"] = self->meters_[i].name;
        init["DeviceType"] = self->meters_[i].deviceType;
        init["ProductId"] = 0xFFFF;
        init["ErrorCode"] = 0;
        return;
    }
}

// Non-contract messages forwarded from the lib's NotOurs path: our Command topic.
// Contract traffic (DBus binding, online self-heal, cookie re-announce, rebind
// detection) is consumed inside GxClient.
void Runtime::onProjectMessageCb(const char* topic, const uint8_t* payload,
                                 size_t length, void* ctx) {
    Runtime* self = static_cast<Runtime*>(ctx);
    char buffer[1024];
    if (length >= sizeof(buffer)) {
        DEBUG_PRINTLN("MQTT payload too large");
        return;
    }
    memcpy(buffer, payload, length);
    buffer[length] = '\0';

    DEBUG_PRINTF("MQTT received [%s]: %s\n", topic, buffer);

    if (self->topicCommand_ == topic) {
        self->handleCommand(buffer);
        return;
    }
}

void Runtime::mqttCallbackCb(char* topic, byte* payload, unsigned int length) {
    if (s_active && s_active->gxClient_) {
        s_active->gxClient_->handleMessage(topic, payload, length);
    }
}

// ============================================================================
// Publishing
// ============================================================================

// {"topicPath":"W/<portal>/grid/<inst>","values":{...}} - null when not bound. The
// meter's projection fills the values (default = direct per-leg copy).
void Runtime::publishVirtualMeter(Meter& meter) {
    JsonDocument doc;
    JsonObject values = gxSession_->proxyValues(doc, meter.serviceId);
    if (values.isNull()) return;   // not bound

    if (!meter.project(canMeter_.data(), values)) return;  // nothing fresh to publish

    gxClient_->publishProxy(doc);   // streamed to device/<id>/Proxy, non-retained
}

void Runtime::publishAllMeters() {
    for (size_t i = 0; i < meterCount_; i++) {
        publishVirtualMeter(meters_[i]);
    }
}

void Runtime::publishCapturedFrames() {
    if (!canMeter_.captureEnabled() || canMeter_.captureCount() == 0) return;
    if (!mqttClient_.connected()) return;

    unsigned long now = millis();
    if (now - lastCapturePublish_ < CAN_CAPTURE_INTERVAL_MS) return;
    lastCapturePublish_ = now;

    JsonDocument doc;
    JsonArray frames = doc.to<JsonArray>();

    for (int i = 0; i < canMeter_.captureCount(); i++) {
        const CanCapture& cap = canMeter_.capture(i);
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
        mqttClient_.publish(topicCapture_.c_str(), payload);
    }

    canMeter_.clearCapture();
}

// ============================================================================
// Commands
// ============================================================================

void Runtime::handleCommand(const char* cmd) {
    String command = String(cmd);
    command.trim();
    command.toUpperCase();

    DEBUG_PRINTF("Command received: %s\n", command.c_str());

    if (command == "STATUS") {
        const MeterData& md = canMeter_.data();
        String response = "--- Status ---\n";
        response += "Version: " + String(version_) + "\n";
        response += "State: " + String(deviceState_) + "\n";
        response += "WiFi: " + WiFi.localIP().toString() + "\n";
        response += "MQTT: " + String(mqttClient_.connected() ? "Connected" : "Disconnected") + "\n";
        response += "OTA: http://" + WiFi.localIP().toString() + ":" + String(HTTP_OTA_PORT) + "/update\n";
        response += "Freq: " + String(md.frequency, 2) + " Hz\n";
        response += "Total Energy: " + String(md.totalEnergyForward, 3) + " kWh\n";

        response += "\nPhysical Phases:\n";
        const char* phaseNames[] = {"L1", "L2", "L3"};
        for (int i = 0; i < 3; i++) {
            const PhaseMeasurement& data = md.phases[i];
            response += "  " + String(phaseNames[i]) + ": ";
            response += "V=" + String(data.voltage, 1);
            response += " I=" + String(data.current, 2);
            response += " P=" + String(data.power, 0);
            response += " PF=" + String(data.powerFactor, 3);
            response += " (" + String(data.valid ? "ok" : "stale") + ")\n";
        }

        // Virtual meters (the lib binds all-or-nothing)
        response += "\nVirtual Meters (";
        response += String(gxSession_->bound() ? "registered" : "pending");
        response += "):\n";
        for (size_t i = 0; i < meterCount_; i++) {
            response += "  " + String(meters_[i].name) + " inst=";
            response += String(gxSession_->instance(meters_[i].serviceId)) + "\n";
        }

        publishLog(response);
    }
    else if (command == "CANDUMP") {
        canMeter_.setSerialDump(!canMeter_.serialDump());
        publishLog(String("Serial CAN dump: ") + (canMeter_.serialDump() ? "ON" : "OFF"));
    }
    else if (command == "CAPTURE") {
        canMeter_.setCaptureEnabled(!canMeter_.captureEnabled());
        publishLog(String("MQTT CAN capture: ") + (canMeter_.captureEnabled() ? "ON" : "OFF"));
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

void Runtime::handleSerialInput() {
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
// LED status
// ============================================================================

#if ENABLE_LED_STATUS
void Runtime::ledSetup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);
}

void Runtime::ledSolid() {
    digitalWrite(LED_PIN, LED_ON);
}

void Runtime::ledOff() {
    digitalWrite(LED_PIN, LED_OFF);
}

void Runtime::ledBlink(unsigned long intervalMs) {
    unsigned long now = millis();
    if (now - lastLedToggle_ >= intervalMs) {
        lastLedToggle_ = now;
        ledState_ = !ledState_;
        digitalWrite(LED_PIN, ledState_ ? LED_ON : LED_OFF);
    }
}

void Runtime::ledDoubleBlink() {
    unsigned long phase = (millis() / 100) % 20;
    if (phase == 0 || phase == 2) {
        digitalWrite(LED_PIN, LED_ON);
    } else {
        digitalWrite(LED_PIN, LED_OFF);
    }
}

void Runtime::updateLedStatus() {
    if (otaInProgress_) {
        ledBlink(50);   // Very fast blink during OTA
        return;
    }

    switch (deviceState_) {
        case STATE_WIFI_CONNECT:
            ledBlink(500);      // 1Hz - slow blink
            break;
        case STATE_MQTT_CONNECT:
            ledBlink(125);      // 4Hz - fast blink
            break;
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
void Runtime::ledSetup() {}
void Runtime::ledSolid() {}
void Runtime::ledOff() {}
void Runtime::ledBlink(unsigned long) {}
void Runtime::ledDoubleBlink() {}
void Runtime::updateLedStatus() {}
#endif

// ============================================================================
// HTTP OTA
// ============================================================================

#if ENABLE_HTTP_OTA

void Runtime::handleOtaIndex() {
    httpServer_.send(200, "text/html",
        String("<html><head><title>Patroclus OTA</title></head><body>"
        "<h1>Patroclus OTA Update</h1>"
        "<p>Version: ") + version_ + "</p>"
        "<form method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin'><br><br>"
        "<input type='submit' value='Upload Firmware'>"
        "</form></body></html>"
    );
}

void Runtime::handleOtaUpdate() {
    httpServer_.sendHeader("Connection", "close");
    if (Update.hasError()) {
        httpServer_.send(500, "text/plain", "Update failed: " + String(Update.errorString()));
    } else {
        httpServer_.send(200, "text/plain", "OK - Rebooting...");
        delay(500);
        ESP.restart();
    }
}

void Runtime::handleOtaUpload() {
    HTTPUpload& upload = httpServer_.upload();

    if (upload.status == UPLOAD_FILE_START) {
        otaInProgress_ = true;
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
        otaInProgress_ = false;
    }
    else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        otaInProgress_ = false;
        DEBUG_PRINTLN("OTA Aborted");
        publishLog("OTA aborted");
    }
}

void Runtime::handleOtaVersion() {
    String json = String("{\"version\":\"") + version_ + "\",\"client_id\":\"" + clientId_ + "\"}";
    httpServer_.send(200, "application/json", json);
}

void Runtime::httpOtaSetup() {
    httpServer_.on("/", HTTP_GET, [this]() { handleOtaIndex(); });
    httpServer_.on("/version", HTTP_GET, [this]() { handleOtaVersion(); });
    httpServer_.on("/update", HTTP_POST, [this]() { handleOtaUpdate(); }, [this]() { handleOtaUpload(); });
    httpServer_.begin();
    DEBUG_PRINTF("HTTP OTA server on port %d\n", HTTP_OTA_PORT);
}

#else
void Runtime::httpOtaSetup() {}
#endif

}  // namespace patroclus
