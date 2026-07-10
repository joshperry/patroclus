/*
 * Runtime - the generic Patroclus firmware an app builds on.
 *
 * Mirrors the composition split used by the sibling switchy/hypnos projects: this
 * library owns hardware (CAN via CanMeter), WiFi, MQTT, the gx-device-projector
 * contract, the publish loop, HTTP OTA, the LED, and the command interface. A thin
 * per-instance app (src/apps/<name>/app.cpp) owns setup()/loop(), composes its virtual
 * meters through the provided functions, and drives the runtime:
 *
 *   static Runtime rt;
 *   void setup() {
 *       rt.setIdentity("patroclus01");
 *       rt.addMeter("grid_shore", "Shore Power", "grid").mapPhase(0, 0).mapPhase(1, 1);
 *       rt.begin();
 *   }
 *   void loop() { rt.loop(); }
 *
 * The build selects which app compiles (platformio.ini build_src_filter per env), so
 * one codebase flashes multiple physically-distinct boards.
 */
#ifndef PATROCLUS_RUNTIME_H
#define PATROCLUS_RUNTIME_H

#include <Arduino.h>
#include <WiFi.h>
#include "patroclus_config.h"

#if MQTT_USE_TLS
#include <WiFiClientSecure.h>
#endif
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <GxSession.h>
#include <GxClient.h>

#if ENABLE_HTTP_OTA
#include <WebServer.h>
#endif

#include "CanMeter.h"
#include "VirtualMeter.h"

namespace patroclus {

// Registration itself rides gxClient.connect(); WAIT polls the async DBus binding.
enum DeviceState {
    STATE_INIT,
    STATE_WIFI_CONNECT,
    STATE_MQTT_CONNECT,
    STATE_WAIT_REGISTRATION,
    STATE_RUNNING
};

class Runtime {
public:
    Runtime();

    // --- app-facing composition (call before begin) --------------------------
    void setIdentity(const char* clientId, const char* version = DEVICE_VERSION);
    Meter& addMeter(const char* serviceId, const char* name, const char* deviceType);

    // --- lifecycle -----------------------------------------------------------
    void begin();   // bring up CAN, WiFi, MQTT, OTA, LED; build the projector session
    void loop();    // pump everything (call from Arduino loop())

private:
    // state machine
    void changeState(DeviceState newState);
    void runStateMachine();

    // wifi
    void wifiSetup();
    bool wifiConnect();
    bool wifiIsConnected();
    bool resolveMqttServer();

    // mqtt / projector
    void mqttSetup();
    bool mqttConnect();
    void publishLog(const char* message);
    void publishLog(const String& message);

    // publishing
    void publishVirtualMeter(Meter& meter);
    void publishAllMeters();
    void publishCapturedFrames();

    // commands
    void handleCommand(const char* cmd);
    void handleSerialInput();

    // led
    void ledSetup();
    void ledSolid();
    void ledOff();
    void ledBlink(unsigned long intervalMs);
    void ledDoubleBlink();
    void updateLedStatus();

    // http ota
    void httpOtaSetup();
#if ENABLE_HTTP_OTA
    void handleOtaIndex();
    void handleOtaUpdate();
    void handleOtaUpload();
    void handleOtaVersion();
#endif

    // projector callbacks (plain fn pointers + ctx = this)
    static void fillMeterInitCb(const char* tag, JsonObject init, void* ctx);
    static void onProjectMessageCb(const char* topic, const uint8_t* payload,
                                   size_t length, void* ctx);
    // PubSubClient callback is a bare fn pointer (no ctx) -> route via s_active.
    static void mqttCallbackCb(char* topic, byte* payload, unsigned int length);
    static Runtime* s_active;

    // --- identity / meters ---------------------------------------------------
    const char* clientId_ = CLIENT_ID_DEFAULT;
    const char* version_ = DEVICE_VERSION;
    Meter meters_[GX_MAX_SERVICES];
    gx::ServiceDef services_[GX_MAX_SERVICES];
    size_t meterCount_ = 0;

    // --- networking / projector ----------------------------------------------
#if MQTT_USE_TLS
    WiFiClientSecure wifiClient_;
#else
    WiFiClient wifiClient_;
#endif
    PubSubClient mqttClient_;
    gx::GxSession* gxSession_ = nullptr;
    gx::GxClient* gxClient_ = nullptr;
    IPAddress mqttServerIP_;

#if ENABLE_HTTP_OTA
    WebServer httpServer_;
#endif

    CanMeter canMeter_;

    // --- topics --------------------------------------------------------------
    String topicCommand_;
    String topicLog_;
    String topicCapture_;

    // --- state ---------------------------------------------------------------
    DeviceState deviceState_ = STATE_INIT;
    bool otaInProgress_ = false;

    unsigned long lastPublishTime_ = 0;
    unsigned long stateEnteredTime_ = 0;
    unsigned long lastWiFiAttempt_ = 0;
    unsigned long lastMQTTAttempt_ = 0;
    unsigned long lastLedToggle_ = 0;
    unsigned long lastCapturePublish_ = 0;
    bool ledState_ = false;
};

}  // namespace patroclus

#endif  // PATROCLUS_RUNTIME_H
