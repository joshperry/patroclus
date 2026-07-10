#ifndef PATROCLUS_CONFIG_H
#define PATROCLUS_CONFIG_H
#if __has_include("secrets.h")
#include "secrets.h"
#endif

// ============================================================================
// Shared configuration.
//
// This header holds only cross-instance concerns: hardware pins, the CAN protocol,
// networking, timing, and feature toggles. Per-instance identity and the physical->
// virtual meter mapping are composed by each app (src/apps/<name>/app.cpp) through the
// Runtime API - see lib/PatroclusRuntime.
// ============================================================================

// ============================================================================
// WiFi Configuration
// Override via build flags: -DWIFI_SSID=\"...\" -DWIFI_PASS=\"...\"
// ============================================================================
#ifndef WIFI_SSID
#define WIFI_SSID "your_ssid"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "your_password"
#endif

// ============================================================================
// MQTT Configuration (Victron GX Device)
// ============================================================================
#ifndef MQTT_SERVER
#define MQTT_SERVER "venus.local"
#endif
#define MQTT_PORT 8883
#define MQTT_USE_TLS 1

#ifndef MQTT_USER
#define MQTT_USER "victron"
#endif
#ifndef MQTT_PASS
#define MQTT_PASS ""
#endif

// ============================================================================
// Device Identity
// ============================================================================
// The client id (MQTT client / mDNS host / OTA target / topic prefix) is set per
// instance by the app via Runtime::setIdentity(); this is only the fallback for an
// app that never calls it.
#define CLIENT_ID_DEFAULT "patroclus"
#define DEVICE_VERSION "v0.7.0"

// ============================================================================
// HTTP OTA Configuration
// ============================================================================
#define ENABLE_HTTP_OTA 1
#define HTTP_OTA_PORT 8080

// ============================================================================
// CAN / TWAI Configuration
// ============================================================================

// Pin assignments (XIAO ESP32-S3). CAN sits on D0/D1 (GPIO1/GPIO2), matching the
// single-wire tap in the README. Deliberately NOT the TX/RX header pins (GPIO43/44,
// silkscreen D6/D7): those are UART0, so the boot-ROM console + IDF error logging fight
// the CAN line and hang the TWAI driver (interrupt-WDT panic) on a busy bus.
// Overridable via -D build flags for a board wired differently.
#ifndef CAN_RX_PIN
#define CAN_RX_PIN GPIO_NUM_2    // D1 - reads the shared bus (RX, bus-direct)
#endif
#ifndef CAN_TX_PIN
#define CAN_TX_PIN GPIO_NUM_1    // D0 - drives the bus through its diode (active-mode ACK)
#endif

// CAN bus speed - VE.CAN uses 250 kbps
#define CAN_BITRATE 250000

// Operating mode
// LISTEN_ONLY: Passive sniffing, no ACKs (use when GX is on bus)
// ACTIVE: Full CAN participation with ACKs (use for bench testing)
#ifndef CAN_ACTIVE_MODE
#define CAN_ACTIVE_MODE 1           // 0 = listen-only, 1 = active
#endif

// ============================================================================
// CAN Message IDs - VE.CAN Energy Meter Format
// ============================================================================
// Source address 0x40, 29-bit extended IDs

// Per-phase voltage/frequency messages (0x19F30xxx)
// Format: [seq_lo][seq_hi][val1_lo][val1_hi][val2_b0][val2_b1][val2_b2][val2_b3]
// val1 = Voltage × 10, val2 bytes 6-7 = Frequency × 10
#define CAN_ID_L1_VOLTAGE    0x19F30340
#define CAN_ID_L2_VOLTAGE    0x19F30440
#define CAN_ID_L3_VOLTAGE    0x19F30540

// Per-phase current/power messages (0x19F30xxx)
// val1 = Current × 10, val2 = signed power in Watts
#define CAN_ID_L1_POWER      0x19F30040
#define CAN_ID_L2_POWER      0x19F30140
#define CAN_ID_L3_POWER      0x19F30240

// Register-based messages (0x19EFFF40)
// Format: [seq_lo][seq_hi][register][sub_seq][value 32-bit LE]
#define CAN_ID_REGISTER      0x1CEFFF40

// EFFF Register addresses
#define REG_ENERGY_FORWARD   0x50    // Total energy in Wh
#define REG_L1_PF            0x55    // L1 Power Factor × 1000
#define REG_L2_PF            0x5A    // L2 Power Factor × 1000
#define REG_L3_PF            0x09    // L3 Power Factor × 1000

// ============================================================================
// Timing Configuration (milliseconds)
// ============================================================================
#define PUBLISH_INTERVAL_MS     500        // Publish to MQTT every 500 ms
#define HEARTBEAT_INTERVAL_MS   60000       // Keepalive every 60 seconds
#define CAN_TIMEOUT_MS          5000        // No CAN data = stale

// Connection timeouts
#define WIFI_CONNECT_TIMEOUT_MS     30000
#define MQTT_CONNECT_TIMEOUT_MS     10000
#define REGISTRATION_TIMEOUT_MS     10000

// Reconnection delays
#define WIFI_RECONNECT_DELAY_MS     5000
#define MQTT_RECONNECT_DELAY_MS     2000

// Remote CAN capture
#define CAN_CAPTURE_BUFFER_SIZE     64      // Max frames to buffer for MQTT publish
#define CAN_CAPTURE_INTERVAL_MS     100     // Publish captured frames every N ms

// ============================================================================
// Hardware Configuration (XIAO ESP32-S3)
// ============================================================================
#define LED_PIN LED_BUILTIN
#define LED_ON LOW                  // Active low on XIAO
#define LED_OFF HIGH

// ============================================================================
// Feature Toggles
// ============================================================================
#define ENABLE_SERIAL_DEBUG     1
#define ENABLE_LED_STATUS       1
#define ENABLE_CAN_DUMP         1   // Enable CANDUMP serial command

#define SERIAL_BAUD 115200

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

#endif // PATROCLUS_CONFIG_H
