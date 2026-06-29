#ifndef CONFIG_H
#define CONFIG_H
#if __has_include("secrets.h")
#include "secrets.h"
#endif

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
#define CLIENT_ID "patroclus01"
#define DEVICE_VERSION "v0.6.3"

// ============================================================================
// HTTP OTA Configuration
// ============================================================================
#define ENABLE_HTTP_OTA 1
#define HTTP_OTA_PORT 8080

// ============================================================================
// CAN / TWAI Configuration
// ============================================================================

// Pin assignments (XIAO ESP32-S3)
// Use D0/D1 to avoid boot ROM UART spew on GPIO43/44
#define CAN_RX_PIN GPIO_NUM_44   // D1 - connects to meter TXD
#define CAN_TX_PIN GPIO_NUM_43   // D0 - for active mode ACK

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
// Virtual Meter Configuration
// ============================================================================
// Maps physical meter phases to virtual meters published to Venus OS
//
// Physical phases from MV-3P75CT: L1, L2, L3 (indices 0, 1, 2)
// Each virtual meter can have 1-3 phases mapped from physical phases
//
// Example: Split-phase shore + single-phase inverter input
//   Meter 0 "Shore Power":    Physical L1 → Virtual L1, Physical L2 → Virtual L2
//   Meter 1 "Inverter Input": Physical L3 → Virtual L1

#define METER_COUNT 2

// --- Meter 0: Shore Power (2-phase) ---
#define METER0_SERVICE_ID    "grid_shore"
#define METER0_NAME          "Shore Power"
#define METER0_DEVICE_TYPE   "grid"
#define METER0_PHASE_COUNT   2
// Physical phase index for each virtual phase (-1 = not used)
#define METER0_VIRT_L1_PHYS  0    // Physical L1 → Virtual L1
#define METER0_VIRT_L2_PHYS  1    // Physical L2 → Virtual L2
#define METER0_VIRT_L3_PHYS  -1   // Not used

// --- Meter 1: Inverter Input (single-phase) ---
#define METER1_SERVICE_ID    "grid_inverter"
#define METER1_NAME          "Inverter Input"
#define METER1_DEVICE_TYPE   "acload"
#define METER1_PHASE_COUNT   1
#define METER1_VIRT_L1_PHYS  2    // Physical L3 → Virtual L1
#define METER1_VIRT_L2_PHYS  -1   // Not used
#define METER1_VIRT_L3_PHYS  -1   // Not used

// ============================================================================
// Generator Handoff (transfer-switched inlet meter -> genset output)
// ============================================================================
// When a generator runs, the AC transfer switch routes its output through the same
// inlet a "grid" meter measures, so that meter now reads the GENERATOR's output.
// Reporting it on BOTH the grid meter AND a genset service double-counts it in Venus
// systemcalc (Consumption = grid + genset). With handoff enabled, while a generator
// is running we move that meter's readings to the genset's AC paths and zero the grid
// meter, so exactly one service reports the power.
//
// Deliberately GENERIC - watches any Venus genset's /StatusCode, no dependency on a
// particular controller. Degrades safely: with no genset present, or a genset that
// has no writable AC paths, the inlet meter just keeps reporting as grid (the power
// is still counted once - never dropped).
#ifndef ENABLE_GENSET_HANDOFF
#define ENABLE_GENSET_HANDOFF 1
#endif

// Virtual-meter index that sits AFTER the transfer switch (the shared inlet).
#define GENSET_HANDOFF_METER        0

// Genset device instance to follow, or -1 to track whichever genset is running.
#define GENSET_INSTANCE             -1

// /StatusCode value meaning "running / producing AC" (Venus genset: 8 = Running).
#define GENSET_RUNNING_STATUSCODE   8

// Keepalive cadence that keeps the GX pushing N/ genset topics to us (< ~60s cutoff).
#define GENSET_KEEPALIVE_MS         25000

// Every Nth keepalive triggers a full republish (re-learn genset presence / AC
// capability for a genset that appeared after we connected). First one always does.
#define GENSET_REPUBLISH_EVERY      10

// ============================================================================
// Timing Configuration (milliseconds)
// ============================================================================
#define PUBLISH_INTERVAL_MS     500        // Publish to MQTT every 1 second
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

#endif // CONFIG_H
