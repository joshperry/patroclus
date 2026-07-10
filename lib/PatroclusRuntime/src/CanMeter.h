/*
 * CanMeter - VE.CAN energy-meter decode for the MV-3P75CT.
 *
 * Owns the TWAI (CAN) peripheral, drains frames, and decodes the VE.CAN energy-meter
 * format into a physical MeterData (L1/L2/L3 + frequency + total energy). This is the
 * per-board physical layer; how those physical phases project onto virtual meters is
 * the app's business (see VirtualMeter.h / Runtime).
 *
 * Decode logic is lifted unchanged from the original monolithic main.cpp.
 */
#ifndef PATROCLUS_CAN_METER_H
#define PATROCLUS_CAN_METER_H

#include <Arduino.h>
#include <driver/twai.h>
#include "patroclus_config.h"

namespace patroclus {

// Per-phase measurements from CAN (physical phases from the meter)
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

// Physical meter data (3 phases from an MV-3P75CT)
struct MeterData {
    float frequency;            // Hz (common to all phases)
    float totalEnergyForward;   // kWh (total from meter register 0x50)
    PhaseMeasurement phases[3]; // Physical L1, L2, L3
};

// One buffered CAN frame (for the MQTT capture stream)
struct CanCapture {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    bool extd;
};

// Is a physical phase present and fresh (within CAN_TIMEOUT_MS)?
inline bool isPhaseValid(const MeterData& d, int physicalPhase) {
    if (physicalPhase < 0 || physicalPhase >= 3) return false;
    const PhaseMeasurement& data = d.phases[physicalPhase];
    if (!data.valid) return false;
    unsigned long age = millis() - data.lastUpdate;
    return age <= CAN_TIMEOUT_MS;
}

class CanMeter {
public:
    // Install + start the TWAI driver. Returns false on failure (CAN won't work, but
    // the rest of the firmware still runs).
    bool begin();

    // Drain all pending frames into the decoded MeterData (non-blocking).
    void poll();

    const MeterData& data() const { return data_; }

    // --- debug / capture -----------------------------------------------------
    void setSerialDump(bool on) { serialDump_ = on; }
    bool serialDump() const { return serialDump_; }

    void setCaptureEnabled(bool on) { captureEnabled_ = on; captureCount_ = 0; }
    bool captureEnabled() const { return captureEnabled_; }

    int captureCount() const { return captureCount_; }
    const CanCapture& capture(int i) const { return captureBuffer_[i]; }
    void clearCapture() { captureCount_ = 0; }

private:
    void processFrame(const twai_message_t& msg);

    MeterData data_ = {};
    bool serialDump_ = false;
    bool captureEnabled_ = false;
    CanCapture captureBuffer_[CAN_CAPTURE_BUFFER_SIZE];
    int captureCount_ = 0;
};

}  // namespace patroclus

#endif  // PATROCLUS_CAN_METER_H
