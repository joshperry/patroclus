/*
 * VirtualMeter - one virtual meter an app projects onto the GX.
 *
 * A Meter names a gx-projector service (serviceId), its display name and DeviceType,
 * and a projection: how the physical MeterData becomes the meter's published Ac/*
 * values. The common case is a direct per-leg copy of physical CTs (mapPhase); the
 * setProjection() escape hatch lets an app compute derived/composite values instead
 * (e.g. subtracting an inverter's charge draw for proper GX accounting).
 *
 * The app composes these through Runtime::addMeter(); the runtime owns the array.
 */
#ifndef PATROCLUS_VIRTUAL_METER_H
#define PATROCLUS_VIRTUAL_METER_H

#include <ArduinoJson.h>
#include "CanMeter.h"

namespace patroclus {

class Meter;

// Fill one meter's Ac/* values from the physical MeterData. Return false to skip
// publishing this cycle (e.g. no fresh mapped phase). `ctx` is the pointer passed to
// setProjection(); the Meter itself is passed so a projection can read its phase map.
typedef bool (*ProjectionFn)(const Meter& meter, const MeterData& phys,
                             JsonObject values, void* ctx);

// The default projection: a direct per-leg copy of the mapped physical CTs, matching
// the original firmware's publishVirtualMeter output (per-leg V/I/P/PF/energy plus the
// summed Ac/Power, Ac/Frequency and total energy). Declared here, defined below.
bool defaultProjection(const Meter& meter, const MeterData& phys, JsonObject values);

class Meter {
public:
    const char* serviceId = "";     // gx-projector tag (dbus bus-name suffix)
    const char* name = "";          // CustomName shown on the GX
    const char* deviceType = "grid";// rides the registration init's DeviceType

    // Virtual leg (0..2) -> physical CT index (0..2); -1 = unused.
    int physicalPhase[3] = {-1, -1, -1};

    // Optional computed projection; when null the default per-leg copy is used.
    ProjectionFn projection = nullptr;
    void* projectionCtx = nullptr;

    // Map virtual leg <- physical CT (direct copy). Fluent.
    Meter& mapPhase(int virtualLeg, int physicalCt) {
        if (virtualLeg >= 0 && virtualLeg < 3) physicalPhase[virtualLeg] = physicalCt;
        return *this;
    }

    // Replace the direct copy with a computed projection (derived/composite values).
    Meter& setProjection(ProjectionFn fn, void* ctx = nullptr) {
        projection = fn;
        projectionCtx = ctx;
        return *this;
    }

    int phaseCount() const {
        int n = 0;
        for (int i = 0; i < 3; i++) if (physicalPhase[i] >= 0) n++;
        return n;
    }

    // Fill `values` using the projection (or the default). Returns whether to publish.
    bool project(const MeterData& phys, JsonObject values) const {
        if (projection) return projection(*this, phys, values, projectionCtx);
        return defaultProjection(*this, phys, values);
    }
};

inline bool defaultProjection(const Meter& meter, const MeterData& phys, JsonObject values) {
    // Publish only when at least one mapped phase has fresh data.
    bool hasValidData = false;
    for (int v = 0; v < 3; v++) {
        if (isPhaseValid(phys, meter.physicalPhase[v])) { hasValidData = true; break; }
    }
    if (!hasValidData) return false;

    float totalPower = 0;
    float totalEnergyRev = 0;
    const char* phasePaths[] = {"Ac/L1", "Ac/L2", "Ac/L3"};

    for (int v = 0; v < 3; v++) {
        int phys_i = meter.physicalPhase[v];
        if (phys_i < 0 || phys_i >= 3) continue;
        if (!isPhaseValid(phys, phys_i)) continue;

        const PhaseMeasurement& d = phys.phases[phys_i];
        String base = String(phasePaths[v]);
        values[base + "/Voltage"] = d.voltage;
        values[base + "/Current"] = d.current;
        values[base + "/Power"] = d.power;
        values[base + "/PowerFactor"] = d.powerFactor;
        values[base + "/Energy/Forward"] = d.energyForward;
        values[base + "/Energy/Reverse"] = d.energyReverse;

        totalPower += d.power;
        totalEnergyRev += d.energyReverse;
    }

    values["Ac/Power"] = totalPower;
    values["Ac/Frequency"] = phys.frequency;
    values["Ac/Energy/Forward"] = phys.totalEnergyForward;
    values["Ac/Energy/Reverse"] = totalEnergyRev;
    return true;
}

}  // namespace patroclus

#endif  // PATROCLUS_VIRTUAL_METER_H
