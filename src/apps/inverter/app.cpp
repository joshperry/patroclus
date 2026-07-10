/*
 * apps/inverter - patroclus02: the inverter's input and output.
 *
 * This board's MV-3P75CT has one CT on the inverter's (single-phase, L1-fed) input
 * conductor and one on its output-bus conductor. Each is published as its own
 * single-phase virtual meter, so the accurate inverter in/out power the Magnum RS485
 * protocol can't provide (hypnos registers the inverter as a vebus device but has no
 * real AC power for it) becomes visible on the GX.
 *
 * Device types here are provisional observation labels. The end state (see the plan's
 * "Accounting" section) is to feed these numbers onto hypnos's vebus service so Venus
 * systemcalc computes ConsumptionOnInput/Output natively - a follow-up that slots in
 * via a custom projection / direct W/ writes without changing the runtime.
 *
 * Build: pio run -e inverter           (OTA: pio run -e inverter_ota -t upload)
 */
#include "Runtime.h"

using namespace patroclus;

static Runtime rt;

void setup() {
    rt.setIdentity("patroclus02");

    // One CT on the inverter input conductor (physical L1 -> virtual L1).
    rt.addMeter("inv_input", "Inverter Input", "grid")
        .mapPhase(0, 0);

    // One CT on the inverter output bus (physical L2 -> virtual L1).
    rt.addMeter("inv_output", "Inverter Output", "acload")
        .mapPhase(0, 1);

    rt.begin();
}

void loop() {
    rt.loop();
}
