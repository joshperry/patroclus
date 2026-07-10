/*
 * apps/split_phase - patroclus01: the split-phase transfer-switch input.
 *
 * This board's MV-3P75CT has its two CTs on the L1 + L2 legs coming out of the
 * transfer switch (shore / generator) into the panel's split-phase input busses. We
 * publish them as a single 2-phase "grid" meter, so the GX sees the whole input feed.
 *
 * The inverter's own input/output metering lives on a separate board (apps/inverter,
 * patroclus02) with its CTs clamped at the inverter - see that app and the plan's
 * accounting notes.
 *
 * Build: pio run -e split_phase        (OTA: pio run -e split_phase_ota -t upload)
 */
#include "Runtime.h"

using namespace patroclus;

static Runtime rt;

void setup() {
    rt.setIdentity("patroclus01");

    // Split-phase shore/gen input: physical L1 -> virtual L1, physical L2 -> virtual L2.
    rt.addMeter("grid_shore", "Shore Power", "grid")
        .mapPhase(0, 0)
        .mapPhase(1, 1);

    rt.begin();
}

void loop() {
    rt.loop();
}
