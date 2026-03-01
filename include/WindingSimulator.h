#pragma once
#include "WindingGeometry.h"

struct SimulationSummary {
    long targetTurns;
    long turnsPerPass;
    long passes;
    float effectiveWidth_mm;
    float packingFactor;
    float lateralSlack_mm;
};

// Simulate winding given geometry and parameters.
// packingFactor: >1 means looser packing (wire takes more space including gaps)
// lateralSlack_mm: extra effective spacing per turn introduced by human play (mm)
SimulationSummary simulateWinding(const WindingGeometry& g, long targetTurns,
                                  float packingFactor = 1.0f,
                                  float lateralSlack_mm = 0.0f);

// Helper that prints a human readable summary to Serial
void printSimulation(const SimulationSummary& s);
