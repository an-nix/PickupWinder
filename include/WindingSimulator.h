#pragma once
#include "WindingGeometry.h"

/**
 * @brief Summary of a winding simulation run.
 */
struct SimulationSummary {
	/** Target turns requested by the simulation. */
    long targetTurns;
	/** Effective turns-per-pass used for the simulation. */
    long turnsPerPass;
	/** Number of passes required to reach target turns. */
    long passes;
	/** Usable winding width derived from geometry. */
    float effectiveWidth_mm;
	/** Packing multiplier applied to nominal spacing. */
    float packingFactor;
	/** Additional lateral spacing caused by mechanical slack/play. */
    float lateralSlack_mm;
};

/**
 * @brief Simulate winding progression from geometry and process parameters.
 *
 * @param g Winding geometry.
 * @param targetTurns Requested turns.
 * @param packingFactor Packing multiplier; >1.0 means looser packing.
 * @param lateralSlack_mm Extra spacing per turn in mm due to mechanical play.
 * @return Aggregated simulation summary.
 */
SimulationSummary simulateWinding(const WindingGeometry& g, long targetTurns,
                                  float packingFactor = 1.0f,
                                  float lateralSlack_mm = 0.0f);

/**
 * @brief Print a human-readable simulation summary to Serial.
 * @param s Summary to print.
 */
void printSimulation(const SimulationSummary& s);
