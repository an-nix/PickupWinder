#include "WindingSimulator.h"
#include <Arduino.h>

SimulationSummary simulateWinding(const WindingGeometry& g, long targetTurns,
                                  float packingFactor, float lateralSlack_mm) {
    SimulationSummary s{};
    s.targetTurns = targetTurns;
    s.packingFactor = packingFactor;
    s.lateralSlack_mm = lateralSlack_mm;

    float effW = g.effectiveWidth();
    s.effectiveWidth_mm = effW;

    // effective wire spacing including packing and human slack
    float wireSpacing = g.wireDiameter_mm * packingFactor + lateralSlack_mm;
    if (wireSpacing <= 0.0f) wireSpacing = g.wireDiameter_mm;

    long tpp = (long)floor(effW / wireSpacing);
    if (tpp < 1) tpp = 1;
    s.turnsPerPass = tpp;

    s.passes = (long)((targetTurns + tpp - 1) / tpp);
    return s;
}

void printSimulation(const SimulationSummary& s) {
    Serial.println("--- Winding simulation summary ---");
    Serial.printf("Target turns : %ld\n", s.targetTurns);
    Serial.printf("Effective width : %.2f mm\n", s.effectiveWidth_mm);
    Serial.printf("Packing factor : %.3f, lateral slack per turn : %.3f mm\n",
                  s.packingFactor, s.lateralSlack_mm);
    Serial.printf("Turns per pass : %ld\n", s.turnsPerPass);
    Serial.printf("Estimated passes (aller/retour) : %ld\n", s.passes);
    Serial.println("----------------------------------");
}
