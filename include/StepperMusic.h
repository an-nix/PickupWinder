#pragma once
#include <Arduino.h>

// ── Musical note frequencies (Hz) ────────────────────────────────────────────
#define NOTE_REST  0
// Octave 4
#define NOTE_C4   262
#define NOTE_D4   294
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_Fs4  370   // F#4
#define NOTE_G4   392
#define NOTE_A4   440
#define NOTE_Bb4  466
#define NOTE_B4   494
// Octave 5
#define NOTE_C5   523
#define NOTE_D5   587
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_Fs5  740   // F#5
#define NOTE_G5   784

// ── Note definition ──────────────────────────────────────────────────────────
struct MusicNote {
    uint16_t freq;       // Frequency in Hz (0 = rest)
    uint16_t durationMs; // Duration ms  (0 = end marker)
};

// ── Adeste Fideles (O Come All Ye Faithful) ──────────────────────────────────
// Key: G major.  Tempo ≈ 92 BPM → Q=652ms  E=326ms  S=163ms  H=1304ms
//
// A short GAP rest is inserted between consecutive identical notes so each
// note sounds distinct on the monophonic stepper motor.
//
// Sections:
//   V1  "Adeste fideles"
//   V2  "Laeti triumphantes"
//   V3  "Venite venite / in Bethlehem"
//   V4  "Natum videte"
//   V5  "Regem angelorum"
//   R   Refrain "Venite adoremus" × 3 + "Dominum"

#define Q    652
#define Qd   978   // dotted quarter
#define E    326
#define S    163
#define H   1304
#define Hd  1956   // dotted half
#define GAP   50   // gap between repeated identical notes

static const MusicNote MELODY_LA_BAMBA[] = {

    // ════ V1 — "A-des-te  fi-de-les" ════
    // G4  G4    D5   B4   G5    F#5  D5 (held)
    {NOTE_G4,  Q},  {NOTE_REST, GAP}, {NOTE_G4,  Q},
    {NOTE_D5,  Q},  {NOTE_B4,   Q},
    {NOTE_G5,  Qd}, {NOTE_Fs5,  E},   {NOTE_D5,  H},
    {NOTE_REST, E},

    // ════ V2 — "lae-ti tri-um-phan-tes" ════
    // E5  E5    F#5  E5   D5   B4 (held)
    {NOTE_E5,  Q},  {NOTE_REST, GAP}, {NOTE_E5,  Q},
    {NOTE_Fs5, E},  {NOTE_E5,   E},
    {NOTE_D5,  Q},  {NOTE_B4,   Hd},
    {NOTE_REST, E},

    // ════ V3 — "Ve-ni-te, ve-ni-te" ════
    // C5   D5   C5   B4   A4   G4 (held)
    {NOTE_C5,  Q},  {NOTE_D5,  Q},
    {NOTE_C5,  Q},  {NOTE_B4,  Q},
    {NOTE_A4,  Q},  {NOTE_G4,  Hd},
    {NOTE_REST, E},

    // ════ V3b — "in  Beth-le-hem" ════
    // G4   A4    B4    C5    D5 (held)
    {NOTE_G4,  Q},  {NOTE_A4,  Q},
    {NOTE_B4,  Q},  {NOTE_C5,  Q},
    {NOTE_D5,  Hd},
    {NOTE_REST, E},

    // ════ V4 — "Na-tum  vi-de-te" ════
    // G5   F#5   E5   D5   C5   B4   A4
    {NOTE_G5,  Q},  {NOTE_Fs5, Q},
    {NOTE_E5,  Q},  {NOTE_D5,  E},  {NOTE_C5,  E},
    {NOTE_B4,  Q},  {NOTE_A4,  Q},
    {NOTE_REST, S},

    // ════ V5 — "Re-gem  an-ge-lo-rum" ════
    // D5   C5   B4   A4   G4 (held)
    {NOTE_D5,  Q},  {NOTE_C5,  Q},
    {NOTE_B4,  Q},  {NOTE_A4,  Q},
    {NOTE_G4,  Hd},
    {NOTE_REST, E},

    // ════ R — Refrain "Ve-ni-te a-do-re-mus" (1st) ════
    // D5  D5    E5   D5   C5   B4   G4  A4  B4  C5
    {NOTE_D5,  Q},  {NOTE_REST, GAP}, {NOTE_D5, Q},
    {NOTE_E5,  Q},  {NOTE_D5,  Q},
    {NOTE_C5,  Q},  {NOTE_B4,  Q},
    {NOTE_G4,  E},  {NOTE_A4,  E},   {NOTE_B4,  E},  {NOTE_C5, E},
    {NOTE_REST, S},

    // ════ R — "Ve-ni-te a-do-re-mus" (2nd) ════
    {NOTE_D5,  Q},  {NOTE_REST, GAP}, {NOTE_D5, Q},
    {NOTE_E5,  Q},  {NOTE_D5,  Q},
    {NOTE_C5,  Q},  {NOTE_B4,  Q},
    {NOTE_G4,  E},  {NOTE_A4,  E},   {NOTE_B4,  E},  {NOTE_C5, E},
    {NOTE_REST, S},

    // ════ R — "Ve-ni-te a-do-re-mus" (3rd — ornamental cadence) ════
    // D5   C5   B4   A4   B4  C5  D5    C5   B4
    {NOTE_D5,  Q},  {NOTE_C5,  Q},
    {NOTE_B4,  Q},  {NOTE_A4,  Q},
    {NOTE_B4,  E},  {NOTE_C5,  E},
    {NOTE_D5,  Qd}, {NOTE_C5,  E},   {NOTE_B4,  Q},
    {NOTE_REST, S},

    // ════ R — "Do-mi-num" ════
    // A4   G4 (held)
    {NOTE_A4,  Q},
    {NOTE_G4,  Hd},

    {NOTE_REST, 0},  // End marker
};

#undef Q
#undef Qd
#undef E
#undef S
#undef H
#undef Hd
#undef GAP

static const size_t LA_BAMBA_LENGTH =
    sizeof(MELODY_LA_BAMBA) / sizeof(MELODY_LA_BAMBA[0]);
