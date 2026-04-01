# Realtime and Memory Review Checklist

## Control loop safety

- [ ] No `delay()` in runtime control paths.
- [ ] Control loop remains bounded and non-blocking.
- [ ] Worst control/comms loop time is monitored and reviewed.

## Logging

- [ ] High-frequency logs are gated by `DIAG_VERBOSE`.
- [ ] Production profile logs are concise and actionable.
- [ ] No secrets are logged (SSID/password/tokens).

## Command pipeline

- [ ] Every externally visible command is declared in `CommandRegistry`.
- [ ] Aliases and schema validation are centralized.
- [ ] Rejection/drop counters are monitored.

## Memory and JSON

- [ ] Recurrent code paths avoid dynamic `String` usage.
- [ ] JSON parsing/serialization uses bounded `StaticJsonDocument` where possible.
- [ ] Large stack buffers are justified and reviewed.
- [ ] Heap min/free telemetry is tracked over representative runs.

## Protocol evolution

- [ ] Version constants updated in `Protocol.h` when schema changes.
- [ ] `/protocol.json` and `/capabilities.json` remain backward compatible.
- [ ] UART startup banner version remains synchronized.

## Test and CI

- [ ] Unit tests cover command normalization and validation.
- [ ] CI compiles firmware profiles and tests.
- [ ] New command or recipe changes include tests and README updates.
