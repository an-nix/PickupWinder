#include "ControlHardware.h"
#include "Diag.h"

volatile int32_t ControlHardware::_encCount = 0;
volatile uint8_t  ControlHardware::_encLastAB = 0;
volatile uint32_t ControlHardware::_encLastUs = 0;

const int8_t ControlHardware::QEM[16] = { 0,-1, 1, 0,
                                         1, 0, 0,-1,
                                        -1, 0, 0, 1,
                                         0, 1,-1, 0 };


ControlHardware::ControlHardware()
{
}

void ControlHardware::begin()
{
    // Initialize the filtered potentiometer input pipeline (inlined SpeedInput logic).
    _initPotSmoothing();

    // Initialize footswitch input as active-low with internal pull-up.
    pinMode(FOOTSWITCH_PIN, INPUT_PULLUP);
    const bool initialFootswitchPressed =
        (digitalRead(FOOTSWITCH_PIN) == (FOOTSWITCH_ACTIVE_LOW ? LOW : HIGH));
    _lastFootswitchRaw = initialFootswitchPressed;
    _footswitchStable = initialFootswitchPressed;
    _lastFootswitchChangeMs = millis();

    // Initialize encoder pins and attach the quadrature ISR on both channels.
    pinMode(ENC1_CLK, INPUT_PULLUP);
    pinMode(ENC1_DT,  INPUT_PULLUP);
    _encLastAB = ((uint8_t)digitalRead(ENC1_CLK) << 1) | (uint8_t)digitalRead(ENC1_DT);
    attachInterrupt(digitalPinToInterrupt(ENC1_CLK), ControlHardware::_encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_DT),  ControlHardware::_encISR, CHANGE);
    Diag::info("[ControlHardware] initialized (pot + footswitch + encoder)");
}

void ControlHardware::tick(uint32_t now, SessionController::TickInput& out)
{
    // Footswitch processing with simple time-based debounce.
    const bool footswitchPressed =
        (digitalRead(FOOTSWITCH_PIN) == (FOOTSWITCH_ACTIVE_LOW ? LOW : HIGH));
    if (footswitchPressed != _lastFootswitchRaw) {
        _lastFootswitchRaw = footswitchPressed;
        _lastFootswitchChangeMs = now;
    }
    if (_footswitchStable != _lastFootswitchRaw &&
        now - _lastFootswitchChangeMs >= FOOTSWITCH_DEBOUNCE_MS) {
        _footswitchStable = _lastFootswitchRaw;
        out.hasFootswitch = true;
        out.footswitch = _footswitchStable;
    }

    // Convert raw quadrature count into a bounded delta and propagate via TickInput.
    int32_t cur = _encCount;
    int32_t delta = cur - _lastEncConsumed;
    if (delta != 0) {
        constexpr int32_t MAX_ENC_DELTA = 4;
        if (delta >  MAX_ENC_DELTA) delta =  MAX_ENC_DELTA;
        if (delta < -MAX_ENC_DELTA) delta = -MAX_ENC_DELTA;
        _lastEncConsumed = cur;
        out.encoderDelta = delta;
    } else {
        out.encoderDelta = 0;
    }
    if (now - _lastEncMs >= 50) {
        _lastEncMs = now;
        if (cur != _lastPrinted) {
            _lastPrinted = cur;
            Serial.printf("[Encoder] count = %ld\n", (long)cur);
        }
    }


    // Sample the potentiometer at a controlled rate to avoid excessive ADC work.
    if (now - _lastPotMs >= POT_READ_INTERVAL) {
        _lastPotMs = now;
        _lastPotHz = _readPotHz();
        float level = (float)_lastPotHz / (float)SPEED_HZ_MAX;
        out.hasPot = true;
        out.potLevel = level;
    }

    // Always propagate the control-loop timestamp.
    out.now = now;
}

// --- Inlined SpeedInput logic ---

void ControlHardware::_initPotSmoothing() {
    // Pre-fill the entire filter buffer with the current ADC reading.
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) {
        _potSamples[i] = analogRead(POT_PIN);
    }
    _potIdx = 0;
    _potLastHz = 0;
    Serial.println("[ControlHardware] Pot smoothing initialized");
}

uint32_t ControlHardware::_readPotHz() {
    // Write the new sample into the circular buffer at the current index.
    _potSamples[_potIdx] = analogRead(POT_PIN);
    _potIdx = (_potIdx + 1) % POT_FILTER_SIZE;

    // Compute the mean of all samples in the buffer.
    uint32_t sum = 0;
    for (uint8_t i = 0; i < POT_FILTER_SIZE; i++) sum += _potSamples[i];
    uint32_t avg = sum / POT_FILTER_SIZE;

    // Invert ADC reading if the pot is wired in reverse.
#if POT_INVERTED
    avg = 4095 - avg;
#endif

    // Zero band: when the pot is physically at the low stop, return zero immediately.
    if (avg <= POT_ADC_ZERO_BAND) {
        _potLastHz = 0;
        return 0;
    }

    uint32_t hz;
    if (avg >= POT_ADC_FULL_BAND) {
        hz = SPEED_HZ_MAX;
    } else {
        // Exponential response curve for finer low-speed control.
        float t = (float)(avg - POT_ADC_ZERO_BAND) / (float)(POT_ADC_FULL_BAND - POT_ADC_ZERO_BAND);
        t = constrain(t, 0.0f, 1.0f);
        static const float invDenom = 1.0f / (expf(POT_EXP_K) - 1.0f);
        hz = (uint32_t)((expf(POT_EXP_K * t) - 1.0f) * invDenom * (float)SPEED_HZ_MAX);
    }
    _potLastHz = hz;
    return hz;
}

void IRAM_ATTR ControlHardware::_encISR()
{
    // Keep the ISR minimal: debounce, decode one quadrature step, and exit.
    uint32_t now = micros();
    if (now - _encLastUs < ENC_DEBOUNCE_US) return;
    _encLastUs = now;
    uint8_t a = digitalRead(ENC1_CLK);
    uint8_t b = digitalRead(ENC1_DT);
    uint8_t ab = (a << 1) | b;
    _encCount += QEM[(_encLastAB << 2) | ab];
    _encLastAB = ab;
}
