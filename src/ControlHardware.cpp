#include "ControlHardware.h"
#include "Diag.h"

volatile int32_t ControlHardware::_encCount = 0;
volatile uint8_t  ControlHardware::_encLastAB = 0;
volatile uint32_t ControlHardware::_encLastUs = 0;

const int8_t ControlHardware::QEM[16] = { 0,-1, 1, 0,
                                         1, 0, 0,-1,
                                        -1, 0, 0, 1,
                                         0, 1,-1, 0 };

ControlHardware::ControlHardware(WinderApp& winder)
    : _winder(winder)
{
}

void ControlHardware::begin()
{
    // Initialize the filtered potentiometer input pipeline.
    _pot.begin();

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

    // Convert raw quadrature count into a bounded delta and forward it to the
    // winding domain for paused-position trim adjustments.
    int32_t cur = _encCount;
    int32_t delta = cur - _lastEncConsumed;
    if (delta != 0) {
        constexpr int32_t MAX_ENC_DELTA = 4;
        if (delta >  MAX_ENC_DELTA) delta =  MAX_ENC_DELTA;
        if (delta < -MAX_ENC_DELTA) delta = -MAX_ENC_DELTA;
        _lastEncConsumed = cur;
        _winder.handleEncoderDelta(delta);
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
        _lastPotHz = _pot.readHz();
        float level = (float)_lastPotHz / (float)SPEED_HZ_MAX;
        out.hasPot = true;
        out.potLevel = level;
    }

    // Always propagate the control-loop timestamp.
    out.now = now;
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
