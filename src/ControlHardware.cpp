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
    // initialize pot
    _pot.begin();

    // initialize encoder pins and ISR
    pinMode(ENC1_CLK, INPUT_PULLUP);
    pinMode(ENC1_DT,  INPUT_PULLUP);
    _encLastAB = ((uint8_t)digitalRead(ENC1_CLK) << 1) | (uint8_t)digitalRead(ENC1_DT);
    attachInterrupt(digitalPinToInterrupt(ENC1_CLK), ControlHardware::_encISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_DT),  ControlHardware::_encISR, CHANGE);
    Diag::info("[ControlHardware] initialized (pot + encoder)");
}

void ControlHardware::tick(uint32_t now, SessionController::TickInput& out)
{
    // Encoder processing
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

    // Potentiometer reading
    if (now - _lastPotMs >= POT_READ_INTERVAL) {
        _lastPotMs = now;
        _lastPotHz = _pot.readHz();
        float level = (float)_lastPotHz / (float)SPEED_HZ_MAX;
        out.hasPot = true;
        out.potLevel = level;
    }

    // propagate timestamp
    out.now = now;
}

uint32_t ControlHardware::readPotHz()
{
    return _lastPotHz;
}

int32_t ControlHardware::getEncoderCount()
{
    return _encCount;
}

void ControlHardware::resetEncoderCount(int32_t v)
{
    _encCount = v;
}

void IRAM_ATTR ControlHardware::_encISR()
{
    uint32_t now = micros();
    if (now - _encLastUs < ENC_DEBOUNCE_US) return;
    _encLastUs = now;
    uint8_t a = digitalRead(ENC1_CLK);
    uint8_t b = digitalRead(ENC1_DT);
    uint8_t ab = (a << 1) | b;
    _encCount += QEM[(_encLastAB << 2) | ab];
    _encLastAB = ab;
}
