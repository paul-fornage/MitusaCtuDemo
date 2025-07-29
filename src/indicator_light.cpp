
#include "indicator_light.h"
#include "Arduino.h"

IndicatorLight::IndicatorLight(Connector* outputPin)
    : inverted(false)
    , pin(outputPin)
    , pattern(LightPattern::OFF)
    , period(100)
    , tick_counter(0)
    , output_state(false) {
}

IndicatorLight::IndicatorLight()
    : inverted(false)
    , pin(nullptr)
    , pattern(LightPattern::OFF)
    , period(100)
    , tick_counter(0)
    , output_state(false) {
}

void IndicatorLight::setPin(Connector* outputPin) volatile {
    pin = outputPin;
    resetPattern(); // Reset the pattern to ensure proper initialization of the new pin
}


void IndicatorLight::setPattern(const LightPattern newPattern) volatile {
    pattern = newPattern;
    resetPattern();
}

void IndicatorLight::setPeriod(const uint32_t newPeriod) volatile {
    period = newPeriod;
    resetPattern();
}

void IndicatorLight::resetPattern() volatile {
    tick_counter = 0;
    output_state = false;
    if (pin) {
        pin->State(output_state);
    }
}

bool IndicatorLight::calculateOutput() const volatile {
    const uint32_t position = tick_counter % (period*8);
    const uint32_t phase = position / period; // should be 0-7 inclusive

    // ConnectorUsb.SendLine(phase);
    
    switch (pattern) {
        case LightPattern::OFF:
            return false;
            
        case LightPattern::ON:
            return true;
            
        case LightPattern::FLASH1:
            return phase == 0;
            
        case LightPattern::FLASH2: {
            return phase == 0 || phase == 2;
        }
            
        case LightPattern::FLASH3: {
            return phase == 0 || phase == 2 || phase == 4;
        }
        case LightPattern::BLINK:
            return !(phase % 2);
        case LightPattern::STROBE:
            return !(tick_counter % 2);
    }
    // should never happen
    return false;
}

void IndicatorLight::tick() volatile {
    tick_counter++;
    if (!pin) return;
    
    // Calculate new output state
    output_state = calculateOutput();
    
    // Update output pin
    pin->State(output_state != inverted);

}

void IndicatorLight::setInverted(bool is_inverted) volatile {
  inverted = is_inverted;
}
