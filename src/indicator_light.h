//
// Created by Paul Fornage on 4/24/2025.
//



#ifndef INDICATOR_LIGHT_H
#define INDICATOR_LIGHT_H
#include "clearcore.h"


enum class LightPattern {
    OFF,        // Always off
    ON,         // Always on
    FLASH1,     // Flash 1 / 4
    FLASH2,     // Flash 2 / 4
    FLASH3,     // Flash 3 / 4
    BLINK,      // Blink at constant rate ( flash 4/4 )
    STROBE,     // Strobes very fast, always uses a period of 2ms, regardless of what period is set to
};

class IndicatorLight {
public:
    explicit IndicatorLight(Connector* outputPin);

    IndicatorLight(); // Default constructor

    void setPin(Connector* outputPin) volatile; // Method to change pin


    /**
     * Set the pattern.
     * @param newPattern See `LightPattern` enum
     */
    void setPattern(LightPattern newPattern) volatile;

    void setInverted(bool is_inverted) volatile;

    /**
     * Set the period
     * @param newPeriod The period over which to repeat the pattern. Measured in 8ms; eg period of 4 repeats over 32ms
     *
     */
    void setPeriod(uint32_t newPeriod) volatile;

    /**
     * Call this function every ms
     * This calculates and writes the state
     */
    void tick() volatile;

    // Get current pattern
    LightPattern getPattern() const volatile { return pattern; }

private:
    bool inverted;
    Connector* pin;
    LightPattern pattern;
    uint32_t period;
    uint32_t tick_counter;
    bool output_state;

    // Helper function to reset the pattern
    void resetPattern() volatile;

    // Calculate output based on pattern and counter
    bool calculateOutput() const volatile;
};


#endif //INDICATOR_LIGHT_H
