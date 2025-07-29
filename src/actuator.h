
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "ClearCore.h"

class Actuator {
public:
    virtual ~Actuator() = default;

    void set_actuator_pin(Connector* actuator_pin, bool inverted = false);

    /**
     * Sets the commanded state
     * @return True for success. Only failure mode is pins being uninitialized
     */
    bool set_commanded_state(bool new_state);

    /**
     * Gets the last commanded state
     * @return the last commanded state
     */
    bool get_commanded_state() const;

    /**
     * Does the actuator need more time to get to it's commanded state.
     * For blind actuators this always returns false
     * @return Does the actuator need more time to get to it's commanded state?
     */
    virtual bool is_mismatch() const;

    virtual bool is_sensed() const;

    virtual bool is_fully_engaged() const;

    virtual bool is_fully_disengaged() const;

protected:
    Connector* actuator_pin{nullptr};
    bool actuator_inverted{false};
    bool commanded_state{false};
};


class SensedActuator final : public Actuator {
public:
    SensedActuator() = default;
    void set_sense_pin(Connector* sense_pin, bool inverted = false);

    /**
     * Gets the measured state. Returns false if the sense pin has not been initialized
     * @return the measured state, or false if the sense pin has not been initialized
     */
    bool get_measured_state() const;


    /**
     * checks if the commanded state does not equal the measured state
     * @return get_commanded_state() != get_measured_state()
     */
    bool is_mismatch() const override;

    bool is_sensed() const override;

    bool is_fully_engaged() const override;

    bool is_fully_disengaged() const override;

private:
    Connector* sense_pin{nullptr};
    bool sensor_inverted{false};
};

#endif //ACTUATOR_H