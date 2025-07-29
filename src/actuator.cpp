
#include "actuator.h"

bool Actuator::set_commanded_state(const bool new_state) {
    if (actuator_pin == nullptr) {return false;}
    commanded_state = new_state;
    actuator_pin->State(new_state != actuator_inverted);
    return true;
}

bool Actuator::get_commanded_state() const {
    return commanded_state;
}

// BlindActuator implementation
void Actuator::set_actuator_pin(Connector* actuator_pin_in, const bool inverted) {
    this->actuator_inverted = inverted;
    this->actuator_pin = actuator_pin_in;
}

bool Actuator::is_mismatch() const {
    return false;
}

bool Actuator::is_sensed() const {
    return false;
}

bool Actuator::is_fully_engaged() const {
    return this->get_commanded_state();
}
bool Actuator::is_fully_disengaged() const {
    return !this->get_commanded_state();
}

// SensedActuator implementation


void SensedActuator::set_sense_pin(Connector* sense_pin_in, const bool inverted) {
    this->sensor_inverted = inverted;
    this->sense_pin = sense_pin_in;
}

bool SensedActuator::get_measured_state() const {
    if (sense_pin == nullptr) {return false;}
    return (sense_pin->State()) != sensor_inverted;
}

bool SensedActuator::is_mismatch() const {
    return get_commanded_state() != get_measured_state();
}


bool SensedActuator::is_sensed() const {
    return true;
}

bool SensedActuator::is_fully_engaged() const {
    return this->get_commanded_state() && get_measured_state();
}
bool SensedActuator::is_fully_disengaged() const {
    return !this->get_commanded_state() && !get_measured_state();
}