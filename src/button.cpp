//
// Created by Paul Fornage on 4/29/2025.
//

#include "button.h"

Button::Button()
    : current_state(false)
    , last_state(false) {
}

Button::Button(const bool initial_state, const bool past_state)
    : current_state(initial_state)
    , last_state(past_state) {
}

bool Button::is_rising() const {
    return current_state && !last_state;
}
bool Button::is_falling() const {
    return !current_state && last_state;
}
bool Button::is_changing() const {
    return current_state ^ last_state;
}

bool Button::get_current_state() const {
    return current_state;
}
void Button::new_reading(const bool new_state) {
    last_state = current_state;
    current_state = new_state;
}