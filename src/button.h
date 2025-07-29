//
// Created by Paul Fornage on 4/29/2025.
//

#ifndef BUTTON_H
#define BUTTON_H

class Button {
public:
    Button();
    Button(bool initial_state, bool past_state);
    bool is_rising() const;
    bool is_falling() const;
    bool is_changing() const;
    bool get_current_state() const;
    void new_reading(bool new_state);
private:
    bool current_state;
    bool last_state;
};

#endif //BUTTON_H
