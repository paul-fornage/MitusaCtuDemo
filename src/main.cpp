#include <Arduino.h>
#include <ClearCore.h>
#include <utility>
#include <RsTypeNames.h>
#include <ModBussy.h>
#include <Ethernet.h>
#include <NvmManager.h>


#define MB_COIL_BITS 256
#define MB_DISCRETE_BITS 16
#define MB_HREGS 128
#define MB_IREGS 1

#define ABS(x) ((x) < 0 ? -(x) : (x))

#define PRINT(expr) ConnectorUsb.Send(expr)
#define PRINTLN(expr) ConnectorUsb.SendLine(expr)

#define CARRIAGE_MOTOR ConnectorM0
#define MOTOR_MAX_STEPS 20000

#define STEPS_PER_HUNDRETH 8
#define START_POS_STEPS 800

#define ESTOP_CONNECTOR ConnectorDI6
#define ESTOP_CON_SAFE_STATE true
#define ESTOP_SAFE (ESTOP_CONNECTOR.State() == ESTOP_CON_SAFE_STATE)

static constexpr u16 NUM_POSITIONS = 16;

// Hregs
static constexpr u16 SELECTED_POSITION_OFFSET = 10; // position to go to when GO_TO_POSITION_LATCH is latched
static constexpr u16 CURRENT_POSITION_INDEX_OFFSET = 11;
static constexpr u16 SPEED_OFFSET = 12;
static constexpr u16 CURRENT_POSITION_OFFSET = 13;
static constexpr u16 POSITIONS_OFFSET = 16;
static constexpr u16 DELAYS_OFFSET = POSITIONS_OFFSET + NUM_POSITIONS;

// Coils
static constexpr u16 START_CYCLE_LATCH_OFFSET = 10;
static constexpr u16 HOME_LATCH_OFFSET = 11;
static constexpr u16 IS_HOMED_OFFSET = 12;

static constexpr u16 GO_TO_POSITION_LATCH_OFFSET = 14;
static constexpr u16 JOB_ACTIVE_OFFSET = 15;
static constexpr u16 STOP_CYCLE_LATCH_OFFSET = 16;
static constexpr u16 IN_ESTOP_OFFSET = 20;
static constexpr u16 ERROR_OFFSET = 21;
static constexpr u16 ENABLED_POSITIONS_OFFSET = 32;

bool home_latched = false;
bool start_cycle_latched = false;
bool stop_cycle_latched = false;
bool go_to_position_latched = false;


u16 coils[(MB_COIL_BITS + 15) / 16];
u16 discretes[(MB_DISCRETE_BITS + 15) / 16];
u16 holding[MB_HREGS];
u16 input[MB_IREGS];

u8 nv_ram[4];

enum class State : u16{
    IDLE,
    START_HOMING,
    WAIT_FOR_HOMING,
    RETURN_TO_START,
    WAIT_FOR_RETURN_TO_START,
    MANUAL_GO_TO_POSITION,
    MANUAL_GO_TO_POSITION_WAIT,
    JOB_CALC_NEXT_POSITION,
    JOB_MOVE_NEXT_POSITION,
    JOB_WAIT_POSITION,
    JOB_WAIT_DELAY,
    ESTOP_START,
    ESTOP,
    ERROR_STATE, // unrecoverable by design
};

bool is_homed = false;
bool in_estop = false;
u32 last_estop_time = 0;
auto last_state_before_estop = State::IDLE;
static constexpr u32 ESTOP_DEACTIVATE_COOLDOWN_MS = 1000;
u16 manual_target_pos_index = 65535; // 65535 always out of bounds
u16 cycle_last_index = 0; // gets set to `cycle_target_index` after reaching target and waiting delay
u16 cycle_target_index = 0; // gets set to the next valid target position at start of cycle
u32 cycle_reached_target_time = 0; // `millis()` at the instant the carriage reached the target pos
u32 millis_delay = 0; // `millis()` at the instant the carriage reached the target pos

ModBussy mb(502, coils, discretes, holding, input);

void print_motor_alerts();
void ethernet_setup(bool use_dhcp, const IPAddress &ip, u16 max_dhcp_attempts);
State state_machine_iter(State state_in);
void estop();
void print_state(State state_in);
State estop_resume_state(State state_in);
bool latch_handler(u16 offset);

auto machine_state = State::IDLE;
auto last_state = State::IDLE;

// Speed to weld in hundreths of an inch per second
u16 speed = 0;



int main() {
    ESTOP_CONNECTOR.Mode(Connector::INPUT_DIGITAL);
    ConnectorUsb.PortOpen();
    ConnectorUsb.Speed(115200);
    const u32 startTime = millis();
    while (!ConnectorUsb.PortIsOpen() && millis() - startTime < 5000) {  }

    if (ConnectorUsb.PortIsOpen()) {
        PRINTLN("USB connected");
    }

    ethernet_setup(true, IPAddress(192, 168, 1, 59), 10);
    mb.begin();
    mb.Coil(IS_HOMED_OFFSET, is_homed);
    mb.Coil(IN_ESTOP_OFFSET, in_estop);
    mb.Coil(START_CYCLE_LATCH_OFFSET, false);
    mb.Coil(HOME_LATCH_OFFSET, false);
    mb.Coil(GO_TO_POSITION_LATCH_OFFSET, false);
    mb.Coil(JOB_ACTIVE_OFFSET, false);

    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    CARRIAGE_MOTOR.AccelMax(15000);
    CARRIAGE_MOTOR.VelMax(5000);



    mb.Hreg(SPEED_OFFSET) = 100;

    while (true) {
        mb.task();
        mb.Coil(IS_HOMED_OFFSET, is_homed);
        mb.Coil(IN_ESTOP_OFFSET, in_estop);
        mb.Coil(ERROR_OFFSET, machine_state == State::ERROR_STATE);
        mb.Coil(JOB_ACTIVE_OFFSET,
            machine_state == State::JOB_WAIT_DELAY
            || machine_state == State::JOB_CALC_NEXT_POSITION
            || machine_state == State::JOB_WAIT_POSITION);

        const u16 speed = mb.Hreg(SPEED_OFFSET);
        CARRIAGE_MOTOR.VelMax(speed * STEPS_PER_HUNDRETH);

        home_latched = latch_handler(HOME_LATCH_OFFSET);
        start_cycle_latched = latch_handler(START_CYCLE_LATCH_OFFSET);
        stop_cycle_latched = latch_handler(STOP_CYCLE_LATCH_OFFSET);
        go_to_position_latched = latch_handler(GO_TO_POSITION_LATCH_OFFSET);

        mb.Hreg(CURRENT_POSITION_INDEX_OFFSET) = cycle_target_index;
        mb.Hreg(CURRENT_POSITION_OFFSET) = CARRIAGE_MOTOR.PositionRefCommanded();

        if (!ESTOP_SAFE && !in_estop) {
            estop();
        }
        last_state = machine_state;

        if (in_estop && machine_state != State::ESTOP && machine_state != State::ESTOP_START && machine_state != State::ERROR_STATE) {
            PRINTLN("`in_estop` was true but state didn't match. setting state to ESTOP_START");
            machine_state = State::ESTOP_START;
        }

        machine_state = state_machine_iter(machine_state);
        // print_state(machine_state);
        // PRINTLN("");
        if (last_state != machine_state) {
            PRINT("State changed from ");
            print_state(last_state);
            PRINT(" to ");
            print_state(machine_state);
            PRINTLN("");
        }
    }
}

State state_machine_iter(const State state_in) {
    switch (state_in) {
        case State::IDLE: {
            if (go_to_position_latched) {
                if (is_homed) {
                    return State::MANUAL_GO_TO_POSITION;
                } else {
                    PRINTLN("Can't return to start, not homed");
                    return State::IDLE;
                }
            }
            if (start_cycle_latched) {
                if (is_homed) {
                    PRINTLN("starting cycle");
                    return State::JOB_CALC_NEXT_POSITION;
                } else {
                    PRINTLN("Can't start job, not homed");
                    return State::IDLE;
                }
            }

            if (home_latched) {
                PRINTLN("Homing");

                return State::START_HOMING;
            }
            return State::IDLE;
        }
        case State::START_HOMING: {
            CARRIAGE_MOTOR.EnableRequest(false);
            delay(50);
            CARRIAGE_MOTOR.EnableRequest(true);
            delay(50);
            CARRIAGE_MOTOR.PositionRefSet(0);
            CARRIAGE_MOTOR.MoveVelocity(-1600);
            // PRINT("CARRIAGE_MOTOR.StatusReg().bit.StepsActive = ");
            // PRINTLN(CARRIAGE_MOTOR.StatusReg().bit.StepsActive);
            delay(50);
            return State::WAIT_FOR_HOMING;
        }
        case State::WAIT_FOR_HOMING: {
            if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
                PRINTLN("Motor homed");
                CARRIAGE_MOTOR.MoveStopAbrupt();
                CARRIAGE_MOTOR.PositionRefSet(0);
                is_homed = true;
                return State::RETURN_TO_START;
            }
            if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
                PRINTLN("Motor alert during homing");
                print_motor_alerts();
                is_homed = false;
                estop();
                return State::ESTOP_START;
            }
            if (CARRIAGE_MOTOR.PositionRefCommanded() > MOTOR_MAX_STEPS) {
                PRINTLN("Motor traveled more than max steps during homing");
                estop();
                return State::ESTOP_START;
            }
            return State::WAIT_FOR_HOMING;
        }
        case State::RETURN_TO_START: {
            CARRIAGE_MOTOR.Move(START_POS_STEPS, StepGenerator::MOVE_TARGET_ABSOLUTE);
            return State::WAIT_FOR_RETURN_TO_START;
        }
        case State::WAIT_FOR_RETURN_TO_START: {
            if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED && CARRIAGE_MOTOR.StepsComplete()) {
                PRINTLN("Motor returned to home");
                return State::IDLE;
            }
            if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
                PRINTLN("Motor alert during homing");
                print_motor_alerts();
                is_homed = false;
                estop();
                return State::ESTOP_START;
            }
            return State::WAIT_FOR_RETURN_TO_START;
        }
        case State::MANUAL_GO_TO_POSITION: {
            manual_target_pos_index = mb.Hreg(SELECTED_POSITION_OFFSET);
            PRINT("going to position index ");
            PRINTLN(manual_target_pos_index);
            if (manual_target_pos_index >= NUM_POSITIONS) {
                PRINTLN("Position index out of bounds.");
                return State::IDLE;
            }
            const u16 target_hundreths = mb.Hreg(POSITIONS_OFFSET + manual_target_pos_index);
            PRINT("position in hundreths: ");
            PRINTLN(target_hundreths);
            const i32 target_steps = target_hundreths * STEPS_PER_HUNDRETH;
            PRINT("position in steps: ");
            PRINTLN(target_steps);
            if (target_steps > MOTOR_MAX_STEPS) {
                PRINTLN("Target position too far for manual move");
                return State::ERROR_STATE;
            }
            CARRIAGE_MOTOR.Move(target_steps, StepGenerator::MOVE_TARGET_ABSOLUTE);
            return State::MANUAL_GO_TO_POSITION_WAIT;
        }
        case State::MANUAL_GO_TO_POSITION_WAIT: {
            if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED && CARRIAGE_MOTOR.StepsComplete()) {
                PRINTLN("Motor reached position");
                return State::IDLE;
            }
            if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
                PRINTLN("Motor alert during homing");
                print_motor_alerts();
                estop();
                return State::ESTOP_START;
            }
            return State::MANUAL_GO_TO_POSITION_WAIT;
        }
        case State::JOB_CALC_NEXT_POSITION: {
            u16 next_target_index = cycle_last_index + 1;
            for (u16 i = 0; i < NUM_POSITIONS; i++) {
                if (mb.Coil(ENABLED_POSITIONS_OFFSET + next_target_index)) {
                    break;
                }
                next_target_index = (next_target_index + 1) % NUM_POSITIONS;
            }
            if (next_target_index == cycle_last_index) {
                PRINTLN("No enabled positions left");
                return State::IDLE;
            }
            cycle_target_index = next_target_index;

            PRINT("Calculated next position index: ");
            PRINTLN(cycle_target_index);

            return State::JOB_MOVE_NEXT_POSITION;
        }

        case State::JOB_MOVE_NEXT_POSITION: {
            PRINT("Going to position at index ");
            PRINTLN(cycle_target_index);

            const u16 target_hundreths = mb.Hreg(POSITIONS_OFFSET + cycle_target_index);
            const i32 target_steps = target_hundreths * STEPS_PER_HUNDRETH;

            if (target_steps > MOTOR_MAX_STEPS) {
                PRINTLN("Target position too far");
                return State::ERROR_STATE;
            }

            CARRIAGE_MOTOR.Move(target_steps, StepGenerator::MOVE_TARGET_ABSOLUTE);
            return State::JOB_WAIT_POSITION;
        }

        case State::JOB_WAIT_POSITION: {
            if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED && CARRIAGE_MOTOR.StepsComplete()) {
                PRINT("Motor reached position ");
                PRINTLN(cycle_target_index);
                cycle_reached_target_time = millis();
                millis_delay = mb.Hreg(POSITIONS_OFFSET + cycle_target_index) * 100;
                return State::JOB_WAIT_DELAY;
            }
            if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
                PRINTLN("Motor alert during homing");
                print_motor_alerts();
                estop();
                return State::ESTOP_START;
            }
            return State::JOB_WAIT_POSITION;
        }


        case State::JOB_WAIT_DELAY: {
            const u32 time_waited = millis() - cycle_reached_target_time;
            if (time_waited < millis_delay) {
                return State::JOB_WAIT_DELAY;
            }
            return State::JOB_CALC_NEXT_POSITION;
        }




        case State::ESTOP_START: {
            return State::ESTOP;
        }
        case State::ESTOP: {
            if (ESTOP_SAFE && (millis() - last_estop_time) > ESTOP_DEACTIVATE_COOLDOWN_MS) {
                in_estop = false;
                return estop_resume_state(last_state_before_estop);
            }
            return State::ESTOP;
        }

        case State::ERROR_STATE: {
            if (!in_estop) {
                estop();
            }
            return State::ERROR_STATE;
        }
    }
    return State::IDLE;
}

bool latch_handler(const u16 offset) {
    if (mb.Coil(offset)) {
        mb.Coil(offset, false);
        return true;
    }
    return false;
}

void estop() {
    in_estop = true;
    last_estop_time = millis();
    last_state_before_estop = machine_state;
    machine_state = State::ESTOP_START;
    CARRIAGE_MOTOR.MoveStopAbrupt();
}


void ethernet_setup(const bool use_dhcp, const IPAddress &ip, const u16 max_dhcp_attempts) {
    u32 attempt_number = 0;
    while (true) {
        attempt_number++;
        PRINT("Connecting Ethernet. Attempt #");
        PRINTLN(attempt_number);

        const u32 eth_start_time_ms = millis();

        if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
            // As best I can tell from the PHY datasheet and how I would implement this API, the link is only 'LinkOn'
            // if ambient traffic is sensed on the cable and a link can be negotiated. This might not happen when connecting
            // point-to-point if both devices are waiting for traffic to try anything.
            PRINTLN("Ethernet has no link, waiting up to ten seconds");
            while (Ethernet.linkStatus() == LinkOFF && ((millis() - eth_start_time_ms) < 10000)) { delay(2); }
        }

        if (Ethernet.linkStatus() != EthernetLinkStatus::LinkON) {
            PRINTLN("Attempting to continue without Link. Could just be P2P conn");
        } else {
            PRINTLN("Ethernet link is up");
        }
        delay(50);
        Ethernet.setRetransmissionCount(5);
        Ethernet.setRetransmissionTimeout(200);

        u16 dhcp_attempts = max_dhcp_attempts;

        while (use_dhcp && dhcp_attempts > 0) {
            PRINT("Attempting DHCP. ");

            if (!Ethernet.begin(nullptr/*CC automatically gets mac address from flash*/)) {
                dhcp_attempts--;
                PRINT("DHCP failed. ");
                PRINT(dhcp_attempts);
                PRINTLN(" attempts remaining");
            } else {

                if (Ethernet.linkStatus() == EthernetLinkStatus::LinkON) {
                    PRINT("DHCP gives IP: ");
                    const auto new_ip = Ethernet.localIP();
                    PRINT(new_ip[0]);
                    PRINT('.');
                    PRINT(new_ip[1]);
                    PRINT('.');
                    PRINT(new_ip[2]);
                    PRINT('.');
                    PRINTLN(new_ip[3]);

                    return;
                } else {
                    PRINTLN("Unexpected code path: ethernet dhcp success and no LinkON. Proceeding as if connected");
                    return;
                }
            }
            delay(10);
        }
        PRINTLN("Start manual ethernet");
        Ethernet.begin(nullptr, ip);
        if (Ethernet.linkStatus() == EthernetLinkStatus::LinkON) {
            PRINT("Ethernet set up on ");
            PRINT(ip[0]);
            PRINT('.');
            PRINT(ip[1]);
            PRINT('.');
            PRINT(ip[2]);
            PRINT('.');
            PRINTLN(ip[3]);

            return;
        } else {
            PRINTLN("Unexpected code path: ethernet began manually and no LinkON. Proceeding as if connected");
            return;
        }
    }
}


/**
 * Prints active alerts.
 *
 * @pre requires "CARRIAGE_MOTOR" to be defined as a ClearCore motor connector
 */
void print_motor_alerts(){
    // report status of alerts
    PRINTLN("ClearPath Alerts present: ");
    if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledInAlert){
        PRINTLN("    MotionCanceledInAlert "); }
    if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledPositiveLimit){
        PRINTLN("    MotionCanceledPositiveLimit "); }
    if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledNegativeLimit){
        PRINTLN("    MotionCanceledNegativeLimit "); }
    if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledSensorEStop){
        PRINTLN("    MotionCanceledSensorEStop "); }
    if(CARRIAGE_MOTOR.AlertReg().bit.MotionCanceledMotorDisabled){
        PRINTLN("    MotionCanceledMotorDisabled "); }
    if(CARRIAGE_MOTOR.AlertReg().bit.MotorFaulted){
        PRINTLN("    MotorFaulted ");
    }
}



void print_state(const State state_in) {
    switch (state_in) {
        case State::IDLE:
            PRINT("IDLE");
            break;
        case State::START_HOMING:
            PRINT("START_HOMING");
            break;
        case State::WAIT_FOR_HOMING:
            PRINT("WAIT_FOR_HOMING");
            break;
        case State::MANUAL_GO_TO_POSITION:
            PRINT("MANUAL_GO_TO_POSITION");
            break;
        case State::MANUAL_GO_TO_POSITION_WAIT:
            PRINT("MANUAL_GO_TO_POSITION_WAIT");
            break;
        case State::JOB_CALC_NEXT_POSITION:
            PRINT("JOB_CALC_NEXT_POSITION");
            break;
        case State::JOB_MOVE_NEXT_POSITION:
            PRINT("JOB_MOVE_NEXT_POSITION");
            break;
        case State::JOB_WAIT_POSITION:
            PRINT("JOB_WAIT_POSITION");
            break;
        case State::JOB_WAIT_DELAY:
            PRINT("JOB_WAIT_DELAY");
            break;
        case State::ESTOP_START:
            PRINT("ESTOP_START");
            break;
        case State::ESTOP:
            PRINT("ESTOP");
            break;
        case State::RETURN_TO_START:
            PRINT("RETURN_TO_START");
            break;
        case State::WAIT_FOR_RETURN_TO_START:
            PRINT("WAIT_FOR_RETURN_TO_START");
            break;
        case State::ERROR_STATE:
            PRINT("ERROR_STATE");
            break;
    }
}

State estop_resume_state(const State state_in) {
    switch (state_in) {
        case State::IDLE:
            break;
        case State::START_HOMING:
        case State::WAIT_FOR_HOMING:
            return State::IDLE;
        case State::RETURN_TO_START:
        case State::WAIT_FOR_RETURN_TO_START:
            return State::RETURN_TO_START;
        case State::MANUAL_GO_TO_POSITION:
        case State::MANUAL_GO_TO_POSITION_WAIT:
            return State::MANUAL_GO_TO_POSITION;
        case State::JOB_CALC_NEXT_POSITION:
            return State::JOB_CALC_NEXT_POSITION;
        case State::JOB_MOVE_NEXT_POSITION:
        case State::JOB_WAIT_POSITION:
            return State::JOB_MOVE_NEXT_POSITION;
        case State::JOB_WAIT_DELAY:
            return State::JOB_WAIT_DELAY;
        case State::ESTOP_START:
        case State::ESTOP:
        case State::ERROR_STATE:
            return State::ERROR_STATE;
    }
    return State::ERROR_STATE;
}
