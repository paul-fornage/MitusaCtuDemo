#include <Arduino.h>
#include <ClearCore.h>
#include <utility>
#include <RsTypeNames.h>
#include <ModBussy.h>
#include <Ethernet.h>
#include <NvmManager.h>
#include <StepsConversions.h>

#define MB_COIL_BITS 256
#define MB_DISCRETE_BITS 16
#define MB_HREGS 128
#define MB_IREGS 1

#define ABS(x) ((x) < 0 ? -(x) : (x))

#define PRINT(expr) ConnectorUsb.Send(expr)
#define PRINTLN(expr) ConnectorUsb.SendLine(expr)

#define CARRIAGE_MOTOR ConnectorM0

#define ESTOP_CONNECTOR ConnectorA12
#define ESTOP_CON_SAFE_STATE true
#define ESTOP_SAFE (ESTOP_CONNECTOR.State() == ESTOP_CON_SAFE_STATE)

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

const IPAddress IP_ADDRESS(192,168,1,66);
static constexpr bool USE_DHCP = true;

static constexpr u16 NUM_POSITIONS = 16;

// Hregs
static constexpr u16 SELECTED_SUBROUTINE_OFFSET = 8;
static constexpr u16 EXECUTE_SR_LATCH_OFFSET = 9; // Sub-routine input latch. not an EE 'SR latch'
static constexpr u16 SELECTED_POSITION_OFFSET = 10; // position to go to when GO_TO_POSITION_LATCH is latched
static constexpr u16 CURRENT_POSITION_INDEX_OFFSET = 11;
static constexpr u16 SPEED_OFFSET = 12;
static constexpr u16 CURRENT_POSITION_OFFSET = 13;
static constexpr u16 MACHINE_STATE_OFFSET = 14;
static constexpr u16 DELAY_REMAINING_OFFSET = 15;
static constexpr u16 POSITIONS_OFFSET = 16;
static constexpr u16 SUBROUTINE_OFFSET = POSITIONS_OFFSET + NUM_POSITIONS;


// Coils
static constexpr u16 ARM_ENABLE = 8;
static constexpr u16 ARM_RUNNING = 9;
static constexpr u16 START_CYCLE_LATCH_OFFSET = 10;
static constexpr u16 HOME_LATCH_OFFSET = 11;
static constexpr u16 IS_HOMED_OFFSET = 12;
static constexpr u16 RESET_CYCLE_LATCH_OFFSET = 13;
static constexpr u16 GO_TO_POSITION_LATCH_OFFSET = 14;
static constexpr u16 JOB_ACTIVE_OFFSET = 15;
static constexpr u16 STOP_CYCLE_LATCH_OFFSET = 16;
static constexpr u16 JOB_MOVING_OFFSET = 17;
static constexpr u16 JOB_WAITING_OFFSET = 18;
static constexpr u16 SET_ESTOP_OFFSET = 19;
static constexpr u16 IN_ESTOP_OFFSET = 20;
static constexpr u16 ERROR_OFFSET = 21;
static constexpr u16 ENABLED_POSITIONS_OFFSET = 32;

volatile u32 delay_countdown = 0;

bool home_latched = false;
bool start_cycle_latched = false;
bool stop_cycle_latched = false;
bool go_to_position_latched = false;
bool reset_cycle_latched = false;
bool pause_cycle_latched = false;
bool execute_sr_latched = false;

enum class MotorWaitResult {
    DONE,
    NOT_DONE,
    OUT_OF_BOUNDS_POSITIVE,
    OUT_OF_BOUNDS_NEGATIVE,
    ERROR,
};

u16 coils[(MB_COIL_BITS + 15) / 16];
u16 discretes[(MB_DISCRETE_BITS + 15) / 16];
u16 holding[MB_HREGS];
u16 input[MB_IREGS];

enum class State : u16{
    IDLE,
    START_HOMING,
    WAIT_FOR_HOMING,
    RETURN_TO_START,
    WAIT_FOR_RETURN_TO_START,
    MANUAL_GO_TO_POSITION,
    MANUAL_GO_TO_POSITION_WAIT,
    MANUAL_EXECUTE_SR,
    MANUAL_EXECUTE_SR_WAIT,
    JOB_CALC_NEXT_POSITION,
    JOB_MOVE_NEXT_POSITION = 16,
    JOB_WAIT_POSITION = 17,
    JOB_WAIT_DELAY = 18,
    JOB_STOP_DELAY = 19,
    ESTOP_START,
    ESTOP,
    ERROR_STATE, // unrecoverable by design
};


static constexpr i32 MOTOR_MAX_VEL_SPS = 3000;
static constexpr i32 MOTOR_MAX_VEL_HPM = sps_to_hpm(MOTOR_MAX_VEL_SPS);

static constexpr i32 MOTOR_HOMING_VEL_HPM = 1000;
static constexpr i32 MOTOR_HOMING_VEL_SPS = hpm_to_sps(MOTOR_HOMING_VEL_HPM);

// in steps per second squared
static constexpr i32 MOTOR_MAX_ACC = 100000;

static constexpr i32 START_POS_HUNDRETHS = 100;
static constexpr i32 START_POS_STEPS = hundreths_to_steps(START_POS_HUNDRETHS);

static constexpr i32 MOTOR_MAX_POS_HUNDRETHS = 10000;
static constexpr i32 MOTOR_MAX_POS_STEPS = hundreths_to_steps(MOTOR_MAX_POS_HUNDRETHS);

bool is_homed = false;
bool in_estop = false;
bool is_cycle_reset = true; // should the next position calculation return the first enabled position?
bool hmi_commands_estop = false;
u32 last_estop_time = 0;
auto last_state_before_estop = State::IDLE;
static constexpr u32 ESTOP_DEACTIVATE_COOLDOWN_MS = 1000;
u16 manual_target_pos_index = 65535; // 65535 always out of bounds
u16 cycle_last_index = 0; // gets set to `cycle_target_index` after reaching target and waiting delay
u16 cycle_target_index = 0; // gets set to the next valid target position at start of cycle
u32 cycle_reached_target_time = 0; // `millis()` at the instant the carriage reached the target pos
u32 millis_delay = 0; // `millis()` at the instant the carriage reached the target pos
u16 manual_go_to_pos_target_hundreths = 0;
u32 last_estop_dbg_millis = 0;
i32 job_target_steps = 0;
// Jog speed in hundreths of an inch per minute
u16 speed = 0;

ModBussy mb(502, coils, discretes, holding, input);

void print_motor_alerts();
void ethernet_setup(bool use_dhcp, const IPAddress &ip, u16 max_dhcp_attempts);
State state_machine_iter(State state_in);
bool estop_conditions_met();
void estop();
const char* state_name(State state_in);
State estop_resume_state(State state_in);
bool latch_handler(u16 offset);
void ConfigurePeriodicInterrupt(uint32_t frequencyHz);
MotorWaitResult wait_for_motor_motion(MotorDriver &Motor);
extern "C" void TCC2_0_Handler(void) __attribute__((
            alias("PeriodicInterrupt")));

volatile auto machine_state = State::IDLE;
auto last_state = State::IDLE;


int main() {
    ESTOP_CONNECTOR.Mode(Connector::INPUT_DIGITAL);
    ConnectorUsb.PortOpen();
    ConnectorUsb.Speed(115200);
    const u32 startTime = millis();
    while (!ConnectorUsb.PortIsOpen() && millis() - startTime < 5000) {  }

    if (ConnectorUsb.PortIsOpen()) {
        PRINTLN("USB connected");
    }

    ConfigurePeriodicInterrupt(1000);

    ethernet_setup(USE_DHCP, IP_ADDRESS, 1);
    mb.begin();
    mb.Coil(IS_HOMED_OFFSET, is_homed);
    mb.Coil(IN_ESTOP_OFFSET, in_estop);
    mb.Coil(START_CYCLE_LATCH_OFFSET, false);
    mb.Coil(HOME_LATCH_OFFSET, false);
    mb.Coil(GO_TO_POSITION_LATCH_OFFSET, false);
    mb.Coil(JOB_ACTIVE_OFFSET, false);
    mb.Coil(STOP_CYCLE_LATCH_OFFSET, false);
    mb.Coil(RESET_CYCLE_LATCH_OFFSET, false);
    mb.Coil(ERROR_OFFSET, false);
    mb.Coil(JOB_MOVING_OFFSET, false);
    mb.Coil(JOB_WAITING_OFFSET, false);
    mb.Coil(SET_ESTOP_OFFSET, false);

    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    CARRIAGE_MOTOR.AccelMax(MOTOR_MAX_ACC);
    CARRIAGE_MOTOR.VelMax(MOTOR_MAX_VEL_SPS);


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
        mb.Coil(JOB_MOVING_OFFSET, machine_state == State::JOB_WAIT_POSITION);
        mb.Coil(JOB_WAITING_OFFSET, machine_state == State::JOB_WAIT_DELAY);

        mb.Hreg(MACHINE_STATE_OFFSET) = static_cast<u16>(machine_state);
        mb.Hreg(CURRENT_POSITION_INDEX_OFFSET) = cycle_target_index;
        mb.Hreg(DELAY_REMAINING_OFFSET) = static_cast<u16>(delay_countdown/100);
        if (is_homed) {
            mb.Hreg(CURRENT_POSITION_OFFSET) = steps_to_hundreths(CARRIAGE_MOTOR.PositionRefCommanded());
        } else {
            mb.Hreg(CURRENT_POSITION_OFFSET) = 0;
        }

        hmi_commands_estop = mb.Coil(SET_ESTOP_OFFSET);


        const u16 temp_speed = mb.Hreg(SPEED_OFFSET);
        if (temp_speed > MOTOR_MAX_VEL_HPM) {
            speed = MOTOR_MAX_VEL_HPM;
            mb.Hreg(SPEED_OFFSET) = MOTOR_MAX_VEL_HPM;
        } else {
            speed = temp_speed;
        }

        home_latched = latch_handler(HOME_LATCH_OFFSET);
        start_cycle_latched = latch_handler(START_CYCLE_LATCH_OFFSET);
        stop_cycle_latched = latch_handler(STOP_CYCLE_LATCH_OFFSET);
        go_to_position_latched = latch_handler(GO_TO_POSITION_LATCH_OFFSET);
        reset_cycle_latched = latch_handler(RESET_CYCLE_LATCH_OFFSET);

        mb.Hreg(CURRENT_POSITION_INDEX_OFFSET) = cycle_target_index;

        if (estop_conditions_met() && !in_estop) {
            PRINTLN("ESTOP triggered");
            PRINT("\tESTOP_SAFE: ");
            PRINTLN(ESTOP_SAFE?"TRUE":"FALSE");
            PRINT("\tEthernet.linkStatus() == EthernetLinkStatus::LinkOFF: ");
            PRINTLN(Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF?"TRUE":"FALSE");
            PRINT("\tmb.hasClient(): ");
            PRINTLN(mb.hasClient()?"TRUE":"FALSE");
            PRINT("\thmi_commands_estop: ");
            PRINTLN(hmi_commands_estop?"TRUE":"FALSE");
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
            PRINT(state_name(last_state));
            PRINT(" to ");
            PRINTLN(state_name(machine_state));
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
                    PRINTLN("Can't go to manual pos, not homed");
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
            if(reset_cycle_latched) {
                is_cycle_reset = true;
                PRINTLN("Cycle reset");
                return State::IDLE;
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
            CARRIAGE_MOTOR.MoveVelocity(-MOTOR_HOMING_VEL_SPS);
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
            if (CARRIAGE_MOTOR.PositionRefCommanded() < -MOTOR_MAX_POS_STEPS) {
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
            switch (wait_for_motor_motion(CARRIAGE_MOTOR)) {
                case MotorWaitResult::DONE: {
                    PRINTLN("Motor returned to start");
                    return State::IDLE;
                }
                case MotorWaitResult::ERROR: {
                    PRINTLN("Motor error during return to start");
                    estop();
                    return State::ERROR_STATE;
                }
                case MotorWaitResult::NOT_DONE: {
                    return State::WAIT_FOR_RETURN_TO_START;
                }
                case MotorWaitResult::OUT_OF_BOUNDS_NEGATIVE: {
                    if (START_POS_STEPS > CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::WAIT_FOR_RETURN_TO_START;
                    } else {
                        // this can sometimes trigger when going to start pos of 0,
                        // but stopping there is the desired behavior anyway
                        CARRIAGE_MOTOR.MoveStopAbrupt();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
                case MotorWaitResult::OUT_OF_BOUNDS_POSITIVE: {
                    if (START_POS_STEPS < CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::WAIT_FOR_RETURN_TO_START;
                    } else {
                        CARRIAGE_MOTOR.MoveStopAbrupt();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
            }
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
            const i32 target_steps = hundreths_to_steps(target_hundreths);
            PRINT("position in steps: ");
            PRINTLN(target_steps);
            if (target_steps > MOTOR_MAX_POS_STEPS || target_steps < 0) {
                PRINTLN("Target position too far for manual move");
                return State::ERROR_STATE;
            }
            CARRIAGE_MOTOR.Move(target_steps, StepGenerator::MOVE_TARGET_ABSOLUTE);
            return State::MANUAL_GO_TO_POSITION_WAIT;
        }
        case State::MANUAL_GO_TO_POSITION_WAIT: {
            switch (wait_for_motor_motion(CARRIAGE_MOTOR)) {
                case MotorWaitResult::DONE: {
                    PRINTLN("Motor got to manual position");
                    return State::IDLE;
                }
                case MotorWaitResult::ERROR: {
                    PRINTLN("Motor error during return to start");
                    estop();
                    return State::ERROR_STATE;
                }
                case MotorWaitResult::NOT_DONE: {
                    return State::MANUAL_GO_TO_POSITION_WAIT;
                }
                case MotorWaitResult::OUT_OF_BOUNDS_NEGATIVE: {
                    if (hundreths_to_steps(manual_go_to_pos_target_hundreths) > CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::MANUAL_GO_TO_POSITION_WAIT;
                    } else {
                        // this can sometimes trigger when going to start pos of 0,
                        // but stopping there is the desired behavior anyway
                        CARRIAGE_MOTOR.MoveStopAbrupt();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
                case MotorWaitResult::OUT_OF_BOUNDS_POSITIVE: {
                    if (hundreths_to_steps(manual_go_to_pos_target_hundreths) < CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::MANUAL_GO_TO_POSITION_WAIT;
                    } else {
                        CARRIAGE_MOTOR.MoveStopAbrupt();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
            }
        }
        case State::MANUAL_EXECUTE_SR: {
            return State::MANUAL_EXECUTE_SR_WAIT;
        }
        case State::MANUAL_EXECUTE_SR_WAIT: {
            return State::IDLE;
        }
        case State::JOB_CALC_NEXT_POSITION: {
            if (stop_cycle_latched) {
                PRINTLN("Stopping cycle");
                return State::IDLE;
            }

            if (is_cycle_reset) {
                for (u16 i = 0; i < NUM_POSITIONS; i++) {
                    if (mb.Coil(ENABLED_POSITIONS_OFFSET + i)) {
                        cycle_target_index = i;
                        is_cycle_reset = false;
                        PRINT("Reset cycle mode, found first en pos: ");
                        PRINTLN(cycle_target_index);
                        return State::JOB_MOVE_NEXT_POSITION;
                    }
                }
                PRINTLN("No enabled positions left");
                return State::IDLE;
            }

            u16 next_target_index = cycle_last_index;

            for (u16 i = 0; i < NUM_POSITIONS; i++) {
                next_target_index = (next_target_index + 1) % NUM_POSITIONS;

                if (mb.Coil(ENABLED_POSITIONS_OFFSET + next_target_index)) {
                    break;
                }
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

            if (stop_cycle_latched) {
                PRINTLN("Stopping cycle");
                return State::IDLE;
            }

            PRINT("Going to position at index ");
            PRINTLN(cycle_target_index);

            const u16 target_hundreths = mb.Hreg(POSITIONS_OFFSET + cycle_target_index);
            const i32 target_steps = hundreths_to_steps(target_hundreths);

            if (target_steps > MOTOR_MAX_POS_STEPS) {
                PRINTLN("Target position too far");
                return State::ERROR_STATE;
            }

            CARRIAGE_MOTOR.Move(target_steps, StepGenerator::MOVE_TARGET_ABSOLUTE);
            return State::JOB_WAIT_POSITION;
        }

        case State::JOB_WAIT_POSITION: {
            if (stop_cycle_latched) {
                CARRIAGE_MOTOR.MoveStopAbrupt();
                PRINTLN("Stopping cycle");
                return State::IDLE;
            }
            switch (wait_for_motor_motion(CARRIAGE_MOTOR)) {
                case MotorWaitResult::DONE: {
                    PRINT("Motor reached position ");
                    PRINTLN(cycle_target_index);
                    delay_countdown = 1000;
                    return State::JOB_WAIT_DELAY;
                }
                case MotorWaitResult::ERROR: {
                    PRINTLN("Motor error during cycle");
                    print_motor_alerts();
                    estop();
                    return State::ERROR_STATE;
                }
                case MotorWaitResult::NOT_DONE: {
                    return State::JOB_WAIT_POSITION;
                }
                case MotorWaitResult::OUT_OF_BOUNDS_NEGATIVE: {
                    if (job_target_steps > CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::JOB_WAIT_POSITION;
                    } else {
                        // this can sometimes trigger when going to start pos of 0,
                        // but stopping there is the desired behavior anyway
                        CARRIAGE_MOTOR.MoveStopAbrupt();
                        PRINTLN("Motor OUT_OF_BOUNDS_NEGATIVE on job, but target is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
                case MotorWaitResult::OUT_OF_BOUNDS_POSITIVE: {
                    if (job_target_steps < CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::JOB_WAIT_POSITION;
                    } else {
                        CARRIAGE_MOTOR.MoveStopAbrupt();
                        PRINTLN("Motor OUT_OF_BOUNDS_POSITIVE on job, but target is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
            }
            return State::JOB_WAIT_POSITION;
        }
        case State::JOB_WAIT_DELAY: {
            if (stop_cycle_latched) {
                PRINTLN("Stopping cycle");
                return State::IDLE;
            }
            if (delay_countdown == 0) {
                cycle_last_index = cycle_target_index;
                return State::JOB_CALC_NEXT_POSITION;
            }
            return State::JOB_WAIT_DELAY;
        }

        case State::JOB_STOP_DELAY: {
            return State::IDLE;
        }


        case State::ESTOP_START: {
            return State::ESTOP;
        }
        case State::ESTOP: {const bool safe_to_deactivate = !estop_conditions_met()
                && (millis() - last_estop_time) > ESTOP_DEACTIVATE_COOLDOWN_MS;

            if (safe_to_deactivate) {
                in_estop = false;
                const State resuming_state = estop_resume_state(last_state_before_estop);
                return resuming_state;
            } else {
                if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
                    if (millis() - last_estop_dbg_millis > 1000) {
                        PRINTLN("Ethernet is down in estop, trying to connect");
                        last_estop_dbg_millis = millis();
                    }
                    ethernet_setup(USE_DHCP, IPAddress(IP_ADDRESS), 1);
                    mb.begin();
                } else if (!mb.hasClient()) {
                    if (millis() - last_estop_dbg_millis > 1000) {
                        PRINTLN("No mb client in estop, but eth is good. trying to connect");
                        last_estop_dbg_millis = millis();
                    }
                    mb.task();
                }
            }
            return State::ESTOP;
        }

        case State::ERROR_STATE: {
            if (!in_estop) {
                in_estop = true;
                CARRIAGE_MOTOR.MoveStopAbrupt();
            }
            return State::ERROR_STATE;
        }
    }
    return State::IDLE;
}

/**
 * Interrupt handler gets automatically called every ms
 */
extern "C" void PeriodicInterrupt(void) {
    if (machine_state == State::JOB_WAIT_DELAY) {
        if (delay_countdown > 0) {
            delay_countdown--;
        }
    }
    // Acknowledge the interrupt to clear the flag and wait for the next interrupt.
    TCC2->INTFLAG.reg = TCC_INTFLAG_MASK; // This is critical
}

bool latch_handler(const u16 offset) {
    if (mb.Coil(offset)) {
        mb.Coil(offset, false);
        return true;
    }
    return false;
}

bool estop_conditions_met() {
    return !mb.hasClient()
        || !ESTOP_SAFE
        || Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF
        || hmi_commands_estop;
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



MotorWaitResult wait_for_motor_motion(MotorDriver &Motor) {
    if (Motor.HlfbState() == MotorDriver::HLFB_ASSERTED && Motor.StepsComplete()) {
        return MotorWaitResult::DONE;
    }
    if (Motor.StatusReg().bit.AlertsPresent) {
        is_homed = false;
        return MotorWaitResult::ERROR;
    }
    if (CARRIAGE_MOTOR.PositionRefCommanded() >= MOTOR_MAX_POS_STEPS) {
        PRINTLN("Max position reached");
        return MotorWaitResult::OUT_OF_BOUNDS_POSITIVE;
    }
    if (CARRIAGE_MOTOR.PositionRefCommanded() <= 0) {
        PRINTLN("Min position reached");
        return MotorWaitResult::OUT_OF_BOUNDS_NEGATIVE;
    }
    return MotorWaitResult::NOT_DONE;
}

const char* state_name(const State state_in) {
    switch (state_in) {
        case State::IDLE: return "IDLE";
        case State::START_HOMING: return "START_HOMING";
        case State::WAIT_FOR_HOMING: return "WAIT_FOR_HOMING";
        case State::RETURN_TO_START: return "RETURN_TO_START";
        case State::WAIT_FOR_RETURN_TO_START: return "WAIT_FOR_RETURN_TO_START";
        case State::MANUAL_GO_TO_POSITION: return "MANUAL_GO_TO_POSITION";
        case State::MANUAL_GO_TO_POSITION_WAIT: return "MANUAL_GO_TO_POSITION_WAIT";
        case State::MANUAL_EXECUTE_SR: return "MANUAL_EXECUTE_SR";
        case State::MANUAL_EXECUTE_SR_WAIT: return "MANUAL_EXECUTE_SR_WAIT";
        case State::JOB_CALC_NEXT_POSITION: return "JOB_CALC_NEXT_POSITION";
        case State::JOB_MOVE_NEXT_POSITION: return "JOB_MOVE_NEXT_POSITION";
        case State::JOB_WAIT_POSITION: return "JOB_WAIT_POSITION";
        case State::JOB_WAIT_DELAY: return "JOB_WAIT_DELAY";
        case State::JOB_STOP_DELAY: return "JOB_STOP_DELAY";
        case State::ESTOP_START: return "ESTOP_START";
        case State::ESTOP: return "ESTOP";
        case State::ERROR_STATE: return "ERROR_STATE";
        default: return "UNKNOWN_STATE";
    }
}


State estop_resume_state(const State state_in) {
    switch (state_in) {
        case State::IDLE:
        case State::START_HOMING:
        case State::WAIT_FOR_HOMING:
            return State::IDLE;
        case State::RETURN_TO_START:
        case State::WAIT_FOR_RETURN_TO_START:
            return State::RETURN_TO_START;
        case State::MANUAL_GO_TO_POSITION:
        case State::MANUAL_GO_TO_POSITION_WAIT:
            return State::MANUAL_GO_TO_POSITION;
        case State::MANUAL_EXECUTE_SR:
        case State::MANUAL_EXECUTE_SR_WAIT:
            return State::IDLE;
        case State::JOB_CALC_NEXT_POSITION:
            return State::JOB_CALC_NEXT_POSITION;
        case State::JOB_MOVE_NEXT_POSITION:
        case State::JOB_WAIT_POSITION:
            return State::JOB_MOVE_NEXT_POSITION;
        case State::JOB_WAIT_DELAY:
            return State::JOB_WAIT_DELAY;
        case State::JOB_STOP_DELAY:
            return State::IDLE;
        case State::ESTOP_START:
        case State::ESTOP:
        case State::ERROR_STATE:
            return State::ERROR_STATE;
    }
    return State::ERROR_STATE;
}


/**
 * Only used to configure the ms interrupt.
 * Should only be called once other IO is configured.
 * If you are confused, don't worry, this won't make sense without knowing the
 * internal architecture of the hardware. It was stolen from
 * https://teknic-inc.github.io/ClearCore-library/_periodic_interrupt_8cpp-example.html
 *
 * @param frequencyHz
 */
void ConfigurePeriodicInterrupt(const uint32_t frequencyHz) {
    // Enable the TCC2 peripheral.
    // TCC2 and TCC3 share their clock configuration and they
    // are already configured to be clocked at 120 MHz from GCLK0.
    CLOCK_ENABLE(APBCMASK, TCC2_);

    // Disable TCC2.
    TCC2->CTRLA.bit.ENABLE = 0;
    SYNCBUSY_WAIT(TCC2, TCC_SYNCBUSY_ENABLE);

    // Reset the TCC module so we know we are starting from a clean state.
    TCC2->CTRLA.bit.SWRST = 1;
    while (TCC2->CTRLA.bit.SWRST) {}

    // If the frequency requested is zero, disable the interrupt and bail out.
    if (!frequencyHz) {
        NVIC_DisableIRQ(TCC2_0_IRQn);
        return;
    }

    // Determine the clock prescaler and period value needed to achieve the
    // requested frequency.
    uint32_t period = (CPU_CLK + frequencyHz / 2) / frequencyHz;
    uint8_t prescale;
    // Make sure period is >= 1.
    period = max(period, 1U);

    // Prescale values 0-4 map to prescale divisors of 1-16,
    // dividing by 2 each increment.
    for (prescale = TCC_CTRLA_PRESCALER_DIV1_Val;
            prescale < TCC_CTRLA_PRESCALER_DIV16_Val && (period - 1) > UINT16_MAX;
            prescale++) {
        period = period >> 1;
    }
    // Prescale values 5-7 map to prescale divisors of 64-1024,
    // dividing by 4 each increment.
    for (; prescale < TCC_CTRLA_PRESCALER_DIV1024_Val && (period - 1) > UINT16_MAX;
            prescale++) {
        period = period >> 2;
    }
    // If we have maxed out the prescaler and the period is still too big,
    // use the maximum period. This results in a ~1.788 Hz interrupt.
    if (period > UINT16_MAX) {
        TCC2->PER.reg = UINT16_MAX;
    }
    else {
        TCC2->PER.reg = period - 1;
    }
    TCC2->CTRLA.bit.PRESCALER = prescale;

    // Interrupt every period on counter overflow.
    TCC2->INTENSET.bit.OVF = 1;
    // Enable TCC2.
    TCC2->CTRLA.bit.ENABLE = 1;

    // Set the interrupt priority and enable it.
    NVIC_SetPriority(TCC2_0_IRQn, PERIODIC_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(TCC2_0_IRQn);
}