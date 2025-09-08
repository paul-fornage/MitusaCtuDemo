#include <Arduino.h>
#include <ClearCore.h>
#include <utility>
#include <RsTypeNames.h>
#include <ModBussy.h>
#include <Ethernet.h>
#include <NvmManager.h>
#include <StepsConversions.h>
#include <DummyMotor.h>

#define MB_COIL_BITS 256
#define MB_DISCRETE_BITS 16
#define MB_HREGS 128
#define MB_IREGS 1

#define ABS(x) ((x) < 0 ? -(x) : (x))

#define PRINT(expr) ConnectorUsb.Send(expr)
#define PRINTLN(expr) ConnectorUsb.SendLine(expr)

//DummyMotor FakeCarriageMotor("Carriage");

 #define CARRIAGE_MOTOR ConnectorM0
//#define CARRIAGE_MOTOR FakeCarriageMotor

#define ESTOP_CONNECTOR ConnectorA12
#define ESTOP_CON_SAFE_STATE true
#define ESTOP_SAFE (ESTOP_CONNECTOR.State() == ESTOP_CON_SAFE_STATE)

#define ESTOP_OUT_CONNECTOR ConnectorIO5
#define INVERT_ESTOP_OUT true

/// Interrupt priority for the periodic interrupt. 0 is highest priority, 7 is lowest.
#define PERIODIC_INTERRUPT_PRIORITY 5

const IPAddress IP_ADDRESS(192,168,1,68);
static constexpr bool USE_DHCP = false;

static constexpr u16 NUM_POSITIONS = 16;

// Hregs
static constexpr u16 CURRENT_SUBROUTINE_OFFSET = 8; // sr index read by arm
static constexpr u16 SELECTED_STATION_OFFSET = 10; // position to go to when GO_TO_POSITION_LATCH is latched
static constexpr u16 CURRENT_POSITION_INDEX_OFFSET = 11; // cycle target index
static constexpr u16 SPEED_OFFSET = 12; // speed in hundreths of an inch / minute
static constexpr u16 CURRENT_POSITION_OFFSET = 13; // current position in hundreths of an inch
static constexpr u16 MACHINE_STATE_OFFSET = 14; // machine_state
static constexpr u16 POSITIONS_OFFSET = 16; // offset for the first position in hundreths
static constexpr u16 SUBROUTINE_OFFSET = POSITIONS_OFFSET + NUM_POSITIONS; // offset for the first SR

// Coils
static constexpr u16 PROGRAM_SELECT_OFFSET = 4;
static constexpr u16 MANUAL_SR_ACTIVE_OFFSET = 5;
static constexpr u16 MANUAL_SR_CANCEL_LATCH_OFFSET = 6;
static constexpr u16 EXECUTE_SR_LATCH_OFFSET = 7; // Sub-routine input latch. not an EE 'SR latch'
static constexpr u16 ARM_ENABLE_OFFSET = 8;
static constexpr u16 ARM_RUNNING_OFFSET = 9;
static constexpr u16 START_CYCLE_LATCH_OFFSET = 10;
static constexpr u16 HOME_LATCH_OFFSET = 11;
static constexpr u16 IS_HOMED_OFFSET = 12;
static constexpr u16 RESET_CYCLE_LATCH_OFFSET = 13;
static constexpr u16 GO_TO_POSITION_LATCH_OFFSET = 14;
static constexpr u16 JOB_ACTIVE_OFFSET = 15;
static constexpr u16 STOP_CYCLE_LATCH_OFFSET = 16;
static constexpr u16 JOB_MOVING_OFFSET = 17;
static constexpr u16 JOB_SR_ACTIVE_OFFSET = 18;
static constexpr u16 SET_ESTOP_OFFSET = 19;
static constexpr u16 IN_ESTOP_OFFSET = 20;
static constexpr u16 ERROR_OFFSET = 21;
static constexpr u16 ENABLED_POSITIONS_OFFSET = 32;

bool home_latched = false;
bool start_cycle_latched = false;
bool stop_cycle_latched = false;
bool go_to_position_latched = false;
bool reset_cycle_latched = false;
bool execute_sr_latched = false;
bool manual_sr_cancel_latched = false;

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
    MANUAL_EXECUTE_SR_START_SETTLE,
    MANUAL_EXECUTE_SR_WAIT_START,
    MANUAL_EXECUTE_SR_WAIT_FINISH,
    JOB_CALC_NEXT_POSITION,
    JOB_MOVE_NEXT_POSITION = 16,
    JOB_WAIT_POSITION = 17,
    JOB_START_SR = 18,
    JOB_START_SR_SETTLE,
    JOB_WAIT_SR_START,
    JOB_WAIT_SR_FINISH,
    JOB_WAIT_SR_COOLDOWN,
    STOP_SR_DELAY,
    ESTOP_START,
    ESTOP,
    ERROR_STATE, // unrecoverable by design
};


static constexpr i32 MOTOR_MAX_VEL_HPM = 50000;
static constexpr i32 MOTOR_MAX_VEL_SPS = hpm_to_sps(MOTOR_MAX_VEL_HPM);

static constexpr i32 MOTOR_HOMING_VEL_HPM = 5000;
static constexpr i32 MOTOR_HOMING_VEL_SPS = hpm_to_sps(MOTOR_HOMING_VEL_HPM);

static constexpr i32 MOTOR_MAX_ACC = 10000; // in steps per second squared
static constexpr i32 MOTOR_ESTOP_MAX_ACC = 100000; // in steps per second squared

static constexpr i32 START_POS_HUNDRETHS = 100;
static constexpr i32 START_POS_STEPS = hundreths_to_steps(START_POS_HUNDRETHS);

static constexpr i32 MOTOR_MAX_POS_HUNDRETHS = 24000;  // 4.37 inches on slide, 24000 inches on ctu
static constexpr i32 MOTOR_MAX_POS_STEPS = hundreths_to_steps(MOTOR_MAX_POS_HUNDRETHS);

static constexpr i32 DEFAULT_MOTOR_VEL_HPM = 10000;

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
u16 manual_go_to_pos_target_hundreths = 0;
u32 last_estop_dbg_millis = 0;
i32 job_target_steps = 0;
// Jog speed in hundreths of an inch per minute
u16 speed = 0;
u16 manual_sr_index = 0;
volatile u32 stop_sr_delay = 0;
volatile u32 sr_start_timeout = 0;
volatile u32 sr_finish_timeout = 0;
volatile u32 sr_cooldown_timeout = 0;
volatile u32 start_settle_timeout = 0;

ModBussy mb(502, coils, discretes, holding, input);

void print_motor_alerts(MotorDriver &Motor);
void ethernet_setup(bool use_dhcp, const IPAddress &ip, u16 max_dhcp_attempts);
State state_machine_iter(State state_in);
bool estop_conditions_met();
void estop();
const char* state_name(State state_in);
State estop_resume_state(State state_in);
bool latch_handler(u16 offset);
void ConfigurePeriodicInterrupt(uint32_t frequencyHz);
MotorWaitResult wait_for_motor_motion(MotorDriver &Motor);
MotorWaitResult wait_for_motor_motion(DummyMotor &Motor);
void set_estop_out(bool new_state);
void print_motor_alerts(DummyMotor &Motor);
extern "C" void TCC2_0_Handler(void) __attribute__((
            alias("PeriodicInterrupt")));

volatile auto machine_state = State::IDLE;
auto last_state = State::IDLE;


int main() {
    ESTOP_CONNECTOR.Mode(Connector::INPUT_DIGITAL);
    ESTOP_OUT_CONNECTOR.Mode(Connector::OUTPUT_DIGITAL);
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
    mb.Coil(JOB_SR_ACTIVE_OFFSET, false);
    mb.Coil(SET_ESTOP_OFFSET, false);
    mb.Coil(ARM_ENABLE_OFFSET, false);
    mb.Coil(EXECUTE_SR_LATCH_OFFSET, false);
    mb.Coil(MANUAL_SR_ACTIVE_OFFSET, false);
    mb.Coil(MANUAL_SR_CANCEL_LATCH_OFFSET, false);
    mb.Coil(PROGRAM_SELECT_OFFSET, false);

    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);

    CARRIAGE_MOTOR.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    CARRIAGE_MOTOR.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    CARRIAGE_MOTOR.AccelMax(MOTOR_MAX_ACC);
    CARRIAGE_MOTOR.EStopDecelMax(MOTOR_ESTOP_MAX_ACC);
    CARRIAGE_MOTOR.VelMax(MOTOR_MAX_VEL_SPS);

    mb.Hreg(SPEED_OFFSET) = DEFAULT_MOTOR_VEL_HPM;

    while (true) {
        mb.task();
        mb.Coil(IS_HOMED_OFFSET, is_homed);
        mb.Coil(IN_ESTOP_OFFSET, in_estop);
        mb.Coil(ERROR_OFFSET, machine_state == State::ERROR_STATE);
        const bool job_sr_active = (machine_state == State::JOB_START_SR
                                    || machine_state == State::JOB_START_SR_SETTLE
                                    || machine_state == State::JOB_WAIT_SR_START
                                    || machine_state == State::JOB_WAIT_SR_FINISH
                                    || machine_state == State::JOB_WAIT_SR_COOLDOWN);
        const bool job_moving = (machine_state == State::JOB_CALC_NEXT_POSITION
                                || machine_state == State::JOB_MOVE_NEXT_POSITION
                                || machine_state == State::JOB_WAIT_POSITION);
        const bool manual_sr_active = (machine_state == State::MANUAL_EXECUTE_SR
                                    || machine_state == State::MANUAL_EXECUTE_SR_START_SETTLE
                                    || machine_state == State::MANUAL_EXECUTE_SR_WAIT_START
                                    || machine_state == State::MANUAL_EXECUTE_SR_WAIT_FINISH);
        mb.Coil(JOB_ACTIVE_OFFSET,
                job_sr_active || job_moving);
        mb.Coil(JOB_MOVING_OFFSET, job_moving);
        mb.Coil(JOB_SR_ACTIVE_OFFSET, job_sr_active);
        mb.Coil(MANUAL_SR_ACTIVE_OFFSET, machine_state == State::MANUAL_EXECUTE_SR
            || machine_state == State::MANUAL_EXECUTE_SR_WAIT_START
            || machine_state == State::MANUAL_EXECUTE_SR_WAIT_FINISH);

        mb.Hreg(MACHINE_STATE_OFFSET) = static_cast<u16>(machine_state);
        mb.Hreg(CURRENT_POSITION_INDEX_OFFSET) = cycle_target_index;
        if (is_homed) {
            mb.Hreg(CURRENT_POSITION_OFFSET) = steps_to_hundreths(CARRIAGE_MOTOR.PositionRefCommanded());
        } else {
            mb.Hreg(CURRENT_POSITION_OFFSET) = 0;
        }

        hmi_commands_estop = mb.Coil(SET_ESTOP_OFFSET);

        set_estop_out(in_estop);

        const u16 temp_speed = mb.Hreg(SPEED_OFFSET);
        if(speed != temp_speed) {
            if (temp_speed > MOTOR_MAX_VEL_HPM) {
                PRINT("New speed out of bounds, capping at ");
                PRINT(MOTOR_MAX_VEL_HPM);
                PRINTLN(" hundreths per minute");
                speed = MOTOR_MAX_VEL_HPM;
                mb.Hreg(SPEED_OFFSET) = MOTOR_MAX_VEL_HPM;
            } else {
                PRINT("New speed: ");
                PRINT(temp_speed);
                PRINTLN(" hundreths per minute");
                speed = temp_speed;
                CARRIAGE_MOTOR.VelMax(speed);
            }
        }


        home_latched = latch_handler(HOME_LATCH_OFFSET);
        start_cycle_latched = latch_handler(START_CYCLE_LATCH_OFFSET);
        stop_cycle_latched = latch_handler(STOP_CYCLE_LATCH_OFFSET);
        go_to_position_latched = latch_handler(GO_TO_POSITION_LATCH_OFFSET);
        reset_cycle_latched = latch_handler(RESET_CYCLE_LATCH_OFFSET);
        execute_sr_latched = latch_handler(EXECUTE_SR_LATCH_OFFSET);
        manual_sr_cancel_latched = latch_handler(MANUAL_SR_CANCEL_LATCH_OFFSET);


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

        if (!job_sr_active &&
            !manual_sr_active &&
            (mb.Coil(PROGRAM_SELECT_OFFSET) || mb.Coil(ARM_ENABLE_OFFSET))
        ) {
            PRINTLN("Program select or arm enable true while not in SR");
            estop();
            mb.Coil(PROGRAM_SELECT_OFFSET, false);
            mb.Coil(ARM_ENABLE_OFFSET, false);
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
            if (execute_sr_latched) {
                if (is_homed) {
                    return State::MANUAL_EXECUTE_SR;
                } else {
                    PRINTLN("Can't execute SR, not homed");
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
            CARRIAGE_MOTOR.MoveVelocity(-MOTOR_HOMING_VEL_SPS);
            delay(50);
            return State::WAIT_FOR_HOMING;
        }
        case State::WAIT_FOR_HOMING: {
            if (CARRIAGE_MOTOR.HlfbState() == MotorDriver::HLFB_ASSERTED) {
                PRINTLN("Motor homed");
                CARRIAGE_MOTOR.MoveStopDecel();
                CARRIAGE_MOTOR.PositionRefSet(0);
                is_homed = true;
                return State::RETURN_TO_START;
            }
            if (CARRIAGE_MOTOR.StatusReg().bit.AlertsPresent) {
                PRINTLN("Motor alert during homing");
                print_motor_alerts(CARRIAGE_MOTOR);
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
                    print_motor_alerts(CARRIAGE_MOTOR);
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
                        CARRIAGE_MOTOR.MoveStopDecel();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
                case MotorWaitResult::OUT_OF_BOUNDS_POSITIVE: {
                    if (START_POS_STEPS < CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::WAIT_FOR_RETURN_TO_START;
                    } else {
                        CARRIAGE_MOTOR.MoveStopDecel();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
            }
        }
        case State::MANUAL_GO_TO_POSITION: {
            manual_target_pos_index = mb.Hreg(SELECTED_STATION_OFFSET);
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
                    PRINTLN("Motor error during got to manual position");
                    print_motor_alerts(CARRIAGE_MOTOR);
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
                        CARRIAGE_MOTOR.MoveStopDecel();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
                case MotorWaitResult::OUT_OF_BOUNDS_POSITIVE: {
                    if (hundreths_to_steps(manual_go_to_pos_target_hundreths) < CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::MANUAL_GO_TO_POSITION_WAIT;
                    } else {
                        CARRIAGE_MOTOR.MoveStopDecel();
                        PRINTLN("Motor returned to start, but start is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
            }
        }
        case State::MANUAL_EXECUTE_SR: {
            mb.Coil(PROGRAM_SELECT_OFFSET, true);
            const u16 station_idx = mb.Hreg(SELECTED_STATION_OFFSET);
            PRINT("Executing manual SR from station #");
            PRINTLN(station_idx);
            manual_sr_index = mb.Hreg(SUBROUTINE_OFFSET + station_idx);
            PRINT("SR IDX: ");
            PRINTLN(manual_sr_index);
            mb.Hreg(CURRENT_SUBROUTINE_OFFSET) = manual_sr_index;
            start_settle_timeout = 200;
            return State::MANUAL_EXECUTE_SR_START_SETTLE;
        }
        case State::MANUAL_EXECUTE_SR_START_SETTLE: {
            if(start_settle_timeout == 0) {
                mb.Coil(ARM_ENABLE_OFFSET, true);
                sr_start_timeout = 1000;
                return State::MANUAL_EXECUTE_SR_WAIT_START;
            }
            return State::MANUAL_EXECUTE_SR_START_SETTLE;
        }
        case State::MANUAL_EXECUTE_SR_WAIT_START: {
            if (mb.Coil(ARM_RUNNING_OFFSET)) {
                PRINTLN("Arm confirms job");
                sr_finish_timeout = 60000;
                mb.Coil(PROGRAM_SELECT_OFFSET, false);
                return State::MANUAL_EXECUTE_SR_WAIT_FINISH;
            }
            if (sr_start_timeout == 0) {
                PRINTLN("SR start timeout. Arm is not responding");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                mb.Coil(PROGRAM_SELECT_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            if (manual_sr_cancel_latched) {
                PRINTLN("SR cancel latched while waiting for arm to confirm receipt of SR. Stopping SR");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                mb.Coil(PROGRAM_SELECT_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            return State::MANUAL_EXECUTE_SR_WAIT_START;
        }
        case State::MANUAL_EXECUTE_SR_WAIT_FINISH: {
            if (manual_sr_cancel_latched) {
                PRINTLN("Stop manual SR. Setting enable to false and waiting 1 second");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            if (!mb.Coil(ARM_RUNNING_OFFSET)) {
                PRINTLN("SR finished");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                return State::IDLE;
            }
            if (sr_finish_timeout == 0) {
                PRINTLN("SR finish timeout. Arm has reported running for 60 seconds. Stopping SR and calling ESTOP");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                estop(); // arm is unresponsive and preports to be moving
                return State::ERROR_STATE;
            }
            return State::MANUAL_EXECUTE_SR_WAIT_FINISH;
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
                CARRIAGE_MOTOR.MoveStopDecel();
                PRINTLN("Stopping cycle");
                return State::IDLE;
            }
            switch (wait_for_motor_motion(CARRIAGE_MOTOR)) {
                case MotorWaitResult::DONE: {
                    PRINT("Motor reached position ");
                    PRINTLN(cycle_target_index);
                    return State::JOB_START_SR;
                }
                case MotorWaitResult::ERROR: {
                    PRINTLN("Motor error during cycle");
                    print_motor_alerts(CARRIAGE_MOTOR);
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
                        CARRIAGE_MOTOR.MoveStopDecel();
                        PRINTLN("Motor OUT_OF_BOUNDS_NEGATIVE on job, but target is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
                case MotorWaitResult::OUT_OF_BOUNDS_POSITIVE: {
                    if (job_target_steps < CARRIAGE_MOTOR.PositionRefCommanded()) {
                        // currently out of bounds, but going back in. this is ok.
                        return State::JOB_WAIT_POSITION;
                    } else {
                        CARRIAGE_MOTOR.MoveStopDecel();
                        PRINTLN("Motor OUT_OF_BOUNDS_POSITIVE on job, but target is further out of bounds than the current position");
                        return State::IDLE;
                    }
                }
            }
            return State::JOB_WAIT_POSITION;
        }
        case State::JOB_START_SR: {
            if (stop_cycle_latched) {
                PRINTLN("Stopping cycle");
                return State::IDLE;
            }
            mb.Coil(PROGRAM_SELECT_OFFSET, true);
            const u16 sr_index = mb.Hreg(SUBROUTINE_OFFSET + cycle_target_index);
            PRINT("Executing SR #");
            PRINTLN(sr_index);
            mb.Hreg(CURRENT_SUBROUTINE_OFFSET) = sr_index;
            start_settle_timeout = 200;
            return State::JOB_START_SR_SETTLE;
        }

        case State::JOB_START_SR_SETTLE: {
            if(start_settle_timeout == 0) {
                sr_start_timeout = 1000;
                mb.Coil(ARM_ENABLE_OFFSET, true);
                return State::JOB_WAIT_SR_START;
            }
            return State::JOB_START_SR_SETTLE;
        }

        case State::JOB_WAIT_SR_START: {
            if(stop_cycle_latched) {
                PRINTLN("Stopping cycle");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                mb.Coil(PROGRAM_SELECT_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            if (mb.Coil(ARM_RUNNING_OFFSET)) {
                PRINTLN("Arm confirms job");
                mb.Coil(PROGRAM_SELECT_OFFSET, false);
                sr_finish_timeout = 60000;
                return State::JOB_WAIT_SR_FINISH;
            }
            if (sr_start_timeout == 0) {
                PRINTLN("SR start timeout. Arm is not responding");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                mb.Coil(PROGRAM_SELECT_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            return State::JOB_WAIT_SR_START;
        }

        case State::JOB_WAIT_SR_FINISH: {
            if(stop_cycle_latched) {
                PRINTLN("Stopping cycle");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            if (!mb.Coil(ARM_RUNNING_OFFSET)) {
                PRINTLN("SR finished move, moving to cooldown");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                cycle_last_index = cycle_target_index;
                sr_cooldown_timeout = 200;
                return State::JOB_WAIT_SR_COOLDOWN;
            }
            if(sr_finish_timeout == 0){
                PRINTLN("SR finish timeout. Arm is not responding");
                mb.Coil(ARM_ENABLE_OFFSET, false);
                stop_sr_delay = 1000;
                return State::STOP_SR_DELAY;
            }
            return State::JOB_WAIT_SR_FINISH;
        }

        case State::JOB_WAIT_SR_COOLDOWN: {
            if (sr_cooldown_timeout == 0) {
                PRINTLN("SR cooldown finished, next position");
                return State::JOB_CALC_NEXT_POSITION;
            }
            return State::JOB_WAIT_SR_COOLDOWN;
        }

        case State::STOP_SR_DELAY: {
            if (stop_sr_delay == 0) {
                if (mb.Coil(ARM_RUNNING_OFFSET)) {
                    PRINTLN("SR was supposed to be stopped, but it's not. "
                            "Trying to stop again, and then declaring unrecoverable error");
                    mb.Coil(ARM_ENABLE_OFFSET, false);
                    estop();
                    return State::ERROR_STATE;
                }
                return State::IDLE;
            }
            return State::STOP_SR_DELAY;
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
                CARRIAGE_MOTOR.MoveStopDecel();
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
    if(machine_state == State::MANUAL_EXECUTE_SR_START_SETTLE || machine_state == State::JOB_START_SR_SETTLE) {
        if(start_settle_timeout > 0){
            start_settle_timeout--;
        }
    }
    if (machine_state == State::JOB_WAIT_SR_COOLDOWN) {
        if (sr_cooldown_timeout > 0) {
            sr_cooldown_timeout--;
        }
    }
    if (machine_state == State::STOP_SR_DELAY) {
        if (stop_sr_delay > 0) {
            stop_sr_delay--;
        }
    }
    if (machine_state == State::MANUAL_EXECUTE_SR_WAIT_START || machine_state == State::JOB_WAIT_SR_START) {
        if (sr_start_timeout > 0) {
            sr_start_timeout--;
        }
    }
    if (machine_state == State::MANUAL_EXECUTE_SR_WAIT_FINISH || machine_state == State::JOB_WAIT_SR_FINISH) {
        if (sr_finish_timeout > 0) {
            sr_finish_timeout--;
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
    mb.Coil(ARM_ENABLE_OFFSET, false);
    mb.Coil(PROGRAM_SELECT_OFFSET, false);
    CARRIAGE_MOTOR.MoveStopDecel();
}

void set_estop_out(bool new_state){
    ESTOP_OUT_CONNECTOR.State(static_cast<bool>(new_state ^ INVERT_ESTOP_OUT));
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
void print_motor_alerts(MotorDriver &Motor){
    // report status of alerts
    PRINTLN("ClearPath Alerts present: ");
    if(Motor.AlertReg().bit.MotionCanceledInAlert){
        PRINTLN("    MotionCanceledInAlert "); }
    if(Motor.AlertReg().bit.MotionCanceledPositiveLimit){
        PRINTLN("    MotionCanceledPositiveLimit "); }
    if(Motor.AlertReg().bit.MotionCanceledNegativeLimit){
        PRINTLN("    MotionCanceledNegativeLimit "); }
    if(Motor.AlertReg().bit.MotionCanceledSensorEStop){
        PRINTLN("    MotionCanceledSensorEStop "); }
    if(Motor.AlertReg().bit.MotionCanceledMotorDisabled){
        PRINTLN("    MotionCanceledMotorDisabled "); }
    if(Motor.AlertReg().bit.MotorFaulted){
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

MotorWaitResult wait_for_motor_motion(DummyMotor &Motor) {
    return MotorWaitResult::DONE;
}
void print_motor_alerts(DummyMotor &Motor) {
    PRINTLN("No alerts present");
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
        case State::MANUAL_EXECUTE_SR_START_SETTLE: return "MANUAL_EXECUTE_SR_START_SETTLE";
        case State::MANUAL_EXECUTE_SR_WAIT_START: return "MANUAL_EXECUTE_SR_WAIT_START";
        case State::MANUAL_EXECUTE_SR_WAIT_FINISH: return "MANUAL_EXECUTE_SR_WAIT_FINISH";
        case State::JOB_CALC_NEXT_POSITION: return "JOB_CALC_NEXT_POSITION";
        case State::JOB_MOVE_NEXT_POSITION: return "JOB_MOVE_NEXT_POSITION";
        case State::JOB_WAIT_POSITION: return "JOB_WAIT_POSITION";
        case State::JOB_START_SR: return "JOB_START_SR";
        case State::JOB_START_SR_SETTLE: return "JOB_START_SR_SETTLE";
        case State::JOB_WAIT_SR_START: return "JOB_WAIT_SR_START";
        case State::JOB_WAIT_SR_FINISH: return "JOB_WAIT_SR_FINISH";
        case State::JOB_WAIT_SR_COOLDOWN: return "JOB_WAIT_SR_COOLDOWN";
        case State::STOP_SR_DELAY: return "STOP_SR_DELAY";
        case State::ESTOP_START: return "ESTOP_START";
        case State::ESTOP: return "ESTOP";
        case State::ERROR_STATE: return "ERROR_STATE";
    }
    return "UNKNOWN_STATE";
}


State estop_resume_state(const State state_in) {
    switch (state_in) {
        case State::IDLE:
        case State::START_HOMING:
        case State::WAIT_FOR_HOMING:
        case State::RETURN_TO_START:
        case State::WAIT_FOR_RETURN_TO_START:
        case State::MANUAL_GO_TO_POSITION:
        case State::MANUAL_GO_TO_POSITION_WAIT:
        case State::MANUAL_EXECUTE_SR:
        case State::MANUAL_EXECUTE_SR_START_SETTLE:
        case State::MANUAL_EXECUTE_SR_WAIT_START:
        case State::MANUAL_EXECUTE_SR_WAIT_FINISH:
        case State::JOB_CALC_NEXT_POSITION:
        case State::JOB_MOVE_NEXT_POSITION:
        case State::JOB_WAIT_POSITION:
        case State::JOB_START_SR:
        case State::JOB_START_SR_SETTLE:
        case State::JOB_WAIT_SR_START:
        case State::JOB_WAIT_SR_FINISH:
        case State::JOB_WAIT_SR_COOLDOWN:
        case State::STOP_SR_DELAY:
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