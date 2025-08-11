//
// Created by paulw on 8/11/2025.
//

#include "DummyMotor.h"
#define PRINTLN(expr) ConnectorUsb.SendLine(expr)
#define PRINT(expr) ConnectorUsb.Send(expr)

#define BLUE_COLOR "\033[34m"
#define RESET_COLOR "\033[0m"
#define START_BLUE PRINT(BLUE_COLOR)
#define END_BLUE PRINT(RESET_COLOR)


DummyMotor::DummyMotor(char const *name) : name(name) {}
static auto DEFAULT_STATUS = MotorDriver::StatusRegMotor();
static auto DEFAULT_HLFB_STATE = MotorDriver::HlfbStates::HLFB_ASSERTED;

void DummyMotor::HlfbMode(const ClearCore::MotorDriver::HlfbModes mode) const {
    START_BLUE;
    PRINT("Set HLFB mode on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINTLN(mode);
    END_BLUE;
}
void DummyMotor::HlfbCarrier(const ClearCore::MotorDriver::HlfbCarrierFrequency freq) const {
    START_BLUE;
    PRINT("Set HLFB carrier on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINTLN(freq);
    END_BLUE;
}
void DummyMotor::AccelMax(const u32 max) const {
    START_BLUE;
    PRINT("Set AccelMax on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINTLN(max);
    END_BLUE;
}
void DummyMotor::VelMax(const u32 max) const {
    START_BLUE;
    PRINT("Set VelMax on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINTLN(max);
    END_BLUE;
}
i32 DummyMotor::PositionRefCommanded(){
    return 0;
}
void DummyMotor::EnableRequest(const bool enable) const {
    START_BLUE;
    PRINT("Set enable on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINTLN(enable?"TRUE":"FALSE");
    END_BLUE;
}
void DummyMotor::MoveVelocity(const i32 vel) const {
    START_BLUE;
    PRINT("MoveVelocity on motor \"");
    PRINT(name);
    PRINT("\" with vel: ");
    PRINTLN(vel);
    END_BLUE;
}
void DummyMotor::MoveStopAbrupt() const {
    START_BLUE;
    PRINT("MoveStopAbrupt on motor \"");
    PRINT(name);
    PRINTLN("\".");
    END_BLUE;
}
void DummyMotor::PositionRefSet(const i32 pos) const {
    START_BLUE;
    PRINT("Set PosRefSteps on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINTLN(pos);
    END_BLUE;
}
ClearCore::MotorDriver::StatusRegMotor &DummyMotor::StatusReg(){
    return DEFAULT_STATUS;
}
ClearCore::MotorDriver::HlfbStates &DummyMotor::HlfbState(){
    return DEFAULT_HLFB_STATE;
}
void DummyMotor::Move(const int32_t dist, const ClearCore::MotorDriver::MoveTarget moveTarget) const {
    START_BLUE;
    PRINT("Move on motor \"");
    PRINT(name);
    PRINT("\" to ");
    PRINT(dist);
    PRINTLN(moveTarget==ClearCore::StepGenerator::MOVE_TARGET_REL_END_POSN?" rel end posn":" abs end posn");
    END_BLUE;
}
