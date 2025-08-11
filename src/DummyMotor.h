//
// Created by paulw on 8/11/2025.
//

#ifndef DUMMYMOTOR_H
#define DUMMYMOTOR_H
#include <ClearCore.h>
#include <RsTypeNames.h>


class DummyMotor {
public:
    char const *name;

    explicit DummyMotor(char const *name);
    void HlfbMode(ClearCore::MotorDriver::HlfbModes mode) const;
    void HlfbCarrier(ClearCore::MotorDriver::HlfbCarrierFrequency freq) const;
    void AccelMax(u32 max) const;
    void VelMax(u32 max) const;

    static i32 PositionRefCommanded();
    void EnableRequest(bool enable) const;
    void MoveVelocity(i32 vel) const;
    void MoveStopAbrupt() const;
    void PositionRefSet(i32 pos) const;

    static ClearCore::MotorDriver::StatusRegMotor &StatusReg();

    static ClearCore::MotorDriver::HlfbStates &HlfbState();
    void Move(int32_t dist, ClearCore::MotorDriver::MoveTarget moveTarget = ClearCore::StepGenerator::MOVE_TARGET_REL_END_POSN) const;
};


#endif