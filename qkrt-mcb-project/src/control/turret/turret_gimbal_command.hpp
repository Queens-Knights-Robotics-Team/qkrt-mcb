#pragma once

#include "tap/control/command.hpp"

namespace control
{
class ControlOperatorInterface;
}

namespace control::turret
{
class TurretSubsystem;

/**
 * @brief Commands a Turret subsystem using remote controller. The pitch and yaw of the turret are
 * commanded independently by this command. This command executes constantly, taking remote inputs
 * in from a control operator interface, transforming remote input into gimbal speed.
 */
class TurretGimbalCommand : public tap::control::Command
{
public:
    static constexpr float MAX_GIMBAL_SPEED_MPS = 3.0f;

    /**
     * @brief Construct a new Turret Gimbal Command object
     *
     * @param turret turret to control.
     */
    TurretGimbalCommand(TurretSubsystem &turret, ControlOperatorInterface &operatorInterface);

    const char *getName() const override { return "Turret Gimbal"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    TurretSubsystem &turret;

    ControlOperatorInterface &operatorInterface;
};
}  // namespace control::turret