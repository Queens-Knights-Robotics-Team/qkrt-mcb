#include "turret_gimbal_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/control_operator_interface.hpp"

#include "turret_subsystem.hpp"

using tap::algorithms::limitVal;

namespace control::turret
{
// Constructor
TurretGimbalCommand::TurretGimbalCommand(
    TurretSubsystem &turret,
    ControlOperatorInterface &operatorInterface)
    : turret(turret),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&turret);
}

// execute function
void TurretGimbalCommand::execute()
{
    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_GIMBAL_SPEED_MPS;
    };

    turret.setVelocityGimbal(
        scale(operatorInterface.getTurretPitchInput()),
        scale(operatorInterface.getTurretYawInput()));
}

// end function
void TurretGimbalCommand::end(bool) { turret.setVelocityGimbal(0, 0); }
};  // namespace control::turret