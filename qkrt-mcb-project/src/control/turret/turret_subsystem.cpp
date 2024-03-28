#include "turret_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

using tap::algorithms::limitVal;

namespace control::turret
{
// create constructor
TurretSubsystem::TurretSubsystem(Drivers &drivers, const TurretConfig &config)
    : tap::control::Subsystem(&drivers),
      desiredOutput{},
      pidControllers{},
      motors{
          Motor(&drivers, config.pitchId, config.canBus, false, "PITCH"),
          Motor(&drivers, config.yawId, config.canBus, false, "YAW"),
      }
{
    for (auto &controller : pidControllers)
    {
        controller.setParameter(config.velocityPidConfig);
    }
}

// Initialize function
void TurretSubsystem::initialize()
{
    for (auto &motor : motors)
    {
        motor.initialize();
    }
}

// setVelocityGimbal function
void TurretSubsystem::setVelocityGimbal(float pitch, float yaw)
{
    pitch = mpsToRpm(pitch);
    yaw = mpsToRpm(yaw);

    pitch = limitVal(pitch, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    yaw = limitVal(yaw, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);

    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitch;
    desiredOutput[static_cast<uint8_t>(MotorId::YAW)] = yaw;
}

// refresh function
void TurretSubsystem::refresh()
{
    auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
        pid.update(desiredOutput - motor.getShaftRPM());
        motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }
}
}  // namespace control::turret