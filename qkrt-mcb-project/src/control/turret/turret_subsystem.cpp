/*
 * Copyright (c) 2020-2021 Queen's Knights Robotics Team
 *
 * This file is part of qkrt-mcb.
 *
 * qkrt-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * qkrt-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with qkrt-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

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
          Motor(&drivers, config.yawId,   config.canBus, true, "YAW"),
      }
{
    pidControllers[1].setParameter(config.turret_yaw_VelocityPidConfig);
    pidControllers[0].setParameter(config.turret_pitch_VelocityPidConfig);
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
    pitch = limitVal(rpmToMilliVolts(pitch), -MAX_MV, MAX_MV);
    yaw   = limitVal(rpmToMilliVolts(yaw), -MAX_MV, MAX_MV);

    if (pitch < 0){
        pitch = pitch/4.0;
    }

    // desiredOutput takes milliVolts as input
    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitch;
    desiredOutput[static_cast<uint8_t>(MotorId::YAW)]   = yaw;
}

// refresh function
void TurretSubsystem::refresh()
{
    auto runPid = [](Pid &pid, Motor &motor, float desiredOutput) {
        pid.update(desiredOutput - motor.getShaftRPM());
        if (pid.getValue() < 100 && pid.getValue() > -100)
            motor.setDesiredOutput(0);
        else
            motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii]);
    }
}
}  // namespace control::turret