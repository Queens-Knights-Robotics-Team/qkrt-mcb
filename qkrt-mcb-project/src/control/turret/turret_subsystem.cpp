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

#include "../internal.hpp"

using tap::algorithms::limitVal;

static constexpr uint16_t ENC_RESOLUTION = 8192;

namespace control::turret
{
// create constructor
TurretSubsystem::TurretSubsystem(Drivers &drivers, const TurretConfig &config)
    : tap::control::Subsystem(&drivers),
      desiredOutput{},
      pidControllers{},
      motors{
          Motor(&drivers, config.pitchId, config.canBus, true, "PITCH"),
          Motor(&drivers, config.yawId,   config.canBus, false, "YAW"),
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


float TurretSubsystem::getYawEnc(float angle)
{
    return (angle  * ENC_RESOLUTION) / 360;
}


// setVelocityGimbal function
void TurretSubsystem::setVelocityGimbal(float pitch, float yaw)
{
    float start_pitch= getYawEnc(27);

    float pitch_angle=getYawEnc(pitch);

    yaw = limitVal(rpmToMilliVolts(yaw), -MAX_MV, MAX_MV);

    desiredOutput[static_cast<uint8_t>(MotorId::YAW)] = yaw;

    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitch_angle+start_pitch;

}

static constexpr float NumAngles   = (2 << 12);    // the number of distinguishable angles of the motor
static constexpr float TurretError = 0x0430;

void TurretSubsystem::refresh()
{
    //pitch position control
    pidControllers[0].update(desiredOutput[0]-motors[0].getEncoderUnwrapped());
    motors[0].setDesiredOutput(pidControllers[0].getValue());

    //yaw velocity control
    pidControllers[1].update(desiredOutput[1]-motors[1].getShaftRPM());
    if (pidControllers[1].getValue() < 100 && pidControllers[1].getValue() > -100)
        motors[1].setDesiredOutput(0);
    else
        motors[1].setDesiredOutput(pidControllers[1].getValue());

    float rawAngle =static_cast<float>(
        motors[static_cast<uint8_t>(MotorId::YAW)].getEncoderUnwrapped()) * 0.5f - TurretError;
    internal::turretYaw =  rawAngle / NumAngles * M_TWOPI;
}
}  // namespace control::turret,,,,,,,,,,,,,,,,,,,,,j
