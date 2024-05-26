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

static constexpr uint16_t ENC_RESOLUTION = 8192;
static constexpr float AGITATOR_GEAR_RATIO_M2006 = 1.0f;
float pitch_angle;
float min_pitch=-36;
float max_pitch=46;


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
    pitch_angle=360 / static_cast<float>(ENC_RESOLUTION) * pitch / AGITATOR_GEAR_RATIO_M2006;
    float lim_pitch_angle = limitVal(pitch_angle,min_pitch, max_pitch);
    
    float start_pitch= (27* AGITATOR_GEAR_RATIO_M2006 * ENC_RESOLUTION) / 360;

    if(lim_pitch_angle!=pitch_angle)
    {
        pitch= (lim_pitch_angle * AGITATOR_GEAR_RATIO_M2006 * ENC_RESOLUTION) / 360;
    }

    yaw   = limitVal(rpmToMilliVolts(yaw), -MAX_MV, MAX_MV);

    desiredOutput[static_cast<uint8_t>(MotorId::YAW)]   = yaw;

    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitch+start_pitch;


}

// refresh function
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


}
}  // namespace control::turre