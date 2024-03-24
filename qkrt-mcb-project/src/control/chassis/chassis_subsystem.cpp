/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "chassis_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "drivers.hpp"

using tap::algorithms::limitVal;

namespace control::chassis
{
ChassisSubsystem::ChassisSubsystem(Drivers& drivers, const ChassisConfig& config)
    : Subsystem(&drivers),
      desiredOutput{0.0f, 0.0f, 0.0f, 0.0f},
      pidControllers{},
      motors{
          Motor(&drivers, config.leftFrontId, config.canBus, false, "a"),
          Motor(&drivers, config.leftBackId, config.canBus, false, "b"),
          Motor(&drivers, config.rightFrontId, config.canBus, true, "c"),
          Motor(&drivers, config.rightBackId, config.canBus, true, "d")}
{
    for (auto& pid : pidControllers)
    {
        pid.setParameter(config.wheelVelocityPidConfig);
    }
};
// STEP 1 (Tank Drive): create constructor

// STEP 2 (Tank Drive): initialize function

void ChassisSubsystem::initialize()
{
    for (auto& motor : motors)
    {
        motor.initialize();
    }
}

// STEP 3 (Tank Drive): setVelocityTankDrive function

void ChassisSubsystem::setVelocityTankDrive(float left, float right)
{
    float newLeft = mpsToRpm(left);
    float newRight = mpsToRpm(right);

    desiredOutput[static_cast<int>(MotorId::LF)] =
        limitVal<float>(newLeft, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    desiredOutput[static_cast<int>(MotorId::LB)] =
        limitVal<float>(newLeft, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    desiredOutput[static_cast<int>(MotorId::RF)] =
        limitVal<float>(newRight, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
    desiredOutput[static_cast<int>(MotorId::RB)] =
        limitVal<float>(newRight, -MAX_WHEELSPEED_RPM, MAX_WHEELSPEED_RPM);
}

// STEP 4 (Tank Drive): refresh function

void ChassisSubsystem::refresh()
{
    pidControllers[static_cast<int>(MotorId::LF)].update(
        desiredOutput[static_cast<int>(MotorId::LF)] -
        motors[static_cast<int>(MotorId::LF)].getShaftRPM());
    pidControllers[static_cast<int>(MotorId::LB)].update(
        desiredOutput[static_cast<int>(MotorId::LB)] -
        motors[static_cast<int>(MotorId::LB)].getShaftRPM());
    pidControllers[static_cast<int>(MotorId::RF)].update(
        desiredOutput[static_cast<int>(MotorId::RF)] -
        motors[static_cast<int>(MotorId::RF)].getShaftRPM());
    pidControllers[static_cast<int>(MotorId::RB)].update(
        desiredOutput[static_cast<int>(MotorId::RB)] -
        motors[static_cast<int>(MotorId::RB)].getShaftRPM());

    motors[static_cast<int>(MotorId::LF)].setDesiredOutput(
        pidControllers[static_cast<int>(MotorId::LF)].getValue());
    motors[static_cast<int>(MotorId::LB)].setDesiredOutput(
        pidControllers[static_cast<int>(MotorId::LB)].getValue());
    motors[static_cast<int>(MotorId::RF)].setDesiredOutput(
        pidControllers[static_cast<int>(MotorId::RF)].getValue());
    motors[static_cast<int>(MotorId::RB)].setDesiredOutput(
        pidControllers[static_cast<int>(MotorId::RB)].getValue());
}
}  // namespace control::chassis
