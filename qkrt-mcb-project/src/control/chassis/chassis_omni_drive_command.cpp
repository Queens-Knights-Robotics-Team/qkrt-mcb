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

#include "chassis_omni_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

using tap::algorithms::limitVal;

namespace control::chassis
{
ChassisOmniDriveCommand::ChassisOmniDriveCommand(
    ChassisSubsystem &chassis,
    ControlOperatorInterface &operatorInterface)
    : chassis(chassis),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&chassis);
}

void ChassisOmniDriveCommand::execute()
{
    operatorInterface.pollInputDevices();

    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * MAX_CHASSIS_SPEED_MPS;
    };

    chassis.setVelocityOmniDrive(
        operatorInterface.getChassisOmniLeftFrontInput(),
        operatorInterface.getChassisOmniLeftBackInput(),
        operatorInterface.getChassisOmniRightFrontInput(),
        operatorInterface.getChassisOmniRightBackInput()
    );
}

void ChassisOmniDriveCommand::end(bool) { chassis.setVelocityOmniDrive(.0f, .0f, .0f, .0f); }

};  // namespace control::chassis
