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

#include "turret_gimbal_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "control/control_operator_interface.hpp"

#include "turret_subsystem.hpp"

#include "../internal.hpp"

using tap::algorithms::limitVal;

namespace control::turret
{

TurretGimbalCommand::TurretGimbalCommand(
    TurretSubsystem& turret,
    ControlOperatorInterface& operatorInterface)
    : turret(turret),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&turret);
}

void TurretGimbalCommand::execute()
{
    auto scale = [](float raw) -> float {
        return limitVal(raw, -1.0f, 1.0f) * 0.004f;
    };

    turret.adjustPositionGimbal(
        scale(operatorInterface.getTurretPitchInput()),
        scale(operatorInterface.getTurretYawInput())
    );
}

void TurretGimbalCommand::end(bool) { turret.adjustPositionGimbal(.0f, .0f); }

};  // namespace control::turret