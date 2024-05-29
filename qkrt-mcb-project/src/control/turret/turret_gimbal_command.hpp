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
    static constexpr float MAX_TURRET_SPEED_MPS = 1.0f;
    static constexpr float TURRET_GEAR_RATIO = 2.0f;


    /**
     * @brief Construct a new Turret Gimbal Command object
     *
     * @param turret turret to control.
     */
    TurretGimbalCommand(TurretSubsystem& turret, ControlOperatorInterface& operatorInterface);

    const char *getName() const override { return "Turret Gimbal"; }

    void initialize() override {}

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const { return false; }

private:
    TurretSubsystem& turret;

    ControlOperatorInterface& operatorInterface;
};

}  // namespace control::turret