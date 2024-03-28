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

#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/setpoint/commands/move_integral_command.hpp"

#include "control/agitator/velocity_agitator_subsystem.hpp"
#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_omni_drive_command.hpp"

class Drivers;

namespace control
{
class Robot
{
public:
    Robot(Drivers &drivers);

    void initSubsystemCommands();

private:
    void initializeSubsystems();
    void registerSoldierSubsystems();
    void setDefaultSoldierCommands();
    void startSoldierCommands();
    void registerSoldierIoMappings();

    Drivers &drivers;

    chassis::ChassisSubsystem chassis;
    chassis::ChassisOmniDriveCommand chassisOmniDrive;

    tap::motor::DjiMotor agitator;
 
    algorithms::EduPidConfig eduPidConfig; 
    tap::control::setpoint::MoveIntegralCommand::Config moveIntegralConfig;
    agitator::VelocityAgitatorSubsystem velocityAgitatorSubsystem;
    
    tap::control::setpoint::MoveIntegralCommand moveIntegralCommand;
    tap::control::HoldRepeatCommandMapping rightSwitchUp;
    tap::control::HoldCommandMapping HCM;
};
  
}  // namespace control