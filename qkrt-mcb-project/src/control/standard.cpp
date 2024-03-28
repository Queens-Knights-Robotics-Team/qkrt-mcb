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

#include "standard.hpp"
#include "drivers_singleton.hpp"

#include "tap/util_macros.hpp"
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_omni_drive_command.hpp"

#include "flywheel/flywheel_subsystem.hpp"
#include "flywheel/flywheel_on_command.hpp"

#include "drivers.hpp"
#include "drivers_singleton.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

using namespace tap::control;
using namespace control;
using namespace tap::communication::serial;

driversFunc drivers = DoNotUse_getDrivers;
control::flywheel::FlywheelSubsystem theFlywheel(drivers());
control::flywheel::FlywheelOnCommand flywheelCommand(&theFlywheel);

namespace control
{

HoldRepeatCommandMapping leftSwitchUp(
    drivers(),
    {&flywheelCommand},
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP), true, -1);

Robot::Robot(Drivers &drivers)
        : drivers(drivers),
          chassis(drivers, chassis::ChassisConfig {
                .leftFrontId = MotorId::MOTOR2,
                .leftBackId = MotorId::MOTOR3,
                .rightBackId = MotorId::MOTOR4,
                .rightFrontId = MotorId::MOTOR1,
                .canBus = CanBus::CAN_BUS1,
                .wheelVelocityPidConfig = modm::Pid<float>::Parameter(10,0,0,0,1000),
            }),
          chassisOmniDrive(chassis, drivers.controlOperatorInterface),
          turret(drivers, turret::TurretConfig {
                .pitchId = MotorId::MOTOR6,
                .yawId = MotorId::MOTOR5,
                .canBus = CanBus::CAN_BUS1,
                .turretVelocityPidConfig = modm::Pid<float>::Parameter(10,0,0,0,1000),
            }),
          turretGimbal(turret, drivers.controlOperatorInterface),
          agitator(&drivers, MotorId::MOTOR7, CanBus::CAN_BUS1, false, "e"),
          eduPidConfig{
              .kp = 1000,
              .ki = 0,
              .kd = 0,
              .maxICumulative = 0,
              .maxOutput = 16000
          },
          moveIntegralConfig{
              .targetIntegralChange = M_TWOPI / 10.0f,
              .desiredSetpoint = M_TWOPI,
              .integralSetpointTolerance = 0
          },
          velocityAgitatorSubsystem(drivers, eduPidConfig, agitator), // FIX LATER
          moveIntegralCommand(velocityAgitatorSubsystem, moveIntegralConfig),
          rightSwitchUp(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP), false),
          HCM(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP))
{
}

void Robot::initSubsystemCommands()
{
    initializeSubsystems();
    registerSoldierSubsystems();
    setDefaultSoldierCommands();
    startSoldierCommands();
    registerSoldierIoMappings();
}

void Robot::initializeSubsystems()
{
    chassis.initialize();
    turret.initialize();
    velocityAgitatorSubsystem.initialize();
    theFlywheel.initialize();
}

void Robot::registerSoldierSubsystems()
{
    drivers.commandScheduler.registerSubsystem(&chassis);
    drivers.commandScheduler.registerSubsystem(&turret);
    drivers.commandScheduler.registerSubsystem(&velocityAgitatorSubsystem);
    drivers.commandScheduler.registerSubsystem(&theFlywheel);
}

void Robot::setDefaultSoldierCommands()
{
   chassis.setDefaultCommand(&chassisOmniDrive);
   turret.setDefaultCommand(&turretGimbal);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{
    drivers.commandMapper.addMap(&rightSwitchUp);
    drivers.commandMapper.addMap(&HCM);
    drivers.commandMapper.addMap(&leftSwitchUp);
}
  
}  // namespace control