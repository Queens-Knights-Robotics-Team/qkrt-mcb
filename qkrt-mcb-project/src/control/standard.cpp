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

#include "tap/util_macros.hpp"

#include "control/chassis/chassis_subsystem.hpp"
#include "control/chassis/chassis_omni_drive_command.hpp"

#include "control/turret/turret_subsystem.hpp"
#include "control/turret/turret_gimbal_command.hpp"

#include "drivers.hpp"

using tap::can::CanBus;
using tap::communication::serial::Remote;
using tap::control::RemoteMapState;
using tap::motor::MotorId;

namespace control
{
Robot::Robot(Drivers &drivers) : drivers(drivers),
// construct ChassisSubsystem and ChassisTankDriveCommand
chassis(drivers, chassis::ChassisConfig{
    .leftFrontId = MotorId::MOTOR2,
    .leftBackId = MotorId::MOTOR3,
    .rightBackId = MotorId::MOTOR4,
    .rightFrontId = MotorId::MOTOR1,
    .canBus = CanBus::CAN_BUS1,
    .wheelVelocityPidConfig = modm::Pid<float>::Parameter(10,0,0,0,1000),
    
}),
chassisOmniDrive(chassis, drivers.controlOperatorInterface),

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
// construct VelocityAgitatorSubsystem and MoveIntegralCommand
velocityAgitatorSubsystem(drivers, eduPidConfig, agitator), // FIX LATER
moveIntegralCommand(velocityAgitatorSubsystem, moveIntegralConfig),
// construct HoldRepeatCommandMapping and HoldCommandMapping
rightSwitchUp(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP), false),
HCM(&drivers, {&moveIntegralCommand}, RemoteMapState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP)),

// construct TurretSubsystem and TurretGimbalCommand
turret(
    drivers,
    turret::TurretConfig{
    // gimbal motors
        .pitchId = MotorId::MOTOR5,
        .yawId = MotorId::MOTOR6,
        .canBus = CanBus::CAN_BUS1,
        .velocityPidConfig = modm::Pid<float>::Parameter(10, 0, 0, 0, 16'000),
    }),
turretGimbal(turret, drivers.controlOperatorInterface)

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
    // STEP 4 (Tank Drive): initialize declared ChassisSubsystem
    chassis.initialize();
    // STEP 4 (Agitator Control): initialize declared VelocityAgitatorSubsystem
    velocityAgitatorSubsystem.initialize();

    // initialize declared turret Subsystem
    turret.initialize();
}

void Robot::registerSoldierSubsystems()
{
    // STEP 5 (Tank Drive): register declared ChassisSubsystem
    drivers.commandScheduler.registerSubsystem(&chassis);
    // STEP 5 (Agitator Control): register declared VelocityAgitatorSubsystem
    drivers.commandScheduler.registerSubsystem(&velocityAgitatorSubsystem);

    // register declared ChassisSubsystem
    drivers.commandScheduler.registerSubsystem(&turret);
}

void Robot::setDefaultSoldierCommands()
{
    // STEP 6 (Tank Drive): set ChassisTanKDriveCommand as default command for ChassisSubsystem
   chassis.setDefaultCommand(&chassisOmniDrive); 

   // set TurretGimbalCommand as default command for TurretSubsystem
    turret.setDefaultCommand(&turretGimbal);
}

void Robot::startSoldierCommands() {}

void Robot::registerSoldierIoMappings()
{   
    // STEP 9 (Agitator Control): register HoldRepeatCommandMapping and HoldCommandMapping
    drivers.commandMapper.addMap(&rightSwitchUp);
    drivers.commandMapper.addMap(&HCM);
    
}
}  // namespace control