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

namespace control::turret
{

static const auto PITCH_MOTOR = static_cast<uint8_t>(TurretSubsystem::MotorId::PITCH);
static const auto YAW_MOTOR   = static_cast<uint8_t>(TurretSubsystem::MotorId::YAW);

static constexpr float ENCODER_RESOLUTION = static_cast<float>(2 << 12);
static constexpr uint16_t ENCODER_YAW_OFFSET   = 0x0430;
static constexpr uint16_t ENCODER_PITCH_OFFSET = 0x02A7;
static constexpr uint16_t ENCODER_PITCH_MIN = 0x0100;
static constexpr uint16_t ENCODER_PITCH_MAX = 0x04DA;

#define ENC_TO_RAD(encoderValue) \
    static_cast<float>(encoderValue) / ENCODER_RESOLUTION * M_TWOPI


// create constructor
TurretSubsystem::TurretSubsystem(Drivers &drivers, const TurretConfig &config)
    : tap::control::Subsystem(&drivers),
      desiredOutput{
        ENC_TO_RAD(ENCODER_PITCH_OFFSET),
        ENC_TO_RAD(ENCODER_YAW_OFFSET)
      },
      pidControllers{},
      imu(drivers.bmi088),
      motors{
          Motor(&drivers, config.pitchId, config.canBus, config.pitchMotorInverted, "PITCH"),
          Motor(&drivers, config.yawId,   config.canBus, config.yawMotorInverted,   "YAW"),
      },
      yawGearRatio(config.yawGearRatio),
      imuInverted(config.imuInverted),
      imuRotationFactor(config.imuRotationFactor),
      encoderYawOffset(config.encoderYawOffset),
      leway(config.leway)
{
    pidControllers[PITCH_MOTOR].setParameter(config.turretPitchPidConfig);
    pidControllers[YAW_MOTOR].setParameter(config.turretYawPidConfig);
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
void TurretSubsystem::adjustPositionGimbal(float pitchInput, float yawInput)
{
#if defined(TARGET_SENTRY)
    yawInput   = limitVal(rpmToMilliVolts(yawInput), -MAX_MV, MAX_MV);
#else
    float imuCounterRotation = rpmToMilliVolts(imu.getGz()) * yawGearRatio / imuRotationFactor;
    if (imuInverted) imuCounterRotation = -imuCounterRotation;
    yawInput   = limitVal(rpmToMilliVolts(yawInput), -MAX_MV, MAX_MV) + imuCounterRotation;
#endif
    
    pitchInput = limitVal(rpmToMilliVolts(pitchInput), -MAX_MV, MAX_MV);

    desiredOutput[static_cast<uint8_t>(MotorId::PITCH)] = pitchInput;
    desiredOutput[static_cast<uint8_t>(MotorId::YAW)]   = yawInput;
}

void TurretSubsystem::refresh()
{
    auto runPid = [](Pid &pid, Motor &motor, float desiredOutput, int leway) {
        pid.update(desiredOutput - motor.getShaftRPM());

        // removes overshoot and drift
        if(pid.getValue() < leway && pid.getValue() > -leway)
            motor.setDesiredOutput(0);
        else
            motor.setDesiredOutput(pid.getValue());
    };

    for (size_t ii = 0; ii < motors.size(); ii++)
    {
        runPid(pidControllers[ii], motors[ii], desiredOutput[ii], leway);
    }
    
    float rawYawAngle = static_cast<float>(
        motors[YAW_MOTOR].getEncoderUnwrapped()) / yawGearRatio - encoderYawOffset;
    internal::turretYaw = rawYawAngle / ENCODER_RESOLUTION * M_TWOPI;
}
}  // namespace control::turret
