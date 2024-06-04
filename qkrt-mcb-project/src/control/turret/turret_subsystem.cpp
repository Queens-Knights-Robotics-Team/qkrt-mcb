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
static constexpr float YAW_GEAR_RATIO     = 2.0f;
static constexpr float YAW_GEAR_RATIO_INV = 1.0f / YAW_GEAR_RATIO;
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
      motors{
          Motor(&drivers, config.pitchId, config.canBus, true,  "PITCH"),
          Motor(&drivers, config.yawId,   config.canBus, false, "YAW"),
      }
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
    /* adjust and bound gimbal pitch value */
    static constexpr float RAD_PITCH_MIN = ENC_TO_RAD(ENCODER_PITCH_MIN);
    static constexpr float RAD_PITCH_MAX = ENC_TO_RAD(ENCODER_PITCH_MAX);
    auto& desiredPitch = desiredOutput[PITCH_MOTOR];
    desiredPitch -= pitchInput; // decrease desired pitch to tilt upwards
    desiredPitch = desiredPitch < RAD_PITCH_MIN ? RAD_PITCH_MIN :
                   desiredPitch > RAD_PITCH_MAX ? RAD_PITCH_MAX : desiredPitch;

    /* adjust yaw value but do not bound */
    desiredOutput[YAW_MOTOR] += yawInput;
}

void TurretSubsystem::refresh()
{
    // pitch position control
    pidControllers[PITCH_MOTOR].update(
        desiredOutput[PITCH_MOTOR] - ENC_TO_RAD(motors[PITCH_MOTOR].getEncoderUnwrapped())
    );
    motors[PITCH_MOTOR].setDesiredOutput(pidControllers[PITCH_MOTOR].getValue());

    // yaw velocity control
    pidControllers[YAW_MOTOR].update(
        desiredOutput[YAW_MOTOR] - ENC_TO_RAD(motors[YAW_MOTOR].getEncoderUnwrapped())
    );
    motors[YAW_MOTOR].setDesiredOutput(pidControllers[YAW_MOTOR].getValue());
    
    float rawYawAngle = static_cast<float>(
        motors[YAW_MOTOR].getEncoderUnwrapped()) * YAW_GEAR_RATIO_INV - ENCODER_YAW_OFFSET;
    internal::turretYaw = rawYawAngle / ENCODER_RESOLUTION * M_TWOPI;
}
}  // namespace control::turret
