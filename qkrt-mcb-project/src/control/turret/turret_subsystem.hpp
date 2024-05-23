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

#include <array>

#include "tap/control/subsystem.hpp"
#include "tap/util_macros.hpp"

#include "modm/math/filter/pid.hpp"
#include "modm/math/geometry/angle.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

// Get rid of useless files later

class Drivers;

namespace control::turret
{
struct TurretConfig
{
    tap::motor::MotorId pitchId;
    tap::motor::MotorId yawId;
    tap::can::CanBus canBus;
    modm::Pid<float>::Parameter turretYawPidConfig;
    modm::Pid<float>::Parameter turretPitchPidConfig;
};

///
/// @brief This subsystem encapsulates four motors that control the turret.
///
class TurretSubsystem : public tap::control::Subsystem
{
public:
    /// @brief Motor ID to index into the velocityPid and motors object.
    enum class MotorId : uint8_t
    {
        PITCH = 0,  ///< Pitch
        YAW,        ///< Yaw
        NUM_MOTORS,
    };

    using Pid = modm::Pid<float>;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::DjiMotor;
#endif

    static constexpr float MAX_GIMBAL_SPEED_RPM = 300;

    TurretSubsystem(Drivers& drivers, const TurretConfig& config);

    ///
    /// @brief Initializes the turret motors.
    ///
    void initialize() override;

    ///
    /// @brief Control the chassis using tank drive. Sets the wheel velocity of the four drive
    /// motors based on the input left/right desired velocity.
    ///
    /// @param pitch Desired turret speed in m/s of the pitch of the Turret. Positive speed is
    /// forward, negative is backwards.
    /// @param yaw Desired turret speed in m/s of the yaw of the Turret.
    ///
    mockable void setVelocityGimbal(float pitch, float yaw);

    ///
    /// @brief Runs velocity PID controllers for the drive motors.
    ///
    void refresh() override;

    const char* getName() override { return "Turret"; }

private:
    float MAX_RPM = 300.0f;
    float MAX_MV = 25000.0f;
    
    float rpmToMilliVolts(float rpm)
    {
        return rpm * MAX_MV / MAX_RPM;
    }
    /// Desired wheel output for each motor
    std::array<float, static_cast<uint8_t>(MotorId::NUM_MOTORS)> desiredOutput;

    /// PID controllers. Input desired wheel velocity, output desired motor current.
    std::array<Pid, static_cast<uint8_t>(MotorId::NUM_MOTORS)> pidControllers;

protected:
    /// Motors.
    std::array<Motor, static_cast<uint8_t>(MotorId::NUM_MOTORS)> motors;
};  // class TurretSubsystem
}  // namespace control::turret