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

#include "control_operator_interface.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/communication/sensors/imu/bmi088/bmi088.hpp"
#include "tap/architecture/clock.hpp"

#include "internal.hpp"

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;
using tap::communication::sensors::imu::bmi088::Bmi088;

/* TODO
 *
 * - manage power consumption
 * - make code more readable & add comments if needed
 * - allow user to toggle between 1v1 and 3v3 mode (infantry)
 * - 
 */

namespace control
{

#define WHEEL_DEADZONE  100.0f

struct ControlState {
    // basic movement inputs
    float x = 0.0;
    float y = 0.0;
    float moveSpeed = 1.0f;
    float beyblade = 0.0;
    float normFactor = 1.0;
    bool  invertMovement = false;
    // turret movement inputs
    float pitch = 0.0;
    float pitchSensitivity = 0.0075f;
    float yaw = 0.0;
    float yawSensitivity = 0.0075f;
    // turret head functionality
    bool  flywheel = false;
    bool  agitator = false;
};

static ControlState control_s;

ControlOperatorInterface::ControlOperatorInterface(Remote &remote)
    : activeDevice(DeviceType::CONTROLLER), remote(remote)
{
}

void ControlOperatorInterface::pollInputDevices() {
    static float rawX, rawY;

    /* toggle between controller and keyboard mode */
    static bool toggledDeviceLastInterval = false;
    if (remote.keyPressed(Remote::Key::C)) {
        if (!toggledDeviceLastInterval) {
            activeDevice = (activeDevice == DeviceType::CONTROLLER) ?
                    DeviceType::KEYBOARDMOUSE : DeviceType::CONTROLLER;
            toggledDeviceLastInterval = true;
        }
        else return;
    }
    else toggledDeviceLastInterval = false;

#if defined(TARGET_STANDARD)
    /* during the 1v1, the standard can boost the indexer speed to shoot
       faster */
    static bool toggledIndexerBoostInterval = false;
    if (remote.keyPressed(Remote::Key::Z)) {
        if (!toggledIndexerBoostInterval) {
            internal::indexerBoost = !internal::indexerBoost; 
            toggledIndexerBoostInterval = true;
        }
        else return;
    }
    else toggledIndexerBoostInterval = false;
#endif

#if defined(TARGET_HERO)
    /* in the event that the hero dies with the turret facing the wrong
       way, you can invert the movement manually */
    static bool toggledInvertedMovementLastInterval = false;
    if (remote.keyPressed(Remote::Key::R)) {
        if (!toggledInvertedMovementLastInterval) {
            control_s.invertMovement = !control_s.invertMovement;
            toggledInvertedMovementLastInterval = true;
        }
        else return;
    }
    else toggledInvertedMovementLastInterval = false;
#endif

#if defined(TARGET_SENTRY)
    /* allow the sentry to switch to autonomous mode by flicking down the
       left switch of the controller  */
    if (remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::DOWN) {
        control_s.beyblade =
            remote.getWheel() >  WHEEL_DEADZONE ? 1.0f :
            remote.getWheel() < -WHEEL_DEADZONE ? 0.0f :
            control_s.beyblade;
        rawX = rawY = 0.0;
        control_s.pitch = control_s.yaw = 0.0;
        control_s.flywheel = control_s.agitator = false;
    } else
#endif
    switch (activeDevice) {
        case DeviceType::CONTROLLER:
            rawX = remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
            rawY = remote.getChannel(Remote::Channel::LEFT_VERTICAL);
            control_s.pitch = remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
            control_s.yaw   = remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
            control_s.flywheel = remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
            control_s.agitator = remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
            // you can mess with these values for beyblade depending on available power
            control_s.beyblade =
                remote.getWheel() >  WHEEL_DEADZONE ? 0.45f :
                remote.getWheel() < -WHEEL_DEADZONE ? 0.00f :
                control_s.beyblade;
            break;
        case DeviceType::KEYBOARDMOUSE:
            // you can adjust the move speed after pressing shift
            control_s.moveSpeed = remote.keyPressed(Remote::Key::SHIFT) ? 4 : 1.5;
            rawX = (remote.keyPressed(Remote::Key::D) * control_s.moveSpeed) - (remote.keyPressed(Remote::Key::A) * control_s.moveSpeed);
            rawY = (remote.keyPressed(Remote::Key::W) * control_s.moveSpeed) - (remote.keyPressed(Remote::Key::S) * control_s.moveSpeed);
            control_s.pitch = -static_cast<float>(remote.getMouseY()) * control_s.pitchSensitivity;
            control_s.yaw   =  static_cast<float>(remote.getMouseX()) * control_s.yawSensitivity;
            control_s.flywheel = remote.getMouseR();
            control_s.agitator = remote.getMouseL();
            // you can mess with these values for beyblade depending on available power
            control_s.beyblade =
                remote.keyPressed(Remote::Key::E) ?  1.75f :
                remote.keyPressed(Remote::Key::Q) ? -3.25f :
                remote.keyPressed(Remote::Key::F) ?  0.00f :
                control_s.beyblade;
            break;
        default:
            rawX = rawY = 0.0;
            control_s.pitch = control_s.yaw = 0.0;
            control_s.flywheel = control_s.agitator = false;
    }
    
    rawX = control_s.invertMovement ? -rawX : rawX;
    rawY = control_s.invertMovement ? -rawY : rawY;

    control_s.x = std::cos(-internal::turretYaw) * rawX - std::sin(-internal::turretYaw) * rawY;
    control_s.y = std::sin(-internal::turretYaw) * rawX + std::cos(-internal::turretYaw) * rawY;
    control_s.normFactor = std::max(std::abs(control_s.x) + std::abs(control_s.y) + std::abs(control_s.beyblade), 1.0f);
}

float ControlOperatorInterface::getChassisOmniLeftFrontInput() {
    return (control_s.y + control_s.x + control_s.beyblade) / control_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniLeftBackInput() {
    return (control_s.y - control_s.x + control_s.beyblade) / control_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniRightFrontInput() {
    return (control_s.y - control_s.x - control_s.beyblade) / control_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniRightBackInput() {
    return (control_s.y + control_s.x - control_s.beyblade) / control_s.normFactor;
}

float ControlOperatorInterface::getTurretPitchInput() {
    return control_s.pitch;
}

float ControlOperatorInterface::getTurretYawInput() {
    return control_s.yaw;
}

bool ControlOperatorInterface::getFlyWheelInput() {
    return control_s.flywheel;
}

bool ControlOperatorInterface::getAgitatorInput() {
    return control_s.agitator;
}


}  // namespace control
