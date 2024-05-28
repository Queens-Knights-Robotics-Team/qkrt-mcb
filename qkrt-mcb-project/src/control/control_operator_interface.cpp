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

namespace control
{

#define WHEEL_DEADZONE      100.0f
#define TURRET_GEAR_RATIO   2.0f

struct InputState {
    float x = 0.0;
    float y = 0.0;
    float w = 0.0;
    float normFactor = 1.0;
    float pitch = 0.0;
    float yaw = 0.0;
    bool flywheel = false;
    bool agitator = false;
    bool beyblade = false;
};

static InputState input_s;

ControlOperatorInterface::ControlOperatorInterface(Remote &remote, Bmi088& imu)
        : activeDevice(DeviceType::CONTROLLER), remote(remote), imu(imu) {
}

void ControlOperatorInterface::pollInputDevices() {
    /* toggle between controller and keyboard mode */
    static bool pressedLastInterval = false;

    if (remote.keyPressed(Remote::Key::Q)) {
        if (!pressedLastInterval) {
            activeDevice = (activeDevice == DeviceType::CONTROLLER) ?
                    DeviceType::KEYBOARDMOUSE : DeviceType::CONTROLLER;
            pressedLastInterval = true;
        }
        else return;
    }
    else pressedLastInterval = false;

    /* poll for input using the chosen active device */
    static float rawX, rawY;
    static int16_t wheelInput;

    switch (activeDevice) {
        case DeviceType::CONTROLLER:
            rawX = remote.getChannel(Remote::Channel::LEFT_HORIZONTAL);
            rawY = remote.getChannel(Remote::Channel::LEFT_VERTICAL);
            input_s.pitch = remote.getChannel(Remote::Channel::RIGHT_VERTICAL);
            input_s.yaw   = remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL);
            input_s.flywheel = remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
            input_s.agitator = remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
            wheelInput = remote.getWheel();
            input_s.beyblade =
                wheelInput >  WHEEL_DEADZONE ? true  :
                wheelInput < -WHEEL_DEADZONE ? false :
                input_s.beyblade;
            break;
        case DeviceType::KEYBOARDMOUSE:
            rawX = remote.keyPressed(Remote::Key::D) * 0.5f + remote.keyPressed(Remote::Key::A) * -0.5f;
            rawY = remote.keyPressed(Remote::Key::W) * 0.5f + remote.keyPressed(Remote::Key::S) * -0.5f;
            input_s.pitch = static_cast<float>(remote.getMouseY() / -100);
            input_s.yaw   = static_cast<float>(remote.getMouseX() /  100);
            input_s.flywheel = remote.getMouseR();
            input_s.agitator = remote.getMouseL();
            input_s.beyblade =
                remote.keyPressed(Remote::Key::Z) ? true  :
                remote.keyPressed(Remote::Key::X) ? false :
                input_s.beyblade;
            break;
        default:
            rawX = rawY = 0.0;
            input_s.pitch = input_s.yaw = 0.0;
    }

    input_s.x = std::cos(-internal::turretYaw) * rawX - std::sin(-internal::turretYaw) * rawY;
    input_s.y = std::sin(-internal::turretYaw) * rawX + std::cos(-internal::turretYaw) * rawY;
    input_s.w = input_s.beyblade ? 0.2f : 0.0f;
    input_s.normFactor = std::max(std::abs(input_s.x) + std::abs(input_s.y) + std::abs(input_s.w), 1.0f);
}

float ControlOperatorInterface::getChassisOmniLeftFrontInput() {
    return (input_s.y + input_s.x + input_s.w) / input_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniLeftBackInput() {
    return (input_s.y - input_s.x + input_s.w) / input_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniRightFrontInput() {
    return (input_s.y - input_s.x - input_s.w) / input_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniRightBackInput() {
    return (input_s.y + input_s.x - input_s.w) / input_s.normFactor;
}

float ControlOperatorInterface::getTurretPitchInput() {
    return input_s.pitch;
}

float ControlOperatorInterface::getTurretYawInput() {
    return input_s.yaw;
}

bool ControlOperatorInterface::getFlyWheelInput() {
    return input_s.flywheel;
}

bool ControlOperatorInterface::getAgitatorInput() {
    return input_s.agitator;
}


}  // namespace control
