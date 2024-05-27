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

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;
using tap::communication::sensors::imu::bmi088::Bmi088;

namespace control
{

#define PI 3.1415927f

struct ControlState {
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double normFactor = 1.0;
    double pitch = 0.0;
    double yaw = 0.0;
    bool   flywheel = false;
    bool   agitator = false;
};

static ControlState control_s;

ControlOperatorInterface::ControlOperatorInterface(Remote &remote, Bmi088& imu)
        : activeDevice(DeviceType::CONTROLLER), remote(remote), imu(imu) {
}

void ControlOperatorInterface::pollInputDevices() {
    // /* toggle between controller and keyboard mode */
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

    /* update input state with desired device */
    // static int16_t lastMouseX = remote.getMouseX();
    // static int16_t lastMouseY = remote.getMouseY();
    // int16_t currMouseX = remote.getMouseY();
    // int16_t currMouseY = remote.getMouseY();

    double rawX, rawY;

    switch (activeDevice) {
        case DeviceType::CONTROLLER:
            rawX = static_cast<double>(std::clamp(remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),  -1.0f, 1.0f));
            rawY = static_cast<double>(std::clamp(remote.getChannel(Remote::Channel::LEFT_VERTICAL),    -1.0f, 1.0f));
            control_s.pitch = static_cast<double>(std::clamp(remote.getChannel(Remote::Channel::RIGHT_VERTICAL),   -1.0f, 1.0f));
            control_s.yaw   = static_cast<double>(std::clamp(remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL), -1.0f, 1.0f));
            control_s.flywheel = remote.getSwitch(Remote::Switch::LEFT_SWITCH) == Remote::SwitchState::UP;
            control_s.agitator = remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP;
            break;
        case DeviceType::KEYBOARDMOUSE:
            rawX = static_cast<double>(remote.keyPressed(Remote::Key::D)) * (double)(0.5)
                 + static_cast<double>(remote.keyPressed(Remote::Key::A)) * (double)(-0.5);
            rawY = static_cast<double>(remote.keyPressed(Remote::Key::W)) * (double)(0.5)
                 + static_cast<double>(remote.keyPressed(Remote::Key::S)) * (double)(-0.5);
            control_s.pitch = static_cast<double>(remote.getMouseY() / -100);
            control_s.yaw   = static_cast<double>(remote.getMouseX() /  100);
            control_s.flywheel = remote.getMouseR();
            control_s.agitator = remote.getMouseL();
            break;
        default:
            rawX = rawY = 0.0;
            control_s.pitch = control_s.yaw = 0.0;
    }

    double turretYaw = static_cast<double>(modm::toRadian(imu.getYaw()));
    control_s.x = -rawX * std::cos(turretYaw) + rawY * std::sin(turretYaw);
    control_s.y = -rawX * std::sin(turretYaw) - rawY * std::cos(turretYaw);
    control_s.w = 0.0;
    control_s.normFactor = std::max(std::abs(control_s.x) + std::abs(control_s.y) + std::abs(control_s.w), (double)(1.0));
}

float ControlOperatorInterface::getChassisOmniLeftFrontInput() {
    return (control_s.y + control_s.x + control_s.w) / control_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniLeftBackInput() {
    return (control_s.y - control_s.x + control_s.w) / control_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniRightFrontInput() {
    return (control_s.y - control_s.x - control_s.w) / control_s.normFactor;
}

float ControlOperatorInterface::getChassisOmniRightBackInput() {
    return (control_s.y + control_s.x - control_s.w) / control_s.normFactor;
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
