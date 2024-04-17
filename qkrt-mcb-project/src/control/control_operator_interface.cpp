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

ControlOperatorInterface::ControlOperatorInterface(Remote &remote, Bmi088& imu)
        : usingController(true), remote(remote), imu(imu) {}

void ControlOperatorInterface::pollSwitchInputDevice() {
    static bool pressedLastInterval = false;

    if (remote.keyPressed(Remote::Key::Q)) {
        if (!pressedLastInterval) {
            usingController = !usingController;
            pressedLastInterval = true;
        }
        else return;
    }
    else pressedLastInterval = false;
}

std::tuple<double, double, double> ControlOperatorInterface::getControllerInput() const {
    if (!remote.isConnected()) return std::make_tuple(0.0, 0.0, 0.0);

    /* use doubles for enhanced precision when processing return values */
    double x    = static_cast<double>(std::clamp(remote.getChannel(Remote::Channel::LEFT_HORIZONTAL),  -1.0f, 1.0f));
    double y    = static_cast<double>(std::clamp(remote.getChannel(Remote::Channel::LEFT_VERTICAL),    -1.0f, 1.0f));
    double yaw  = static_cast<double>(modm::toRadian(imu.getYaw()));
    double rotX = x * std::cos(-yaw) - y * std::sin(-yaw);
    double rotY = x * std::sin(-yaw) + y * std::cos(-yaw);

    /* constant spin speed for beyblade */
    double rx = 0.0;
    
    return std::make_tuple(rotX, rotY, rx);
}

std::tuple<double, double, double> ControlOperatorInterface::getKeyboardInput() const {
    if (!remote.isConnected()) return std::make_tuple(0.0, 0.0, 0.0);

    double x = remote.keyPressed(Remote::Key::D) ? 1.0 : remote.keyPressed(Remote::Key::A) ? -1.0 : 0.0;
    double y = remote.keyPressed(Remote::Key::W) ? 1.0 : remote.keyPressed(Remote::Key::S) ? -1.0 : 0.0;
    double yaw  = static_cast<double>(modm::toRadian(imu.getYaw()));
    double rotX = x * std::cos(-yaw) - y * std::sin(-yaw);
    double rotY = x * std::sin(-yaw) + y * std::cos(-yaw);

    double rx = remote.keyPressed(Remote::Key::SHIFT) ? 0.2 : 0.0;

    return std::make_tuple(rotX, rotY, rx);
}

float ControlOperatorInterface::getChassisOmniLeftFrontInput() {
    auto [vx, vy, w] = usingController ? getControllerInput() : getKeyboardInput();
    double denom = std::max(std::abs(vy) + std::abs(vx) + std::abs(w), static_cast<double>(1.0));
    return (vy + vx + w) / denom;
}

float ControlOperatorInterface::getChassisOmniLeftBackInput() {
    auto [vx, vy, w] = usingController ? getControllerInput() : getKeyboardInput();
    double denom = std::max(std::abs(vy) + std::abs(vx) + std::abs(w), static_cast<double>(1.0));
    return (vy - vx + w) / denom;
}

float ControlOperatorInterface::getChassisOmniRightFrontInput() {
    auto [vx, vy, w] = usingController ? getControllerInput() : getKeyboardInput();
    double denom = std::max(std::abs(vy) + std::abs(vx) + std::abs(w), static_cast<double>(1.0));
    return (vy - vx - w) / denom;
}

float ControlOperatorInterface::getChassisOmniRightBackInput() {
    auto [vx, vy, w] = usingController ? getControllerInput() : getKeyboardInput();
    double denom = std::max(std::abs(vy) + std::abs(vx) + std::abs(w), static_cast<double>(1.0));
    return (vy + vx - w) / denom;
}

float ControlOperatorInterface::getTurretPitchInput() {
    return std::clamp(remote.getChannel(Remote::Channel::RIGHT_VERTICAL),  -1.0f, 1.0f);
}

float ControlOperatorInterface::getTurretYawInput() {
    return std::clamp(remote.getChannel(Remote::Channel::RIGHT_HORIZONTAL),  -1.0f, 1.0f);
}

}  // namespace control
