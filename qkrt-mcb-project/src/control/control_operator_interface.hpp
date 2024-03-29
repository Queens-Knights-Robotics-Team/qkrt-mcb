/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-edu.
 *
 * aruw-edu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-edu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-edu.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <tuple>

namespace tap::communication
{
namespace serial { class Remote; }
namespace sensors::imu::bmi088 { class Bmi088; }
}

namespace control
{
class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::communication::serial::Remote& remote,
                             tap::communication::sensors::imu::bmi088::Bmi088& imu);

    std::tuple<double, double, double> pollInput();

    float getChassisOmniLeftFrontInput();
    float getChassisOmniLeftBackInput();
    float getChassisOmniRightFrontInput();
    float getChassisOmniRightBackInput();

    float getTurretPitchInput() { return 0.0f;}
    float getTurretYawInput() { return 0.0f; }
private:
    tap::communication::serial::Remote& remote;
    tap::communication::sensors::imu::bmi088::Bmi088& imu;
};
}  // namespace control
