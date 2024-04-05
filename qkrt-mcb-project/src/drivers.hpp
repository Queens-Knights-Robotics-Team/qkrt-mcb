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

#include "tap/drivers.hpp"

#ifdef ENV_UNIT_TESTS
#include "control/mock_control_operator_interface.hpp"
#else
#include "control/control_operator_interface.hpp"
#endif

class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers(), controlOperatorInterface(remote, bmi088) {}

public:
#ifdef ENV_UNIT_TESTS
    control::MockControlOperatorInterface controlOperatorInterface;
#else
    control::ControlOperatorInterface controlOperatorInterface;
#endif
};  // class Drivers
