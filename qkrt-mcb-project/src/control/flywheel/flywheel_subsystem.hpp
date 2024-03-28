#ifndef FLYWHEEL_SUBSYSTEM_HPP_
#define FLYWHEEL_SUBSYSTEM_HPP_

#include "tap/drivers.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/communication/gpio/pwm.hpp"
#include "tap/util_macros.hpp"

namespace control
{
namespace flywheel
{

class FlywheelSubsystem : public tap::control::Subsystem
{
public:
    FlywheelSubsystem(tap::Drivers *drivers);
        
    ~FlywheelSubsystem() = default;

    void initialize() override;

    void setDesiredOutput(float output);

    void refresh() override;

private:
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN1 = tap::gpio::Pwm::C1;
    static constexpr tap::gpio::Pwm::Pin FLYWHEEL_MOTOR_PIN2 = tap::gpio::Pwm::C2;
    static constexpr float MAX_SNAIL_OUTPUT = 0.50f;    // max pwm input value
    static constexpr float MIN_SNAIL_OUTPUT = 0.25f;    // min pwm input value
    tap::Drivers *drivers;
};

}  // flywheel
}  // control

#endif