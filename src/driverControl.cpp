#include "driverControl.h"

#include "configure.h"
#include "dashboard.h"
#include "robotActions.h"

#include <array>
#include <cstdint>

namespace
{
    constexpr std::uint32_t TOP_ROLLER_REVERSAL_MS = 125;

    constexpr std::uint32_t DEFAULT_DEBOUNCE_MS = 120;
    constexpr std::uint32_t TOGGLE_DEBOUNCE_MS = 220;

    bool matchLoaderDown = false;
    bool intakeLiftUp = false;
    bool wing4BarUp = false;
    bool wingUp = true;
    bool midGoalUp = true;
    bool hoodUp = true;
    bool downIntakeActive = false;
    std::uint32_t downIntakeStartMs = 0;
    bool intakeDriveCutActive = false;

    bool basketUp = false;
    bool leverUp = false;

    bool leverReturningDown = false;
    bool lastL1State = false;
    constexpr double LEVER_DOWN_POSITION_THRESHOLD = 5.0; // degrees

    std::array<std::uint32_t, 32> lastButtonPressTimeMs = {0};

    pros::v5::Motor &drivetrainPort6Motor()
    {
        static pros::v5::Motor motor(6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
        return motor;
    }

    pros::v5::Motor &drivetrainPort20Motor()
    {
        static pros::v5::Motor motor(-20, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
        return motor;
    }

    void applyWingHold(bool held)
    {
        wingUp = !held;
        robotactions::setWingUp(wingUp);
    }

    void applyMatchLoaderHold(bool held)
    {
        matchLoaderDown = held;
        robotactions::setMatchLoaderDown(matchLoaderDown);
    }

    void applyIntakeDriveMotorCut()
    {
        const bool intakeRunning = robotactions::isAnyIntakeRunning();
        const bool leverRunning = robotactions::isLeverRunning();
        auto &port6 = drivetrainPort6Motor();
        auto &port20 = drivetrainPort20Motor();

        if (intakeRunning)
        {
            if (!intakeDriveCutActive)
            {
                port6.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                port20.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                intakeDriveCutActive = true;
            }

            port6.move_voltage(0);
            port20.move_voltage(0);
            return;
        }

        if (intakeDriveCutActive)
        {
            port6.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            port20.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            intakeDriveCutActive = false;
        }
    }

    bool isDebouncedNewPress(pros::controller_digital_e_t button, std::uint32_t debounceMs = DEFAULT_DEBOUNCE_MS)
    {
        if (!master.get_digital_new_press(button))
        {
            return false;
        }

        const auto nowMs = static_cast<std::uint32_t>(pros::millis());
        const auto index = static_cast<std::size_t>(button);

        if (index < lastButtonPressTimeMs.size() && lastButtonPressTimeMs[index] != 0U && (nowMs - lastButtonPressTimeMs[index]) < debounceMs)
        {
            return false;
        }

        if (index < lastButtonPressTimeMs.size())
        {
            lastButtonPressTimeMs[index] = nowMs;
        }

        return true;
    }
} // namespace

void runDriveControl()
{
    const DriverProfile profile = getSelectedDriverProfile();
    // if (profile == DriverProfile::Pink) {
    //     chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y), true);
    //     return;
    // }

    chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X), true);

    // chassis.curvature(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));
}

void runScoringControl()
{
    hoodUp = true;
    robotactions::setHoodUp(true);
    // robotactions::spinAllIntake(robotactions::kFullPowerMv);
    // robotactions::spinLiftMotors(robotactions::kFullPowerMv);
}

void runWingHoldControl()
{
    bool l2Pressed = master.get_digital(DIGITAL_L2);
    applyWingHold(l2Pressed);
}

void runR1IntakeControl()
{
    if (master.get_digital(DIGITAL_R1) && !leverUp)
    {
        robotactions::spinAllIntake(robotactions::kFullPowerMv);
    }
}

void runL1LeverControl()
{
    static bool lastL1State = false;
    static bool leverReturningDown = false;
    const double LEVER_DOWN_POSITION_THRESHOLD = 5.0;

    bool currentL1State = master.get_digital(DIGITAL_L1);

    if (currentL1State && basketUp)
    {
        robotactions::setLeverUp(true);
        leverUp = true;
    }
    else
    {
        robotactions::setLeverUp(false);
        leverUp = false;
    }

    int leverPower = 0;
    if (currentL1State)
    {
        if (basketUp)
        {
            leverPower = robotactions::kFullPowerMv;
        }
        else
        {
            leverPower = robotactions::kFullPowerMv / 2;
        }
    }

    if (lastL1State && !currentL1State)
    {
        leverReturningDown = true;
    }
    lastL1State = currentL1State;

    if (currentL1State)
    {
        leverReturningDown = false;
        robotactions::spinLever(leverPower);
    }
    else if (leverReturningDown)
    {
        double pos = lever.get_position();
        if (pos > LEVER_DOWN_POSITION_THRESHOLD)
        {
            pros::delay(100);
            robotactions::spinLever(-robotactions::kFullPowerMv);
        }
        else
        {
            robotactions::stopLever();
            leverReturningDown = false;
        }
    }
    else
    {
        robotactions::stopLever();
    }
}

void runDownIntakeTimedControl()
{
    if (!master.get_digital(DIGITAL_DOWN))
    {
        downIntakeActive = false;
        return;
    }

    if (!downIntakeActive)
    {
        downIntakeActive = true;
        downIntakeStartMs = static_cast<std::uint32_t>(pros::millis());
    }

    const std::uint32_t elapsedMs = static_cast<std::uint32_t>(pros::millis()) - downIntakeStartMs;
    const int topRollerVoltage = elapsedMs < TOP_ROLLER_REVERSAL_MS ? -robotactions::kFullPowerMv : robotactions::kFullPowerMv;
    const bool midGoalShouldBeUp = elapsedMs < TOP_ROLLER_REVERSAL_MS ? true : false;
    robotactions::setIntakeVoltages(-robotactions::kFullPowerMv);
    midGoalUp = midGoalShouldBeUp;
    robotactions::setMidGoalUp(midGoalShouldBeUp);
}

void runOuttakeControl()
{
    if (master.get_digital(DIGITAL_R2))
    {
        robotactions::runOuttake();
    }
}

void runIntakeLiftOutControl()
{
    runLowGoalControl();
}

void runLowGoalControl()
{
    const bool pressed = master.get_digital(DIGITAL_A);
    intakeLiftUp = !pressed;
    robotactions::setIntakeLiftUp(intakeLiftUp);

    if (pressed)
    {
        robotactions::runLowGoalScore();
    }
}

void runMatchLoaderHoldControl()
{
    applyMatchLoaderHold(master.get_digital(DIGITAL_RIGHT));
}

void runWingFourBarToggleControl()
{
    if (isDebouncedNewPress(DIGITAL_Y, TOGGLE_DEBOUNCE_MS))
    {
        wing4BarUp = !wing4BarUp;
    }

    robotactions::setWingFourBarUp(wing4BarUp);
}

void runInertialRecalibrationControl()
{
    if (!isDebouncedNewPress(DIGITAL_UP, 500))
    {
        return;
    }

    if (!inertial.is_calibrating())
    {
        inertial.reset(false);
    }
}

void runBasketControl()
{
    if (isDebouncedNewPress(DIGITAL_X, TOGGLE_DEBOUNCE_MS))
    {
        basketUp = !basketUp;
        robotactions::setBasketUp(basketUp);
    }
}

void runWingAlignControl(){
    return;
}


void runDriverControl()
{
    runInertialRecalibrationControl();
    runDriveControl();
    runWingHoldControl();
    runMatchLoaderHoldControl();
    runWingFourBarToggleControl();
    runBasketControl();

    robotactions::stopAllIntake();

    hoodUp = false;
    robotactions::setHoodUp(false);

    midGoalUp = true;
    robotactions::setMidGoalUp(true);

    intakeLiftUp = false;
    robotactions::setIntakeLiftUp(false);

    wing4BarUp = true;
    robotactions::setWingFourBarUp(true);

    basketUp = false;
    robotactions::setBasketUp(false);

    leverUp = false;
    robotactions::setLeverUp(false);

    runL1LeverControl();

    if (master.get_digital(DIGITAL_R2))
    {
        downIntakeActive = false;
        runOuttakeControl();
    }
    else if (master.get_digital(DIGITAL_A))
    {
        downIntakeActive = false;
        runLowGoalControl();
    }
    else if (master.get_digital(DIGITAL_R1))
    {
        downIntakeActive = false;
        runR1IntakeControl();
    }
    else if (master.get_digital(DIGITAL_DOWN))
    {
        runDownIntakeTimedControl();
    }
    else if (master.get_digital(DIGITAL_RIGHT))
    {
        matchLoader.set_value(true);
    }
    else
    {
        downIntakeActive = false;
    }

    applyIntakeDriveMotorCut();
}

bool isMatchLoaderDown()
{
    return matchLoaderDown;
}

bool isIntakeLiftUp()
{
    return intakeLiftUp;
}

bool isWingFourBarUp()
{
    return wing4BarUp;
}

bool isWingUp()
{
    return wingUp;
}

bool isMidGoalUp()
{
    return midGoalUp;
}

bool isHoodUp()
{
    return hoodUp;
}

bool isBasketUp()
{
    return basketUp;
}

bool isLeverUp()
{
    return leverUp;
}

int getFullIntakeVoltage()
{
    return robotactions::kFullPowerMv;
}

int getPartialOuttakeVoltage()
{
    return -robotactions::kPartialOuttakeMv;
}
