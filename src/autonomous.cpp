#include "autonomous.h"

#include "main.h"
#include "configure.h"
#include "robotActions.h"

#include <cmath>

namespace {
void autonDoNothing() {
    // chassis.setPose(0, 0, 0);
    // moveDistance(-24, 2000, {.forwards = false});
    robotactions::runOuttake();
    pros::delay(20);
}

void autonLeft() {
    chassis.setPose(0, 0, -90);
    chassis.moveToPose(-33.5, 0, -90, 1500);
    chassis.turnToHeading(180, 600);
    matchLoader.set_value(true);
    chassis.moveToPoint(chassis.getPose().x, -14, 1500, {.maxSpeed = 100});
    pros::delay(500);
    chassis.waitUntilDone();
    // robotactions::spinFirstThreeIntake(robotactions::kFullPowerMv);

    pros::delay(1200);

    chassis.moveToPose(chassis.getPose().x, 18, 180, 1800, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    robotactions::runIntakeIn();
    // robotactions::setHoodUp(true);
    hood.set_value(1);
    pros::delay(1500);

    chassis.moveToPoint(chassis.getPose().x, 0, 1800);
    chassis.turnToHeading(90, 1200);
    pros::delay(1000);

    chassis.turnToHeading(180, 600);
    chassis.moveToPoint(chassis.getPose().x, -14, 1800);
    pros::delay(20);
}

void autonRight() {
    chassis.setPose(0, 0, 90);
    robotactions::runOuttake();
    pros::delay(150);
    robotactions::stopAllIntake();
    chassis.moveToPose(33.5, 0, 90, 1500);
    chassis.turnToHeading(180, 600);
    matchLoader.set_value(true);
    chassis.moveToPoint(chassis.getPose().x, -14, 1500, {.maxSpeed = 70});
    pros::delay(500);
    chassis.waitUntilDone();
    // robotactions::spinFirstThreeIntake(robotactions::kFullPowerMv);

    pros::delay(1200);

    chassis.moveToPose(chassis.getPose().x, 18, 180, 1800, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    robotactions::runIntakeIn();
    // robotactions::setHoodUp(true);
    hood.set_value(1);
    pros::delay(1500);

    chassis.moveToPoint(chassis.getPose().x, 0, 1800);
    chassis.turnToHeading(-90, 1200);
    pros::delay(2000);

    chassis.turnToHeading(180, 600);
    chassis.moveToPoint(chassis.getPose().x, -14, 1800);
    pros::delay(2000);
    chassis.moveToPoint(chassis.getPose().x, -20, 2000, {.forwards = false});
    pros::delay(20);
}
} // namespace

void moveDistance(float distance, int timeout, lemlib::MoveToPointParams params, bool async) {
    // Convert heading to standard-position radians for correct trig.
    const lemlib::Pose pose = chassis.getPose(true, true);
    const float targetX = pose.x + (distance * std::cos(pose.theta));
    const float targetY = pose.y + (distance * std::sin(pose.theta));
    chassis.moveToPoint(targetX, targetY, timeout, params, async);
}

const char* autonRoutineName(AutonRoutine routine) {
    switch (routine) {
    case AutonRoutine::Left:
        return "Left";
    case AutonRoutine::Right:
        return "Right";
    default:
        return "Do Nothing";
    }
}

void runAutonRoutine(AutonRoutine routine) {
    switch (routine) {
    case AutonRoutine::Left:
        autonLeft();
        break;
    case AutonRoutine::Right:
        autonRight();
        break;
    default:
        autonDoNothing();
        break;
    }
}
