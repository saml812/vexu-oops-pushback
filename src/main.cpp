#include "main.h"

#include "autonomous.h"
#include "configure.h"
#include "dashboard.h"
#include "driverControl.h"

void initialize() {
    initializeRobot();
    initDashboard();
    setDashboardMode(DashboardMode::Disabled);
}

void disabled() {
    setDashboardMode(DashboardMode::Disabled);
}

void competition_initialize() {
    setDashboardMode(DashboardMode::Disabled);
}

void autonomous() {
    setDashboardMode(DashboardMode::Autonomous);
    runAutonRoutine(getSelectedAutonRoutine());
}

void opcontrol() {
    setDashboardMode(DashboardMode::Driver);
    while (true) {
        runDriverControl();
        pros::delay(10);
    }
}
