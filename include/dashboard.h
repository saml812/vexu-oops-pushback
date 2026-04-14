#ifndef DASHBOARD_H_
#define DASHBOARD_H_

#include "autonomous.h"
#include "main.h"

enum class DashboardMode {
    Disabled,
    Autonomous,
    Auton = Autonomous,
    Driver,
    Opcontrol = Driver
};

enum class DriverProfile { White, Pink };

void initDashboard();
void setDashboardMode(DashboardMode mode);

AutonRoutine getSelectedAutonRoutine();
const char* getSelectedAutonName();
DriverProfile getSelectedDriverProfile();

#endif
