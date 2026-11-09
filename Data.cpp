#include "Data.h"

// Define shared motor command, motor telemetry and imu telemetry structures

MotorCommands motorCommands = {};

MotorTelemetry motorTelemetry = {};

IMUTelemetry imuTelemetry = {};


// Which motors are active for test
const bool ACTIVE_MOTORS[4] = {
    true,   // Motor 1
    false,  // Motor 2
    false,  // Motor 3
    false   // Motor 4
};

