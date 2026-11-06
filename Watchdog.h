#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>

// Initialize safety flags and watchdog timers
void initializeSafety();

// Record successful communication timestamps between Simulink and teensys
void recordValidCommand();
void recordCANTelemetry(uint8_t motorIndex);

// Periodically check system communication and update status
void checkWatchdogs();

// Detect when Simulink stops sending commands
bool checkCommandTimeout();

// Detect when a Teensy stops sending telemetry
bool checkCANTimeout();

// Trigger motors input to zero when a fault occurs
void enterSafeState();

// Clear faults when acceptable
void resetSafetyFlags();

#endif