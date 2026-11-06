#include <Arduino.h>

#include "Watchdog.h"
#include "Data.h"
#include "CAN_Com.h"

// Watchdog timeout limits
const uint32_t COMMAND_TIMEOUT_MS = 500; // Receiving current cmds from Simulink 
const uint32_t CAN_TIMEOUT_MS = 500;     // Receiving telemetry cmds from Teensys
const uint32_t STARTUP_GRACE_PERIOD_MS = 1000;

// Command communication tracking of simulink command times
uint32_t lastCommandTime = 0;
bool commandReceived = false;

// CAN communication tracking of packet times
uint32_t lastCANTelemetryTime[4] = {};
bool canTelemetryReceived[4] = {};

// Startup timer
uint32_t startupTime = 0;

// Safety state
bool safeStateActive = false;


// Initialize safety flags and watchdog timers
void initializeSafety() {

    startupTime = millis();

    lastCommandTime = startupTime;
    commandReceived = false;

    safeStateActive = false;

    for (int i = 0; i < 4; i++) {
        lastCANTelemetryTime[i] = startupTime;
        canTelemetryReceived[i] = false;
    }

}


// Record successful Simulink command reception
void recordValidCommand() {

    lastCommandTime = millis();
    commandReceived = true;

}


// Record successful CAN telemetry reception
void recordCANTelemetry(uint8_t motorIndex) {

    if (motorIndex >= 4) {
        return;
    }

    lastCANTelemetryTime[motorIndex] = millis();
    canTelemetryReceived[motorIndex] = true;

}


// Detect when Simulink stops sending commands
bool checkCommandTimeout() {

    uint32_t currentTime = millis();

    // Allow startup grace period before first command
    if (!commandReceived) {
        return (currentTime - startupTime) > STARTUP_GRACE_PERIOD_MS;
    }

    return (currentTime - lastCommandTime) > COMMAND_TIMEOUT_MS;

}


// Detect when a Teensy stops sending telemetry
bool checkCANTimeout() {

    uint32_t currentTime = millis();

    for (int i = 0; i < 4; i++) {

        // Allow startup grace period before first telemetry
        if (!canTelemetryReceived[i]) {

            if ((currentTime - startupTime) > STARTUP_GRACE_PERIOD_MS) {
                return true;
            }

        }
        else if ((currentTime - lastCANTelemetryTime[i]) > CAN_TIMEOUT_MS) {
            return true;
        }

    }

    return false;

}


// Trigger motors input to zero when a fault occurs
void enterSafeState() {

    if (safeStateActive) {
        return;
    }

    Serial.println("SAFETY FAULT: Entering safe operating state!");

    // Set all four motor current commands to zero
    for (int i = 0; i < 4; i++) {
        motorCommands.current_cmd[i] = 0.0f;
    }

    // Transmit zero-current commands over CAN
    transmitCANCommands();

    safeStateActive = true;

}


// Periodically check system communication and update status
void checkWatchdogs() {

    if (checkCommandTimeout()) {
        Serial.println("WARNING: Simulink command timeout!");
        enterSafeState();
        Serial.println("WARNING RESPONSE: Attempting to commnad motors to 0 A");
    }

    if (checkCANTimeout()) {
        Serial.println("WARNING: CAN telemetry timeout!");
        enterSafeState();
        Serial.println("WARNING RESPONSE: Attempting to commnad motors to 0 A");
    }

}


// Clear faults when communication has recovered, could implement into Simulink 
void resetSafetyFlags() {

    // Only allow reset when both communication paths are healthy
    if (!checkCommandTimeout() && !checkCANTimeout()) {

        safeStateActive = false;

        Serial.println("Safety flags reset.");

    }

}
