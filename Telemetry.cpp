#include <Arduino.h>
#include <stdio.h>

#include "Telemetry.h"
#include "Data.h"
#include "Wifi_Access_Point.h"

// Telemetry packet buffer
char telemetryBuffer[512] = {};

// Check whether a valid telemetry packet has been created
bool telemetryPacketReady = false;

// Telemetry transmission rate
const uint32_t TELEMETRY_INTERVAL_US = 10000; // 100 Hz

uint32_t previousTelemetryTime = 0;


// Initialize telemetry structures
void initializeTelemetry() {

    memset(telemetryBuffer, 0, sizeof(telemetryBuffer));

    telemetryPacketReady = false;

    previousTelemetryTime = micros();

}


// Format telemetry measurements into a CSV packet, size 27
void packTelemetry() {

    int length = snprintf(
        telemetryBuffer,
        sizeof(telemetryBuffer),

        "%lu," // 1

        // Motor q-axis currents
        "%.6f,%.6f,%.6f,%.6f," //4

        // Motor d-axis currents
        "%.6f,%.6f,%.6f,%.6f," // 4

        // Motor velocities
        "%.6f,%.6f,%.6f,%.6f," // 4

        // Motor angles
        "%.6f,%.6f,%.6f,%.6f," // 4

        // Gyroscope
        "%.6f,%.6f,%.6f," // 3

        // Accelerometer
        "%.6f,%.6f,%.6f," // 3

        // Magnetometer
        "%.6f,%.6f,%.6f," // 3

        // IMU temperature
        "%d\n", // 1

        // IMU timestamp
        (unsigned long)imuTelemetry.timestamp_us,

        // Motor q-axis currents
        motorTelemetry.current_q[0],
        motorTelemetry.current_q[1],
        motorTelemetry.current_q[2],
        motorTelemetry.current_q[3],

        // Motor d-axis currents
        motorTelemetry.current_d[0],
        motorTelemetry.current_d[1],
        motorTelemetry.current_d[2],
        motorTelemetry.current_d[3],

        // Motor velocities
        motorTelemetry.velocity[0],
        motorTelemetry.velocity[1],
        motorTelemetry.velocity[2],
        motorTelemetry.velocity[3],

        // Motor angles
        motorTelemetry.angle[0],
        motorTelemetry.angle[1],
        motorTelemetry.angle[2],
        motorTelemetry.angle[3],

        // Gyroscope
        imuTelemetry.gyro_x,
        imuTelemetry.gyro_y,
        imuTelemetry.gyro_z,

        // Accelerometer
        imuTelemetry.accel_x,
        imuTelemetry.accel_y,
        imuTelemetry.accel_z,

        // Magnetometer
        imuTelemetry.mag_x,
        imuTelemetry.mag_y,
        imuTelemetry.mag_z,

        // Temperature
        (int)imuTelemetry.temperature
    );

    // Check whether packet formatting succeeded
    if (length > 0 && length < sizeof(telemetryBuffer)) {
        telemetryPacketReady = true;
    }
    else {
        telemetryPacketReady = false;
        Serial.println("Telemetry packet formatting failed!");
    }

}


// Send the completed telemetry packet to Simulink
void sendTelemetry() {

    if (!telemetryPacketReady) {
        return;
    }

    sendWifiTelemetry(telemetryBuffer);
    Serial.println("Telemetry packet sending to Simulink");

    telemetryPacketReady = false;

}


// Combine the latest motor and IMU measurements
void updateTelemetry() {

    uint32_t currentTime = micros();

    // Limit telemetry transmission to 100 Hz
    if ((uint32_t)(currentTime - previousTelemetryTime) <
        TELEMETRY_INTERVAL_US) {
        return;
    }

    previousTelemetryTime = currentTime;

    packTelemetry();

    sendTelemetry();

}