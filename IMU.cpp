#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "IMU.h"
#include "Data.h"

// Set IMU sample rate
const uint32_t BNO055_SAMPLERATE_DELAY_MS = 10; // 100Hz

// Initialize BNO055 at I2C address 0x28
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Sensor status
bool imuReady = false;
bool newIMUData = false;

// Timestamp
uint32_t imuTimestamp = 0;

// Sensor measurement containers
sensors_event_t gyroData;
sensors_event_t accelData;
sensors_event_t magData;

// Board temperature
int8_t boardTemp = 0;

// Sampling timer
uint32_t previousSampleTime = 0;


// Initialize I2C and the BNO055
void setupIMU() {

    Wire.begin();

    Serial.println("Initializing BNO055...");

    // Initialize directly in AMG mode
    if (!bno.begin(OPERATION_MODE_AMG)) {

        Serial.println("BNO055 not detected!");
        imuReady = false;
        return;
    }

    delay(1000);

    imuReady = true;

    Serial.println("BNO055 initialized and is ready!");

}


// Retrieve measurements from the BNO055
void readIMU() {

    if (!imuReady) {
        return;
    }

    // Maintain 100 Hz sampling rate without blocking the main loop
    uint32_t currentTime = micros();

    if ((uint32_t)(currentTime - previousSampleTime) <
        BNO055_SAMPLERATE_DELAY_MS * 1000UL) {
        return;
    }

    previousSampleTime = currentTime;

    // Retrieve raw gyroscope, magnetometer, and accelerometer measurements
    bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // Record timestamp
    imuTimestamp = micros();

    // Retrieve sensor temperature
    boardTemp = bno.getTemp();

    newIMUData = true;

}

// Update the shared IMU telemetry structure
void updateIMUData() {

    if (!newIMUData) {
        return;
    }

    // Timestamp
    imuTelemetry.timestamp_us = imuTimestamp;

    // Gyroscope [rad/s]
    imuTelemetry.gyro_x = gyroData.gyro.x;
    imuTelemetry.gyro_y = gyroData.gyro.y;
    imuTelemetry.gyro_z = gyroData.gyro.z;

    // Accelerometer [m/s^2]
    imuTelemetry.accel_x = accelData.acceleration.x;
    imuTelemetry.accel_y = accelData.acceleration.y;
    imuTelemetry.accel_z = accelData.acceleration.z;

    // Magnetometer [uT]
    imuTelemetry.mag_x = magData.magnetic.x;
    imuTelemetry.mag_y = magData.magnetic.y;
    imuTelemetry.mag_z = magData.magnetic.z;

    // Temperature [degrees C]
    imuTelemetry.temperature = boardTemp;

    newIMUData = false;

}

// Check whether the sensor initialized successfully
bool checkIMUStatus() {

    return imuReady;

}

