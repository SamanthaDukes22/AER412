#ifndef IMU_H
#define IMU_H

// Declares functions for ESP32 and BNO055 IMU Communication

// Initialize I2C and the BNO055
void setupIMU();

// Retrieve measurements from the BNO055
void readIMU();	

// Update the shared IMU telemetry structure
void updateIMUData();	

// Check whether the sensor is communicating properly
bool checkIMUStatus();	


#endif