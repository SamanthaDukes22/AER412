#ifndef TELEMETRY_H
#define TELEMETRY_H

// Initialize telemetry structures
void initializeTelemetry();	

// Combine the latest motor and IMU measurements
void updateTelemetry();	

// Format telemetry into a packet for TCP transmission (ESP32 to Simulink)
void packTelemetry();	

// Send the completed telemetry packet to Simulink
void sendTelemetry();	

#endif