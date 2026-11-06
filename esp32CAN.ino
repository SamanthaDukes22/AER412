#include <Arduino.h>

#include "CAN_Com.h"
#include "Command_Processing.h"
#include "Data.h"
#include "IMU.h"
#include "Telemetry.h"
#include "Watchdog.h"
#include "Wifi_Access_Point.h"

// Output command buffer for Simulink commands
char receivedCommandBuffer[128] = {};

void setup() {

    Serial.begin(115200);

    // Initialize command variables and processing state
    initializeCommandProcessing(); // Sets all commands = 0 and bools = false

    // Initialize WiFi access point
    setupWifi(); // Starts wifi server

    // Init CAN Communication bt ESP32 and Teensys
    setupCAN(); 

    // Initialize IMU 
    setupIMU();

    // Initialize telemetry structures
    initializeTelemetry();

    // Initialize safety flags and watchdog timers
    initializeSafety();

}


void loop() {

// Receive commands from Simulink, unpack, validate, record last cmd time from esp32, put into CAN structure, trasmit to teensys

    // Connect to Simulink, read incoming command from Simulink, store command values in receivedCommandBuffer
    if (receiveWifiCommand(receivedCommandBuffer, sizeof(receivedCommandBuffer))) { 

        unpackCommands(); // Unpack Simulink cmds and check formatting

        if (processCommands()) { // Checks cmd formatting and make sures within current limits then adds cmd to shared motor cmd struct
            recordValidCommand(); // Record time validated command was received for watchdog timer
            transmitCANCommands(); // Send validated motor commands to Teensys over CAN
        }
    }

    // Receive motor telemetry
    receiveCANTelemetry(); // If telemetry from motors is available, check packet size and id, read bytes into CANbuffer,
                           // move to processCANMessage() -> Identify type of packet and motor its from,
                           // write to shared motor telemetry structure, record time valid packet received for watchdog

    // Read IMU
    readIMU();             // Read IMU data
    updateIMUData();       // update IMU structure

    // Update and transmit telemetry to simulink
    updateTelemetry();    // Set telemetry transmission rate, move to packTelemetry() -> Formats all telemetry from motors and IMU 
                          // into a CSV packet, check its formatting, if formatting good, send packet to simulink

    // Check com watchdogs
    checkWatchdogs(); // Check that Simulink and Teensys are still communicating, if not within allowable timeframe,
                      // attempt to command all motors to be zero

}



