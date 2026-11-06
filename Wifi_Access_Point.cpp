#include <string.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

#include "Wifi_Access_Point.h"

// Implementation of funcitons for wifi access point between esp32 and simulink

// WiFi configuration
const char* ssid = "Hampter"; // Wifi name
const char* password = "Whirligig"; // Wifi Password

// TCP server
WiFiServer server(80);

// Connected Simulink client
WiFiClient client;

// Incoming command buffer to hold Simulink data
char rxBuffer[128];
size_t rxIndex = 0;


// Initialize WiFi access point and TCP server
void setupWifi() {

    Serial.println();
    Serial.println("Configuring access point...");

    // Create WiFi access point
    if (!WiFi.softAP(ssid, password)) {

        Serial.println("WiFi failed!");

        return;
    }

    // Print ESP32 IP address
    Serial.print("ESP32 IP address: ");
    Serial.println(WiFi.softAPIP());

    // Start TCP server
    server.begin();

    Serial.println("TCP server started on port 80");

}


// Receive commands from Simulink
bool receiveWifiCommand(char* buffer, size_t bufferSize) {

    // Accept a new client if one is not connected
    if (!client || !client.connected()) {

        if (client) {
            client.stop();
            Serial.println("Simulink disconnected");
        }

        client = server.available();

        if (client) {
            Serial.println("Simulink connected!");
            rxIndex = 0;
        }

        return false;
    }

    // Read incoming data without blocking
    while (client.available()) {

        char c = client.read();

        // Newline indicates end of command packet
        if (c == '\n') {

            // Remove carriage return if present
            if (rxIndex > 0 && rxBuffer[rxIndex - 1] == '\r') {
                rxIndex--;
            }

            // Check output buffer size
            if (rxIndex >= bufferSize) {

                rxIndex = 0;
                return false;

            }

            // Copy completed command
            memcpy(buffer, rxBuffer, rxIndex);

            buffer[rxIndex] = '\0';

            rxIndex = 0;

            return true;

        }

        // Store received character
        if (rxIndex < sizeof(rxBuffer) - 1) {

            rxBuffer[rxIndex++] = c;

        }
        else {

            // Discard oversized command
            rxIndex = 0;

        }

    }

    return false;

}


// Send telemetry to Simulink
void sendWifiTelemetry(const char* data) {

    if (client && client.connected()) {

        client.println(data);

    }

}


// Check whether a computer is connected to the access point
bool checkWiFiConnection() {

    return WiFi.softAPgetStationNum() > 0;

}

