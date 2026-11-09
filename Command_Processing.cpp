#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

#include "Command_Processing.h"
#include "Data.h"

// Temporary storage for incoming commands from Simulink that have not been validated
float receivedCommands[4];

// Check if command received is in valid format
bool commandFormatValid = false;

// Check if atleast one valid command has been accepted (format and current limit checks are passed)
bool validCommandReceived = false;

// Maximum magnitude of allowed current to be commanded 
const float MAX_CURRENT = 1.1f; // [A]



// Initialize command variables and processing state
void initializeCommandProcessing() {

    // Initialize received motor commands
    for (int i = 0; i < 4; i++) {   // 4 motor commands to be initalized
        receivedCommands[i] = 0.0f; // Initialize unvalidated current commands from simulink to 0 A
        motorCommands.current_cmd[i] = 0.0f; // Initialize validated current commands that will packaged to CAN and sent to teensys to 0 A
    }

    // Initialize command processing state 
    validCommandReceived = false; // Initializes command state to false i.e. no valid command has been received from simulink
    commandFormatValid = false; // Initializes that no command has been accepted

}

// Convert received TCP data from Simulink into four floating-point current commands. Store in receivedCommands
void unpackCommands()	{

    // Assume received current format is invalid
    commandFormatValid = false;

    // Create pointer for analyzing command from Simulink, points to first character
    char* ptr = receivedCommandBuffer;

    // Extract 4 current commands from Simulink
    for (int i = 0; i < 4; i++) { // For each current command

        // Create pointer to store position immediately after the extracted number
        char* endPtr;

        // Convert the command from a string to a float
        receivedCommands[i] = strtof(ptr, &endPtr); // Convert number beginning at ptr and store the position after that number in endPtr

        // Check if number was successfully extracted
        if (ptr == endPtr) {
            Serial.println("Command from Simulink is not a number");
            return;
        }

        // Move pointer past the extracted number
        ptr = endPtr;

        // *ptr: Check if character after extracted number is space || tab || cariage return
        while (*ptr == ' ' || *ptr == '\t' || *ptr == '\r') {
            ptr++; // If it is, then move pointer forward
        }

        // Now check if pointer postion is at a comma
        if (i < 3) {

            // if its not, exit function
            if (*ptr != ',') {
                Serial.println("Cannot find comma delimination in Simulink Command");
                return;
            }

            // if it is, move pointer forward
            ptr++;
        }
    }

    // Ensure there is no additional data after the fourth value
    while (*ptr == ' ' || *ptr == '\t' || *ptr == '\r') {
        ptr++;
    }

    // Check for command terminator
    if (*ptr != '\0') {
        Serial.println("Cannot find command terminator in Simulink Command");
        return;
    }

    // All four values were successfully extracted
    commandFormatValid = true;

}

// Check that the received command packet is valid
bool validateCommands()	{

    // Check whether the command from Simulink was successfully received
    if (!commandFormatValid) {
        Serial.println("Invalid command format from Simulink");
        return false;
    }

    // Check that all four commands are valid finite numbers
    for (int i = 0; i < 4; i++) {

        if (!isfinite(receivedCommands[i])) {
            Serial.println("Command contains an invalid numerical value");
            return false;
        }
    }

    // All four commands passed validation
    return true;

}

// Ensure all four current commands are within allowable limits
bool checkCurrentLimits()	{
    
    // Check each motor's current command
    for (int i = 0; i < 4; i++) {

        // Check whether the absolute current exceeds the maximum allowable current
        if (fabs(receivedCommands[i]) > MAX_CURRENT) {

            Serial.print("Motor ");
            Serial.print(i + 1);
            Serial.println(" current command exceeds allowable limit");

            return false;
        }
    }

    // All four current commands are within allowable limits
    return true;

}

// Update the shared motor command structure after validation
bool processCommands()	{

    // Check that the received command packet is valid
    if (!validateCommands()) {
        return false;
    }

    // Check that all current commands are within allowable limits
    if (!checkCurrentLimits()) {
        return false;
    }

    // Copy validated commands into the shared motor command structure
    for (int i = 0; i < 4; i++) {
        motorCommands.current_cmd[i] = receivedCommands[i];
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.println("current command has been validated");       
    }

    // Indicate that at least one valid command has been received
    validCommandReceived = true;

}

// Track command sequence numbers to detect missing or repeated packets
void updateCommandSequence() {

  
}