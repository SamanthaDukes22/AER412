#ifndef COMMAND_PROCESSING_H
#define COMMAND_PROCESSING_H

// Declares funcitons for processing data from Simulink

// Initialize command variables and processing state
void initializeCommandProcessing();

// Convert received TCP data into four floating-point current commands.
void unpackCommands();	

// Check that the received command packet is valid
bool validateCommands();	

// Ensure all four current commands are within allowable limits
bool checkCurrentLimits();	

// Update the shared motor-command structure after validation
bool processCommands();	

// Track command sequence numbers to detect missing or repeated packets
void updateCommandSequence();	


#endif


