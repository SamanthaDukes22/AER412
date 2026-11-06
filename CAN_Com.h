#ifndef CAN_COM_H
#define CAN_COM_H

// Declares funcitons for commanding and receiving with Teensys/Motors over CAN 

// Initialize ESP32 TWAI controller and CAN configuration
void setupCAN();		

// // Package CAN frame and send motor q current to the four Teensys
void transmitCANCommands();

// Receive incoming CAN frames from the Teensys
void receiveCANTelemetry();	

// Identify the sending Teensy and interpret its CAN data
bool processCANMessage();	


// Check CAN bus status and communication errors.
bool checkCANStatus();	

#endif