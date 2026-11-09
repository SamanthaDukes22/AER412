#ifndef DATA_H
#define DATA_H

#include <stdint.h>

// Define data structures and shared varibales

// CAN ids: Lower values get higher priority
// Motor commands: ESP32 to Teensys
// ids: 0x10x where x is motor #
#define CAN_ID_MOTOR1_COMMAND  0x101
#define CAN_ID_MOTOR2_COMMAND  0x102
#define CAN_ID_MOTOR3_COMMAND  0x103
#define CAN_ID_MOTOR4_COMMAND  0x104

// Motor telemetry: Teensys to ESP32 
// current data ids: 0x11x where x is motor #, q and d current measurements from motors
// dynamic data ids: 0x12x where x is motor #, velocity and angle measurements from motors
#define CAN_ID_MOTOR1_CURRENT_TELEMETRY  0x111
#define CAN_ID_MOTOR1_DYNAMIC_TELEMETRY  0x121

#define CAN_ID_MOTOR2_CURRENT_TELEMETRY  0x112
#define CAN_ID_MOTOR2_DYNAMIC_TELEMETRY  0x122

#define CAN_ID_MOTOR3_CURRENT_TELEMETRY  0x113
#define CAN_ID_MOTOR3_DYNAMIC_TELEMETRY  0x123

#define CAN_ID_MOTOR4_CURRENT_TELEMETRY  0x114
#define CAN_ID_MOTOR4_DYNAMIC_TELEMETRY  0x124

// Shared commanded current received from simulink buffer
extern char receivedCommandBuffer[128];

// Active motors, true or false for each motor
extern const bool ACTIVE_MOTORS[4];

// CAN configuration
const uint32_t CAN_BAUD_RATE = 500000; // 5 kbps same asteensys


struct MotorCommands {

    float current_cmd[4]; // q-axis current [A]

};

// Shared motor command structure
extern MotorCommands motorCommands;

// Motor telemetry 16 bytes so need two 8 byte CAN data frames per motor
struct MotorTelemetry {

    float current_q[4]; // measured d-current [mA]
    float current_d[4]; // measured q-current [mA]
    float velocity[4];  // measured ang vel [rad/s]
    float angle[4];     // measured motor position angle [rad]
  //  float timestamp[4];  // time of measurements [microsec?]
  //  float current_set[4]; // set q-current from teensy [A]

};

// Shared motor telemetry structure
extern MotorTelemetry motorTelemetry;

struct IMUTelemetry {

    uint32_t timestamp_us;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float accel_x;
    float accel_y;
    float accel_z;

    float mag_x;
    float mag_y;
    float mag_z;

    int8_t temperature;
};

// Shared IMU telemetry structure
extern IMUTelemetry imuTelemetry;

#endif