/*
Wezley Mitchell 2026-09-15
Teensy4.1 DO NOT CONNECT PINS TO 5V ONLY USE 3.3v
TMC6300 6PWM motor driver
AS508A Encoder over SPI.
*/
#include <SimpleFOC.h>
//#include <SPI.h>
// MagneticSensorSPI(int cs, float_cpr, int_angle_register, long_clock_speed)
// cs             - SPI chip select pin. Test : 10
// bit_resolution - magnetic sensor resolution - AS508A : 14
// angle_register - (optional) angle read register - default 0x3FFF
// clock_speed    - (optional) SPI clock speed - default 1MHz -
MagneticSensorSPI motorSensor01 = MagneticSensorSPI(10, 14, 0x3FFF);
// BLDCMotor(pole pair number, phase resistance [Ohms], KV rating [rpm/V], q-axis inductance [H], d-axis inductance [H])
// Inductance only needed for high speed applications. Has the least impact on performance of the motor.
BLDCMotor motorA = BLDCMotor( 11, 6.34, 117, 0.00108, 0.00108 );
//  BLDCDriver6PWM( int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en)
//  - phA_h, phA_l - A phase pwm pin high/low pair
//  - phB_h, phB_l - B phase pwm pin high/low pair
//  - phB_h, phC_l - C phase pwm pin high/low pair
//  - enable pin    - (optional input)
BLDCDriver6PWM driver = BLDCDriver6PWM(6,9, 8,7, 2,3);
// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motorA, cmd); }
void setup() {
 Serial.begin(115200); // to output the debug information to the serial
 SimpleFOCDebug::enable(&Serial);
// Sensor Initialization Encoder uses SPI_MODE1
 motorSensor01.spi_mode = SPI_MODE1;
// I fried SPI0 on the first Teensy.
 motorSensor01.init(&SPI1);
 motorA.linkSensor(&motorSensor01);
 motorA.current_limit = 1.0; // set current limit to 1 A
 // contoller configuration based on the control type 
 motorA.LPF_velocity.Tf = 0.5; 
// pwm frequency to be used [Hz]
// for atmega328 either 4k or 32kHz
// esp32/stm32/teensy configurable
 driver.pwm_frequency = 100000;
// dead_zone [0,1] - default 0.02 - 2%
 driver.dead_zone = 0.02;
// power supply voltage [V]
 driver.voltage_power_supply = 11.0;
// Max DC voltage allowed - default 0.5 power supply.
 //driver.voltage_limit = 6;
 Serial.print("Driver init ");
// init driver
 if (driver.init()) Serial.println("success!");
 else{
  Serial.println("failed!");
  return;
 }
 motorA.linkDriver(&driver);
 // Downsample Motion
 //motorA.motion_downsample = 5; // downsample 5 times
// set the torque control type
 motorA.torque_controller = TorqueControlType::estimated_current;
// set motion control loop to be used
 motorA.controller = MotionControlType::torque;
// use monitoring with serial
// comment out if not needed
 motorA.useMonitoring(Serial);
// initialize motor
 motorA.init();
// align sensor and start FOC
 motorA.initFOC();
// add target command M
 command.add('M', doMotor, "motorA");
 Serial.println(F("Motor ready."));
 Serial.println(F("Set the target  using serial terminal:"));
 _delay(1000);
}
void loop() {
// main FOC algorithm function
 motorA.loopFOC();
// Motion control function
 motorA.move();
// user communication
 command.run();
 //Serial.println(motorSensor01.getAngle());
}
