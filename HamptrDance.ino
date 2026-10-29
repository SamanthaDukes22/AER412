/*
Wezley Mitchell 2026-09-15
Teensy4.1 DO NOT CONNECT PINS TO 5V ONLY USE 3.3v
TMC6300 6PWM motor driver
AS508A Encoder over SPI.
ACS71240LLCBTR-010B3 current sense : 132 mV/A
Op-Amp Gain ~ 3
*/
/*
Hardware setup:
---
MagnetSPI:
00 - CS1
01 - MISO1
26 - MOSI1
27 - SCK1 
---
H-Brige driver
09 - UL
06 - UH
07 - VL
08 - VH
03 - WL
02 - WH
40 - Enable
36 - Diag
---
Voltage sensing
41 - Volt divider
---
Current Sensing
16 (A2) - U current
15 (A1) - V current
14 (A0) - W current
---
CAN bus
23 - CAN R
22 - CAN D
---
*/
#include <SimpleFOC.h>
#include <FlexCAN_T4.h>

#define DEBUG = true

#define VOLTSENS = 41
#define VOLTSENS_GAIN = 4.4
#define CAND = 22
#define CANR = 23
#define PROP = 0.034796103
#define INTG = 14408.74411
#define BANDWIDTH = 3142
//Magnetic Sensor SPI, cs, bit_resolution, register
MagneticSensorSPI motorSensor01 = MagneticSensorSPI( 0, 14, 0x3FFF );
//Motor. PP, R, KV, Iq, Id
BLDCMotor motorA = BLDCMotor( 11, 6.34, 117, 0.00108, 0.00108);
//Curent Sense, mV/A, A adc, B adc, C adc
InlineCurrentSense current_sense  = InlineCurrentSense( 132.0f,  A2, A1, A0 );
//Motor Driver AH, AL, BH, BL, CH, CL, enable
BLDCDriver6PWM driver = BLDCDriver6PWM(6, 9, 8, 7, 2, 3, 40);
//Commander interface
Commander command = Commander(Serial);
//CAN BUS
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;
myCan.setBaudRate(1000000);

void doMotor(char* cmd) { command.motor(&motorA, cmd)}

void setup() {
  Serial.begin(115200);
  if(DEBUG){ SimpleFOCDebug::enable(&Serial); }
  //Read the voltge from the voltage divider
  float inputVoltage = 11.00;
  //Sensor setup
  motorSensor01.spi_mode = SPI_MODE1;
  motorSensor01.init(&SPI1);
  motorA.linkSensor(&motorSensor01);
  //MotorSetup
  motorA.current_limit = 1.0;
  motorA.voltage_limit = inputVoltage / 2;
  motorA.LPF_velocity.Tf = 0.1f;
  //Driver Setup
  driver.pwm_frequenct = 100000;
  driver.dead_zone = 0.02;
  driver.voltage_power_supply = inputVoltage;
  //Delay
  delay(1000);
  Serial.print("Driver init ");
  if (driver.init()) Serial.println("success!");
  else{
    // Send error packet on CAN
    Serial.println("failed!");
    return;
  }
  motorA.linkDriver(&driver);
  motorA.linkCurrentSens(&current_sense);
  motorA.torque_controller = TorqueControlType::foc_current;
  motorA.controller = MotionControlType::torque;
  if(DEBUG){ motorA.useMonitoring(Serial); }
  motorA.PID_current_q.P = PROP;
  motorA.PID_current_d.P = PROP;
  motorA.PID_current_q.I = INTG;
  motorA.PID_current_d.I = INTG;
  motorA.LPF_current_d.Tf = 1.0f / (BANDWIDTH * 6.28f * 5.0f);
  motorA.LPF_current_q.Tf = 1.0f / (BANDWIDTH * 6.28f * 5.0f);
  //Initialize FOC
  // init FOC
  if (motor.initFOC())  {
    command.add('M', doMotor, "motorA");
    // Send ready packet on CAN
    Serial.println("FOC init success!");
  else{
    // Send error packet on CAN
    Serial.println("FOC init failed!");
  return;
}

}

void loop() {
  // put your main code here, to run repeatedly:

}
