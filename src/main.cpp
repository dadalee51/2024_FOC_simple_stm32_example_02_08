#include <SimpleFOC.h>
TwoWire i2c1 = TwoWire(PB_7,PA_15);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_5, PB_10, PB_3);
float target_angle = 0;
float target_velocity = 1;

void setup() {
  pinMode(PA_7, OUTPUT);
  digitalWrite(PA_7,1);
  sensor.init(&i2c1);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 8;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  // motor.controller = MotionControlType::velocity;
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.1f;
  motor.PID_velocity.D = 0;
  motor.voltage_limit = 6;
  motor.LPF_velocity.Tf = 0.01f;
  motor.P_angle.P = 20;
  motor.velocity_limit = 20;
  motor.PID_velocity.output_ramp = 1000;
  

  motor.init();
  motor.initFOC();
  _delay(1000);
}


void loop() {
  motor.loopFOC();
  // motor.move(target_angle);
  motor.move(target_velocity);
  motor.monitor();
}