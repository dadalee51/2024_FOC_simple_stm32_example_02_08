#include <SimpleFOC.h>
TwoWire i2c1 = TwoWire(PB_7,PA_15);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA_5, PB_10, PB_3);
float target_angle = 0;
float target_velocity = 10;
HardwareSerial Serial4(PC_11, PC_10);
void setup() {
  Serial4.begin(115200);
  pinMode(PA_7, OUTPUT);
  digitalWrite(PA_7,1);
  sensor.init(&i2c1);
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 8;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.controller = MotionControlType::angle;
  motor.controller = MotionControlType::velocity;
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

  //initialize sensors at PB14, PB12, PB11,PB1,  ir LED @PA_12
  pinMode(PB_14, INPUT_ANALOG);
  pinMode(PB_12, INPUT_ANALOG);
  pinMode(PB_11, INPUT_ANALOG);
  pinMode(PB_1, INPUT_ANALOG);
  pinMode(PA_12, OUTPUT_OPEN_DRAIN);
  digitalWrite(PA_12,1);
  Serial4.println("Hello");
  _delay(1000);
}
int a1=0;
int a2=0;
int a3=0;
int a4=0;
#define HAL_ADC_MODULE_ENABLED
void loop() {
  a1=analogRead(PB_14);
  a2=analogRead(PB_12);
  a3=analogRead(PB_11);
  a4=analogRead(PB_1);
  Serial4.print(a1);
  Serial4.print("\t");
  Serial4.print(a2);
  Serial4.print("\t");
  Serial4.print(a3);
  Serial4.print("\t");
  Serial4.print(a4);
  Serial4.println("");
  motor.loopFOC();
  // motor.move(target_angle);
  motor.move(target_velocity);
  motor.monitor();
  delay(100);
}