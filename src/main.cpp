// ----------------------------------------------
#include <Arduino.h>
#include <SimpleFOC.h>
#include "imu_helpers.h"

// (6, 5, 10, 8)
#define MOT1_A 6
#define MOT1_B 5
#define MOT1_C 10
#define MOT1_EN 8

// (9, 3, 11, 7)
#define MOT2_A 9
#define MOT2_B 3
#define MOT2_C 11
#define MOT2_EN 7

// Serial instance for communication with NVIDIA Jetson
HardwareSerial Ser3(PC11, PC10);

// magnetic sensor instance - SPI
MagneticSensorSPI sensor1 = MagneticSensorSPI(2, 14, 0x3FFF);
MagneticSensorSPI sensor2 = MagneticSensorSPI(4, 14, 0x3FFF);

// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board
SPIClass SPI_2(PB15, PB14, PB13);

// create motor and driver instances
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOT1_A, MOT1_B, MOT1_C, MOT1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOT2_A, MOT2_B, MOT2_C, MOT2_EN);

// control algorithm parameters
// stabilisation pid
PIDController pid_stb{.P = 30, .I = 100, .D = 1, .ramp = 100000, .limit = 7};
// velocity pid
PIDController pid_vel{.P = 0.01, .I = 0, .D = 0, .ramp = 10000, .limit = _PI / 10};

// velocity control filtering
LowPassFilter lpf_pitch_cmd{.Tf = 0.07};
// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf = 0.5};
LowPassFilter lpf_steering{.Tf = 0.1};

// WiFi app variables
float steering = 0;
float throttle = 0;
float max_throttle = 40; // 20 rad/s
float max_steering = 2; // 1 V
int state = 0; // 1 on / 0 off

float pitch=0;
float target_pitch=0;
float voltage_control=0;
float steering_adj=0;

#define  MAX_RANG      (520)//the max measurement vaule of the module is 520cm(a little bit longer than effective max range)
#define  ADC_SOLUTION  (4095.0)//ADC accuracy is 12bit

// motion control tunning using commander
int comm_throttle_timout = 0;
int comm_steering_timout = 0;

Commander commander = Commander(Ser3);

void cntStab(char *cmd)
{
  commander.pid(&pid_stb, cmd);
}
void cntMove(char *cmd)
{
  commander.pid(&pid_vel, cmd);
}
void lpfThrottle(char *cmd)
{
  commander.lpf(&lpf_throttle, cmd);
}
void lpfPitch(char *cmd)
{
  commander.lpf(&lpf_pitch_cmd, cmd);
}
void lpfSteering(char *cmd)
{
  commander.lpf(&lpf_steering, cmd);
}

// ----------------------------------------------
void onMotor(char* cmd){
  comm_throttle_timout = 0;
  comm_steering_timout = 0;
  commander.motor(&motor1,cmd);
  }

void doSend(char* cmd) { 
  comm_throttle_timout = 0;
  comm_steering_timout = 0;
  switch (atoi(cmd))
  {
  case 0:
    Ser3.printf("{p0:%.2fp1:%.2fp2:%.2fp3:%.2fp4:%.2f", pid_stb.P, pid_stb.I, pid_stb.D, pid_stb.output_ramp, pid_stb.limit);
    Ser3.printf("p5:%.2fp6:%.2fp7:%.2fp8:%.2fp9:%.2f", pid_vel.P, pid_vel.I, pid_vel.D, pid_vel.output_ramp, pid_vel.limit);
    Ser3.printf("p10:%.2fp11:%.2fp12:%.2f}\n", lpf_pitch_cmd.Tf, lpf_steering.Tf, lpf_throttle.Tf);
    
    break;
  case 1:
    Ser3.printf("{p0:%.2fp1:%.2fp2:%.2fp3:%.2fp4:%.2f", pid_stb.P, pid_stb.I, pid_stb.D, pid_stb.output_ramp, pid_stb.limit);
    Ser3.printf("p5:%.2fp6:%.2fp7:%.2fp8:%.2fp9:%.2f", pid_vel.P, pid_vel.I, pid_vel.D, pid_vel.output_ramp, pid_vel.limit);
    Ser3.printf("p10:%.2fp11:%.2fp12:%.2f}\n", lpf_pitch_cmd.Tf, lpf_steering.Tf, lpf_throttle.Tf);
    break;
  default:
    break;
  }
}

void doGetVar(char* cmd) { 
  comm_throttle_timout = 0;
  comm_steering_timout = 0;
  switch (atoi(cmd))
  {
  case 0:
    Ser3.printf("{p13:%.2fp14:%.2fp15:%.2fp16:%.2fp17:%.2fp18:%.2f}\n", motor1.target, motor2.target, pitch, target_pitch, voltage_control, steering_adj);
    break;
  default:
    break;
  }
}

void doOnOff(char* cmd) {
  comm_throttle_timout = 0;
  comm_steering_timout = 0;
    if(atoi(cmd)) state=1; 
    else state=0;
    Serial.printf("O%d\n", atoi(cmd));
}

void doThrottle(char* cmd) {
  comm_throttle_timout = 0;
  comm_steering_timout = 0;
  commander.scalar(&throttle, cmd);
  if(throttle > 100.0f)  throttle = 100.0f;
  if(throttle < -100.0f)  throttle = -100.0f;
  throttle = max_throttle * ((float)throttle) / 100.0;
  // Serial.printf("Set throttle: %.2f\n",throttle);
}
void doSteering(char* cmd) {
  comm_throttle_timout = 0;
  comm_steering_timout = 0;
  commander.scalar(&steering, cmd);
  if(steering > 100.0f)  steering = 100.0f;
  if(steering < -100.0f)  steering = -100.0f;
  steering = max_steering * ((float)steering) / 100.0;
  // Serial.printf("Set steering: %.2f\n",steering);
}

// ----------------------------------------------
void setup() {

  // use monitoring with Serial for motor init
  // monitoring port
  Serial.begin(115200);

  Ser3.begin(115200);

  // initialise magnetic sensor hardware
  sensor1.init(&SPI_2);
  sensor2.init(&SPI_2);
  Serial.println(F("Sensors ready"));
  Ser3.println(F("Sensors ready"));

  _delay(1000);

  // imu init and configure
  if ( !initIMU() ) {
    Serial.println(F("IMU connection problem... Disabling!"));
    Ser3.println(F("IMU connection problem... Disabling!"));
    return;
  }
  Serial.println(F("IMU ready"));
  Ser3.println(F("IMU ready"));
  
  _delay(1000);

  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // driver config
  driver1.pwm_frequency = 20000;
  driver2.pwm_frequency = 20000;
  // power supply voltage [V]
  driver1.voltage_power_supply = 8;
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 8;
  driver2.init();
  motor2.linkDriver(&driver2);

  // set control loop type to be used
  // using voltage torque mode
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // enable monitoring
  motor1.useMonitoring(Ser3);
  motor2.useMonitoring(Ser3);

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  // add the configuration commands
  commander.add('A', cntStab, "pid stab");
  commander.add('B', cntMove, "pid vel");
  commander.add('C', lpfThrottle, "lpf vel command");
  commander.add('D', lpfPitch, "lpf throttle");
  commander.add('E', lpfSteering, "lpf steering");

  commander.add('M',onMotor,"my motor motion");
  commander.add('O', doOnOff, "State OnOff");
  commander.add('T', doThrottle, "Throttle"); // add Throttle command T
  commander.add('S', doSteering, "Steering");
  commander.add('U', doSend, "Update");

  commander.add('G', doGetVar, "Get variables");

  Serial.println(F("Balancing robot ready!"));
  Ser3.println(F("Balancing robot ready!"));

  // state = 1;
}

// ----------------------------------------------
void loop() {

  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  if (!state)
  { // if balancer disabled
    throttle = 0;
    steering = 0;
    motor1.target = 0;
    motor2.target = 0;
  }
  else if (hasDataIMU())
  { // when IMU has received the package
    // read pitch from the IMU
    // float pitch = getPitchIMU();
    pitch = getPitchIMU();
    
    //-#- Overturning protect
    // Serial.println(pitch);
    if( (pitch < -0.4) || (pitch > 0.4) )
      state = 0;

    // -#- Timeout
    comm_throttle_timout++;
    comm_steering_timout++;
    if(comm_throttle_timout > 100)
      throttle = 0;
    if(comm_steering_timout > 100)
      steering = 0;

    // calculate the target angle for throttle control
    target_pitch = lpf_pitch_cmd(pid_vel((motor1.shaft_velocity + motor2.shaft_velocity) / 2 - lpf_throttle(throttle)));
    // calculate the target voltage
    voltage_control = pid_stb(target_pitch - pitch);

    // filter steering
    steering_adj = lpf_steering(steering);
    // set the tergat voltage value
    motor1.target = voltage_control + steering_adj;
    motor2.target = voltage_control - steering_adj;
  }

  // read the tuning commands from Serial
  commander.run(Ser3, 10);
}
// ----------------------------------------------