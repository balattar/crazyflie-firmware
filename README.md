# Crazyflie Firmware (modified)  [![Build Status](https://api.travis-ci.org/bitcraze/crazyflie-firmware.svg)](https://travis-ci.org/bitcraze/crazyflie-firmware)

The documentation for the original package, including some installation instructions: https://github.com/bitcraze/crazyflie-firmware.

This package contains modified stm32 firmware for the crazyflie. 

## Main Additions to Firmware

1. Setpoint for attitude rate.
   * This setpoint takes rpy rates and thrust as inputs
   * Arguments: 
   ```c++
   (float rollRate, float PitchRate, float YawRate, float Thrust)
   ```

2. Setpoint for Geometric Tracking Controller (GTC) command.
   * This setpoint takes the flight mode and three corresponding commands
   * Arguments: 
   ```c++
   (uint16_t mode, float cmd1, float cmd2, float cmd3)
   ```

   
Mode | Description | Command 1 | Command 2 | Command 3 | Units |
------------ | ------------ | ------------ | ------------ | ------------ | ------------ |
2 | Linear Velocity | Vx | Vy | Vz | m/s
3 | Attitude | roll | pitch | yaw | rad
4 | Attitude Rate | rollRate | pitchRate | yawRate | rad/s
else | Stop | N/A | N/A | N/A | N/A

3. Geometric Tracking Controller added to onboard controller framework
   * Has 3 Seperate flight modes
      1. Linear Velocity (mode = 2) 
      2. Attitude (mode = 3) 
      3. Attitude Rate (mode = 4) 

4. Boolean parameter to switch between rpyt and pwm commands for the motors
   * PWM values are stored in the control data struct 


## Paramaters and Logging Variables

The following tables include most newly added parameters and logging variables that support the geometric tracking controller and flipped crazyflie mounting setup.

Parameter | Group | Type | File | Description
------------ | ------------- | ------------- | ------------- | ------------- |
 __*h_ceiling*__ | deck | FLOAT | zranger2.c | Defines height of ceiling for relative z position measurement
 __*CmdMotors*__ | motorPowerSet | UINT8 | power_distribution_stock.c | true to send direct PWM commands (for GTC)
 __*kp_v*__ | GtcGain | FLOAT | controller_gtc.c | Proportional velocity error gain 
 __*kp_R*__ | GtcGain | FLOAT | controller_gtc.c | Proportional orientation gain 
 __*kd_R*__ | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain (attitude rate control)
 __*kd_R2*__ | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain (else)

To conserve space, not all additional logging variables will be mentioned.

 Log | Group | Type | File | Description
------------ | ------------- | ------------- | ------------- | ------------- | 
__*f_thrust*__ | GtcForce | FLOAT | controller_gtc.c | Desired thrust force 
__*tau1*__ | GtcForce | FLOAT | controller_gtc.c | Desired torque on roll axis
__*tau2*__ | GtcForce | FLOAT | controller_gtc.c | Desired torque on pitch axis
__*tau3*__ | GtcForce | FLOAT | controller_gtc.c | Desired torque on yaw axis
__*xy*__ | stateEstimateZ | UINT32 | stabilizer.c | compressed x and y position



