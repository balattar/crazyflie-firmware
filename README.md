# Crazyflie Firmware (modified)  [![Build Status](https://api.travis-ci.org/bitcraze/crazyflie-firmware.svg)](https://travis-ci.org/bitcraze/crazyflie-firmware)

The documentation for the original package, including some installation instructions: https://github.com/bitcraze/crazyflie-firmware.

This package contains modified stm32 firmware for the crazyflie. 

## Paramaters and Logging Variables

The following tables include most newly added parameters and logging variables that support the geometric tracking controller and flipped crazyflie mounting setup.

Parameter | Group | Type | File | Description
------------ | ------------- | ------------- | ------------- | ------------- | 
 __*h_ceiling*__ | deck | FLOAT | zranger2.c | Defines height of ceiling for relative z position measurement
 __*CmdMotors*__ | motorPowerSet | UINT8 | power_distribution_stock.c | true to send direct PWM commands (for GTC)
 __*kp_v*__ | GtcGain | FLOAT | controller_gtc.c | Proportional velocity gain constant
 __*kp_R*__ | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain constant
 __*kd_R*__ | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain constant
 __*kd_R2*__ | GtcGain | FLOAT | controller_gtc.c | Derivative orientation 2 gain constant

To conserve space, not all additional logging variables will be mentioned.

 Log | Group | Type | File | Description
------------ | ------------- | ------------- | ------------- | ------------- | 
__*f_thrust*__ | GtcForce | FLOAT | controller_gtc.c | Desired thrust force 
__*tau1*__ | GtcForce | FLOAT | controller_gtc.c | Desired torque on roll axis
__*tau2*__ | GtcForce | FLOAT | controller_gtc.c | Desired torque on pitch axis
__*tau3*__ | GtcForce | FLOAT | controller_gtc.c | Desired torque on yaw axis
__*xy*__ | stateEstimateZ | UINT32 | stabilizer.c | compressed x and y position



