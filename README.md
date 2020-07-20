# Crazyflie Firmware (modified)  [![Build Status](https://api.travis-ci.org/bitcraze/crazyflie-firmware.svg)](https://travis-ci.org/bitcraze/crazyflie-firmware)

The documentation for the original package, including some installation instructions: https://github.com/bitcraze/crazyflie-firmware.

This package contains modified stm32 firmware for the crazyflie. 

## Paramaters and Logging Variables

The following tables include any newly added parameters and logging variables that support the geometric tracking controller and crazyflie mounting setup.

Parameter | Group | Type | File | Description
------------ | ------------- | ------------- | ------------- | ------------- | 
 *h_ceiling* | deck | FLOAT | zranger2.c | Defines height of ceiling for relative z position measurement
 *CmdMotors* | motorPowerSet | UINT8 | power_distribution_stock.c | true to send direct PWM commands (for GTC)
 *kp_v* | GtcGain | FLOAT | controller_gtc.c | Proportional velocity gain constant
 *kp_R* | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain constant
 *kd_R* | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain constant
 *kd_R2* | GtcGain | FLOAT | controller_gtc.c | Derivative orientation 2 gain constant

To conserve space, not all additional logging variables will be mentioned.

 Log | Group | Type | File | Description
------------ | ------------- | ------------- | ------------- | ------------- | 
*f_thrust* | GtcForce | FLOAT | controller_gtc.c | Desired thrust force 
*tau1* | GtcForce | FLOAT | controller_gtc.c | Desired torque on roll axis
*tau2* | GtcForce | FLOAT | controller_gtc.c | Desired torque on pitch axis
*tau3* | GtcForce | FLOAT | controller_gtc.c | Desired torque on yaw axis
*xy* | stateEstimateZ | UINT32 | stabilizer.c | compressed x and y position



