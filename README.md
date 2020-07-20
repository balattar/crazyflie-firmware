# Crazyflie Firmware (modified)  [![Build Status](https://api.travis-ci.org/bitcraze/crazyflie-firmware.svg)](https://travis-ci.org/bitcraze/crazyflie-firmware)

The documentation for the original package, including some installation instructions: http://crazyswarm.readthedocs.io/en/latest/.

This package contains modified stm32 firmware for the crazyflie. 

## Paramaters and Logging Variables

The following tables include any newly added parameters and logging variables that support the geometric tracking controller and crazyflie mounting setup.

Parameter | Group | Type | Location | Description
------------ | -------------
 h_ceiling | deck | FLOAT | zranger2.c | Defines height of ceiling for relative z position measurement
 CmdMotors | motorPowerSet | UINT8 | power_distribution_stock.c | true to send direct PWM commands (for GTC)
 kp_v | GtcGain | FLOAT | controller_gtc.c | Proportional velocity gain constant
 kp_R | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain constant
 kd_R | GtcGain | FLOAT | controller_gtc.c | Derivative orientation gain constant
 kd_R2 | GtcGain | FLOAT | controller_gtc.c | Derivative orientation 2 gain constant






PARAM_GROUP_START(GtcGain)
PARAM_ADD(PARAM_FLOAT, kp_v, &kp_v)
PARAM_ADD(PARAM_FLOAT, kp_R, &kp_R)
PARAM_ADD(PARAM_FLOAT, kd_R, &kd_R)
PARAM_ADD(PARAM_FLOAT, kd_R2, &kd_R2)
PARAM_GROUP_STOP(GtcGain)

## Logging Variables

