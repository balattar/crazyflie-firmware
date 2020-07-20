/**
 *
 * controller_gtc.h - Geometric Tracking Controller Interface
 */
#ifndef __CONTROLLER_GTC_H__
#define __CONTROLLER_GTC_H__

#include "stabilizer_types.h"
//#include "math_linear_algebra.h"

typedef struct _StateFull {
    double data[14];
} StateFull;

typedef struct _MotorCommand {
    double data[4];
} MotorCommand;



void controllerGtcInit(void);
bool controllerGtcTest(void);
void controllerGtc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_GTC_H__
