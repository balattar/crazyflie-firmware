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


/*void myMemCpy (void *dest, void *src, int n);
void hat(double *result, double *vector);
void dehat(double *result, double *matrix);
double mla_dot(double *v1, double *v2, int n);
void matTranspose(double *result, double *matrix, int n);
void matAddsMat(double *result, double *m1, double *m2, int size, int flag);
void matTimesScalar(double *result, double *m1, double s, int size, int flag);
void matTimesVec(double *result, double *matrix, double *vector, int n);
void matTimesMat(double *result, double *m1, double *m2);
void quat2rotm_Rodrigue(double * result, double * quaternion);
*/

void controllerGtcInit(void);
bool controllerGtcTest(void);
void controllerGtc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_GTC_H__
