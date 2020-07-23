/* 
* adapted from crazyflie_gazebo controller 
* @Pan Liu
*/

#include "controller_gtc.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "math3d.h"
#include "log.h"
#include "param.h"

#include <math.h>
#include <stdint.h>

static double state_full[14];
static float motorspeed[4];

static double position[3];
static double orientation_q[4];
static double eul[3];
static double vel[3];
static double omega[3];

static double R[3][3];
static double R_d[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
static double e_R[3];
static double omega_d[3];
static double e_omega[3];
static double b1_d[3];
static double b2_d[3];
static double b3_d[3];
static double b2_d_hat[3][3];
static double b3_d_hat[3][3];
static double p_d[3];
static double v_d[3];// = {0.0, 0, 1.0};
static double e_x[3];
static double e_v[3];

static double k_x = 0.8;
static double k_v = 0.2;
static double k_R = 3e-2;
static double k_omega = 2.3e-3;
// static double c_T = 1.2819184e-8; // needs double precisoin
// static double Gamma_inv[4][4] = {  {0.25,  -7.6859225, -7.6859225, -41.914296}, {0.25, 7.6859225,  7.6859225,  -41.914296},
//                             {0.25,  7.6859225,  -7.6859225, 41.914296},  {0.25, -7.6859225, 7.6859225,  41.914296}};    // calculated by Matlab
static double J[3][3] = {{1.65717e-05, 0, 0}, {0, 1.66556e-05, 0}, {0, 0, 2.92617e-05}};
static double mass = (0.025 + 0.00075*4);
static double gravity = 9.8;

static double f_thrust;
static double tau[3];

static double tmp1[3][3];  static double tmp2[3][3];  static double tmp3[3][3];  static double tmp4[3][3];  static double tmp5[3][3];  static double tmp6[3][3];
//static double tmp7[3];  not used
static double tmp8[3]; static double tmp9[3]; static double tmp10[3];    static double tmp11[3][3]; static double tmp12[3];    static double tmp13[3];    static double tmp14[3];
static double tmp21[3];    static double tmp22[3];    static double tmp23[3];    static double tmp24[3];    static double tmp25[3];

static float x;
static float y;
static float z;
static float vx;
static float vy;
static float vz;
static float f1;
static float tau1;
static float tau2;
static float tau3;
static float q1;
static float q2;
static float q3;
static float q4;
static float w1;
static float w2;
static float w3;

static float RREV;

static uint16_t type;

void controllerGtcInit(void)
{
    motorspeed[0] = 0;
    motorspeed[1] = 0;
    motorspeed[2] = 0;
    motorspeed[3] = 0;

    e_R[0] =0;
    e_R[1] =0;
    e_R[2] =0;

    e_omega[0]=0;
    e_omega[1]=0;
    e_omega[2]=0;

    e_x[0]=0;
    e_x[1]=0;
    e_x[2]=0;
    
    e_v[0]=0;
    e_v[1]=0;
    e_v[2]=0;

    f_thrust = 0;
    tau[0]=0;
    tau[1]=0;
    tau[2]=0;

    tmp8[0]=0;tmp8[1]=0;tmp8[2]=0;
    tmp9[0]=0;tmp9[1]=0;tmp9[2]=0;
    tmp10[0]=0;tmp10[1]=0;tmp10[2]=0;
    tmp12[0]=0;tmp12[1]=0;tmp12[2]=0;
    tmp13[0]=0;tmp13[1]=0;tmp13[2]=0;
    tmp14[0]=0;tmp14[1]=0;tmp14[2]=0;
    tmp21[0]=0;tmp21[1]=0;tmp21[2]=0;
    tmp22[0]=0;tmp22[1]=0;tmp22[2]=0;
    tmp23[0]=0;tmp23[1]=0;tmp23[2]=0;
    tmp24[0]=0;tmp24[1]=0;tmp24[2]=0;
    tmp25[0]=0;tmp25[1]=0;tmp25[2]=0;

    controllerGtcTest();
}


bool controllerGtcTest(void)
{
    return true;
}

void controllerGtc(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
    // Need to enter state_full information from state esimators (*sesnors / *state)
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) // 500 Hz
    {
        state_full[0] = (double)state->position.x;
        state_full[1] = (double)state->position.y;
        state_full[2] = (double)state->position.z;

        x = (float)state_full[0];
        y = (float)state_full[1];
        z = (float)state_full[2];

        
        state_full[3] = (double)state->attitudeQuaternion.w; // scalar
        state_full[4] = (double)state->attitudeQuaternion.x; // vector
        state_full[5] = (double)state->attitudeQuaternion.y;
        state_full[6] = (double)state->attitudeQuaternion.z;

        q1 = (float)state_full[3];
        q2 = (float)state_full[4];
        q3 = (float)state_full[5];
        q4 = (float)state_full[6];

        state_full[7] = (double)state->velocity.x;
        state_full[8] = (double)state->velocity.y;
        state_full[9] = (double)state->velocity.z;

        vx = (float)state_full[7];
        vy = (float)state_full[8];
        vz = (float)state_full[9];

        state_full[10] = radians2((double)sensors->gyro.x);
        state_full[11] = radians2((double)sensors->gyro.y);
        state_full[12] = radians2((double)sensors->gyro.z);

        w1 = (float)state_full[10];
        w2 = (float)state_full[11];
        w3 = (float)state_full[12];

        RREV = vz/(1.5f - z);

        myMemCpy(position, state_full, sizeof(position));
        myMemCpy(orientation_q, state_full+3,  sizeof(orientation_q));
        myMemCpy(vel, state_full+7, sizeof(vel));
        myMemCpy(omega, state_full+10, sizeof(omega));
        quat2rotm_Rodrigue((double *) R, orientation_q);

        type = setpoint->gtc_mode;          // 1:position, 2:velocity, 3:attitude, 4:angular rate control
        if (type == 1 || type == 2)
        {
            if (type==1)    // position control
            {
                p_d[0] = (double)setpoint->position.x;
                p_d[1] = (double)setpoint->position.y;
                p_d[2] = (double)setpoint->position.z;
                matAddsMat(e_x, position, p_d, 3, 2);       // e_x = pos - p_d
                myMemCpy(e_v, vel, sizeof(vel));            // e_v = v - v_d
            }
            else            // velocity control
            {
                v_d[0] = (double)setpoint->velocity.x;
                v_d[1] = (double)setpoint->velocity.y;
                v_d[2] = (double)setpoint->velocity.z;
                e_x[0]=0; e_x[1]=0; e_x[2]=0;
                matAddsMat(e_v, vel, v_d, 3, 2);            // e_v = v - v_d
            }
                                       
            matTimesScalar(tmp24, e_x, -k_x, 3, 1);                     // -k_x * e_x
            matTimesScalar(tmp21, e_v, -k_v, 3, 1);                     // -k_v * e_v
            tmp22[0] = 0; tmp22[1] = 0; tmp22[2] = mass*gravity;        // mg * e_3
            matAddsMat(tmp23, tmp21, tmp22, 3, 1);                      // k_v*e_v + mg*e_3
            matAddsMat(tmp25, tmp23, tmp24, 3, 1);                      // -k_x*e_x + -k_v*e_v + mg*e_3
            if (tmp25[2]<0)
                tmp25[2] = 1e-2;
            matTimesScalar(b3_d, tmp25, (double)sqrt(mla_dot(tmp25, tmp25, 3)), 3, 2);     // normalize
            b1_d[0] = 1; b1_d[1] = 0; b1_d[2] = 0;
            hat((double *) b3_d_hat, b3_d);
            matTimesVec(b2_d,(double *) b3_d_hat, b1_d, 3);
            matTimesScalar(b2_d, b2_d, (double)sqrt(mla_dot(b2_d, b2_d, 3)), 3, 2);     // normalize
            hat((double *) b2_d_hat, b2_d);
            matTimesVec(b1_d,(double *) b2_d_hat, b3_d, 3);
            R_d[0][0] = b1_d[0];    R_d[0][1] = b2_d[0];    R_d[0][2] = b3_d[0];
            R_d[1][0] = b1_d[1];    R_d[1][1] = b2_d[1];    R_d[1][2] = b3_d[1];
            R_d[2][0] = b1_d[2];    R_d[2][1] = b2_d[2];    R_d[2][2] = b3_d[2];
            
            matTranspose((double *) tmp1,(double *) R_d, 3);                                // R_d'
            matTranspose((double *) tmp2,(double *) R, 3);                                  // R'
            matTimesMat((double *) tmp3,(double *) tmp1,(double *) R);                      // R_d' * R
            matTimesMat((double *) tmp4,(double *) tmp2,(double *) R_d);                    // R' * R_d
            matAddsMat((double *) tmp5,(double *) tmp3,(double *) tmp4, 3*3, 2);
            matTimesScalar((double *) tmp6,(double *) tmp5, 0.5, 3*3, 1);
            dehat(e_R,(double *) tmp6);
            myMemCpy(e_omega, omega, sizeof(omega));

            double b3[3] = {R[0][2], R[1][2], R[2][2]};
            f_thrust = mla_dot(tmp25, b3, 3);

            matTimesScalar(tmp8, e_R, -k_R, 3, 1);                  // -k_R * e_R
            matTimesScalar(tmp9, e_omega, -k_omega, 3, 1);          // -k_omega * e_omega
            matTimesVec(tmp10, (double *) J, omega, 3);             // J * omega
            hat((double *) tmp11, omega);                           // omega_hat
            matTimesVec(tmp12, (double *) tmp11, tmp10, 3);         // omega x J*omega
            matAddsMat(tmp13, tmp8, tmp9, 3, 1);
            matAddsMat(tau, tmp13, tmp12, 3, 1);

        }
        else if (type == 3 || type==4)
        {
            if (type == 3)          // attitude control
            {
                eul[0] = (double)setpoint->attitude.roll;
                eul[1] = (double)setpoint->attitude.pitch;
                eul[2] = (double)setpoint->attitude.yaw;
                
                R_d[0][0] = (double)cos(eul[1]);    R_d[0][1] = 0;      R_d[0][2] = (double)sin(eul[1]);
                R_d[1][0] = 0;                      R_d[1][1] = 1;      R_d[1][2] = 0;
                R_d[2][0] = -(double)sin(eul[1]);   R_d[2][1] = 0;      R_d[2][2] = (double)cos(eul[1]);
                
                matTranspose((double *) tmp1,(double *) R_d, 3);                                // R_d'
                matTranspose((double *) tmp2,(double *) R, 3);                                  // R'
                matTimesMat((double *) tmp3,(double *) tmp1,(double *) R);                      // R_d' * R
                matTimesMat((double *) tmp4,(double *) tmp2,(double *) R_d);                    // R' * R_d
                matAddsMat((double *) tmp5,(double *) tmp3,(double *) tmp4, 3*3, 2);
                matTimesScalar((double *) tmp6,(double *) tmp5, 0.5, 3*3, 1);
                dehat(e_R,(double *) tmp6);
                myMemCpy(e_omega, omega, sizeof(omega));
            }
            else
            {
                omega_d[0] = (double)setpoint->attitudeRate.roll;
                omega_d[1] = (double)setpoint->attitudeRate.pitch;
                omega_d[2] = (double)setpoint->attitudeRate.yaw;
                
                e_R[0]=0; e_R[1]=0; e_R[2]=0;
                matAddsMat(e_omega, omega, omega_d, 3, 2);            // e_omega = omega - omega_d
            }

            if (R[2][2] > 0.7)
                f_thrust = mass*gravity / R[2][2];
            else
                f_thrust = mass*gravity / 0.7;
            
            matTimesScalar(tmp8, e_R, -k_R, 3, 1);                  // -k_R * e_R
            matTimesScalar(tmp9, e_omega, -k_omega, 3, 1);          // -k_omega * e_omega
            matTimesVec(tmp10, (double *) J, omega, 3);             // J * omega
            hat((double *) tmp11, omega);                           // omega_hat
            matTimesVec(tmp12, (double *) tmp11, tmp10, 3);         // omega x J*omega
            matAddsMat(tmp13, tmp8, tmp9, 3, 1);
            matAddsMat(tau, tmp13, tmp12, 3, 1);

        }

        f1 = (float)f_thrust;
        tau1 = (float)tau[0];
        tau2 = (float)tau[1];
        tau3 = (float)tau[2];
        

        uint32_t scalePWM = 10000; // temporary
        uint32_t minPWM = 1000; // nonlinear below 

        control->thrust = scalePWM*f_thrust;
        control->roll = scalePWM*tau[0];
        control->pitch = scalePWM*tau[1];
        control->yaw = scalePWM*tau[2];

        // shutoff motors if low pwm or incorrect mode
        if ( (control->thrust < minPWM && control->roll < minPWM
            && control->pitch < minPWM && control->yaw < minPWM) 
              || (type != 1 && type != 2 && type != 3 && type != 4) )
        {
          control->thrust = 0;
          control->roll = 0;
          control->pitch = 0;
          control->yaw = 0;

        }

    }
}


LOG_GROUP_START(GtcState)
LOG_ADD(LOG_FLOAT,x, &x)//position[0])
LOG_ADD(LOG_FLOAT,y, &y)//position[1])
LOG_ADD(LOG_FLOAT,z, &z)//position[2])
LOG_ADD(LOG_FLOAT,q1, &q1)
LOG_ADD(LOG_FLOAT,q2, &q2)
LOG_ADD(LOG_FLOAT,q3, &q3)
LOG_ADD(LOG_FLOAT,q4, &q4)
LOG_ADD(LOG_FLOAT,vx, &vx)
LOG_ADD(LOG_FLOAT,vy, &vy)
LOG_ADD(LOG_FLOAT,vz, &vz)
LOG_ADD(LOG_FLOAT,wx, &w1)
LOG_ADD(LOG_FLOAT,wy, &w2)
LOG_ADD(LOG_FLOAT,wz, &w3)
LOG_GROUP_STOP(GtcState)

LOG_GROUP_START(GtcForce)
LOG_ADD(LOG_FLOAT,f_thrust, &f1)//f_thrust)
LOG_ADD(LOG_FLOAT,tau1, &tau1)//[0])
LOG_ADD(LOG_FLOAT,tau2, &tau2)//[1])
LOG_ADD(LOG_FLOAT,tau3, &tau3)//[2])
LOG_GROUP_STOP(GtcForce)

LOG_GROUP_START(GtcGain)
LOG_ADD(LOG_FLOAT, k_x, &k_x)
LOG_ADD(LOG_FLOAT, k_v, &k_v)
LOG_ADD(LOG_FLOAT, k_R, &k_R)
LOG_ADD(LOG_FLOAT, k_omega, &k_omega)
LOG_GROUP_STOP(GtcGain)

PARAM_GROUP_START(GtcGain)
PARAM_ADD(PARAM_FLOAT, k_x, &k_x)
PARAM_ADD(PARAM_FLOAT, k_v, &k_v)
PARAM_ADD(PARAM_FLOAT, k_R, &k_R)
PARAM_ADD(PARAM_FLOAT, k_omega, &k_omega)
PARAM_GROUP_STOP(GtcGain)