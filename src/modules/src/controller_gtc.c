
#include "controller_gtc.h"
#include "stabilizer.h"
#include "stabilizer_types.h"
#include "math3d.h"
//#include "math_linear_algebra.h"

#include "log.h"
#include "param.h"

#include <math.h>
#include <stdint.h>
//#include "math3d.h"

static double state_full[14];
//static StateFull state_full_structure;
static float motorspeed[4];
//static MotorCommand motorspeed_structure;
//static double control_cmd[5];
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
static double v_d[3];// = {0.0, 0, 1.0};
static double e_v[3];

//static double kp_x = 0.15;  // not used
static double kp_v = 3;
static double kp_R = 1e-5;
static double kd_R = 5e-4;
static double kd_R2 = 1e-6; //0.000001
static double c_T = 1.2819184e-8; // needs double precisoin
static double Gamma_inv[4][4] = {  {0.25,  -7.6859225, -7.6859225, -41.914296}, {0.25, 7.6859225,  7.6859225,  -41.914296},
                            {0.25,  7.6859225,  -7.6859225, 41.914296},  {0.25, -7.6859225, 7.6859225,  41.914296}};    // calculated by Matlab
static double J[3][3] = {{1.65717e-05, 0, 0}, {0, 1.66556e-05, 0}, {0, 0, 2.92617e-05}};

static double f_thrust;
static double f_hover = (0.025 + 0.00075*4)*9.8066;
static double tau[3];
static double FT[4];
static double f[4];
static double motorspeed_square[4];

static double tmp1[3][3];  static double tmp2[3][3];  static double tmp3[3][3];  static double tmp4[3][3];  static double tmp5[3][3];  static double tmp6[3][3];
//static double tmp7[3];  not used
static double tmp8[3]; static double tmp9[3]; static double tmp10[3];    static double tmp11[3][3]; static double tmp12[3];    static double tmp13[3];    static double tmp14[3];
static double tmp21[3];    static double tmp22[3];    static double tmp23[3];    static double tmp24[3];    static double tmp25[3];

static bool InvalidCommand = 0;

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

static int16_t m1;
static int16_t m2;
static int16_t m3;
static int16_t m4;

static float RREV;

static uint16_t type;
static float c1;
static float c2;
static float c3;

void controllerGtcInit(void)
{
  //static double state_full[14];
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
      if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) { // 500 Hz

          

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

          type = setpoint->gtc_mode;//flightMode;

          
          if (type == 2) // velocity control
          {
              InvalidCommand = 0;
              v_d[0] = (double)setpoint->velocity.x;
              v_d[1] = (double)setpoint->velocity.y;
              v_d[2] = (double)setpoint->velocity.z;
              c1 = setpoint->velocity.x;
              c2 = setpoint->velocity.y;
              c3 = setpoint->velocity.z;
              //v_d[0]=0.0;v_d[1]=0.0;v_d[2]=0.5;
            //  v_d[0]=setpoint->velocity.x;v_d[1]=0.0;v_d[2]=0.5;
              quat2rotm_Rodrigue((double *) R, orientation_q);

              matAddsMat(e_v, v_d, vel, 3, 2);                      // e_v = v_d - v
              matTimesScalar(tmp21, e_v, kp_v, 3, 1);               // kp_v * e_v
              tmp22[0] = 0; tmp22[1] = 0; tmp22[2] = f_hover;             // mg * e_3
              matAddsMat(tmp23, tmp21, tmp22, 3, 1);                // kp_v*e_v + mg*e_3
              tmp24[0] = 0;   tmp24[1] = 0;   tmp24[2] = 0;               // kd_v * e_a
              matAddsMat(tmp25, tmp23, tmp24, 3, 1);                // kp_v*e_v + kd_v * e_a + mg*e_3
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
              f_thrust = mla_dot(tmp25, b3_d, 3);

          }
          else if (type == 3) // attitude control
          {
            InvalidCommand = 0;
            eul[0] = (double)setpoint->attitude.roll;
            eul[1] = (double)setpoint->attitude.pitch;
            eul[2] = (double)setpoint->attitude.yaw;
            //eul[0]=0.0;eul[1]=0.0;eul[2]=0.0;
            //std::cout<<"Attitude command ["<< eul[0]<<", "<< eul[1]<<", "<< eul[2]<<"]"<<std::endl;
            quat2rotm_Rodrigue((double *) R, orientation_q);
            R_d[0][0] = (double)cos(eul[1]);    R_d[0][1] = 0;      R_d[0][2] = (double)sin(eul[1]);
            R_d[1][0] = 0;              R_d[1][1] = 1;      R_d[1][2] = 0;
            R_d[2][0] = -(double)sin(eul[1]);   R_d[2][1] = 0;      R_d[2][2] = (double)cos(eul[1]);
            f_thrust = 0;

          }
          else if (type == 4) // attitude rate control
          {
            InvalidCommand = 0;

            omega_d[0] = (double)setpoint->attitudeRate.roll;
            omega_d[1] = (double)setpoint->attitudeRate.pitch;
            omega_d[2] = (double)setpoint->attitudeRate.yaw;
            //omega_d[0]=0.0;omega_d[1]=2.0;omega_d[2]=0.0;
            //std::cout<<"Omega command ["<< omega_d[0]<<", "<< omega_d[1]<<", "<< omega_d[2]<<"]"<<std::endl;
            quat2rotm_Rodrigue((double *) R, orientation_q);
            R_d[0][0] = R[0][0];    R_d[0][1] = 0;    R_d[0][2] = R[0][2];
            R_d[1][0] = R[1][0];    R_d[1][1] = 1;    R_d[1][2] = R[1][2];
            R_d[2][0] = R[2][0];    R_d[2][1] = 0;    R_d[2][2] = R[2][2];
            //myMemCpy(R_d, R, sizeof(R));
            f_thrust = 0;

          }
          else {
             InvalidCommand = 1;
          }

        if (!InvalidCommand){
          matTranspose((double *) tmp1,(double *) R_d, 3);
          matTranspose((double *) tmp2,(double *) R, 3);
          matTimesMat((double *) tmp3,(double *) tmp1,(double *) R);
          matTimesMat((double *) tmp4,(double *) tmp2,(double *) R_d);      // R' * R_d
          matAddsMat((double *) tmp5,(double *) tmp3,(double *) tmp4, 3*3, 2);
          matTimesScalar((double *) tmp6,(double *) tmp5, 0.5, 3*3, 1);
          dehat(e_R,(double *) tmp6);
          matTimesVec(tmp14, (double *) tmp4, omega_d, 3);          // R' * R_d * omega_d
          if (type==4){
              matAddsMat(e_omega, omega, tmp14, 3, 2);                  // omega - R'*R_d*omega_d
          }
          else
          {
              //e_omega[0]=0;   e_omega[1]=0;   e_omega[2]=0;
              myMemCpy(e_omega, omega, sizeof(omega));
          }
          /*if ( (type==4) && (k_run%10 == 1))
              std::cout<<"e_omega ["<< e_omega[0]<<", "<< e_omega[1]<<", "<< e_omega[2]<<"]"<<std::endl;*/
          //myMemCpy(e_omega, omega, sizeof(omega));                 // e_omega = omega - R'*R_d*omega_d
          matTimesScalar(tmp8, e_R, -kp_R, 3, 1);               // -kp_R * e_R
          if (type==4){
              matTimesScalar(tmp9, e_omega, -kd_R, 3, 1);           // -kd_R * e_omega
            }
          else {
              matTimesScalar(tmp9, e_omega, -kd_R2, 3, 1);
            }
          matTimesVec(tmp10, (double *) J, omega, 3);           // J * omega
          hat((double *) tmp11, omega);                         // omega_hat
          matTimesVec(tmp12, (double *) tmp11, tmp10, 3);       // omega x J*omega
          matAddsMat(tmp13, tmp8, tmp9, 3, 1);
          matAddsMat(tau, tmp13, tmp12, 3, 1);


          FT[0] = f_thrust;
          FT[1] = tau[0];
          FT[2] = tau[1];
          FT[3] = tau[2];
          f1 = (float)f_thrust;
          tau1 = (float)tau[0];
          tau2 = (float)tau[1];
          tau3 = (float)tau[2];
          matTimesVec(f, (double *) Gamma_inv, FT, 4);
          matTimesScalar(motorspeed_square, f, c_T, 4, 2);
          for(int k_ms_s=0;k_ms_s<4;k_ms_s++)
          {
              if(motorspeed_square[k_ms_s]<0)
                  motorspeed_square[k_ms_s] = 0;
          }
          if(type == 3 || type == 4)
          {
              motorspeed_square[0]=(motorspeed_square[0]+motorspeed_square[2])/2;     motorspeed_square[2]=motorspeed_square[0];

              if(R[2][2]<0)
              {
                  /*if (k_run%100 == 1)
                      std::cout<<"Shutdown motors"<<std::endl;*/
                  motorspeed_square[0] = 0;   motorspeed_square[1] = 0;   motorspeed_square[2] = 0;   motorspeed_square[3] = 0;
              }

          }

            // From System Identification Paper
            motorspeed[0] = (float)(((double)sqrt(motorspeed_square[0]) - 380.8359)/0.04076521);
            motorspeed[1] = (float)(((double)sqrt(motorspeed_square[1]) - 380.8359)/0.04076521);
            motorspeed[2] = (float)(((double)sqrt(motorspeed_square[2]) - 380.8359)/0.04076521);
            motorspeed[3] = (float)(((double)sqrt(motorspeed_square[3]) - 380.8359)/0.04076521);


          }
          else {
            motorspeed[0] = 0.0f;
            motorspeed[1] = 0.0f;
            motorspeed[2] = 0.0f;
            motorspeed[3] = 0.0f;
          }
          // Actually for m1/m2/m3/m4 -> must change usage paramter in power_distribution.c to work properly

          for (int i =0; i<4; i++){
            if (motorspeed[i] > 65535){
              motorspeed[i] = 65535.0f;
            }
            else if (motorspeed[i] < 0) {
              motorspeed[i] = 0.0f;
            }
          }
          m1 = (int16_t)(motorspeed[0]/2);
          m2 = (int16_t)(motorspeed[1]/2);
          m3 = (int16_t)(motorspeed[2]/2);
          m4 = (int16_t)(motorspeed[3]/2);
          // divide by two for 16 bit precsion -> rescale in power distribution

          /*control->thrust = m1;
          control->roll = m2;
          control->pitch = m3;
          control->yaw = m4;*/



          // Change to set *control commands

}
control->thrust = m1;//floor(motorspeed[0]);
control->roll = m2;//(int16_t)floor(motorspeed[1]);
control->pitch = m3;//(int16_t)floor(motorspeed[2]);
control->yaw = m4;//(int16_t)floor(motorspeed[3]);
}



LOG_GROUP_START(GtcState)
LOG_ADD(LOG_FLOAT,vx, &vx)
LOG_ADD(LOG_FLOAT,vy, &vy)
LOG_ADD(LOG_FLOAT,vz, &vz)
LOG_ADD(LOG_FLOAT,x, &x)//position[0])
LOG_ADD(LOG_FLOAT,y, &y)//position[1])
LOG_ADD(LOG_FLOAT,z, &z)//position[2])
LOG_ADD(LOG_FLOAT,q1, &q1)
LOG_ADD(LOG_FLOAT,q2, &q2)
LOG_ADD(LOG_FLOAT,q3, &q3)
LOG_ADD(LOG_FLOAT,q4, &q4)
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

LOG_GROUP_START(GtcMot)
LOG_ADD(LOG_FLOAT,m1, &motorspeed[0])
LOG_ADD(LOG_FLOAT,m2, &motorspeed[1])
LOG_ADD(LOG_FLOAT,m3, &motorspeed[2])
LOG_ADD(LOG_FLOAT,m4, &motorspeed[3])
LOG_GROUP_STOP(GtcMot)

LOG_GROUP_START(GtcGain)
LOG_ADD(LOG_FLOAT, kp_v, &kp_v)
LOG_ADD(LOG_FLOAT, kp_R, &kp_R)
LOG_ADD(LOG_FLOAT, kd_R, &kd_R)
LOG_ADD(LOG_FLOAT, kd_R2, &kd_R2)
LOG_GROUP_STOP(GtcGain)

PARAM_GROUP_START(GtcGain)
PARAM_ADD(PARAM_FLOAT, kp_v, &kp_v)
PARAM_ADD(PARAM_FLOAT, kp_R, &kp_R)
PARAM_ADD(PARAM_FLOAT, kd_R, &kd_R)
PARAM_ADD(PARAM_FLOAT, kd_R2, &kd_R2)
PARAM_GROUP_STOP(GtcGain)

/*
kp_v = 3;
kp_R = 1e-5;
kd_R = 5e-4;
kd_R2 = 1e-6; //0.000001
*/
