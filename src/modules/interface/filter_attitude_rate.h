/**
 *
 * 
 * 
 */

#ifndef __FILTER_ATTITUDE_RATE_H__
#define __FILTER_ATTITUDE_RATE__

#include "stabilizer_types.h"

static float pitch_rate_raw[2] = {0,0};
static float pitch_rate_filter[2] = {0,0};


// try first order filter first 

static uint8_t pos_switch = 1;

// obtained from first order matlab butter function
// fs = 1000 - fc = 80
static float b_bw[2] = {0.0592,0.0592};
static float a_bw[2] = {1,-0.8816};


static inline void filter_attitudeRate(state_t *state,
                                       const sensorData_t *sensors,
                                       const uint32_t tick)
{
  if (RATE_DO_EXECUTE(RATE_MAIN_LOOP, tick)) // 1000 Hz
  {
    uint8_t posinv = pos_switch & 1;
    pitch_rate_raw[pos_switch] = sensors->gyro.y;
    pitch_rate_filter[pos_switch] = a_bw[1]*pitch_rate_filter[posinv] 
                                  - b_bw[0]*pitch_rate_raw[pos_switch]
                                  - b_bw[1]*pitch_rate_raw[posinv];
  
    state->attitudeRate.pitch = 50*pitch_rate_filter[pos_switch];
    pos_switch = posinv;
  }
}

// second order filter
static float pitch_rate_raw2[3] = {0,0,0};
static float pitch_rate_filter2[3] = {0,0,0};

static uint8_t pos_switch2 = 0;

// obtained from first order matlab butter function
// fs = 1000 - fc = 80
static float b_bw2[3] = {0.0036,0.0072,0.0036};
static float a_bw2[3] = {1,-1.8227,0.8372};
static inline void filter_attitudeRate2(state_t *state,
                                       const sensorData_t *sensors,
                                       const uint32_t tick)
     {
  if (RATE_DO_EXECUTE(RATE_MAIN_LOOP, tick)) // 1000 Hz
  {
    
    pitch_rate_raw2[pos_switch2] = sensors->gyro.y;
    pitch_rate_filter2[pos_switch2] = a_bw2[1]*pitch_rate_filter2[(pos_switch2+2)%3] 
                                    +a_bw2[2]*pitch_rate_filter2[(pos_switch2+1)%3] 
                                  - b_bw2[0]*pitch_rate_raw2[pos_switch2]
                                  - b_bw2[1]*pitch_rate_raw2[(pos_switch2+2)%3]
                                  - b_bw2[2]*pitch_rate_raw2[(pos_switch2+1)%3];
    state->attitudeRate.pitch = pitch_rate_filter2[pos_switch2];
    pos_switch2 = (pos_switch2 + 1)%3;
  }
}                                  

#endif //__FILTER_ATTITUDE_RATE__

// --    --     -- 