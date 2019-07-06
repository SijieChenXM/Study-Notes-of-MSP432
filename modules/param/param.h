/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file param.h
 */

#pragma once

#include <stdint.h>

typedef struct {

    // attitude control

    float att_p[3];

    float rate_p[3];
    float rate_i[3];
    float rate_d[3];
    float rate_ff[3];
    float rate_int_lim[3];
    float auto_rate_max[3];
    float mc_rate_max[3];

    // position control

    float thr_hover;
    float thr_min;
    float thr_max;

    float tilt_max_air;
    float tilt_max_land;

    float tko_speed;
    float land_speed;

    float pos_p[3];

    float vel_p[3];
    float vel_i[3];
    float vel_d[3];
    float vel_ff[3];
    float vel_i_max[3];
    float vel_hor_max;
    float vel_up_max;
    float vel_down_max;

    float dec_hor_max;
    float acc_hor_max;
    float acc_z_p;
    float acc_z_i;
    float acc_z_d;
    float acc_up_max;
    float acc_down_max;

    float man_vel_xy_max;
    float man_tilt_max;
    float man_yaw_max;

    float global_yaw_max;

    // sensor calibration

    float gyro_offset[3];
    float accel_offset[3];
    float mag_offset[3];
    float mag_scale[3];

    // radio coding

    uint32_t link_valid;
    uint32_t link_channel;
    uint32_t link_address[5];

} param_t;

#ifdef __cplusplus
extern "C"
{
#endif

extern param_t _params;
extern uint32_t param_count(void);

#ifdef __cplusplus
}
#endif 
