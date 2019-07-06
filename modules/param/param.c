/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file param.c
 */

#include <stdlib.h>
#include <string.h>

#include "param.h"

param_t _params = {

    // attitude control

    .att_p = { 4.5f, 4.5f, 4.5f },
    .rate_p = { 0.12f, 0.12f, 0.20f },
    .rate_i = { 0.05f, 0.05f, 0.10f },
    .rate_d = { 0.005f, 0.005f, 0.0f },
    .rate_ff = { 0.0f, 0.0f, 0.1f },
    .rate_int_lim = { 0.3f, 0.3f, 0.3f },
    .auto_rate_max = { 3.8f, 3.8f, 4.5f },
    .mc_rate_max = { 3.8f, 3.8f, 4.5f },

    // position control

    .thr_hover = 0.35f,
    .thr_min = 0.12f,
    .thr_max = 0.90f,

    .tilt_max_air = 0.78f,
    .tilt_max_land = 0.20f,

    .tko_speed = 1.0f,
    .land_speed = 0.5f,

    .pos_p = { 0.95f, 0.95f, 1.0f },
    .vel_p = { 0.12f, 0.12f, 0.2f },
    .vel_i = { 0.02f, 0.02f, 0.02f },
    .vel_d = { 0.01f, 0.01f, 0.0f },
    .vel_ff = { 0.5f, 0.5f, 0.5f },
    .vel_i_max = { 0.3f, 0.3f, 0.3f },
    .vel_hor_max = 2.0f,
    .vel_down_max = 1.5f,
    .vel_up_max = 1.5f,

    .dec_hor_max = 3.0f,
    .acc_hor_max = 5.0f,
    .acc_z_p = 3.5f,
    .acc_z_i = 0.012f,
    .acc_z_d = 0.0f,
    .acc_up_max = 3.0f,
    .acc_down_max = 2.0f,

    .man_vel_xy_max = 2.0f,
    .man_tilt_max = 0.6f,
    .man_yaw_max = 3.5f,

    .global_yaw_max = 3.5f,

    // sensor calibration

    .gyro_offset = { 0.0f, 0.0f, 0.0f },
    .accel_offset = { 0.0f, 0.0f, 0.0f },
    .mag_offset = {0.0f, 0.0f, 0.0f },
    .mag_scale = { 1.0f, 1.0f, 1.0f },

    // radio coding

    .link_channel = 0.0f,
    .link_address = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

};

uint32_t param_count()
{
    return sizeof(_params) / sizeof(float);
}
