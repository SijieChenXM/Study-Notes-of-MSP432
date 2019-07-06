/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file attitude_estimator.c
 */

#include "attitude_estimator.h"

#include <math.h>
#include <float.h>
#include <string.h>
#include "matrix.h"

static const float accel_gain = 0.2f;
static const float gyro_bias_gain = 0.1f;

static float _q[4];
static float _gyro_bias[3];

static uint8_t _inited = 0;

static inline float vector_length(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/*
 * brief: initialize quaternion
 *
 */
static uint8_t init()
{
    _q[0] = 1.0f;
    _q[1] = _q[2] = _q[3] = 0.0f;

    _gyro_bias[0] = _gyro_bias[1] = _gyro_bias[2] = 0.0f;

    return 0;
}

/*
 * brief: attitude estimator base on quaternion
 *
 * @param dt   : delta time in seconds
 * @param accel: body acceleration in m/s^2
 * @param gyro : body rates in rad/s
 */
static void update(const float dt, const float accel[], const float gyro[])
{
    if (!_inited) {
        if (init() == 0) {
            _inited = 1;
        }

        return;
    }

    float corr[3] = { 0.0f };
    float spin_rate = vector_length(gyro);

    float length = 1.0f / sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float a[3] = {accel[0] * length, accel[1] * length, accel[2] * length};

    float b[3] = {
        2.0f * (_q[1] * _q[3] - _q[0] * _q[2]),
        2.0f * (_q[2] * _q[3] + _q[0] * _q[1]),
        _q[0] * _q[0] - _q[1] * _q[1] - _q[2] * _q[2] + _q[3] * _q[3]
    };

    float accel_corr[3] = {
        b[1] * a[2] - b[2] * a[1],
        b[2] * a[0] - b[0] * a[2],
        b[0] * a[1] - b[1] * a[0],
    };

    for (int i = 0; i < 3; i++) {
        corr[i] += accel_corr[i] * accel_gain;
    }

    for (int i = 0; i < 3; i++) {
        // gyro bias estimation
        if (spin_rate < 0.175f) {
            _gyro_bias[i] += corr[i] * gyro_bias_gain * dt;
            _gyro_bias[i] = _gyro_bias[i] < -0.05f ? -0.05f : _gyro_bias[i] > 0.05f ? 0.05f : _gyro_bias[i];
        }

        corr[i] += gyro[i] + _gyro_bias[i];
    }

    float q_prev[4] = { _q[0], _q[1], _q[2], _q[3] };

    _q[0] += (corr[0] * -q_prev[1] + corr[1] * -q_prev[2] + corr[2] * -q_prev[3]) * 0.5f * dt;
    _q[1] += (corr[0] *  q_prev[0] + corr[1] * -q_prev[3] + corr[2] *  q_prev[2]) * 0.5f * dt;
    _q[2] += (corr[0] *  q_prev[3] + corr[1] *  q_prev[0] + corr[2] * -q_prev[1]) * 0.5f * dt;
    _q[3] += (corr[0] * -q_prev[2] + corr[1] *  q_prev[1] + corr[2] *  q_prev[0]) * 0.5f * dt;

    length = 1.0f / sqrtf(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= length;
    _q[1] *= length;
    _q[2] *= length;
    _q[3] *= length;

    if (!isfinite(_q[0]) || !isfinite(_q[1]) || !isfinite(_q[2]) || !isfinite(_q[3])) {
        _q[0] = q_prev[0];
        _q[1] = q_prev[1];
        _q[2] = q_prev[2];
        _q[3] = q_prev[3];
        _gyro_bias[0] = _gyro_bias[1] = _gyro_bias[2] = 0.0f;
    }
}

// interface

#include "scheduler.h"
#include "topics.h"

static uint64_t t_prev = 0;
static uint64_t gyro_timestamp = 0;

void attitude_estimator()
{
    if (_gyro.timestamp > gyro_timestamp) {
        gyro_timestamp = _gyro.timestamp;

        uint64_t t = time();

        float dt = t_prev > 0 ? (t - t_prev) * 1e-6f : 1e-6f;
        dt = dt < 0.0001f ? 0.0001f : dt > 0.02f ? 0.02f : dt;

        t_prev = t;

        // body frame acceleration in m/s^s
        float a[3] = { _accel.x, _accel.y, _accel.z };

        // body frame angular speed in rad/s
        float g[3] = { _gyro.x, _gyro.y, _gyro.z };

        // update attitude by quaternion
        update(dt, a, g);

        // publish attitude
        _attitude.timestamp = t;

        // generate euler angles
        _attitude.euler[0] = atan2f(2.0f * (_q[0] * _q[1] + _q[2] * _q[3]), 1.0f - 2.0f * (_q[1] * _q[1] + _q[2] * _q[2]));
        _attitude.euler[1] = asinf(2.0f * (_q[0] * _q[2] - _q[3] * _q[1]));
        _attitude.euler[2] = atan2f(2.0f * (_q[0] * _q[3] + _q[1] * _q[2]), 1.0f - 2.0f * (_q[2] * _q[2] + _q[3] * _q[3]));

        // generate corrected angular rates
        _attitude.rate[0] = g[0] + _gyro_bias[0];
        _attitude.rate[1] = g[1] + _gyro_bias[1];
        _attitude.rate[2] = g[2] + _gyro_bias[2];

        // copy quaternion
        _attitude.q[0] = _q[0];
        _attitude.q[1] = _q[1];
        _attitude.q[2] = _q[2];
        _attitude.q[3] = _q[3];

        // generate rotation matrix from quaternion
        float sq[4] = { _q[0] * _q[0], _q[1] * _q[1], _q[2] * _q[2], _q[3] * _q[3] };
        _attitude.R[0] = sq[0] + sq[1] - sq[2] - sq[3];
        _attitude.R[1] = 2.0f * (_q[1] * _q[2] - _q[0] * _q[3]);
        _attitude.R[2] = 2.0f * (_q[0] * _q[2] + _q[1] * _q[3]);
        _attitude.R[3] = 2.0f * (_q[1] * _q[2] + _q[0] * _q[3]);
        _attitude.R[4] = sq[0] - sq[1] + sq[2] - sq[3];
        _attitude.R[5] = 2.0f * (_q[2] * _q[3] - _q[0] * _q[1]);
        _attitude.R[6] = 2.0f * (_q[1] * _q[3] - _q[0] * _q[2]);
        _attitude.R[7] = 2.0f * (_q[0] * _q[1] + _q[2] * _q[3]);
        _attitude.R[8] = sq[0] - sq[1] - sq[2] + sq[3];
    }
}
