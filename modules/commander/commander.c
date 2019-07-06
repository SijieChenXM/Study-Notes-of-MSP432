/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file commander.c
 */

#include "commander.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "scheduler.h"
#include "topics.h"
#include "param.h"
#include "low_pass_filter.h"
#include "led.h"
#include "adc.h"
#include "flash.h"

#ifndef M_ONE_G
#define M_ONE_G 9.80665f
#endif

static uint8_t _require_shutdown;
static uint8_t _calibrate_imu;

float _acc_filter;
float _battery_filter;

// mode: < -1 | -1 | 0           | 1          | > 1
// leds: off  | on | quick flash | slow flash | flash n times
void led_indicate(int mode)
{
    static int times = 0;
    static uint64_t t_start = 0;
    static uint64_t t_prev = 0;

    uint64_t t = time();

    uint64_t t_on, t_off, t_delay;

    char path[3] = "led";
    uint8_t led[4];

    if (times != mode) {
        times = mode;
        t_start = t;
        t_prev = t;
    }

    if (times < -1) {
        t_on = 0;
        t_off = 0;
        t_delay = UINT32_MAX;

    } else if (times < 0) {
        t_on = 0;
        t_off = 0;
        t_delay = 0;

    } else if (times < 2) {
        t_on = times ? 1.2e6 : 6e4;
        t_off = t_on;
        t_delay = t_off << 1;

    } else {
        t_on = 1.2e5;
        t_off = t_on * (times << 1);
        t_delay = t_off + 1.5e6;
    }

    if (t > t_start + t_delay) {
        // turn on
        led[0] = 1;
        led[1] = 1;
        led[2] = 1;
        led[3] = 1;

        write(path, led, sizeof(led));

        t_start = t;
        t_prev = t;

    } else if (t > t_start + t_off) {
        // turn off
        led[0] = 0;
        led[1] = 0;
        led[2] = 0;
        led[3] = 0;

        write(path, led, sizeof(led));

    } else if (t > t_prev + t_on) {
        // toggle
        led[0] = 2;
        led[1] = 2;
        led[2] = 2;
        led[3] = 2;

        write(path, led, sizeof(led));

        t_prev = t;
    }
}

static void calibrate()
{
    static int count = 0;
    static float accel_sample[3] = { 0.0f };
    static float gyro_sample[3] = { 0.0f };
    static time_t gyro_timestamp = 0;

    const int samples = 300;

    if (_calibrate_imu) {
        if (_gyro.timestamp > gyro_timestamp) {
            gyro_timestamp = _gyro.timestamp;

            if (!count) {
                gyro_sample[0] = _gyro.x;
                gyro_sample[1] = _gyro.y;
                gyro_sample[2] = _gyro.z;

                accel_sample[0] = _accel.x;
                accel_sample[1] = _accel.y;
                accel_sample[2] = _accel.z;

                count++;

            } else {

                gyro_sample[0] += _gyro.x;
                gyro_sample[1] += _gyro.y;
                gyro_sample[2] += _gyro.z;

                accel_sample[0] += _accel.x;
                accel_sample[1] += _accel.y;
                accel_sample[2] += _accel.z;

                count++;

                if (count == samples) {
                    for (int i = 0; i < 3; i++) {
                        gyro_sample[i] /= (float)samples;
                        accel_sample[i] /= (float)samples;
                    }

                    accel_sample[2] += M_ONE_G;

                    _params.gyro_offset[0] += gyro_sample[0];
                    _params.gyro_offset[1] += gyro_sample[1];
                    _params.gyro_offset[2] += gyro_sample[2];

                    _params.accel_offset[0] += accel_sample[0];
                    _params.accel_offset[1] += accel_sample[1];
                    _params.accel_offset[2] += accel_sample[2];

                    count = 0;

                    _calibrate_imu = 0;

                    // save parameters to flash
                    char path = 'f';
                    write(&path, NULL, 0);
                }
            }
        }
    }
}

static void commander()
{
    uint64_t t = time();

    char path = '\0';

    // power button
    // 
    // long press 2 seconds to shutdown
    // long press 4 seconds to re-link radio
    static uint64_t button_pressed_timestamp = 0;

    path = 'b';

    uint8_t button;
    read(&path, &button, 1);

    if (!button) {
        button_pressed_timestamp = t;

        if (!button && _require_shutdown) {
            uint8_t power = 0;
            write(&path, &power, 1);
        }

    } else {
        if (t > button_pressed_timestamp + 2e6) {
            _require_shutdown = 1;
        }
    }

    // armed-disarmed
    static uint64_t cmd_armed_timestamp = 0;

    if (_manual.aux[1] == 1 && t > cmd_armed_timestamp + 5e5) {
        cmd_armed_timestamp = t;
        _status.armed = !_status.armed;
    }

    // armed by rc
    //
    //       left    right
    //
    //
    //        O        O
    //         \
    //          \
    //
    static uint64_t arming_timestamp = 0;

    if (!_status.armed && _manual.z < 0.2f && _manual.r > 0.8f) {
        if (t > arming_timestamp + 500000) {
            // safeguard
            if (_attitude.timestamp > 0 && fabsf(_attitude.euler[0]) < 0.3f && fabsf(_attitude.euler[1]) < 0.3f &&
                fabsf(_attitude.rate[0]) < 0.3f && fabsf(_attitude.rate[1]) < 0.3f && fabsf(_attitude.rate[2]) < 0.3f &&
                _local_position.timestamp > 0 && fabsf(_local_position.vz) < 0.2f) {

                _status.armed = 1;
            }
        }

    } else {
        arming_timestamp = t;
    }

    // disarmed by rc
    //
    //       left    right
    //
    //
    //        O        O
    //       /
    //      /
    //
    static uint64_t disarming_timestamp = 0;

    if (_status.armed && _manual.z < 0.2f && _manual.r < -0.8f) {
        if (t > disarming_timestamp + 500000) {
            _status.armed = 0;
        }

    } else {
        disarming_timestamp = t;
    }

    // calibration imu
    //
    //       left    right
    //
    //
    //        O        O
    //       /        /
    //      /        /
    //
    static uint64_t calibrate_imu_timestamp = 0;

    if (!_status.armed && !_calibrate_imu
        && _manual.z < 0.2f && _manual.r < -0.8f && _manual.y < -0.8f && _manual.x < -0.8f) {
        if (t > calibrate_imu_timestamp + 500000) {
            _calibrate_imu = 1;
        }

    } else {
        calibrate_imu_timestamp = t;
    }

    calibrate();

    // landed detect
    static uint64_t landed_timestamp;

    float a_b[3] = { _accel.x, _accel.y, _accel.z };
    float R[3][3];

    memcpy(R, _attitude.R, sizeof(R));

    float a_e[3];

    for (int i = 0; i < 3; i++) {
        a_e[i] = 0.0f;

        for (int j = 0; j < 3; j++) {
            a_e[i] += R[i][j] * a_b[j];
        }
    }
    a_e[2] += M_ONE_G;

    _acc_filter += 0.018f * (sqrtf(a_e[0] * a_e[0] + a_e[1] * a_e[1] + a_e[2] * a_e[2]) - _acc_filter); // dt = 0.03s fp = 1.0

    if (!_status.armed) {
        _status.landed = 1;
        _acc_filter = 0.0f;

    } else if (_status.nav_state == NAV_STATE_STAB) {
        _status.landed = 0;

    } else if (_manual.z > 0.6f) {
        _status.landed = 0;

    } else if (_actuator_controls.control[3] < 0.2f && _acc_filter < 1.0f) {
        if (t > landed_timestamp + 500000) {
            _status.landed = 1;
        }

    } else {
        landed_timestamp = t;
    }

    // baterry detect
    path = 'a';

    union {
        uint8_t c[4];
        float f;
    } adc;

    if (read(&path, adc.c, 4) != -1) {
        float voltage = adc.f;

        if (_status.voltage < 1.0f) {
            _battery_filter = voltage;

        } else {
            _battery_filter += 0.01f * (voltage - _battery_filter); // dt = 0.03s fp = 0.5
        }

        _status.voltage = _battery_filter;
    }

    // mode switch
    if (_manual.aux[0] > 800 && _manual.aux[0] < 1200) {
        if (_local_position.v_xy_valid) {
            if (fabsf(_manual.x) < 0.1f && fabsf(_manual.y) < 0.1f && fabsf(_manual.r) < 0.1f) {
                _status.nav_state = NAV_STATE_POSCTL;

            } else {
                _status.nav_state = NAV_STATE_ALTCTL;
            }

        } else if (_local_position.v_z_valid) {
            _status.nav_state = NAV_STATE_ALTCTL;

        } else {
            _status.nav_state = NAV_STATE_STAB;
        }

    } else if (_manual.aux[0] > 1300 && _manual.aux[0] < 1700) {
        if (_local_position.v_z_valid) {
            _status.nav_state = NAV_STATE_ALTCTL;

        } else {
            _status.nav_state = NAV_STATE_STAB;
        }

    } else if (_manual.aux[0] > 1800 && _manual.aux[0] < 2200) {
        _status.nav_state = NAV_STATE_STAB;
    }

    // publish status
    _status.timestamp = t;

    // control mode
    _control_mode.manual_enabled = 1;
    _control_mode.rates_enabled = 1;
    _control_mode.attitude_enabled = 1;

    if (_status.nav_state < NAV_STATE_ALTCTL) {
        _control_mode.climb_rate_enabled = 0;
        _control_mode.altitude_enabled = 0;

    } else {
        _control_mode.climb_rate_enabled = 1;
        _control_mode.altitude_enabled = 1;
    }

    if (_status.nav_state == NAV_STATE_POSCTL) {
        _control_mode.position_enabled = 1;
        _control_mode.velocity_enabled = 1;

    } else {
        _control_mode.position_enabled = 0;
        _control_mode.velocity_enabled = 0;
    }

    // publish control mode
    _control_mode.timestamp = t;

    // led indicate
    if (_require_shutdown) {
        led_indicate(-2);

    } else if (t > _manual.timestamp + 2e5) {
        led_indicate(0);

    } else if (_calibrate_imu) {
        led_indicate(0);

    } else if (!_status.armed) {
        led_indicate(1);

    } else {
        led_indicate(2);
    }
}

void commander_task()
{
    commander();
}
