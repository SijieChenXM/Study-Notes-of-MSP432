/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file radio_xn297.c
 *
 * @author
 */

#include "radio.h"
#include "xn297.h"
#include "topics.h"
#include "param.h"
#include "flash_rom.h"

extern uint64_t sys_time(void);
extern void usleep(uint32_t us);

/*
≤‚ ‘ø…”√ 20170605
*/
/*
    key8            key9
        key0    key4
        key1    key5
        key2    key6
        key3    key7  
*/

static uint8_t _inited = 0;
static uint8_t _ready = 0;

static int _check = 0;

static manual_t _manual;

uint8_t read_code()
{
    uint8_t valid = 0;
    uint8_t channel[1];
    uint8_t address[5];

    uint32_t v;

    param_get(param_find("LINK_VALID"), &v);
    valid = (uint8_t)v;

    param_get(param_find("LINK_CHANNEL"), &v);
    channel[0] = v;

    param_get(param_find("LINK_ADDRESS0"), &v);
    address[0] =  v;

    param_get(param_find("LINK_ADDRESS1"), &v);
    address[1] = v;

    param_get(param_find("LINK_ADDRESS2"), &v);
    address[2] = v;

    param_get(param_find("LINK_ADDRESS3"), &v);
    address[3] = v;

    param_get(param_find("LINK_ADDRESS4"), &v);
    address[4] = v;

    if(!valid) {
        return 0;
    }

    xn297_set_channel(channel);
    xn297_set_address(address);

    return 1;
}

int coding()
{
    int ret = -1;

    uint8_t read_buffer[12] = { 0 };

    uint8_t channel[1] = { 0 };
    uint8_t address[5] = { 0 };

    xn297_read(read_buffer, sizeof(read_buffer));     

    if (read_buffer[0] == 0xFE) {
        channel[0] = read_buffer[1];    // channel
        address[0] = read_buffer[2];    // address
        address[1] = read_buffer[3];
        address[2] = read_buffer[4];
        address[3] = read_buffer[5];
        address[4] = read_buffer[6];

        uint8_t check = (channel[0] + address[0] + address[1] + address[2] + address[3] + address[4]);

        if (read_buffer[7] == check) {

            xn297_set_channel(channel);
            xn297_set_address(address);

            //  save coding to flash
            uint32_t v;

            v = 1;
            param_set(param_find("LINK_VALID"), &v);

            v = channel[0];
            param_set(param_find("LINK_CHANNEL"), &v);

            v = address[0];
            param_set(param_find("LINK_ADDRESS0"), &v);

            v = address[1];
            param_set(param_find("LINK_ADDRESS1"), &v);

            v = address[2];
            param_set(param_find("LINK_ADDRESS2"), &v);

            v = address[3];
            param_set(param_find("LINK_ADDRESS3"), &v);

            v = address[4];
            param_set(param_find("LINK_ADDRESS4"), &v);

            flash_save();

            ret = 0;
        }
    }

    return ret;
}

static inline float constrain(float val, float min, float max)
{
    return val < min ? min : val > max ? max : val;
}

void radio_xn297_task()
{
    static uint64_t t_prev = 0;

    uint64_t t = sys_time();

    int check = param_check();

    if (_check != check) {
        _check = check;

        uint32_t valid = 0;

        param_get(param_find("LINK_VALID"), &valid);

        if (!valid) {
            _inited = 0;
            _ready = 0;
        }
    }

    if (!_inited) {
        xn297_config();
        _inited = 1;
    }

    if (!_ready) {
        //  read coding from flash
        if (!read_code()) {
            if (coding() == 0) {
                _ready = 1;
            }

        } else {
            _ready = 1;
        }

        return;
    }

    if (t < t_prev + 1e4) {
        return;
    }

    t_prev = t;

    uint8_t read_buffer[12] = { 0 };

    int ret = xn297_read(read_buffer, sizeof(read_buffer));

    if (ret == 0) {
        uint16_t check = 0;

        for (int i = 0; i < 10; i++) {
            check += read_buffer[i];
        }

        if (check == (uint16_t)(read_buffer[10] << 8 | read_buffer[11])) {

            _manual.timestamp = t;

            _manual.x = (float) (((read_buffer[0] << 8)|read_buffer[1]) - 500) / 500.0f;    // roll control     -1..1
            _manual.y = (float) (((read_buffer[2] << 8)|read_buffer[3]) - 500) / 500.0f;    // pitch control    -1..1
            _manual.z = (float) ((read_buffer[4] << 8)|read_buffer[5]) / 1000.0f;           // throttle control  0..1
            _manual.r = (float) (((read_buffer[6] << 8)|read_buffer[7]) - 500) / 500.0f;    // yaw control      -1..1

            _manual.y = constrain(_manual.y, -1.0f, 1.0f);
            _manual.x = constrain(_manual.x, -1.0f, 1.0f);
            _manual.z = constrain(_manual.z, 0.0f, 1.0f);
            _manual.r = constrain(_manual.r, -1.0f, 1.0f);

            _manual.aux[1] = (uint16_t)read_buffer[8];
            _manual.aux[2] = (uint16_t)read_buffer[9];
        }
    }

    // [20ms, 200ms] ~ [1, 0]
    float rssi = constrain((1.0f - (t - _manual.timestamp) / 200000.0f + 20000.0f / 200000.0f), 0.0f, 1.0f);

    _manual.rssi = (uint8_t)(rssi * 100.0f);

    publish(publish_handle("manual_0"), &_manual);
}
