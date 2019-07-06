/***************************************************************************
 *
 *  Copyright (c) 2017 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file radio.c
 *
 * @author zwh <zwh@raaworks.com>
 */

#include <string.h>

#include "radio.h"
#include "topics.h"

extern uint64_t sys_time(void);

static int _manual_0_sub = -1;
static int _manual_1_sub = -1;

static int _manual_pub = -1;

static int _inited = 0;

static manual_t _manual_0;
static manual_t _manual_1;
manual_t _manual;

static void init()
{
    _manual_0_sub = subscribe_handle("manual_0");
    _manual_1_sub = subscribe_handle("manual_1");

    _manual_pub = publish_handle("manual");

    memset(&_manual_0, 0, sizeof(_manual_0));
    memset(&_manual_1, 0, sizeof(_manual_1));
    memset(&_manual, 0, sizeof(_manual));

    _inited = 1;
}

void radio_task()
{
    uint64_t t = sys_time();

    if (!_inited) {
        init();
    }

    if (check_update(_manual_0_sub)) {
        subscribe(_manual_0_sub, &_manual_0);
    }

    if (check_update(_manual_1_sub)) {
        subscribe(_manual_1_sub, &_manual_1);
    }

    // radio select
    if (t < (_manual_1.timestamp + 1e5) &&
       ((t < (_manual_0.timestamp + 1e5) && _manual_0.aux[1] > 800 && _manual_0.aux[1] < 1200) || t > (_manual_0.timestamp + 1e5))) {

        memcpy(&_manual, &_manual_1, sizeof(_manual));

    } else {
        memcpy(&_manual, &_manual_0, sizeof(_manual));
    }

    publish(_manual_pub, &_manual);
}
