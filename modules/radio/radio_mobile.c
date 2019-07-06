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

#include "topics.h"

extern uint64_t sys_time(void);

static int _manual_pub = -1;

static manual_t _manual;

static int _inited = 0;

static void init()
{
    _manual_pub = publish_handle("manual_1");

    memset(&_manual, 0, sizeof(_manual));

    _inited = 1;
}

void radio_ralink_task()
{
    uint64_t t = sys_time();

    if (!_inited) {
        init();
    }

    _manual.timestamp = t;

    publish(_manual_pub, &_manual);
}
