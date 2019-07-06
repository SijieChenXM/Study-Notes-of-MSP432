/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file topics.c
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "topics.h"

struct topic_s {
    char *name;
    void *data;
    uint32_t size;

    uint8_t queue;
    uint8_t push;
    uint8_t pop;

    uint8_t subscriber;
    uint32_t update;

    struct topic_s *next;
};

struct topic_s *head = NULL;

int publish_handle(char *name)
{
    struct topic_s *p;

    int index = 0;

    for (p = head; p; p = p->next) {
        if (!strcmp(p->name, name)) {
            break;
        }

        index++;
    }

    return p ? index << 8 : -1;
}

int subscribe_handle(char *name)
{
    struct topic_s *p;

    int handle;
    int index = 0;

    for (p = head; p; p = p->next) {
        if (!strcmp(p->name, name)) {

            if (p->subscriber < sizeof(p->update) * 8) {
                handle = (index << 8) + (p->subscriber & 0x0f);
                p->subscriber++;

            } else {
                handle = -1;
            }

            break;
        }

        index++;
    }

    return p ? handle : -1;
}

int check_update(int handle)
{
	if (handle < 0) {
		return 0;
	}

	struct topic_s *p;

	uint32_t index = handle >> 8;

	for (p = head; p; p = p->next) {
		if (index > 0) {
			index--;

		} else {
			break;
		}
	}

	if (p && p->queue > 0
		&& (p->update & (1 << (handle & 0x0f)))) {

		return 1;

	} else {
		return 0;
	}
}

int publish(int handle, void *data)
{
    if (handle < 0) {
        return -1;
    }

    struct topic_s *p;

    uint32_t index = handle >> 8;

    for (p = head; p; p = p->next) {
        if (index == 0) {
            break;
        }

        index--;
    }

    if (p && p->queue > 0) {

        memcpy((uint8_t*)p->data + p->size * p->push, data, p->size);

        if ((p->push + 1) % p->queue == p->pop) {
            p->pop = (p->pop + 1) % p->queue;
        }

        p->push = (p->push + 1) % p->queue;

        p->update = ~0;

        return 0;

    } else {
        return -1;
    }
}

int subscribe(int handle, void *data)
{
    if (handle < 0) {
        return -1;
    }

    struct topic_s *p;

    uint32_t index = handle >> 8;

    for (p = head; p; p = p->next) {
        if (index > 0) {
            index--;

        } else {
            break;
        }
    }

    if (p && p->queue > 0) {

        memcpy(data, (uint8_t*)p->data + p->size * p->pop, p->size);

        if ((p->pop + 1) % p->queue == p->push) {
            p->update &= ~(1 << (handle & 0x0f));

        } else {
            p->pop = (p->pop + 1) % p->queue;
        }

        return 0;

    } else {
        return -1;
    }
}

static int topic_register(char *name, uint32_t size, uint8_t queue)
{
    if (!queue) {
        return -1;
    }

    struct topic_s *s = (struct topic_s*)malloc(sizeof(struct topic_s));

    void *data = (uint8_t*)malloc(size * queue);

    if (s && data) {

        memset(data, 0, size * queue);

        s->name = name;
        s->data = data;
        s->size = size;

        s->queue = queue;
        s->push = 0;
        s->pop = queue - 1;

        s->subscriber = 0;
        s->update = 0;

        s->next = NULL;

        if (!head) {
            head = s;

        } else {
            struct topic_s *p;

            for (p = head; p->next; p = p->next) {}

            p->next = s;
        }

        return 0;

    } else {
        return -1;
    }
}

#define TOPIC_DEFINE(_name)                         topic_register(#_name, sizeof(_name##_t), 1);
#define TOPIC_DEFINE_QUEUE(_name, _queue_length)    topic_register(#_name, sizeof(_name##_t), _queue_length);

// creat topics
int topics_config(void)
{
    int ret = 0;

    ret += TOPIC_DEFINE(status);

    ret += TOPIC_DEFINE(manual);
    ret += topic_register("manual_0", sizeof(manual_t), 1);
    ret += topic_register("manual_1", sizeof(manual_t), 1);

    ret += TOPIC_DEFINE(accel);

    ret += TOPIC_DEFINE(gyro);
    ret += TOPIC_DEFINE(mag);
    ret += TOPIC_DEFINE(baro);
    ret += TOPIC_DEFINE(sonar);
    ret += TOPIC_DEFINE(gps);
    ret += TOPIC_DEFINE(optical_flow);

    ret += TOPIC_DEFINE(attitude);
    ret += TOPIC_DEFINE(attitude_setpoint);
    ret += TOPIC_DEFINE(local_position);
    ret += TOPIC_DEFINE(local_position_setpoint);

    ret += TOPIC_DEFINE(control_mode);
    ret += TOPIC_DEFINE(actuator_controls);
    ret += TOPIC_DEFINE(actuator_output);
    ret += TOPIC_DEFINE(saturation_status);

    ret += TOPIC_DEFINE(vision);

    return ret;
}

status_t _status;

manual_t _manual;
manual_t _manual_0, _manual_1;
accel_t _accel;
gyro_t _gyro;
mag_t _mag;
baro_t _baro;
sonar_t _sonar;
gps_t _gps;
optical_flow_t _optical_flow;
vision_t _vision;

attitude_t _attitude;
attitude_setpoint_t _attitude_setpoint;
local_position_t _local_position;
local_position_setpoint_t _local_position_setpoint;

control_mode_t _control_mode;
actuator_controls_t _actuator_controls;
actuator_output_t _actuator_output;
saturation_status_t _saturation_status;
