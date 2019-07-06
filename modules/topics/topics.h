/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file topics.h
 */

#pragma once

#include "topics_type.h"

#ifdef __cplusplus
extern "C" {
#endif

extern status_t _status;

extern manual_t _manual;
extern manual_t _manual_0, _manual_1;
extern accel_t _accel;
extern gyro_t _gyro;
extern mag_t _mag;
extern baro_t _baro;
extern sonar_t _sonar;
extern gps_t _gps;
extern optical_flow_t _optical_flow;
extern vision_t _vision;

extern attitude_t _attitude;
extern attitude_setpoint_t _attitude_setpoint;
extern local_position_t _local_position;
extern local_position_setpoint_t _local_position_setpoint;

extern control_mode_t _control_mode;
extern actuator_controls_t _actuator_controls;
extern actuator_output_t _actuator_output;
extern saturation_status_t _saturation_status;

#ifdef __cplusplus
}
#endif
