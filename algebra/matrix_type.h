/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file matrix_type.h
 */

#pragma once

typedef struct {
    float data[3];
} Vector3;

typedef struct {
    float data[4];
} Quaternion;

typedef struct {
    float data[3][3];
} Matrix3;
