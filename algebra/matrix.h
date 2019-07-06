/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file matrix.h
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "matrix_type.h"

#include "geo.h"

#ifdef __cplusplus
extern "C" {
#endif

bool matrix_inverse(float *A, float *Inv, size_t M);
bool matrix_inverse4x4(float m[], float invOut[]);

#ifdef __cplusplus
}
#endif
