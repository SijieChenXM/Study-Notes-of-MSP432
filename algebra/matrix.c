/***************************************************************************
 *
 *  Copyright (c) 2018 Robotics & Automation Works. All rights reserved.
 *
 *  www.raaworks.com
 ***************************************************************************/

/*
 * @file matrix.c
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "matrix.h"

static inline void swap_rows(float *A, size_t a, size_t b, size_t M)
{
	if (a == b) {
		return;
	}

	size_t aM = a * M;
	size_t bM = b * M;

	for (size_t j = 0; j < M; j++) {
		float tmp = A[aM + j];
		A[aM + j] = A[bM + j];
		A[bM + j] = tmp;
	}
}

static inline void swap_cols(float *A, size_t a, size_t b, size_t M)
{
	if (a == b) {
		return;
	}

	size_t iM;

	for (size_t i = 0; i < M; i++) {

		iM = i * M;

		float tmp = A[iM + a];
		A[iM + a] = A[iM + b];
		A[iM + b] = tmp;
	}
}

bool matrix_inverse(float *A, float *Inv, size_t M)
{
	float *L = (float*)malloc(sizeof(float) * M * M);
	float *U = (float*)malloc(sizeof(float) * M * M);
	float *P = (float*)malloc(sizeof(float) * M * M);

//    memset(Inv, 0, sizeof(float) * M * M);

	if (!L || !U || !P) {
        // free memory
        free(L);
        free(U);
        free(P);
		return false;
	}

	memset(L, 0, sizeof(float) * M * M);
	memset(U, 0, sizeof(float) * M * M);
	memset(P, 0, sizeof(float) * M * M);

	for (size_t i = 0; i < M; i++) {

		for (size_t j = 0; j < M; j++) {
			U[i * M + j] = A[i * M + j];
		}

		L[i * M + i] = 1.0f;
		P[i * M + i] = 1.0f;
	}

	// for all diagonal elements
	for (size_t n = 0; n < M; n++) {

		// if diagonal is zero, swap with row below
		if (fabsf(U[n * M + n]) < 1e-8f) {
			for (size_t i = n + 1; i < M; i++) {
				if (fabsf(U[i * M + n]) > 1e-8f) {
					swap_rows(U, i, n, M);
					swap_rows(P, i, n, M);
					swap_rows(L, i, n, M);
					swap_cols(L, i, n, M);
					break;
				}
			}
		}

		// failsafe, return zero matrix
		if (fabsf(U[n * M + n]) < 1e-8f) {
            // free memory
            free(L);
            free(U);
            free(P);
			return false;
		}

		// for all rows below diagonal
		for (size_t i = n + 1; i < M; i++) {
			L[i * M + n] = U[i * M + n] / U[n * M + n];

			// add i-th row and n-th row
			// multiplied by: -a(i,n)/a(n,n)
			for (size_t k = n; k < M; k++) {
				U[i * M + k] -= L[i * M + n] * U[n * M + k];
			}
		}
	}

	// solve LY=P*I for Y by forward subst

	// for all columns of Y
	for (size_t c = 0; c < M; c++) {
		// for all rows of L
		for (size_t i = 0; i < M; i++) {
			// for all columns of L
			for (size_t j = 0; j < i; j++) {
				// for all existing y
				// subtract the component they
				// constibute to the solution
				P[i * M + c] -= L[i * M + j] * P[j * M + c];
			}
		}
	}

	// for all columns of X
	for (size_t c = 0; c < M; c++) {
		// for all rows of U
		for (size_t k = 0; k < M; k++) {
			// have to go in reverse other
			size_t i = M - 1 - k;

			// for all columns of U
			for (size_t j = i + 1; j < M; j++) {
				// for all existing x
				// subtract the component they
				// contribute to the solution
				P[i * M + c] -= U[i * M + j] * P[j * M + c];
			}

			// divide by the factor
			// on current
			// term to be solved
			//
			// we known that U(i, i) != 0 from above
			P[i * M + c] /= U[i * M + i];
		}
	}

	// check sanity of results
	for (size_t i = 0; i < M; i++) {
		for (size_t j = 0; j < M; j++) {
			if (!isfinite(P[i * M + j])) {
                // free memory
                free(L);
                free(U);
                free(P);
				return false;
			}
		}
	}

	memcpy(Inv, P, sizeof(float) * M * M);

	// free memory
	free(L);
	free(U);
	free(P);

	return true;
}

bool matrix_inverse4x4(float m[], float invOut[])
{
    float inv[16], det;
    uint8_t i;

    inv[0] = m[5]  * m[10] * m[15] -
            m[5]  * m[11] * m[14] -
            m[9]  * m[6]  * m[15] +
            m[9]  * m[7]  * m[14] +
            m[13] * m[6]  * m[11] -
            m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
            m[4]  * m[11] * m[14] +
            m[8]  * m[6]  * m[15] -
            m[8]  * m[7]  * m[14] -
            m[12] * m[6]  * m[11] +
            m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
            m[4]  * m[11] * m[13] -
            m[8]  * m[5] * m[15] +
            m[8]  * m[7] * m[13] +
            m[12] * m[5] * m[11] -
            m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
            m[4]  * m[10] * m[13] +
            m[8]  * m[5] * m[14] -
            m[8]  * m[6] * m[13] -
            m[12] * m[5] * m[10] +
            m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
            m[1]  * m[11] * m[14] +
            m[9]  * m[2] * m[15] -
            m[9]  * m[3] * m[14] -
            m[13] * m[2] * m[11] +
            m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
            m[0]  * m[11] * m[14] -
            m[8]  * m[2] * m[15] +
            m[8]  * m[3] * m[14] +
            m[12] * m[2] * m[11] -
            m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
            m[0]  * m[11] * m[13] +
            m[8]  * m[1] * m[15] -
            m[8]  * m[3] * m[13] -
            m[12] * m[1] * m[11] +
            m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
            m[0]  * m[10] * m[13] -
            m[8]  * m[1] * m[14] +
            m[8]  * m[2] * m[13] +
            m[12] * m[1] * m[10] -
            m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
            m[1]  * m[7] * m[14] -
            m[5]  * m[2] * m[15] +
            m[5]  * m[3] * m[14] +
            m[13] * m[2] * m[7] -
            m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
            m[0]  * m[7] * m[14] +
            m[4]  * m[2] * m[15] -
            m[4]  * m[3] * m[14] -
            m[12] * m[2] * m[7] +
            m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
            m[0]  * m[7] * m[13] -
            m[4]  * m[1] * m[15] +
            m[4]  * m[3] * m[13] +
            m[12] * m[1] * m[7] -
            m[12] * m[3] * m[5];

	inv[14] = -m[0]  * m[5] * m[14] +
            m[0]  * m[6] * m[13] +
            m[4]  * m[1] * m[14] -
            m[4]  * m[2] * m[13] -
            m[12] * m[1] * m[6] +
            m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
            m[1] * m[7] * m[10] +
            m[5] * m[2] * m[11] -
            m[5] * m[3] * m[10] -
            m[9] * m[2] * m[7] +
            m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
            m[0] * m[7] * m[10] -
            m[4] * m[2] * m[11] +
            m[4] * m[3] * m[10] +
            m[8] * m[2] * m[7] -
            m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
            m[0] * m[7] * m[9] +
            m[4] * m[1] * m[11] -
            m[4] * m[3] * m[9] -
            m[8] * m[1] * m[7] +
            m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
            m[0] * m[6] * m[9] -
            m[4] * m[1] * m[10] +
            m[4] * m[2] * m[9] +
            m[8] * m[1] * m[6] -
            m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (fabsf(det) < 1.1755e-38f) {
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++) {
        invOut[i] = inv[i] * det;
    }

    return true;
}


void vector_mult(float *v, float val)
{
}

void vector_cross(float *v1, float *v2, float *v3)
{
}

float vector_dot(float *v1, float *v2)
{
    return 0.0f;
}
