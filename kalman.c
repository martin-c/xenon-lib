/*! \file
 *  kalman.c
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Martin Clemons
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "kalman.h"



/***                Definitions                 ***/
/*! \privatesection */



/***         Private Global Variables           ***/



/***          Public Global Variables           ***/
/*! \publicsection */



/***        Private Function Prototypes         ***/
/*! \privatesection */



/***                   ISRs                     ***/



/***             Private Functions              ***/



/***             Public Functions               ***/
/*! \publicsection */

/*! Initialize Kalman Filter state estimate and state variance
 *  \param initial_estimate The initial estimate of the process state.
 *  \param initial_variance The initial variance of the state estimate. It is best to not
 *  set this value to zero, as a non-zero variance will allow the filter to converge more
 *  quickly. See paper cited above for more information.
 */
void kfInit(struct kf_state_s *state, double *initial_estimate, double *initial_variance)
{
    state->estimate = *initial_estimate;
    state->variance = *initial_variance;
}

/* Kalman Filter update step.
 * This steps projects the state ahead and projects the error covariance ahead, then
 * corrects the state projection with the measurement, and finally corrects the
 * estimate variance.
 * \param state Pointer to struct with KF state estimate values.
 * \param c Pointer to struct with KF control values (U, Q).
 * \param m Pointer to struct with KF measurement values (Z, R).
 * \return function returns the computed Kalman Gain (K).
 */
double kfUpdate(struct kf_state_s *state, struct kf_control_s *c, struct kf_measurement_s *m)
{
    // project ahead
    double state_priori = state->estimate + c->u;
    double variance_priori = state->variance + c->q;
    // compute kalman gain
    double k = variance_priori / (variance_priori + m->r);
    // update state and variance
    state->estimate = state_priori + k * (m->z - state_priori);
    state->variance = (1 - k) * variance_priori;
    return k;
}

