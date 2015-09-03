/*! \file
 *  kalman.h
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

/*! This library implements a basic Kalman filter which utilizes only scalar input
 *  and process models.
 *  The mathematical model of the filter and terminology used is based on the resource
 *  available here:
 *  http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
 *
 */

#pragma once

#include <inttypes.h>
#include <avr/io.h>



/***            Public Variables            ***/

/* See http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information about the terminology
 * used here.
 * The terms below are values carried over from the previous iteration of the filter.
 */
/*! The Kalman filter state estimate values */
struct kf_state_s {
    double estimate;        // the state estimate
    double variance;        // the state estimate variance
};

/*! The Kalman Filter control values */
struct kf_control_s {
    double u;               // the KF control input U
    double q;               // the KF process variance Q
};

/* The Kalman Filter measurement values */
struct kf_measurement_s {
    double z;               // the KF input measurement Z
    double r;               // the KF measurement variance R
};


/***            Public Functions            ***/

void kfInit(struct kf_state_s *state, double *initial_estimate, double *initial_variance);
double kfUpdate(struct kf_state_s *state, struct kf_control_s *c, struct kf_measurement_s *m);

