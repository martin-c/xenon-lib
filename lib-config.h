/*! \file
 *  lib-config.h
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Martin Clemons
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

/*! xenon-lib configuration file.
 *  The various library-wide configuration options are defined in this file.
 *  Edit this file as necessary to configure your project.
 */
#pragma once

/*  ###     Timer Library Configuration     ###
 */
/*  Enable RTC as a timer source in timer library (timer.c).
 *  By enabling the RTC in timer library the rtc-hardware library is no longer available,
 *  as each of these libraries depends on the RTC overflow interrupt, and only one
 *  interrupt handler can be defined at once.
 */
//#define XENON_RTC_AS_TIMER_SOURCE   1

/*! Timer hardware peripheral which runs all software timers.
 *  Needs to be defined in advance so library can assign an
 *  interrupt handler.
 */
#define XENON_TIMER_TC              TCC0
#define XENON_TIMER_TC_ISR          TCC0_OVF_vect

