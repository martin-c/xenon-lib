/*! \file
 *  task-scheduler.h
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

#pragma once

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "embedded-device.h"



/***                Definitions                 ***/

/*! Since the allocation of memory for tasks is static, we need to know the maximum number of simultaneous
 *  tasks to allocate memory for.
 */
#define TS_MAX_TASKS        64



/***          Public Global Variables           ***/

/*  A task data structure
 */
typedef struct tsTask_s task_t;

/*  A task function pointer
 *  A pointer to a function returning void with a single device_t pointer argument.
 */
typedef void (*task_fp)(device_t *);



/***             Public Functions               ***/

task_t *tsNewTimedTask(void (*task)(device_t *), int16_t period);
task_t *tsNewTimedSingleShotTask(void (*task)(device_t *), int16_t period);
task_t *tsEnqueueTask(void (*task)(device_t *));
task_t *tsNewConditionalTask(void (*task)(device_t *), uint8_t (*cb)(device_t *));
task_t *tsNewConditionalSingleShotTask(void (*task)(device_t *), uint8_t (*cb)(device_t *));
void tsReleaseTask(task_t *t);
void tsMain(device_t *dev);