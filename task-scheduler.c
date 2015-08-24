/*! \file
 *  task-scheduler.c
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

#include "task-scheduler.h"
#include <stdio.h>
#include <util/atomic.h>
#include "timer.h"
#include "debug.h"



/***                Definitions                 ***/



/***         Private Global Variables           ***/

// Enum to track task type
enum taskType_e {
    TASK_EMPTY  = 0,                ///< Unallocated task
    TASK_TIMED,                     ///< Timed task, due when timer expires
    TASK_QUEUED,                    ///< Queued task, due at next call to tsMain()
    TASK_CONDITIONAL,               ///< Conditional task, due when (reg & mask) > 0
    TASK_CONDITIONAL_SH,            ///< Conditional task, single shot
};

// parameters unique to timed tasks
struct timedParams_s {
    timer_t dueTimer;               ///< timer indicating when task is due again
    uint16_t period;                ///< period for rescheduling repeating task
};

// parameters unique to queued tasks
struct queuedParams_s {
    ///< empty at this point
};

// parameters unique to conditional tasks
struct conditionalParams_s {
    uint8_t (*cb)(device_t *);      ///< callback function pointer to determine if task should run
};

// a single task
struct tsTask_s {
    task_ptr cb;
    enum taskType_e type;
    union {
        struct timedParams_s        timed;
        struct queuedParams_s       queued;
        struct conditionalParams_s  conditional;
    } params;
};

//! static memory for all task definitions
static struct tsTask_s tasks[TS_MAX_TASKS];



/***          Public Global Variables           ***/



/***        Private function portotypes         ***/

static task_t *allocTask(void);
static void releaseTask(task_t *t);



/***                   ISRs                     ***/



/***             Private Functions              ***/

/*  Return pointer to a new uninitialized task struct. Return NULL if out of memory.
 */
static task_t *allocTask(void)
{
    uint8_t i;
    
    for (i=0; i < sizeof tasks / sizeof tasks[0]; i++) {
        if (tasks[i].type == TASK_EMPTY) {
            return &tasks[i];
        }
    }
    fputs_P(PSTR("allocTask: Out of memory.\r\n"), debug);
    return NULL;
}

/*  Return used task to task memory pool
 */
static void releaseTask(task_t *t)
{
    t->type = TASK_EMPTY;
}



/***             Public Functions               ***/

/*  Create a new timed repeating task.
 *
 */
task_t *tsNewTimedTask(task_ptr task, int16_t period)
{
    task_t *new;
    
    // allocate new task, check if NULL
    new = allocTask();
    if (new == NULL || task == NULL) {
        tsReleaseTask(new);
        return NULL;
    }
    
    // initialize new task
    new->cb = task;
    new->type = TASK_TIMED;
    timerInit(&new->params.timed.dueTimer, period);
    new->params.timed.period = period;
    return new;
}

/*  Create a new timed one-shot task.
 *
 */
task_t *tsNewTimedSingleShotTask(task_ptr task, int16_t period)
{
    task_t *new;
    
    // allocate new task, check if NULL
    new = allocTask();
    if (new == NULL || task == NULL) {
        tsReleaseTask(new);
        return NULL;
    }
    
    // initialize new task
    new->cb = task;
    new->type = TASK_TIMED;
    timerInit(&new->params.timed.dueTimer, period);
    new->params.timed.period = 0;
    return new;
}

/*  Create a new queued task.
 *
 */
task_t *tsEnqueueTask(task_ptr task)
{
    task_t *new;
    
    // allocate new task, check if NULL
    new = allocTask();
    if (new == NULL || task == NULL) {
        tsReleaseTask(new);
        return NULL;
    }
    
    // initialize new task
    new->cb = task;
    new->type = TASK_QUEUED;
    return new;
}

/*  Create a new conditional task.\n
 *  \param task Function pointer to task.
 *  \param cb Function pointer to callback which dtermines if task should run.
 *  Callback should return nonzero to run task, 0 to not run task.
 */
task_t *tsNewConditionalTask(task_ptr task, uint8_t (*cb)(device_t *))
{
    task_t *new;
    
    // allocate new task, check if NULL
    new = allocTask();
    if (new == NULL || task == NULL) {
        tsReleaseTask(new);
        return NULL;
    }
    
    // initialize new task
    new->cb = task;
    new->type = TASK_CONDITIONAL;
    new->params.conditional.cb = cb;
    return new;
}

/*  Create a new conditional task.\n
 *  \param task Function pointer to task.
 *  \param cb Function pointer to callback which dtermines if task should run.
 *  Callback should return nonzero to run task, 0 to not run task.
 */
task_t *tsNewConditionalSingleShotTask(task_ptr task, uint8_t (*cb)(device_t *))
{
    task_t *new;
    
    // allocate new task, check if NULL
    new = allocTask();
    if (new == NULL || task == NULL) {
        tsReleaseTask(new);
        return NULL;
    }
    
    // initialize new task
    new->cb = task;
    new->type = TASK_CONDITIONAL_SH;
    new->params.conditional.cb = cb;
    return new;
}

/*  Release a task and recycle associated memory. Cancels any pending task callbacks.
 *
 */
void tsReleaseTask(task_t *t)
{
    if (t != NULL) {
        releaseTask(t);
    }
}

/*  Main task manager. Iterates through all tasks and manages task execution.
 *
 */
void tsMain(device_t *dev)
{
    uint8_t i;
    
    // first iterate through timed tasks
    for (i=0; i < sizeof tasks / sizeof tasks[0]; i++) {
        if (tasks[i].type == TASK_TIMED) {
            if (timerActive(&tasks[i].params.timed.dueTimer) == 0) {
                // task timer has elapsed
                //fprintf_P(debug, PSTR("t: %p\r\n"), tasks[i].cb);
                tasks[i].cb(dev);
                if (tasks[i].params.timed.period > 0) {
                    // task is not single shot, renew timer
                    timerAddPeriod(&tasks[i].params.timed.dueTimer, tasks[i].params.timed.period);
                } else {
                    // task is single shot, release it
                    tsReleaseTask(&tasks[i]);
                }
            }
        }
    }
    
    // next iterate through conditional and queued tasks
    for (i=0; i < sizeof tasks / sizeof tasks[0]; i++) {
        switch (tasks[i].type) {
            
            case TASK_QUEUED:
                //fprintf_P(debug, PSTR("t: %p\r\n"), tasks[i].cb);
                tasks[i].cb(dev);
                tsReleaseTask(&tasks[i]);
                break;
                
            case TASK_CONDITIONAL:
                if (tasks[i].params.conditional.cb(dev) != 0) {
                    // conditional check passed
                    //fprintf_P(debug, PSTR("t: %p\r\n"), tasks[i].cb);
                    tasks[i].cb(dev);
                }
                break;
                
            case TASK_CONDITIONAL_SH:
                if (tasks[i].params.conditional.cb(dev) != 0) {
                    // conditional check passed
                    //fprintf_P(debug, PSTR("t: %p\r\n"), tasks[i].cb);
                    tasks[i].cb(dev);
                    tsReleaseTask(&tasks[i]);
                }
                break;
                
            default:
                break;
        }
    }
}

