/*! \file
 *  rtc.c
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

/*  Note: THIS FILE IS DEPRECATED. Source is included for reference purposes only.
 *
 */

#include "rtc.h"
#include <string.h>
#include <util/atomic.h>



/***                Globals (Private)           ***/

/*  private implementation of a timer */
struct rtcTimer_s {
    enum {
        TIMER_STATE_EXPIRED = 0,        // timer ready for re-use
        TIMER_STATE_INACTIVE,           // not currently used
        TIMER_STATE_ACTIVE,             // timer running
        TIMER_STATE_ACTIVE_OVF,         // timer running, rtcCount overflow expected before timer expires
    } state;
    uint32_t expireCount;
};

/*  Memory allocated for all timers - maximum of 8 timers at once
 */
static rtcTimer_t timers[8];

/*  RTC single "tick" state */
static volatile enum {
    STATE_NOT_EXPIRED   = 0,
    STATE_EXPIRED       = 1,
} clockState;

/* RTC value */
static uint32_t rtcCount;

/* array of interrupt context callback functions (up to 4) */
static void (*isrCallbacks[4])(void);

/* array of standard context callbacks (up to 16) */
static struct callback_s {
    void (*fp)(void *);         // function pointer
    void        *param;         // function parameter pointer
    rtcTimer_t  execTimer;      // function execution timer
    uint16_t    period;         // function execution period
} callbacks[16];



/***        Private function portotypes         ***/

static rtcTimer_t *allocTimer(void);
static void initTimer(rtcTimer_t *t, uint16_t period);



/***                    ISR                     ***/

/*  RTC overflow ISR.
 *  Updates RTC counters/timers and executes registered callback functions.
 */
ISR(RTC_OVF_vect) {
    uint8_t i;
    
    /* update RTC counters/timers */
    rtcCount++;                         // increment 32-bit rtc counter
    clockState = STATE_EXPIRED;
    
    /* iterate through callbacks */
    for (i = 0; i < sizeof isrCallbacks / sizeof (isrCallbacks[0]); i++) {
        /* chack for valid callback */
        if (isrCallbacks[i] > 0) {
            (*isrCallbacks[i]) ();     // execute callback;
        }
    }
}



/***            Private Functions               ***/

/*  Allocate a new timer. Returns a pointer to a new rtcTimer_t
 */
static rtcTimer_t *allocTimer(void) {
    
    uint8_t i;
    
    // find the first expired timer and recycle it
    for (i=0; i< sizeof timers / sizeof timers[0]; i++) {
        if (timers[i].state == TIMER_STATE_EXPIRED) {
            //memset(&timers[i], 0, sizeof timers[0]);    // clear timer contents
            return &timers[i];                          // return address
        }
    }
    return NULL;    // no memory available for a new timer
}

/*  Initialize a new timer.
 */
static void initTimer(rtcTimer_t *t, uint16_t period) {
    
    uint32_t count;
    
    // initialize timer
    count = rtcGetCounter();
    // if expireCount overflows, so will timer. Therefore period should be preserved through overflow.
    t->expireCount = count + period;
    t->state = (t->expireCount > count) ? TIMER_STATE_ACTIVE : TIMER_STATE_ACTIVE_OVF;
}

/***            Public Functions            ***/
/*! Initialize RTC based on settings in intialization struct.
 *  \param prescale RTC clock prescaler value.
 *  \param clkSource RTC clock source.
 *  \param ovfInterrupt RTC overflow interrupt priority level.
 *  \param period RTC period register value, sets overflow rate.
 */
void rtcInit(enum rtcPrescale_e prescale,
             enum rtcClockSource_e clkSource,
             enum rtCOvfInterrupt_e ovfInterrupt,
             uint16_t period) {
    
    CLK.RTCCTRL = clkSource;        // set RTC clock source
    RTC.INTCTRL = ovfInterrupt;     // set overflow interrupt priority level
    RTC.PER = period;               // set period register
    /* From AVR 1314:
     * The four synchronized registers are CTRL, CNT, PER and COMP. Note that the PER register does
     * not have a separate synchronizer; the synchronization of PER is triggered by synchronization
     * of the CNT, CTRL or COMP registers.
     */
    RTC.CTRL = prescale;            // set prescaler
}

/*! Start initialized RTC
 *
 */
void rtcStart(void) {
    clockState = STATE_NOT_EXPIRED;
    rtcCount = 0;                   // reset RTC counter
    CLK.RTCCTRL |= CLK_RTCEN_bm;    // set enable bit for RTC clock signal
}

/*! Stop RTC
 *
 */
void rtcStop(void) {
    CLK.RTCCTRL &= ~(CLK_RTCEN_bm); // clear enable bit for RTC clock signal
}

/*! Get new timer with preset period, allocates new timer and starts timer.
 *  \param period Timer period in RTC ticks.
 *  \return Returns pointer to new timer or NULL if timer memory is full.
 */
rtcTimer_t *rtcGetNewTimer(uint16_t period) {
    
    rtcTimer_t *t;
    
    t = allocTimer();
    if (t == NULL) {
        return NULL;
    }
    initTimer(t, period);
    return t;
}

/*! Release timer which is no longer needed.
 *  Should be called on all timers which are no longer needed to make space for new timers.
 *  \param timer Pointer to timer to release.
 */
void rtcReleaseTimer(rtcTimer_t *timer) {
    
    if (timer != NULL) {
        timer->state = TIMER_STATE_EXPIRED;
    }
}

/*! Check if a timer is active.
 *  \param timer Pointer to timer to check.
 *  \return Returns 0 if timer has elapsed, or 1 if timer is active.
 */
uint8_t rtcTimerActive(rtcTimer_t *timer) {
    
    uint32_t rtc;
    
    if (timer == NULL) {
        return 0;
    }
    
    rtc = rtcGetCounter();
    
    switch (timer->state) {
        // normal timer logic
        case TIMER_STATE_ACTIVE:
            if (rtc < timer->expireCount) {
                return 1;
            }
            break;
        // if RTC is expected to overflow before timer elapses, then check for overflow in addition
        // to checking timer logic.
        case TIMER_STATE_ACTIVE_OVF:
            if (rtc > UINT32_MAX - UINT16_MAX ||
                rtc < timer->expireCount) {
                return 1;
            }
            break;
        default:
            break;
    }
    return 0;
}

/*! Get 32-bit RTC counter.
 *  
 *  \return 32-bit RTC counter value.\n
 *  Counter starts at 0 on RTC initialization, and increments by 1 on every RTC overflow.
 *  Currently increments every 1/4096 seconds for a total range of ~291.27 hours
 */
uint32_t rtcGetCounter(void) {
    uint32_t c;
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        c = rtcCount;
    }
    return c;
}

/*! Check for RTC overflow
 *
 */
uint8_t rtcTick(void) {
    if (clockState != STATE_EXPIRED) {
        return 0;
    }
    clockState = STATE_NOT_EXPIRED;                 // reset state
    return 1;
}

/*! Register callback function called on every RTC tick, from interrupt context.\n
 *  \param cb Pointer to callback function with no parameters returning void which is to be added.
 */
void rtcRegisterCallbackIsr(void (*cb)(void)) {
    
    uint8_t i;
    
    for (i = 0; i < sizeof isrCallbacks / sizeof (isrCallbacks[1]); i++) {
        if (isrCallbacks[i] == cb) {        // check if callback already exists
            break;
        } else if (isrCallbacks[i] == 0) {
            isrCallbacks[i] = cb;           // add callback to array and break from loop
            break;
        }
    }    
}

/*! Register callback function called on every n RTC ticks, from standard context.\n
 *  Function will be called with pointer to parameter specified every n counter ticks.
 *  The exact timing is approximate. Functions are executed in order of being added to callback
 *  array and only when program execution path is relinquished to rtcTaskScheduler.
 *  \param cb Pointer to callback function which is to be added.
 *  \param param Pointer to parameter to be passed to callback function.
 *  \param n Callback will be called every n timer ticks.
 */
void rtcRegisterCallback(void (*cb)(void *), void *param, uint16_t period) {
    
    uint8_t i;
    
    for (i = 0; i < sizeof callbacks / sizeof (callbacks[1]); i++) {
        if (callbacks[i].fp == cb) {        // check if callback already exists
            break;
        }
        if (callbacks[i].fp == NULL) {      // check for empty callback slot
            callbacks[i].fp = cb;           // add callback function to array
            callbacks[i].param = param;
            callbacks[i].period = period;
            initTimer(&callbacks[i].execTimer, period);
            break;
        }
    }
}

/*! Release registered interrupt context callback function.\n
 *  \param cb Pointer to callback function which is to be released.
 */
void rtcReleaseCallbackIsr(void (*cb)(void)) {
    
    uint8_t i;
    
    for (i = 0; i < sizeof isrCallbacks / sizeof (isrCallbacks[1]); i++) {
        /* chack for matching callback */
        if (isrCallbacks[i] == cb) {
            isrCallbacks[i] = 0;   // remove callback from array and break from loop
            break;
        }
    }    
}

/*! Release registered callback function called from standard context.\n
 *  \param cb Pointer to callback function which is to be released.
 */
void rtcReleaseCallback(void (*cb)(void *)) {
    
    uint8_t i;
    
    for (i = 0; i < sizeof callbacks / sizeof (callbacks[1]); i++) {
        /* chack for empty callback */
        if (callbacks[i].fp == cb) {
            callbacks[i].fp = NULL;     // set function pointer to NULL to indicate unused callback
            break;
        }
    }    
}

/*! Task scheduler
 *
 */
void rtcTaskScheduler(void) {
    
    uint8_t i;
    
    // iterate through callbacks
    for (i = 0; i < sizeof callbacks / sizeof (callbacks[1]); i++) {
        /* chack if callback is due */
        if (callbacks[i].fp != NULL) {
            if (rtcTimerActive(&callbacks[i].execTimer) == 0) {
                // reset new timer
                initTimer(&callbacks[i].execTimer, callbacks[i].period);
                // call callback function
                callbacks[i].fp(callbacks[i].param);
            }
        }
    }
}



