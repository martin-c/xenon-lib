/*! \file
 *  timer.c
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

#include "timer.h"
#include "lib-config.h"
#include <string.h>
#include <util/atomic.h>



/***                Globals (Private)           ***/

//  Internal timer value
static volatile uint16_t timerCount;



/***          Public Global Variables           ***/

volatile uint8_t hwTimerF;



/***        Private Function Prototypes         ***/

static inline void timerTick(void);



/***                    ISR                     ***/

/*  Timer overflow ISR.
 *  Updates internal timerCount and wakes device from sleep.
 */
ISR(XENON_TIMER_TC_ISR) {
    timerTick();
}

#ifdef XENON_RTC_AS_TIMER_SOURCE
ISR(RTC_OVF_vect) {
    timerTick();
}
#endif


/***            Private Functions               ***/

/* Each software timer tick
 */
static inline void __attribute__ ((always_inline)) timerTick(void) {
    timerCount++;
    hwTimerF = 1;
}



/***            Public Functions            ***/

#ifdef XENON_RTC_AS_TIMER_SOURCE
/*! Initialize RTC as timer based on setting provided.
 *  \param prescale RTC clock prescaler value.
 *  \param clkSource RTC clock source.
 *  \param ovfInterrupt RTC overflow interrupt priority level.
 *  \param period RTC period register value, sets overflow rate.
 */
void timerRtcInit(enum rtcPrescale_e prescale,
                  enum rtcClockSource_e clkSource,
                  enum rtcOvfInterrupt_e ovfInterrupt,
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
void timerRtcStart(void) {
    timerCount = 0;                   // reset RTC counter
    CLK.RTCCTRL |= CLK_RTCEN_bm;    // set enable bit for RTC clock signal
}

/*! Stop RTC
 *
 */
void timerRtcStop(void) {
    CLK.RTCCTRL &= ~(CLK_RTCEN_bm); // clear enable bit for RTC clock signal
}
#endif

/*! Initialize and start timer peripheral based on settings provided.
 *  \param timer Pointer to timer hardware peripheral to utilize for timer.\n
 *  *** NOTE: timer specified must match timer specified in TIMER_ISR macro in header file ***
 *  \param clock Timer peripheral clock source.
 *  \param ovfInterrupt Timer overflow interrupt priority level.
 *  \param period Timer period register value.
 */
void timerHwStart(enum timerClockSelect_e clock,
                  enum timerOvfInterrupt_e ovfInterrupt,
                  uint16_t period) {
    
    hwTimerF = 0;
    XENON_TIMER_TC.CTRLB = TC_WGMODE_NORMAL_gc;
    XENON_TIMER_TC.INTCTRLA = ovfInterrupt;
    XENON_TIMER_TC.INTCTRLB = 0;
    XENON_TIMER_TC.CNT = 0;
    XENON_TIMER_TC.PER = period;
    XENON_TIMER_TC.CTRLA = clock;
}

void timerHwStop(void) {

    XENON_TIMER_TC.CTRLA = 0;
}

/*  Initialize a new timer.
 *  \param t Pointer to timer to initialize
 *  \param period Period in timer "ticks" (overflows) for the new timer. Maximum value is 0x7FFF ticks.
 */
void timerInit(timer_t *t, uint16_t period) {
    
    uint16_t count;
    
    if (t == NULL) {
        return;
    }
    period &= 0x7FFF;           // mask highest bit of period - this is reserved for overflow tracking
    // initialize timer
    count = timerGetCounter();
    // if expireCount overflows, so will timer. Therefore period should be preserved through overflow.
    t->expireCount = count + period;
}

/*  Add a time period to an active timer.
 *  \param t Pointer to timer to add time period.
 *  \param period Period in timer "ticks" (overflows) to add.
 */
void timerAddPeriod(timer_t *t, uint16_t period) {
    
    if (t == NULL) {
        return;
    }
    period &= 0x7FFF;
    t->expireCount += period;
}

/*! Check if a timer is active.
 *  \param timer Pointer to timer to check.
 *  \return Returns 0 if timer has elapsed, or 1 if timer is active.
 */
uint8_t timerActive(timer_t *t) {
    
    uint16_t count;
    
    if (t == NULL) {
        return 0;
    }
    
    count = timerGetCounter();
    // compare overflow bits
    if ((t->expireCount & 0x8000) == (count & 0x8000)) {
        if ((count & 0x7FFF) < (t->expireCount & 0x7FFF)) {
            return 1;
        }
    } else {
        if ((count & 0x7FFF) >= (t->expireCount & 0x7FFF)) {
            return 1;
        }
    }
    return 0;
}

/*! Get internal 16-bit timer value (atomic operation).
 *
 *  \return 16-bit internal counter value.\n
 *  Counter starts at 0 on timer initialization, and increments by 1 on every timer peripheral overflow.
 */
uint16_t timerGetCounter(void) {
    
    uint16_t c;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        c = timerCount;
    }
    return c;
}

