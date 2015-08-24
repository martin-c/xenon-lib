/*! \file
 *  rtc-hardware.c
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

#include "rtc-hardware.h"
#include <string.h>
#include <util/atomic.h>



/***                Globals (Private)           ***/

//  RTC value
static uint16_t rtcCount;



/***          Public Global Variables           ***/



/***        Private function portotypes         ***/



/***                    ISR                     ***/

/*  RTC overflow ISR.
 *  Updates RTC counters/timers and executes registered callback functions.
 */
ISR(RTC_OVF_vect) {
    
    // increment RTC counter
    rtcCount++;
}



/***            Private Functions               ***/



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
    rtcCount = 0;                   // reset RTC counter
    CLK.RTCCTRL |= CLK_RTCEN_bm;    // set enable bit for RTC clock signal
}

/*! Stop RTC
 *
 */
void rtcStop(void) {
    CLK.RTCCTRL &= ~(CLK_RTCEN_bm); // clear enable bit for RTC clock signal
}

/*  Initialize a new timer.
 *  \param t Pointer to timer to initialize
 *  \param period Period in RTC "ticks" (overflows) for the new timer. Maximum value is 0x7FFF ticks.
 */
void rtcTimerInit(rtcTimer_t *t, uint16_t period) {
    
    uint16_t count;
    
    if (t == NULL) {
        return;
    }
    period &= 0x7FFF;           // mask highest bit of period - this is reserved for overflow tracking
    // initialize timer
    count = rtcGetCounter();
    // if expireCount overflows, so will timer. Therefore period should be preserved through overflow.
    t->expireCount = count + period;
}

/*  Add a time period to an active timer.
 *  \param t Pointer to timer to add time period.
 *  \param period Period in RTC "ticks" (overflows) to add.
 */
void rtcTimerAddPeriod(rtcTimer_t *t, uint16_t period) {
    
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
uint8_t rtcTimerActive(rtcTimer_t *t) {
    
    uint16_t rtc;
    
    if (t == NULL) {
        return 0;
    }
    
    rtc = rtcGetCounter();
    // compare overflow bits
    if ((t->expireCount & 0x8000) == (rtc & 0x8000)) {
        if ((rtc & 0x7FFF) < (t->expireCount & 0x7FFF)) {
            return 1;
        }
    } else {
        if ((rtc & 0x7FFF) >= (t->expireCount & 0x7FFF)) {
            return 1;
        }
    }
    return 0;
}

/*! Get 16-bit RTC counter.
 *  
 *  \return 16-bit RTC counter value.\n
 *  Counter starts at 0 on RTC initialization, and increments by 1 on every RTC overflow.
 */
uint16_t rtcGetCounter(void) {
    
    uint16_t c;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        c = rtcCount;
    }
    return c;
}

