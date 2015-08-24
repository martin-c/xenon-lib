/*! \file
 *  timer.h
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

#pragma once

#include "lib-config.h"
#include <inttypes.h>
#include <avr/io.h>



/***            Public Variables            ***/

/* Hardware timer overflow flag - set on each HW timer period.
 */
extern volatile uint8_t hwTimerF;

/*! RTC timer prescaler selection */
enum rtcPrescale_e {
    RTC_PRESCALE_OFF    = RTC_PRESCALER_OFF_gc,
    RTC_PRESCALE_1      = RTC_PRESCALER_DIV1_gc,
    RTC_PRESCALE_2      = RTC_PRESCALER_DIV2_gc,
    RTC_PRESCALE_8      = RTC_PRESCALER_DIV8_gc,
    RTC_PRESCALE_16     = RTC_PRESCALER_DIV16_gc,
    RTC_PRESCALE_64     = RTC_PRESCALER_DIV64_gc,
    RTC_PRESCALE_256    = RTC_PRESCALER_DIV256_gc,
    RTC_PRESCALE_1024   = RTC_PRESCALER_DIV1024_gc,
};

/*! RTC timer clock source selection */
enum rtcClockSource_e {
    RTC_CLOCK_SOURCE_ULP    = CLK_RTCSRC_ULP_gc,    ///< 1kHz from internal 32kHz ULP
    RTC_CLOCK_SOURCE_TOSC   = CLK_RTCSRC_TOSC_gc,   ///< 1.024kHz from 32.768kHz crystal on TOSC
    RTC_CLOCK_SOURCE_RCOSC  = CLK_RTCSRC_RCOSC_gc,  ///< 1.024kHz from internal 32.768kHz RC
    RTC_CLOCK_SOURCE_TOSC32 = CLK_RTCSRC_TOSC32_gc, ///< 32.768kHz from 32.768kHz crystal on TOSC
};

/*! RTC timer overflow interrupt priority level */
enum rtcOvfInterrupt_e {
    RTC_OVFINT_OFF      = RTC_OVFINTLVL_OFF_gc,
    RTC_OVFINT_LO       = RTC_OVFINTLVL_LO_gc,
    RTC_OVFINT_MED      = RTC_OVFINTLVL_MED_gc,
    RTC_OVFINT_HI       = RTC_OVFINTLVL_HI_gc,
};

/*! Timer clock selection */
enum timerClockSelect_e {
    TIMER_CLOCK_OFF     = TC_CLKSEL_OFF_gc,
    TIMER_CLOCK_DIV1    = TC_CLKSEL_DIV1_gc,
    TIMER_CLOCK_DIV2    = TC_CLKSEL_DIV2_gc,
    TIMER_CLOCK_DIV4    = TC_CLKSEL_DIV4_gc,
    TIMER_CLOCK_DIV8    = TC_CLKSEL_DIV8_gc,
    TIMER_CLOCK_DIV64   = TC_CLKSEL_DIV64_gc,
    TIMER_CLOCK_DIV256  = TC_CLKSEL_DIV256_gc,
    TIMER_CLOCK_DIV1024 = TC_CLKSEL_DIV1024_gc,
};

/*! timer overflow interrupt priority level */
enum timerOvfInterrupt_e {
    TIMER_OVFINT_OFF    = TC_OVFINTLVL_OFF_gc,
    TIMER_OVFINT_LO     = TC_OVFINTLVL_LO_gc,
    TIMER_OVFINT_MED    = TC_OVFINTLVL_MED_gc,
    TIMER_OVFINT_HI     = TC_OVFINTLVL_HI_gc,
};

/*! Timer type
 *  The internal workings of the timer are private. The definition is here because the
 *  timer library does not actually allocate memory for any timers, so the size of a timer
 *  type must be known externally.
 */
typedef struct timer_s {
    uint16_t expireCount;
} timer_t;



/***            Public Functions            ***/
#ifdef XENON_RTC_AS_TIMER_SOURCE
void timerRtcInit(enum rtcPrescale_e prescale,
                  enum rtcClockSource_e clkSource,
                  enum rtcOvfInterrupt_e ovfInterrupt,
                  uint16_t period);
void timerRtcStart(void);
void timerRtcStop(void);
#endif
void timerHwStart(enum timerClockSelect_e clock,
                  enum timerOvfInterrupt_e ovfInterrupt,
                  uint16_t period);
void timerHwStop(void);
void timerInit(timer_t *t, uint16_t period);
void timerAddPeriod(timer_t *t, uint16_t period);
uint8_t timerActive(timer_t *t);
uint16_t timerGetCounter(void);



