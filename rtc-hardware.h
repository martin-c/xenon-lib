/*! \file
 *  rtc-hardware.h
 *  xenon-lib
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Martin Clemons
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



/***            Public Variables            ***/

/*! RTC prescaler selection */
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

/*! RTC clock source selection */
enum rtcClockSource_e {
    RTC_CLOCK_SOURCE_ULP    = CLK_RTCSRC_ULP_gc,    ///< 1kHz from internal 32kHz ULP
    RTC_CLOCK_SOURCE_TOSC   = CLK_RTCSRC_TOSC_gc,   ///< 1.024kHz from 32.768kHz crystal on TOSC
    RTC_CLOCK_SOURCE_RCOSC  = CLK_RTCSRC_RCOSC_gc,  ///< 1.024kHz from internal 32.768kHz RC
    RTC_CLOCK_SOURCE_TOSC32 = CLK_RTCSRC_TOSC32_gc, ///< 32.768kHz from 32.768kHz crystal on TOSC
};

/*! RTC overflow interrupt priority level */
enum rtCOvfInterrupt_e {
    RTC_OVFINT_OFF      = RTC_OVFINTLVL_OFF_gc,
    RTC_OVFINT_LO       = RTC_OVFINTLVL_LO_gc,
    RTC_OVFINT_MED      = RTC_OVFINTLVL_MED_gc,
    RTC_OVFINT_HI       = RTC_OVFINTLVL_HI_gc,
};

/*  implementation of a timer */
struct rtcTimer_s {
    uint16_t expireCount;
};

/*! RTC Timer type */
typedef struct rtcTimer_s rtcTimer_t;



/***            Public Functions            ***/
void rtcInit(enum rtcPrescale_e prescale,
             enum rtcClockSource_e clkSource,
             enum rtCOvfInterrupt_e ovfInterrupt,
             uint16_t period);
void rtcStart(void);
void rtcStop(void);
void rtcTimerInit(rtcTimer_t *t, uint16_t period);
void rtcTimerAddPeriod(rtcTimer_t *t, uint16_t period);
uint8_t rtcTimerActive(rtcTimer_t *timer);
uint16_t rtcGetCounter(void);

