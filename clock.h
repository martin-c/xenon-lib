/*! \file
 *  clock.h
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

#include <inttypes.h>
#include <avr/io.h>



/***        Public Variables            ***/

/*! Enum defining desired clock source.
 */
enum clockSource_e {
    CLOCK_RC_2MEG   = 0,    ///< RC oscillator 2MHz
    CLOCK_RC_32MEG  = 1,    ///< RC oscillator 32MHz
    CLOCK_RC_32K    = 2,    ///< RC oscillator 32kHz
    CLOCK_XOSC      = 3,    ///< External crystal oscillator
    CLOCK_PLL       = 4,    ///< Internal PLL
};

/*! Enum defining PLL clock source.
 */
enum pllClockSource_e {
    PLL_RC_2MEG     = OSC_PLLSRC_RC2M_gc,   ///< RC Oscillator 2MHz
    PLL_RC_32MEG    = OSC_PLLSRC_RC32M_gc,  ///< RC Oscillator 32 MHz
    PLL_XOSC        = OSC_PLLSRC_XOSC_gc,   ///< External clock source
};

/*! Enum defining Prescaler A division factor.
 */
enum prescaleAFactor_e {
    PRESCALE_A_DIV1     = CLK_PSADIV_1_gc,
    PRESCALE_A_DIV2     = CLK_PSADIV_2_gc,
    PRESCALE_A_DIV4     = CLK_PSADIV_4_gc,
    PRESCALE_A_DIV8     = CLK_PSADIV_8_gc,
    PRESCALE_A_DIV16    = CLK_PSADIV_16_gc,
    PRESCALE_A_DIV32    = CLK_PSADIV_32_gc,
    PRESCALE_A_DIV64    = CLK_PSADIV_64_gc,
    PRESCALE_A_DIV128   = CLK_PSADIV_128_gc,
    PRESCALE_A_DIV256   = CLK_PSADIV_256_gc,
    PRESCALE_A_DIV512   = CLK_PSADIV_512_gc,
};

/*! Enum defining source for DFLL RC oscillator calibration
 */
enum clockDfllCalSource_e {
    DFLL_CAL_SOURCE_INT = 0,
    DFLL_CAL_SOURCE_EXT = 1,
};

/*! Enum defining XOSC frequency range
 */
enum clockXoscFreq_e {
    XOSC_FREQ_04TO2     = OSC_FRQRANGE_04TO2_gc,    ///< Frequency range 0.4MHz to 2MHz
    XOSC_FREQ_2TO9      = OSC_FRQRANGE_2TO9_gc,     ///< Frequency range 2MHz to 9MHz
    XOSC_FREQ_9TO12     = OSC_FRQRANGE_9TO12_gc,    ///< Frequency range 9MHz to 12MHz
    XOSC_FREQ_12TO16    = OSC_FRQRANGE_12TO16_gc,   ///< Frequency range 12MHz to 16MHz
};

/*! Enum defining XOSC selection and startup time
 */
enum clockXoscSelection_e {
    XOSC_EXTCLK         = OSC_XOSCSEL_EXTCLK_gc,        ///< External clock signal, 6 CLK startup time
    XOSC_32KHZ          = OSC_XOSCSEL_32KHz_gc,         ///< 32.768 kHz crystal on TOSC, 16K CLK startup time
    XOSC_XTAL_256       = OSC_XOSCSEL_XTAL_256CLK_gc,   ///< 0.4MHz - 16MHz crystal + 256 CLK startup time
    XOSC_XTAL_1K        = OSC_XOSCSEL_XTAL_1KCLK_gc,    ///< 0.4MHz - 16MHz crystal + 1k CLK startup time
    XOSC_XTAL_16K       = OSC_XOSCSEL_XTAL_16KCLK_gc,   ///< 0.4MHz - 16MHz crystal + 16k CLK startup time
};



/***        Public Functions            ***/

void clockEnableOsc(enum clockSource_e src);
void clockDisableOsc(enum clockSource_e src);
void clockConfigXosc(enum clockXoscFreq_e freq, enum clockXoscSelection_e sel);
void clockSetSource(enum clockSource_e src);
void clockConfigPll(enum pllClockSource_e src, uint8_t fact);
void clockSetPSA(enum prescaleAFactor_e ps);
int8_t clockEnableDfllCalibration(enum clockDfllCalSource_e src);
void clockDisbaleDfllCalibration(void);


