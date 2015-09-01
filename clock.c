/*! \file
 *  clock.c
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

#include <util/delay.h>
#include "clock.h"


/* The period to wait when starting a clock source for the DFLL hardware, in ms * 10.
 * The function will wait for the oscillator ready flag to be set for up to this long.
 * If the flag is not set by this interval the function will not attempt to enable the DFLL
 * and will return with error.
 * The start-up time of the crystal oscillator is up to 16k clock cycles, which
 * translates to approximately 1/2 second. We will wait for up to approximately
 * 1.0 seconds for oscillator to become stable.
 * Since the DFLL is not a critical function is does not make sense to block
 * until oscillator is stable, since in the case of an oscillator failure the
 * firmware will hang.
 */
#define OSCILLATOR_STARTUP_TIMEOUT      100



/*! Enable an oscillator.
 *  Function does not check if oscillator is stable before returning.
 *  \param mask Mask of oscillator(s) to enable.
 */
void clockEnableOsc(enum clockSource_e src)
{
    uint8_t mask = 1 << src;    // mask for control and status registers corresponding to new clock source
    OSC.CTRL |= mask;
}

/*! Disable an oscillator.
 *  \param mask Mask of oscillator(s) to disable.
 */
void clockDisableOsc(enum clockSource_e src)
{
    uint8_t mask = 1 << src;    // mask for control and status registers corresponding to new clock source
    OSC.CTRL &= ~mask;
}

/*! Configure external XTAL oscillator
 *  \param freq Enum defining XOSC frequency range.
 *  \param sel Enum defining XOSC selection and startup time.
 */
void clockConfigXosc(enum clockXoscFreq_e freq, enum clockXoscSelection_e sel)
{
    OSC.XOSCCTRL = freq | sel;
}

/*! Set XMega clock source.\n
 *  \param src New clock source.
 *  Function blocks until desired oscillator is stable, does not deactivate old clock source.
 */
void clockSetSource(enum clockSource_e src)
{
    clockDisbaleDfllCalibration();
    uint8_t mask = 1 << src;    // mask for control and status registers corresponding to new clock source
    
    /* Loop until OSC.STATUS bit is set indicating new oscillator is stable */
    while ((OSC.STATUS & mask) == 0) {
        ;
    }
    
    /* new oscillator is running & stable, set it as the new clock source */
    CCP = 0xD8;             // write signature to configuration change protection register
                            // interrupts ignored for 4 instruction cycles
    CLK.CTRL = src;         // set clock control register
}

/*! Configure the PLL with desired settings.\n
 *  \param src The input source of the PLL.
 *  \param fact The multiplication factor of the PLL. The multiplication factor can be in the range
 *  from 1x to 31x. The output frequency from the PLL should not exceed 200 MHz.
 *  The PLL must have a minimum output frequency of 10 MHz.
 */
void clockConfigPll(enum pllClockSource_e src, uint8_t fact)
{
    fact &= 0x1F;               // mask upper 3 bits of PLL factor, maximum PLL factor is 31.
    OSC.PLLCTRL = src | fact;   // set PLL configuration register.
}

/*! Configure Prescaler A division factor.\n
 *  \param ps Prescaler A division factor.
 */
void clockSetPSA(enum prescaleAFactor_e ps)
{
    uint8_t mask = CLK.PSCTRL;
    mask &= 0x03;           // clear all prescale A control bits
    mask |= ps;             // set desired prescale A control bits
    CCP = 0xD8;             // write signature to configuration change protection register
                            // interrupts ignored for 4 instruction cycles
    CLK.PSCTRL = mask;      // set prescale control register
}

/*! Enable DFLL RC oscillator calibration on current oscillator.\n
 *  \param src Calibration source (internal 32.768kHz or external 32.768kHz).
 *  Oscillator calibration is disabled automatically when clock source is changed.
 */
int8_t clockEnableDfllCalibration(enum clockDfllCalSource_e src)
{
    // enable required oscillator if necessary
    if (src == DFLL_CAL_SOURCE_INT) {
        OSC.CTRL |= OSC_RC32KEN_bm;             // enable internal oscillator for DFLL
        // set both 32MHZ and 2MHZ to use the internal RC32K oscillator
        OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc;
    } else {
        OSC.XOSCCTRL |= OSC_XOSCSEL_32KHz_gc;   // configure external oscillator
        OSC.CTRL |= OSC_XOSCEN_bm;              // enable external oscillator for DFLL
        // set both 32MHZ and 2MHZ DFLL to use external 32K oscillator
        #if defined(OSC_RC32MCREF_bm)
        OSC.DFLLCTRL = OSC_RC2MCREF_bm | OSC_RC32MCREF_bm;
        #elif defined(OSC_RC32MCREF_gm)
        OSC.DFLLCTRL = OSC_RC2MCREF_bm | OSC_RC32MCREF_XOSC32K_gc;
        #else
        #error Unknown DFLL Control register (DFLLCTRL) bitmasks.
        #endif
    }
    // wait for oscillator ready flag
    uint8_t i = 0;
    while ((src == DFLL_CAL_SOURCE_INT && bit_is_clear(OSC.STATUS, OSC_RC32KRDY_bp)) ||
            (src != DFLL_CAL_SOURCE_INT && bit_is_clear(OSC.STATUS, OSC_XOSCRDY_bp))) {
        if (i >= OSCILLATOR_STARTUP_TIMEOUT) {
            // oscillator is not ready after timeout period
            return -1;
        }
        i++;
        _delay_ms(10.0);
    }
    // select which DFLL to activate
    switch (CLK.CTRL & CLK_SCLKSEL_gm) {
        case CLK_SCLKSEL_RC2M_gc:
            // clock source RC 2MHz
            DFLLRC2M.CTRL = DFLL_ENABLE_bm;
            break;
        case CLK_SCLKSEL_RC32M_gc:
            // clock source RC 32MHz
            DFLLRC32M.CTRL = DFLL_ENABLE_bm;
            break;
        case CLK_SCLKSEL_PLL_gc:
            // clock source PLL
            switch (OSC.PLLCTRL & OSC_PLLSRC_gm) {
                case OSC_PLLSRC_RC2M_gc:
                    // PLL source RC 2MHz
                    DFLLRC2M.CTRL = DFLL_ENABLE_bm;
                    break;
                case OSC_PLLSRC_RC32M_gc:
                    // PLL source RC 32MHz
                    DFLLRC32M.CTRL = DFLL_ENABLE_bm;
                    break;
                default:
                    // unsupported PLL source
                    return -1;
            }
            break;
        default:
            // unsupported clock source for DFLL calibration
            return -1;
    }
    return 0;
}

/*! Disable DFLL RC oscillator calibration on any RC oscillator.\n
 *  Function is automatically called when oscillator is changed
 */
void clockDisbaleDfllCalibration(void)
{
    DFLLRC2M.CTRL = 0;
    DFLLRC32M.CTRL = 0;
}



