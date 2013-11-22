/*! \file
 *  quadrature.c
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

/*  Note: THIS FILE IS DEPRECATED. Source is included for reference purposes only.
 *
 */

#include "quadrature.h"
#include <avr/io.h>



/***                Definitions                 ***/

// encoder power
#define ENCODER_POWERUP     PORTB.OUTSET = PIN3_bm
#define ENCODER_POWERDOWN   PORTB.OUTCLR = PIN3_bm;



/***         Private Global Variables           ***/



/***          Public Global Variables           ***/



/***        Private function portotypes         ***/



/***                   ISRs                     ***/



/***             Private Functions              ***/



/***             Public Functions               ***/

/*! Initialize quadrature decoder using event channel 0 and TCC1
 *
 */
void quadratureInit(void) {
    
    // configure pins
    PORTA.PIN3CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;    // enable pull-up, low level sense
    PORTA.PIN4CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
    // configure event channel 0 for quardature input on PA3, PA4
    EVSYS.CH0MUX = EVSYS_CHMUX_PORTA_PIN3_gc;
    // enable quadrature decoding, digital filter 2 samples
    EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
    // TCD1 event action quadrature decode, event channel 0
    TCC1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
}

/*! Enable quardrature decoder, enable encoder power
 *
 */
void quadratureEnable(void) {
    
    ENCODER_POWERUP;
    // enable timer/counter C1
    TCC1.CTRLA = TC_CLKSEL_DIV1_gc;
}

/*! Disable quadrature decoder, shut down encoder power
 *
 */
void quadratureDisable(void) {
    
    // disable timer/counter C1
    TCC1.CTRLA = TC_CLKSEL_OFF_gc;
    ENCODER_POWERDOWN;
}